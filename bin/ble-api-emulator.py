#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "dbus-next==0.2.3",
#   "meshtastic[cli]>=2.7.11,<3",
# ]
# ///
# ruff: noqa: F821

import argparse
import asyncio
import logging
import re
import select
import signal
import struct
import subprocess
import time
from collections import deque
from pathlib import Path

from dbus_next import BusType, DBusError, Variant
from dbus_next.aio import MessageBus
from dbus_next.constants import PropertyAccess
from dbus_next.service import ServiceInterface, dbus_property, method
from meshtastic.protobuf import (
    channel_pb2,
    config_pb2,
    device_ui_pb2,
    mesh_pb2,
    module_config_pb2,
    portnums_pb2,
    telemetry_pb2,
)

MESH_SERVICE_UUID = "6ba1b218-15a8-461f-9fa8-5dcae273eafd"
FROM_RADIO_UUID = "2c55e69e-4993-11ed-b878-0242ac120002"
TO_RADIO_UUID = "f75c76d2-129e-4dad-a1dd-7866124401e7"
FROM_NUM_UUID = "ed9da18c-a800-4f66-a670-aa7547e34453"
LOG_RADIO_UUID = "5a3d6e49-06e6-4423-9944-e9de8cdf9547"

APP_PATH = "/org/meshtastic/ble_emulator"
SERVICE_PATH = f"{APP_PATH}/service0"
ADVERTISEMENT_PATH = "/org/meshtastic/ble_advertisement0"
AGENT_PATH = "/org/meshtastic/ble_agent0"
BLUEZ_SERVICE = "org.bluez"

LOGGER = logging.getLogger("ble-api-emulator")


def protobuf_frame(field, value):
    frame = mesh_pb2.FromRadio()
    target = getattr(frame, field)
    if hasattr(target, "CopyFrom"):
        target.CopyFrom(value)
    else:
        setattr(frame, field, value)
    return frame.SerializeToString()


class MeshtasticProtocolEmulator:
    def __init__(self, node_num):
        self.node_num = node_num
        self.started_at = time.monotonic()
        self.frames = deque()
        self.active_frame = b""
        self.from_num = 0
        self.from_num_callback = None
        self.log_callback = None

    def set_callbacks(self, from_num_callback, log_callback):
        self.from_num_callback = from_num_callback
        self.log_callback = log_callback

    def log(self, message, level=logging.INFO):
        LOGGER.log(level, message)
        if self.log_callback:
            self.log_callback(message, level)

    def queue_frame(self, frame, notify=False):
        self.frames.append(frame)
        if notify:
            self.from_num = (self.from_num + 1) & 0xFFFFFFFF
            if self.from_num_callback:
                self.from_num_callback(self.from_num)

    def read_from_radio(self, offset):
        if offset == 0:
            self.active_frame = self.frames.popleft() if self.frames else b""
            if self.active_frame:
                decoded = mesh_pb2.FromRadio()
                decoded.ParseFromString(self.active_frame)
                self.log(
                    f"FromRadio read: {decoded.WhichOneof('payload_variant')} "
                    f"({len(self.active_frame)} bytes), queued={len(self.frames)}",
                    logging.DEBUG,
                )
            else:
                self.log("FromRadio read: queue empty", logging.DEBUG)

        if offset > len(self.active_frame):
            raise DBusError(
                "org.bluez.Error.InvalidOffset",
                "Read offset is beyond the current frame",
            )
        return self.active_frame[offset:]

    def handle_to_radio(self, payload):
        request = mesh_pb2.ToRadio()
        try:
            request.ParseFromString(payload)
        except Exception as error:
            self.log(
                f"Malformed ToRadio ({len(payload)} bytes): {error}", logging.ERROR
            )
            raise DBusError(
                "org.bluez.Error.InvalidValueLength", "Invalid ToRadio protobuf"
            ) from error

        variant = request.WhichOneof("payload_variant")
        self.log(f"ToRadio write: {variant} ({len(payload)} bytes)")

        if variant == "want_config_id":
            self._queue_configuration(request.want_config_id)
        elif variant == "heartbeat":
            status = mesh_pb2.QueueStatus(res=0, free=16, maxlen=16)
            self.queue_frame(protobuf_frame("queueStatus", status), notify=True)
        elif variant == "packet":
            self._handle_mesh_packet(request.packet)
        elif variant == "disconnect":
            self.log("Client requested API disconnect")
        elif variant is None:
            self.log("Empty ToRadio protobuf", logging.WARNING)
        else:
            self.log(f"Unsupported ToRadio variant: {variant}", logging.WARNING)

    def _queue_configuration(self, nonce):
        self.frames.clear()
        self.active_frame = b""
        for frame in self._configuration_frames(nonce):
            self.queue_frame(frame)
        self.log(
            f"Queued configuration nonce={nonce}: {len(self.frames)} FromRadio frames"
        )

    def _configuration_frames(self, nonce):
        node_id = f"!{self.node_num:08x}"
        uptime = int(time.monotonic() - self.started_at)

        my_info = mesh_pb2.MyNodeInfo(
            my_node_num=self.node_num,
            min_app_version=30200,
            device_id=b"BLE-API-EMULATOR",
            pio_env="linux-ble-emulator",
            firmware_edition=mesh_pb2.FirmwareEdition.VANILLA,
            nodedb_count=1,
        )
        yield protobuf_frame("my_info", my_info)

        ui_config = device_ui_pb2.DeviceUIConfig(
            version=1,
            screen_brightness=100,
            screen_timeout=30,
        )
        yield protobuf_frame("deviceuiConfig", ui_config)

        node_info = mesh_pb2.NodeInfo(
            num=self.node_num,
            last_heard=int(time.time()),
            hops_away=0,
            is_favorite=True,
        )
        node_info.user.id = node_id
        node_info.user.long_name = "BLE API Emulator"
        node_info.user.short_name = "EMU"
        node_info.user.hw_model = mesh_pb2.HardwareModel.PORTDUINO
        node_info.device_metrics.CopyFrom(
            telemetry_pb2.DeviceMetrics(
                battery_level=100,
                voltage=5.0,
                channel_utilization=0.0,
                air_util_tx=0.0,
                uptime_seconds=uptime,
            )
        )
        yield protobuf_frame("node_info", node_info)

        metadata = mesh_pb2.DeviceMetadata(
            firmware_version="2.8.0.ble-emulator",
            device_state_version=25,
            canShutdown=False,
            hasBluetooth=True,
            role=config_pb2.Config.DeviceConfig.Role.CLIENT,
            position_flags=0,
            hw_model=mesh_pb2.HardwareModel.PORTDUINO,
            hasPKC=False,
        )
        yield protobuf_frame("metadata", metadata)

        channel = channel_pb2.Channel(index=0, role=channel_pb2.Channel.Role.PRIMARY)
        channel.settings.psk = b"\x01"
        channel.settings.name = "LongFast"
        yield protobuf_frame("channel", channel)

        configs = [
            config_pb2.Config(
                device=config_pb2.Config.DeviceConfig(
                    role=config_pb2.Config.DeviceConfig.Role.CLIENT,
                    node_info_broadcast_secs=10800,
                    tzdef="UTC0",
                )
            ),
            config_pb2.Config(
                position=config_pb2.Config.PositionConfig(
                    position_broadcast_secs=3600,
                    gps_update_interval=120,
                    gps_mode=config_pb2.Config.PositionConfig.GpsMode.NOT_PRESENT,
                )
            ),
            config_pb2.Config(
                power=config_pb2.Config.PowerConfig(
                    wait_bluetooth_secs=60,
                    sds_secs=0xFFFFFFFF,
                    ls_secs=300,
                    min_wake_secs=10,
                )
            ),
            config_pb2.Config(
                network=config_pb2.Config.NetworkConfig(
                    ntp_server="meshtastic.pool.ntp.org",
                )
            ),
            config_pb2.Config(
                display=config_pb2.Config.DisplayConfig(
                    screen_on_secs=60,
                    enable_message_bubbles=True,
                )
            ),
            config_pb2.Config(
                lora=config_pb2.Config.LoRaConfig(
                    use_preset=True,
                    modem_preset=config_pb2.Config.LoRaConfig.ModemPreset.LONG_FAST,
                    region=config_pb2.Config.LoRaConfig.RegionCode.KZ_433,
                    hop_limit=3,
                    tx_enabled=True,
                    tx_power=10,
                )
            ),
            config_pb2.Config(
                bluetooth=config_pb2.Config.BluetoothConfig(
                    enabled=True,
                    mode=config_pb2.Config.BluetoothConfig.PairingMode.NO_PIN,
                )
            ),
            config_pb2.Config(
                security=config_pb2.Config.SecurityConfig(
                    serial_enabled=True,
                    debug_log_api_enabled=True,
                )
            ),
            config_pb2.Config(sessionkey=config_pb2.Config.SessionkeyConfig()),
            config_pb2.Config(device_ui=device_ui_pb2.DeviceUIConfig()),
        ]
        for config in configs:
            yield protobuf_frame("config", config)

        for field in module_config_pb2.ModuleConfig.DESCRIPTOR.fields:
            module_config = module_config_pb2.ModuleConfig()
            getattr(module_config, field.name).SetInParent()
            yield protobuf_frame("moduleConfig", module_config)

        yield protobuf_frame("config_complete_id", nonce)

    def _handle_mesh_packet(self, packet):
        port_name = portnums_pb2.PortNum.Name(packet.decoded.portnum)
        payload_text = ""
        if packet.decoded.portnum == portnums_pb2.PortNum.TEXT_MESSAGE_APP:
            payload_text = packet.decoded.payload.decode("utf-8", errors="replace")

        self.log(
            f"MeshPacket id=0x{packet.id:08x} to=0x{packet.to:08x} "
            f"port={port_name} want_ack={packet.want_ack} payload={payload_text!r}"
        )

        status = mesh_pb2.QueueStatus(
            res=0,
            free=16,
            maxlen=16,
            mesh_packet_id=packet.id,
        )
        self.queue_frame(protobuf_frame("queueStatus", status), notify=True)

        echoed = mesh_pb2.MeshPacket()
        echoed.CopyFrom(packet)
        setattr(echoed, "from", self.node_num)
        echoed.want_ack = False
        echoed.rx_time = int(time.time())
        self.queue_frame(protobuf_frame("packet", echoed), notify=True)

        if packet.want_ack:
            routing = mesh_pb2.Routing(error_reason=mesh_pb2.Routing.Error.NONE)
            acknowledgement = mesh_pb2.MeshPacket(
                to=self.node_num,
                id=(packet.id + 1) & 0xFFFFFFFF,
                priority=mesh_pb2.MeshPacket.Priority.ACK,
            )
            setattr(acknowledgement, "from", self.node_num)
            acknowledgement.decoded.portnum = portnums_pb2.PortNum.ROUTING_APP
            acknowledgement.decoded.payload = routing.SerializeToString()
            acknowledgement.decoded.request_id = packet.id
            self.queue_frame(protobuf_frame("packet", acknowledgement), notify=True)


class GattService(ServiceInterface):
    def __init__(self, uuid):
        super().__init__("org.bluez.GattService1")
        self.uuid = uuid

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> "s":
        return self.uuid

    @dbus_property(access=PropertyAccess.READ)
    def Primary(self) -> "b":
        return True


class GattCharacteristic(ServiceInterface):
    def __init__(self, uuid, flags):
        super().__init__("org.bluez.GattCharacteristic1")
        self.uuid = uuid
        self.flags = flags
        self.value = b""
        self.notifying = False

    @dbus_property(access=PropertyAccess.READ)
    def UUID(self) -> "s":
        return self.uuid

    @dbus_property(access=PropertyAccess.READ)
    def Service(self) -> "o":
        return SERVICE_PATH

    @dbus_property(access=PropertyAccess.READ)
    def Flags(self) -> "as":
        return self.flags

    @dbus_property(access=PropertyAccess.READ)
    def Value(self) -> "ay":
        return self.value

    @dbus_property(access=PropertyAccess.READ)
    def Notifying(self) -> "b":
        return self.notifying

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":
        raise DBusError(
            "org.bluez.Error.NotSupported", "Characteristic is not readable"
        )

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}"):
        raise DBusError(
            "org.bluez.Error.NotSupported", "Characteristic is not writable"
        )

    @method()
    def StartNotify(self):
        if "notify" not in self.flags and "indicate" not in self.flags:
            raise DBusError(
                "org.bluez.Error.NotSupported", "Characteristic does not notify"
            )
        if self.notifying:
            return
        self.notifying = True
        self.emit_properties_changed({"Notifying": True})
        LOGGER.info("Notifications enabled for %s", self.uuid)

    @method()
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        self.emit_properties_changed({"Notifying": False})
        LOGGER.info("Notifications disabled for %s", self.uuid)

    @staticmethod
    def offset(options):
        value = options.get("offset")
        return value.value if value else 0

    def notify(self, value):
        self.value = value
        if self.notifying:
            self.emit_properties_changed({"Value": value})


class FromRadioCharacteristic(GattCharacteristic):
    def __init__(self, protocol):
        super().__init__(FROM_RADIO_UUID, ["read"])
        self.protocol = protocol

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":
        self.value = self.protocol.read_from_radio(self.offset(options))
        return self.value


class ToRadioCharacteristic(GattCharacteristic):
    def __init__(self, protocol, compatibility_mode):
        flags = ["write"]
        if compatibility_mode:
            flags.append("write-without-response")
        super().__init__(TO_RADIO_UUID, flags)
        self.protocol = protocol

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}"):
        write_type = options.get("type")
        LOGGER.info(
            "ToRadio GATT write: type=%s, len=%d",
            write_type.value if write_type else "unknown",
            len(value),
        )
        offset = self.offset(options)
        if offset:
            raise DBusError(
                "org.bluez.Error.InvalidOffset",
                "Fragmented ToRadio writes are unsupported",
            )
        self.protocol.handle_to_radio(value)


class FromNumCharacteristic(GattCharacteristic):
    def __init__(self, protocol, compatibility_mode, initial_fromnum):
        flags = ["read", "notify"]
        if compatibility_mode:
            flags.append("write")
        super().__init__(FROM_NUM_UUID, flags)
        self.protocol = protocol
        self.initial_fromnum = initial_fromnum
        self.value = struct.pack("<I", protocol.from_num)

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":
        value = struct.pack("<I", self.protocol.from_num)
        offset = self.offset(options)
        if offset > len(value):
            raise DBusError("org.bluez.Error.InvalidOffset", "Invalid FromNum offset")
        self.value = value[offset:]
        LOGGER.debug("FromNum read: %d", self.protocol.from_num)
        return self.value

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}"):
        if len(value) != 4:
            raise DBusError(
                "org.bluez.Error.InvalidValueLength", "FromNum must be uint32"
            )
        requested = struct.unpack("<I", value)[0]
        LOGGER.info("FromNum rewind requested: %d", requested)

    @method()
    def StartNotify(self):
        super().StartNotify()
        if self.initial_fromnum:
            self.notify_counter(self.protocol.from_num)

    def notify_counter(self, value):
        LOGGER.debug("FromNum notify: %d", value)
        self.notify(struct.pack("<I", value))


class LogRadioCharacteristic(GattCharacteristic):
    def __init__(self):
        super().__init__(LOG_RADIO_UUID, ["read", "notify"])

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":
        offset = self.offset(options)
        if offset > len(self.value):
            raise DBusError("org.bluez.Error.InvalidOffset", "Invalid LogRadio offset")
        return self.value[offset:]

    def emit_log(self, message, level):
        protobuf_level = mesh_pb2.LogRecord.Level.INFO
        if level >= logging.ERROR:
            protobuf_level = mesh_pb2.LogRecord.Level.ERROR
        elif level >= logging.WARNING:
            protobuf_level = mesh_pb2.LogRecord.Level.WARNING
        elif level <= logging.DEBUG:
            protobuf_level = mesh_pb2.LogRecord.Level.DEBUG
        record = mesh_pb2.LogRecord(
            message=message[:384],
            time=int(time.time()),
            source="BLEEmulator",
            level=protobuf_level,
        )
        self.notify(record.SerializeToString())


class PairingAgent(ServiceInterface):
    def __init__(self):
        super().__init__("org.bluez.Agent1")

    @method()
    def Release(self):
        LOGGER.debug("BlueZ released the pairing agent")

    @method()
    def RequestPinCode(self, device: "o") -> "s":
        LOGGER.info("Legacy pairing request from %s", device)
        return "000000"

    @method()
    def DisplayPinCode(self, device: "o", pin_code: "s"):
        LOGGER.debug("Pairing PIN for %s: %s", device, pin_code)

    @method()
    def RequestPasskey(self, device: "o") -> "u":
        LOGGER.info("Passkey pairing request from %s", device)
        return 0

    @method()
    def DisplayPasskey(self, device: "o", passkey: "u", entered: "q"):
        LOGGER.debug(
            "Pairing passkey for %s: %06d, entered=%d", device, passkey, entered
        )

    @method()
    def RequestConfirmation(self, device: "o", passkey: "u"):
        LOGGER.info("Accepting Just Works pairing from %s", device)

    @method()
    def RequestAuthorization(self, device: "o"):
        LOGGER.info("Authorizing pairing from %s", device)

    @method()
    def AuthorizeService(self, device: "o", service_uuid: "s"):
        LOGGER.debug("Authorizing %s for %s", service_uuid, device)

    @method()
    def Cancel(self):
        LOGGER.info("Pairing request cancelled")


class Advertisement(ServiceInterface):
    def __init__(self, local_name):
        super().__init__("org.bluez.LEAdvertisement1")
        self.local_name = local_name
        self.tx_power = 0

    @dbus_property(access=PropertyAccess.READ)
    def Type(self) -> "s":
        return "peripheral"

    @dbus_property(access=PropertyAccess.READ)
    def ServiceUUIDs(self) -> "as":
        return [MESH_SERVICE_UUID]

    @dbus_property(access=PropertyAccess.READ)
    def LocalName(self) -> "s":
        return self.local_name

    @dbus_property(access=PropertyAccess.READ)
    def Discoverable(self) -> "b":
        return True

    @dbus_property(access=PropertyAccess.READWRITE)
    def TxPower(self) -> "n":
        return self.tx_power

    @TxPower.setter
    def TxPower(self, value: "n"):
        self.tx_power = value
        LOGGER.debug("BlueZ selected advertisement TxPower=%d dBm", value)

    @method()
    def Release(self):
        LOGGER.info("BlueZ released the advertisement")


class HciAdvertisingController:
    def __init__(self, adapter, local_name):
        self.adapter = adapter
        self.local_name = local_name
        self.process = None

    def start(self):
        match = re.fullmatch(r"hci(\d+)", self.adapter)
        if not match:
            raise RuntimeError(f"Cannot derive adapter index from {self.adapter!r}")

        helper = Path(__file__).with_name("ble-mgmt-advertising.py")
        command = [
            "sudo",
            "-n",
            "/usr/bin/python3",
            str(helper),
            "--adapter-index",
            match.group(1),
            "--name",
            self.local_name,
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

        readable, _, _ = select.select([self.process.stdout], [], [], 5)
        if not readable:
            self.stop()
            raise RuntimeError("Timed out waiting for the MGMT advertising helper")

        status = self.process.stdout.readline().strip()
        if not status.startswith("READY "):
            error = self.process.stderr.read().strip()
            self.stop()
            raise RuntimeError(error or status or "MGMT advertising helper exited")
        LOGGER.warning("Using direct HCI advertising fallback: %s", status)

    def resume(self):
        if not self.process or not self.process.stdin or not self.process.stdout:
            raise RuntimeError("HCI advertising helper is not running")
        self.process.stdin.write("R")
        self.process.stdin.flush()

        readable, _, _ = select.select([self.process.stdout], [], [], 5)
        if not readable:
            raise RuntimeError("Timed out while resuming HCI advertising")
        status = self.process.stdout.readline().strip()
        if status != "RESUMED":
            raise RuntimeError(status or "Failed to resume HCI advertising")
        LOGGER.info("BLE advertising resumed after disconnect")

    def stop(self):
        if not self.process:
            return
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.process.returncode:
            error = self.process.stderr.read().strip()
            if error:
                LOGGER.debug("MGMT advertising helper cleanup: %s", error)
        self.process = None


class BluezPeripheral:
    def __init__(
        self, adapter, local_name, protocol, compatibility_mode, initial_fromnum
    ):
        self.adapter = adapter
        self.adapter_path = f"/org/bluez/{adapter}"
        self.local_name = local_name
        self.protocol = protocol
        self.compatibility_mode = compatibility_mode
        self.initial_fromnum = initial_fromnum
        self.bus = None
        self.gatt_manager = None
        self.advertising_manager = None
        self.adapter_properties = None
        self.agent_manager = None
        self.agent_registered = False
        self.original_pairable = None
        self.stop_event = asyncio.Event()
        self.connected_devices = {}
        self.bluez_advertising = False
        self.hci_advertising = None

    async def start(self):
        self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        self.bus.add_message_handler(self._trace_dbus_message)
        adapter_introspection = await self.bus.introspect(
            BLUEZ_SERVICE, self.adapter_path
        )
        adapter_object = self.bus.get_proxy_object(
            BLUEZ_SERVICE, self.adapter_path, adapter_introspection
        )
        self.gatt_manager = adapter_object.get_interface("org.bluez.GattManager1")
        self.advertising_manager = adapter_object.get_interface(
            "org.bluez.LEAdvertisingManager1"
        )
        self.adapter_properties = adapter_object.get_interface(
            "org.freedesktop.DBus.Properties"
        )

        bluez_introspection = await self.bus.introspect(BLUEZ_SERVICE, "/org/bluez")
        bluez_object = self.bus.get_proxy_object(
            BLUEZ_SERVICE, "/org/bluez", bluez_introspection
        )
        self.agent_manager = bluez_object.get_interface("org.bluez.AgentManager1")

        service = GattService(MESH_SERVICE_UUID)
        from_radio = FromRadioCharacteristic(self.protocol)
        to_radio = ToRadioCharacteristic(self.protocol, self.compatibility_mode)
        from_num = FromNumCharacteristic(
            self.protocol, self.compatibility_mode, self.initial_fromnum
        )
        log_radio = LogRadioCharacteristic()
        advertisement = Advertisement(self.local_name)
        pairing_agent = PairingAgent()

        self.protocol.set_callbacks(from_num.notify_counter, log_radio.emit_log)

        self.bus.export(AGENT_PATH, pairing_agent)
        self.bus.export(SERVICE_PATH, service)
        self.bus.export(f"{SERVICE_PATH}/char0", from_radio)
        self.bus.export(f"{SERVICE_PATH}/char1", to_radio)
        self.bus.export(f"{SERVICE_PATH}/char2", from_num)
        self.bus.export(f"{SERVICE_PATH}/char3", log_radio)
        self.bus.export(ADVERTISEMENT_PATH, advertisement)

        pairable = await self.adapter_properties.call_get(
            "org.bluez.Adapter1", "Pairable"
        )
        self.original_pairable = pairable.value
        await self.agent_manager.call_register_agent(AGENT_PATH, "NoInputNoOutput")
        self.agent_registered = True
        await self.agent_manager.call_request_default_agent(AGENT_PATH)
        if not self.original_pairable:
            await self.adapter_properties.call_set(
                "org.bluez.Adapter1", "Pairable", Variant("b", True)
            )
        LOGGER.info("BLE pairing enabled in Just Works mode (no PIN)")

        await self.gatt_manager.call_register_application(APP_PATH, {})
        try:
            await self.advertising_manager.call_register_advertisement(
                ADVERTISEMENT_PATH, {}
            )
            self.bluez_advertising = True
        except DBusError as error:
            LOGGER.warning("BlueZ advertising registration failed: %s", error)
            self.hci_advertising = HciAdvertisingController(
                self.adapter, self.local_name
            )
            try:
                self.hci_advertising.start()
            except Exception:
                await self.gatt_manager.call_unregister_application(APP_PATH)
                raise

        LOGGER.info("Meshtastic BLE API emulator is advertising")
        LOGGER.info("Name: %s", self.local_name)
        LOGGER.info("Service UUID: %s", MESH_SERVICE_UUID)
        LOGGER.info("Adapter: %s", self.adapter)
        LOGGER.info(
            "GATT profile: %s, initial FromNum=%s",
            "compatibility" if self.compatibility_mode else "nRF52 strict",
            self.initial_fromnum,
        )
        asyncio.create_task(self._monitor_connections())

    @staticmethod
    def _trace_dbus_message(message):
        if (
            LOGGER.isEnabledFor(logging.DEBUG)
            and message.path
            and message.path.startswith("/org/meshtastic")
        ):
            detail = ""
            if (
                message.interface == "org.freedesktop.DBus.Properties"
                and message.member == "Set"
            ):
                detail = f" property={message.body[1]}"
            LOGGER.debug(
                "D-Bus inbound: %s.%s path=%s%s",
                message.interface,
                message.member,
                message.path,
                detail,
            )
        return False

    async def _monitor_connections(self):
        root_introspection = await self.bus.introspect(BLUEZ_SERVICE, "/")
        root_object = self.bus.get_proxy_object(BLUEZ_SERVICE, "/", root_introspection)
        object_manager = root_object.get_interface("org.freedesktop.DBus.ObjectManager")

        while not self.stop_event.is_set():
            managed = await object_manager.call_get_managed_objects()
            connected = {}
            for path, interfaces in managed.items():
                device = interfaces.get("org.bluez.Device1")
                if not device:
                    continue
                if device.get("Connected", Variant("b", False)).value:
                    name = device.get(
                        "Name", Variant("s", path.rsplit("/", 1)[-1])
                    ).value
                    connected[path] = name

            for path in connected.keys() - self.connected_devices.keys():
                LOGGER.info("BLE central connected: %s (%s)", connected[path], path)
            disconnected = self.connected_devices.keys() - connected.keys()
            for path in disconnected:
                LOGGER.info(
                    "BLE central disconnected: %s (%s)",
                    self.connected_devices[path],
                    path,
                )
            if disconnected and self.hci_advertising:
                try:
                    self.hci_advertising.resume()
                except RuntimeError as error:
                    LOGGER.error("Failed to resume BLE advertising: %s", error)
            self.connected_devices = connected

            try:
                await asyncio.wait_for(self.stop_event.wait(), timeout=1)
            except asyncio.TimeoutError:
                pass

    async def stop(self):
        self.stop_event.set()
        if self.hci_advertising:
            self.hci_advertising.stop()
        elif self.advertising_manager and self.bluez_advertising:
            try:
                await self.advertising_manager.call_unregister_advertisement(
                    ADVERTISEMENT_PATH
                )
            except DBusError as error:
                LOGGER.debug("Advertisement cleanup: %s", error)
        if self.gatt_manager:
            try:
                await self.gatt_manager.call_unregister_application(APP_PATH)
            except DBusError as error:
                LOGGER.debug("GATT cleanup: %s", error)
        if self.adapter_properties and self.original_pairable is not None:
            try:
                await self.adapter_properties.call_set(
                    "org.bluez.Adapter1",
                    "Pairable",
                    Variant("b", self.original_pairable),
                )
            except DBusError as error:
                LOGGER.debug("Pairable state cleanup: %s", error)
        if self.agent_manager and self.agent_registered:
            try:
                await self.agent_manager.call_unregister_agent(AGENT_PATH)
            except DBusError as error:
                LOGGER.debug("Pairing agent cleanup: %s", error)
            self.agent_registered = False
        if self.bus:
            self.bus.disconnect()


def self_test(node_num):
    protocol = MeshtasticProtocolEmulator(node_num)
    start = mesh_pb2.ToRadio(want_config_id=0x12345678)
    protocol.handle_to_radio(start.SerializeToString())

    variants = []
    while True:
        payload = protocol.read_from_radio(0)
        if not payload:
            break
        frame = mesh_pb2.FromRadio()
        frame.ParseFromString(payload)
        variants.append(frame.WhichOneof("payload_variant"))

    assert variants[0] == "my_info"
    assert "node_info" in variants
    assert "metadata" in variants
    assert "channel" in variants
    assert variants[-1] == "config_complete_id"

    packet = mesh_pb2.MeshPacket(
        to=0xFFFFFFFF,
        id=0x10203040,
        want_ack=True,
        hop_limit=3,
    )
    packet.decoded.portnum = portnums_pb2.PortNum.TEXT_MESSAGE_APP
    packet.decoded.payload = b"emulator self-test"
    protocol.handle_to_radio(mesh_pb2.ToRadio(packet=packet).SerializeToString())

    responses = []
    while True:
        payload = protocol.read_from_radio(0)
        if not payload:
            break
        frame = mesh_pb2.FromRadio()
        frame.ParseFromString(payload)
        responses.append(frame.WhichOneof("payload_variant"))

    assert responses == ["queueStatus", "packet", "packet"]
    print(
        f"Self-test passed: {len(variants)} config frames, {len(responses)} packet responses"
    )


async def run(args):
    protocol = MeshtasticProtocolEmulator(args.node_num)
    peripheral = BluezPeripheral(
        args.adapter,
        args.name,
        protocol,
        args.compatibility_mode,
        args.initial_fromnum or args.compatibility_mode,
    )
    loop = asyncio.get_running_loop()
    for signal_name in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signal_name, peripheral.stop_event.set)

    try:
        await peripheral.start()
        await peripheral.stop_event.wait()
    finally:
        await peripheral.stop()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Meshtastic Client API BLE peripheral emulator for BlueZ"
    )
    parser.add_argument("--adapter", default="hci0", help="BlueZ adapter name")
    parser.add_argument("--name", default="Meshtastic_0001", help="advertised BLE name")
    parser.add_argument(
        "--node-num",
        type=lambda value: int(value, 0),
        default=0xE0A00001,
        help="emulated node number, decimal or 0x-prefixed",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="test protobuf behavior without Bluetooth",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show every FromRadio read and FromNum notify",
    )
    parser.add_argument(
        "--compatibility-mode",
        action="store_true",
        help="allow write-without-response and send an initial FromNum notification",
    )
    parser.add_argument(
        "--initial-fromnum",
        action="store_true",
        help="send FromNum=0 once immediately after notifications are enabled",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    if args.self_test:
        self_test(args.node_num)
        return
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
