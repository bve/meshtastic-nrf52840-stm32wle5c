#!/usr/bin/env python3

import argparse
import hashlib
import re
import select
import signal
import struct
import subprocess
import sys
import uuid

MESH_SERVICE_UUID = "6ba1b218-15a8-461f-9fa8-5dcae273eafd"
HCI_TOOL = "/usr/bin/hcitool"
HCI_OGF_LE = 0x08
HCI_OCF_SET_ADV_SET_RANDOM_ADDRESS = 0x0035
HCI_OCF_SET_EXT_ADV_PARAMS = 0x0036
HCI_OCF_SET_EXT_ADV_DATA = 0x0037
HCI_OCF_SET_EXT_SCAN_RESPONSE = 0x0038
HCI_OCF_SET_EXT_ADV_ENABLE = 0x0039
HCI_OCF_REMOVE_ADV_SET = 0x003C


class HciAdvertising:
    def __init__(self, adapter_index, local_name):
        self.adapter = f"hci{adapter_index}"
        self.local_name = local_name
        self.handle = 1
        self.registered = False

    def start(self):
        response = self._command(
            HCI_OCF_SET_EXT_ADV_PARAMS,
            self._advertising_parameters(),
        )
        tx_power = struct.unpack("b", response[:1])[0]
        self._command(
            HCI_OCF_SET_ADV_SET_RANDOM_ADDRESS,
            bytes([self.handle]) + self._random_static_address(),
        )

        service_uuid = uuid.UUID(MESH_SERVICE_UUID).bytes[::-1]
        advertising_data = b"\x02\x01\x06" + bytes([0x11, 0x07]) + service_uuid
        self._set_data(HCI_OCF_SET_EXT_ADV_DATA, advertising_data)

        encoded_name = self.local_name.encode("utf-8")[:29]
        scan_response = bytes([len(encoded_name) + 1, 0x09]) + encoded_name
        self._set_data(HCI_OCF_SET_EXT_SCAN_RESPONSE, scan_response)

        self._set_enabled(True)
        self.registered = True
        return tx_power

    def stop(self):
        if not self.registered:
            return
        try:
            self._set_enabled(False)
        finally:
            self._command(HCI_OCF_REMOVE_ADV_SET, bytes([self.handle]))
            self.registered = False

    def resume(self):
        if self.registered:
            self._set_enabled(True)

    def _advertising_parameters(self):
        interval = 0x0800
        return (
            bytes([self.handle])
            + struct.pack("<H", 0x0013)
            + interval.to_bytes(3, "little")
            + interval.to_bytes(3, "little")
            + b"\x07"
            + b"\x01"
            + b"\x00"
            + bytes(6)
            + b"\x00"
            + b"\x7f"
            + b"\x01"
            + b"\x00"
            + b"\x01"
            + b"\x00"
            + b"\x00"
        )

    def _random_static_address(self):
        seed = f"meshtastic-ble-emulator:{self.local_name}".encode()
        address = bytearray(hashlib.sha256(seed).digest()[:6])
        address[5] = (address[5] & 0x3F) | 0xC0
        return bytes(address)

    def _set_data(self, opcode, data):
        parameters = bytes([self.handle, 0x03, 0x01, len(data)]) + data
        self._command(opcode, parameters)

    def _set_enabled(self, enabled):
        parameters = bytes([int(enabled), 1, self.handle, 0, 0, 0])
        self._command(HCI_OCF_SET_EXT_ADV_ENABLE, parameters)

    def _command(self, opcode, payload):
        command = [
            HCI_TOOL,
            "-i",
            self.adapter,
            "cmd",
            f"{HCI_OGF_LE:02x}",
            f"{opcode:04x}",
            *(f"{byte:02x}" for byte in payload),
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        output = completed.stdout + completed.stderr
        if completed.returncode:
            raise RuntimeError(output.strip() or f"HCI opcode 0x{opcode:04x} failed")

        event = output.rpartition("> HCI Event:")[2]
        event_lines = event.splitlines()[1:]
        event_bytes = [
            int(token, 16)
            for line in event_lines
            for token in line.split()
            if re.fullmatch(r"[0-9a-fA-F]{2}", token)
        ]
        if len(event_bytes) < 4:
            raise RuntimeError(
                f"No Command Complete response for HCI opcode 0x{opcode:04x}"
            )

        status = event_bytes[3]
        if status:
            raise RuntimeError(
                f"HCI opcode 0x{opcode:04x} failed with status 0x{status:02x}"
            )
        return bytes(event_bytes[4:])


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--adapter-index", type=int, required=True)
    parser.add_argument("--name", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    advertising = HciAdvertising(args.adapter_index, args.name)
    stopping = False

    def request_stop(_signal_number, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        tx_power = advertising.start()
        print(f"READY handle={advertising.handle} tx_power={tx_power}", flush=True)
        while not stopping:
            readable, _, _ = select.select([sys.stdin], [], [], 1)
            if readable:
                command = sys.stdin.buffer.read(1)
                if not command:
                    break
                if command == b"R":
                    advertising.resume()
                    print("RESUMED", flush=True)
    except Exception as error:  # noqa: BLE001
        print(f"ERROR {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        advertising.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
