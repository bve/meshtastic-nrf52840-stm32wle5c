#!/usr/bin/env python3

import argparse
import binascii
import struct
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:
    raise SystemExit("pyserial is required: python -m pip install pyserial") from error


MAGIC = b"RPDF"
PROTOCOL_VERSION = 1
HEADER = struct.Struct("<4sBBHII")
MAX_PAYLOAD = 256
APPLICATION_ADDRESS = 0x08008000
RAM_START = 0x20000000
RAM_END = 0x20010000

OP_PROBE = 0x01
OP_BEGIN = 0x02
OP_DATA = 0x03
OP_VERIFY = 0x04
OP_RUN = 0x05
OP_HEALTH = 0x07

BRIDGE_HEALTH_NAMES = {
    0: "starting",
    1: "reset failed",
    2: "hello failed",
    3: "configure failed",
    4: "start receive failed",
    5: "ready",
}

STATUS_NAMES = {
    0: "ok",
    1: "bad USB frame",
    2: "bad command",
    3: "SPI transport error",
    4: "RIPULORA bootloader rejected command",
    5: "incompatible RIPULORA bootloader",
    6: "invalid update state",
}

TARGET_STATUS_NAMES = {
    0: "ok",
    1: "busy",
    2: "bad frame",
    3: "bad command",
    4: "radio error",
    5: "overflow",
    6: "not ready",
    7: "invalid address",
    8: "invalid size",
    9: "verification failed",
    10: "invalid state",
    11: "flash operation failed",
}


class DfuError(RuntimeError):
    pass


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def read_exact(port: serial.Serial, length: int, response_timeout: float) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + response_timeout
    while len(data) < length:
        block = port.read(length - len(data))
        if block:
            data.extend(block)
            continue
        if time.monotonic() >= deadline:
            raise DfuError(f"timeout waiting for {length} response bytes")
    return bytes(data)


def transact(
    port: serial.Serial,
    opcode: int,
    payload: bytes = b"",
    argument: int = 0,
    response_timeout: float = 8.0,
) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise DfuError("payload exceeds USB DFU frame capacity")

    request = (
        HEADER.pack(
            MAGIC, PROTOCOL_VERSION, opcode, len(payload), argument, crc32(payload)
        )
        + payload
    )
    port.write(request)
    port.flush()

    header = read_exact(port, HEADER.size, response_timeout)
    magic, version, response_opcode, payload_length, status, payload_crc = (
        HEADER.unpack(header)
    )
    if (
        magic != MAGIC
        or version != PROTOCOL_VERSION
        or response_opcode != (opcode | 0x80)
    ):
        raise DfuError("invalid USB DFU response header")
    if payload_length > MAX_PAYLOAD:
        raise DfuError("invalid USB DFU response length")

    response = read_exact(port, payload_length, response_timeout)
    if crc32(response) != payload_crc:
        raise DfuError("USB DFU response CRC mismatch")
    if status != 0:
        detail = STATUS_NAMES.get(status, f"unknown status {status}")
        if status == 4 and response:
            target = response[0]
            detail += f": {TARGET_STATUS_NAMES.get(target, f'unknown target status {target}') }"
        raise DfuError(detail)
    return response


def probe(port: serial.Serial) -> tuple[int, int, bool, int]:
    response = transact(port, OP_PROBE, response_timeout=1.0)
    if len(response) != 8:
        raise DfuError("unexpected probe response")
    version, max_chunk, active, application_address = struct.unpack("<BHBI", response)
    if version != PROTOCOL_VERSION or application_address != APPLICATION_ADDRESS:
        raise DfuError("incompatible nRF52840 updater")
    return version, max_chunk, bool(active), application_address


def port_priority(info) -> tuple[int, str]:
    description = " ".join(
        value or ""
        for value in (getattr(info, "interface", None), info.description, info.product)
    ).upper()
    return (0 if "RIPULORA" in description else 1, info.device)


def open_updater(explicit_port: str | None) -> serial.Serial:
    if explicit_port:
        candidates = [explicit_port]
    else:
        candidates = [
            info.device for info in sorted(list_ports.comports(), key=port_priority)
        ]

    errors = []
    for device in candidates:
        port = None
        try:
            port = serial.Serial(device, 115200, timeout=0.4, write_timeout=2)
            port.reset_input_buffer()
            probe(port)
            return port
        except (OSError, serial.SerialException, DfuError) as error:
            errors.append(f"{device}: {error}")
            try:
                if port is None:
                    continue
                port.close()
            except AttributeError:
                pass

    detail = "\n".join(errors) if errors else "no serial ports found"
    raise DfuError(f"RIPULORA DFU port was not found\n{detail}")


def validate_image(image: bytes) -> None:
    if len(image) < 8:
        raise DfuError("firmware image is too small")
    stack_pointer, reset_handler = struct.unpack_from("<II", image)
    reset_address = reset_handler & ~1
    if not RAM_START <= stack_pointer <= RAM_END or stack_pointer % 8 != 0:
        raise DfuError(f"invalid initial stack pointer 0x{stack_pointer:08x}")
    if reset_handler & 1 == 0 or reset_address < APPLICATION_ADDRESS:
        raise DfuError(
            f"firmware is not linked at 0x{APPLICATION_ADDRESS:08x} (reset handler 0x{reset_handler:08x})"
        )


def flash(port: serial.Serial, image: bytes) -> None:
    image_crc = crc32(image)
    begin_response = transact(port, OP_BEGIN, struct.pack("<II", len(image), image_crc))
    if len(begin_response) != 12:
        raise DfuError("unexpected RIPULORA bootloader information")

    protocol, application_address, flash_end, max_chunk, _ = struct.unpack(
        "<BIIHB", begin_response
    )
    if protocol != PROTOCOL_VERSION or application_address != APPLICATION_ADDRESS:
        raise DfuError("incompatible RIPULORA bootloader")
    if len(image) > flash_end - application_address:
        raise DfuError(
            f"firmware is too large: {len(image)} bytes, capacity is {flash_end - application_address}"
        )
    chunk_size = min(max_chunk, MAX_PAYLOAD)
    if chunk_size == 0 or chunk_size % 8 != 0:
        raise DfuError(f"unsupported bootloader chunk size {chunk_size}")

    for offset in range(0, len(image), chunk_size):
        chunk = image[offset : offset + chunk_size]
        response = transact(port, OP_DATA, chunk, offset)
        if len(response) != 4 or struct.unpack("<I", response)[0] != offset + len(
            chunk
        ):
            raise DfuError(f"RIPULORA offset mismatch after 0x{offset:08x}")
        progress = (offset + len(chunk)) * 100 // len(image)
        print(f"\rWriting RIPULORA: {progress:3d}%", end="", flush=True)
    print()

    response = transact(port, OP_VERIFY)
    if len(response) != 4 or struct.unpack("<I", response)[0] != image_crc:
        raise DfuError("RIPULORA verification CRC mismatch")
    transact(port, OP_RUN)

    deadline = time.monotonic() + 15
    last_health = 0
    while time.monotonic() < deadline:
        response = transact(port, OP_HEALTH, response_timeout=1.0)
        if len(response) != 1:
            raise DfuError("unexpected RIPULORA health response")
        last_health = response[0]
        if last_health == 5:
            return
        time.sleep(0.1)
    detail = BRIDGE_HEALTH_NAMES.get(last_health, f"unknown state {last_health}")
    raise DfuError(f"RIPULORA application did not reconnect to nRF52840: {detail}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Flash RIPULORA through the nRF52840 USB DFU port"
    )
    parser.add_argument("firmware", type=Path, help="ripu_bridge firmware.bin")
    parser.add_argument(
        "--port", help="second nRF52840 CDC port (auto-detected by default)"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        image = args.firmware.read_bytes()
        validate_image(image)
        with open_updater(args.port) as port:
            print(
                f"Using {port.port}; image {len(image)} bytes, CRC32 0x{crc32(image):08x}"
            )
            flash(port, image)
        print("RIPULORA updated and started successfully")
        return 0
    except (OSError, serial.SerialException, DfuError) as error:
        print(f"error: {error}", file=sys.stderr)
        print(
            "The updater remains recoverable; rerun this command to retry.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
