from __future__ import annotations

from dataclasses import dataclass
import re
import struct
from typing import Iterable, List


USB_FRAME_HEAD1 = 0xA5
USB_FRAME_HEAD2 = 0x5A
USB_FRAME_TAIL = 0xFF
USB_FRAME_OVERHEAD = 7
USB_FRAME_MAX_DATA_LEN = 255

USART_FRAME_HEAD = 0xA5
USART_FRAME_TAIL = 0x5A
USART_FRAME_DATA_LEN = 40


@dataclass(frozen=True)
class UsbFrame:
    cmd: int
    payload: bytes
    raw: bytes

    @property
    def length(self) -> int:
        return len(self.payload)

    @property
    def crc(self) -> int:
        return (self.raw[-3] << 8) | self.raw[-2]

    def hex(self) -> str:
        return bytes_to_hex(self.raw)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def pack_usb_frame(cmd: int, payload: bytes = b"") -> bytes:
    if not 0 <= cmd <= 0xFF:
        raise ValueError("cmd must be 0..255")
    if len(payload) > USB_FRAME_MAX_DATA_LEN:
        raise ValueError("payload is too long")

    body = bytes([USB_FRAME_HEAD1, USB_FRAME_HEAD2, len(payload), cmd]) + payload
    crc = crc16_modbus(body)
    return body + bytes([(crc >> 8) & 0xFF, crc & 0xFF, USB_FRAME_TAIL])


def pack_4float_payload(values: Iterable[float]) -> bytes:
    vals = list(values)
    if len(vals) > 4:
        raise ValueError("USB float payload accepts at most 4 values")
    vals.extend([0.0] * (4 - len(vals)))
    return struct.pack("<4f", *vals)


def pack_4float_frame(cmd: int, values: Iterable[float]) -> bytes:
    return pack_usb_frame(cmd, pack_4float_payload(values))


def pack_usart_remote_frame(data: bytes) -> bytes:
    if len(data) != USART_FRAME_DATA_LEN:
        raise ValueError(f"USART remote DATA must be {USART_FRAME_DATA_LEN} bytes")
    checksum = sum(data) & 0xFF
    return bytes([USART_FRAME_HEAD]) + data + bytes([checksum, USART_FRAME_TAIL])


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def parse_hex(text: str) -> bytes:
    cleaned = text.strip()
    if not cleaned:
        return b""

    cleaned = cleaned.replace(",", " ").replace(";", " ")
    cleaned = re.sub(r"0x", "", cleaned, flags=re.IGNORECASE)

    if re.search(r"\s", cleaned):
        parts = [part for part in re.split(r"\s+", cleaned) if part]
    else:
        compact = re.sub(r"[^0-9A-Fa-f]", "", cleaned)
        if len(compact) % 2:
            raise ValueError("hex string has an odd number of digits")
        parts = [compact[i : i + 2] for i in range(0, len(compact), 2)]

    out = bytearray()
    for part in parts:
        if len(part) > 2:
            if len(part) % 2:
                raise ValueError(f"invalid hex token: {part}")
            out.extend(int(part[i : i + 2], 16) for i in range(0, len(part), 2))
        else:
            out.append(int(part, 16))
    return bytes(out)


class UsbStreamParser:
    """Streaming parser for A5 5A LEN CMD DATA CRC_H CRC_L FF frames."""

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf = bytearray()
        self.max_buffer = max_buffer
        self.dropped_bytes = 0
        self.crc_failures = 0
        self.tail_failures = 0

    def feed(self, data: bytes) -> List[UsbFrame]:
        if not data:
            return []

        self._buf.extend(data)
        if len(self._buf) > self.max_buffer:
            overflow = len(self._buf) - self.max_buffer
            del self._buf[:overflow]
            self.dropped_bytes += overflow

        frames: List[UsbFrame] = []

        while True:
            if len(self._buf) < USB_FRAME_OVERHEAD:
                return frames

            head_pos = self._buf.find(bytes([USB_FRAME_HEAD1]))
            if head_pos < 0:
                self.dropped_bytes += len(self._buf)
                self._buf.clear()
                return frames
            if head_pos:
                self.dropped_bytes += head_pos
                del self._buf[:head_pos]

            if len(self._buf) < 2:
                return frames
            if self._buf[1] != USB_FRAME_HEAD2:
                self.dropped_bytes += 1
                del self._buf[0]
                continue

            if len(self._buf) < 4:
                return frames

            data_len = self._buf[2]
            total_len = data_len + USB_FRAME_OVERHEAD
            if len(self._buf) < total_len:
                return frames

            candidate = bytes(self._buf[:total_len])
            if candidate[-1] != USB_FRAME_TAIL:
                self.tail_failures += 1
                del self._buf[0]
                continue

            recv_crc = (candidate[-3] << 8) | candidate[-2]
            calc_crc = crc16_modbus(candidate[:-3])
            if recv_crc != calc_crc:
                self.crc_failures += 1
                del self._buf[0]
                continue

            frames.append(UsbFrame(cmd=candidate[3], payload=candidate[4 : 4 + data_len], raw=candidate))
            del self._buf[:total_len]
