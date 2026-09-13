from __future__ import annotations

import struct
import threading
import time
import math
from dataclasses import dataclass
from typing import Protocol

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # Allows protocol unit tests without pyserial installed.
    serial = None


SLAVE_ADDRESS = 1
FC_READ_HOLDING = 0x03
FC_READ_INPUT = 0x04
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10

REG_HEARTBEAT = 0x0000
REG_COMMAND = 0x0001
REG_CHASSIS_MODE = 0x0010
REG_CHASSIS_TARGET = 0x0012
REG_DELTA_TARGET = 0x0020

CHASSIS_STOP = 0
CHASSIS_SPEED_LOCAL = 1
CHASSIS_POSITION_WORLD = 2

CMD_ODOMETRY_RESET = 0x0004
CMD_DELTA_STRIKE = 0x0010
CMD_DELTA_HOME = 0x0011
CMD_ROD_PREPARE = 0x0020
CMD_ROD_HOME = 0x0021
CMD_ROD_STRIKE = 0x0022
CMD_EMERGENCY_STOP = 0x00FF


class SerialLike(Protocol):
    timeout: float

    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...
    def close(self) -> None: ...
    def reset_input_buffer(self) -> None: ...


class ModbusError(RuntimeError):
    pass


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16(payload))


def floats_to_registers(values: list[float] | tuple[float, ...]) -> list[int]:
    registers: list[int] = []
    for value in values:
        high, low = struct.unpack(">HH", struct.pack(">f", float(value)))
        registers.extend((high, low))
    return registers


def registers_to_floats(registers: list[int]) -> list[float]:
    if len(registers) % 2:
        raise ValueError("float register count must be even")
    return [
        struct.unpack(">f", struct.pack(">HH", registers[i], registers[i + 1]))[0]
        for i in range(0, len(registers), 2)
    ]


def discover_serial_port() -> str:
    if serial is None:
        raise ModbusError("pyserial is not installed")
    ports = list(serial.tools.list_ports.comports())
    keywords = ("USB", "CH340", "CP210", "ACM")
    matches = [p.device for p in ports if any(k.lower() in p.description.lower() for k in keywords)]
    if not matches:
        raise ModbusError("no USB/TTL serial port found; set serial_port in config.json")
    if len(matches) > 1:
        raise ModbusError(f"multiple serial ports found {matches}; set serial_port in config.json")
    return matches[0]


class ModbusRTUClient:
    def __init__(self, port: str | None = None, baudrate: int = 115200,
                 slave: int = SLAVE_ADDRESS, timeout: float = 0.1,
                 retries: int = 2, serial_instance: SerialLike | None = None):
        self.slave = slave
        self.retries = retries
        self._lock = threading.Lock()
        if serial_instance is not None:
            self.serial = serial_instance
        else:
            if serial is None:
                raise ModbusError("pyserial is not installed")
            self.serial = serial.Serial(port or discover_serial_port(), baudrate=baudrate,
                                        bytesize=8, parity="N", stopbits=1, timeout=timeout)

    def close(self) -> None:
        self.serial.close()

    def _read_exact(self, size: int) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + float(self.serial.timeout or 0.1)
        while len(data) < size and time.monotonic() < deadline:
            block = self.serial.read(size - len(data))
            if block:
                data.extend(block)
        if len(data) != size:
            raise ModbusError(f"response timeout: expected {size} bytes, got {len(data)}")
        return bytes(data)

    def _exchange_once(self, request: bytes, function: int) -> bytes:
        self.serial.reset_input_buffer()
        self.serial.write(request)
        head = self._read_exact(2)
        if head[0] != self.slave:
            raise ModbusError(f"unexpected slave address {head[0]}")
        if head[1] == (function | 0x80):
            tail = self._read_exact(3)
            response = head + tail
            self._validate_crc(response)
            raise ModbusError(f"modbus exception 0x{response[2]:02x}")
        if head[1] != function:
            raise ModbusError(f"unexpected function 0x{head[1]:02x}")
        if function in (FC_READ_HOLDING, FC_READ_INPUT):
            byte_count = self._read_exact(1)
            response = head + byte_count + self._read_exact(byte_count[0] + 2)
        else:
            response = head + self._read_exact(6)
        self._validate_crc(response)
        return response

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        if len(frame) < 4 or struct.unpack("<H", frame[-2:])[0] != crc16(frame[:-2]):
            raise ModbusError("invalid response CRC")

    def _exchange(self, request: bytes, function: int) -> bytes:
        last_error: Exception | None = None
        with self._lock:
            for _ in range(self.retries + 1):
                try:
                    return self._exchange_once(request, function)
                except (ModbusError, OSError) as exc:
                    last_error = exc
            raise ModbusError(str(last_error))

    def read_registers(self, function: int, address: int, count: int) -> list[int]:
        request = with_crc(struct.pack(">BBHH", self.slave, function, address, count))
        response = self._exchange(request, function)
        if response[2] != count * 2:
            raise ModbusError("unexpected register byte count")
        return list(struct.unpack(f">{count}H", response[3:-2]))

    def read_holding(self, address: int, count: int) -> list[int]:
        return self.read_registers(FC_READ_HOLDING, address, count)

    def read_input(self, address: int, count: int) -> list[int]:
        return self.read_registers(FC_READ_INPUT, address, count)

    def write_single(self, address: int, value: int) -> None:
        payload = struct.pack(">BBHH", self.slave, FC_WRITE_SINGLE, address, value & 0xFFFF)
        request = with_crc(payload)
        response = self._exchange(request, FC_WRITE_SINGLE)
        if response[:-2] != payload:
            raise ModbusError("write-single echo mismatch")

    def write_multiple(self, address: int, values: list[int]) -> None:
        if not 1 <= len(values) <= 123:
            raise ValueError("register count must be 1..123")
        body = struct.pack(">BBHHB", self.slave, FC_WRITE_MULTIPLE,
                           address, len(values), 2 * len(values))
        body += struct.pack(f">{len(values)}H", *values)
        response = self._exchange(with_crc(body), FC_WRITE_MULTIPLE)
        expected = struct.pack(">BBHH", self.slave, FC_WRITE_MULTIPLE, address, len(values))
        if response[:-2] != expected:
            raise ModbusError("write-multiple echo mismatch")


@dataclass(frozen=True)
class RobotStatus:
    protocol_version: int
    status_flags: int
    active_striker: int
    last_command: int
    last_result: int
    delta_state: int
    rod_state: int
    chassis_state: int
    odometry: tuple[float, float, float]
    chassis_speed: tuple[float, float, float]
    delta_joint_deg: tuple[float, float, float]
    rod_angle_deg: float


class RobotClient:
    def __init__(self, port: str | None = None, baudrate: int = 115200,
                 timeout: float = 0.1, retries: int = 2,
                 modbus: ModbusRTUClient | None = None):
        self.modbus = modbus or ModbusRTUClient(port, baudrate, timeout=timeout, retries=retries)
        self._desired_mode = CHASSIS_STOP
        self._desired_target = (0.0, 0.0, 0.0)
        self._state_lock = threading.Lock()
        self._run = False
        self._thread: threading.Thread | None = None
        self._heartbeat = 0
        self.latest_status: RobotStatus | None = None
        self.last_error: Exception | None = None

    def start(self) -> None:
        if self._run:
            return
        self._run = True
        self._thread = threading.Thread(target=self._worker, name="robot-modbus", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._run = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        try:
            self.stop_chassis()
            self._write_chassis_once()
        finally:
            self.modbus.close()

    def _worker(self) -> None:
        next_motion = next_housekeeping = time.monotonic()
        while self._run:
            now = time.monotonic()
            try:
                if now >= next_motion:
                    self._write_chassis_once()
                    next_motion = now + 0.02
                if now >= next_housekeeping:
                    self._write_heartbeat_once()
                    self.latest_status = self.read_status()
                    next_housekeeping = now + 0.1
                self.last_error = None
            except Exception as exc:
                self.last_error = exc
                next_motion = now + 0.02
                next_housekeeping = now + 0.1
            time.sleep(0.002)

    def _write_chassis_once(self) -> None:
        with self._state_lock:
            mode = self._desired_mode
            target = self._desired_target
        values = [mode, 0] + floats_to_registers(target)
        self.modbus.write_multiple(REG_CHASSIS_MODE, values)

    def _set_motion(self, mode: int, values: tuple[float, float, float]) -> None:
        if not all(math.isfinite(float(x)) for x in values):
            raise ValueError("motion target must be finite")
        with self._state_lock:
            self._desired_mode = mode
            self._desired_target = tuple(map(float, values))

    def set_chassis_speed(self, vx_mm_s: float, vy_mm_s: float, wz_deg_s: float) -> None:
        self._set_motion(CHASSIS_SPEED_LOCAL, (vx_mm_s, vy_mm_s, wz_deg_s))

    def move_chassis_to(self, x_mm: float, y_mm: float, yaw_deg: float) -> None:
        self._set_motion(CHASSIS_POSITION_WORLD, (x_mm, y_mm, yaw_deg))

    def stop_chassis(self) -> None:
        self._set_motion(CHASSIS_STOP, (0.0, 0.0, 0.0))

    def apply_chassis_command(self) -> None:
        """Immediately send the currently selected chassis mode and target once."""
        self._write_chassis_once()

    def _write_heartbeat_once(self) -> None:
        with self._state_lock:
            self._heartbeat = (self._heartbeat + 1) & 0xFFFF
            heartbeat = self._heartbeat
        self.modbus.write_single(REG_HEARTBEAT, heartbeat)

    def _command(self, command: int) -> None:
        self._write_heartbeat_once()
        self.modbus.write_single(REG_COMMAND, command)

    def reset_odometry(self) -> None: self._command(CMD_ODOMETRY_RESET)
    def strike_delta(self) -> None: self._command(CMD_DELTA_STRIKE)
    def home_delta(self) -> None: self._command(CMD_DELTA_HOME)
    def prepare_rod(self) -> None: self._command(CMD_ROD_PREPARE)
    def home_rod(self) -> None: self._command(CMD_ROD_HOME)
    def strike_rod(self) -> None: self._command(CMD_ROD_STRIKE)
    def emergency_stop(self) -> None: self._command(CMD_EMERGENCY_STOP)

    def set_delta_strike_point(self, x_mm: float, y_mm: float, z_mm: float,
                               timeout_ms: int = 300) -> None:
        if not 100 <= timeout_ms <= 1000:
            raise ValueError("timeout_ms must be 100..1000")
        self.modbus.write_multiple(REG_DELTA_TARGET,
                                    floats_to_registers((x_mm, y_mm, z_mm)) + [timeout_ms])

    def read_status(self) -> RobotStatus:
        summary = self.modbus.read_input(0x0000, 8)
        motion = registers_to_floats(self.modbus.read_input(0x0010, 12))
        joints = registers_to_floats(self.modbus.read_input(0x0020, 6))
        rod = registers_to_floats(self.modbus.read_input(0x0030, 2))[0]
        return RobotStatus(*summary, tuple(motion[:3]), tuple(motion[3:]),
                           tuple(joints), rod)
