"""STM32 protocol v4 framing and feedback decoding (FLU coordinates)."""

from dataclasses import dataclass
from enum import IntEnum
from math import isfinite
import struct


class Command(IntEnum):
    """Firmware command identifiers from Data_Analysis.h."""

    SYS_DISABLE = 0x00
    SYS_ENABLE = 0x01
    SOURCE = 0x02
    STOP = 0x05
    SYSTEM = 0x06
    CHASSIS_ENABLE = 0x11
    MODE = 0x12
    VELOCITY = 0x13
    POSITION = 0x14
    CHASSIS_STOP = 0x15
    CHASSIS = 0x16
    ARM_ENABLE = 0x21
    ARM_TARGET = 0x23
    ARM_STOP = 0x25
    ARM = 0x26
    TOOL_ENABLE = 0x31
    TOOL_SELECT = 0x32
    TOOL_STATE = 0x34
    TOOL_STOP = 0x35
    TOOL = 0x36
    ROBOT = 0x46
    CLIMB_ENABLE = 0x51
    CLIMB_STEP_UP = 0x53
    CLIMB_UP = 0x54
    CLIMB_STOP = 0x55
    CLIMB = 0x56
    CLIMB_TEST = 0x57
    CLIMB_STEP_DOWN = 0x58
    CLIMB_DOWN = 0x59
    CLIMB_GATE_UP = 0x5A
    CLIMB_GATE_DOWN = 0x5B
    CLIMB_UP_PAUSE = 0x5C
    CLIMB_DOWN_PAUSE = 0x5D
    CLIMB_RESUME = 0x5E
    IK = 0x90


USB_HEAD = bytes((0xA5, 0x5A))
USB_TAIL = 0xFF
SYS_SWITCH_SOURCE = Command.SOURCE
ARM_ENABLE = Command.ARM_ENABLE
ARM_SET_TARGET = Command.ARM_TARGET
ARM_STOP = Command.ARM_STOP
RESPONSE_LENGTHS = {
    Command.SYSTEM: 8, Command.CHASSIS: 112, Command.ARM: 64,
    Command.TOOL: 32, Command.ROBOT: 240, Command.CLIMB: 68, Command.IK: 6,
}


def crc16_modbus(data):
    """Return CRC16/Modbus with initial value 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def pack_usb_frame(command, payload=b''):
    """Pack a complete frame with high CRC byte first."""
    if not 0 <= int(command) <= 0xFF:
        raise ValueError('command must fit in one byte')
    payload = bytes(payload)
    if len(payload) > 255:
        raise ValueError('payload cannot exceed 255 bytes')
    body = USB_HEAD + bytes((len(payload), int(command))) + payload
    crc = crc16_modbus(body)
    return body + bytes((crc >> 8, crc & 0xFF, USB_TAIL))


def pack_float_command(command, values):
    """Pack four finite little-endian float32 values."""
    parameters = tuple(float(value) for value in values)
    if len(parameters) != 4 or not all(isfinite(v) for v in parameters):
        raise ValueError('command requires four finite float values')
    try:
        return pack_usb_frame(command, struct.pack('<4f', *parameters))
    except (OverflowError, struct.error) as error:
        raise ValueError('command value is outside float32 range') from error


@dataclass(frozen=True)
class Frame:
    """One validated wire frame."""

    command: int
    payload: bytes


class StreamParser:
    """Recover frames across reads, noise, corrupt length and CRC errors."""

    def __init__(self, expected_lengths=None):
        self.buffer = bytearray()
        self.expected_lengths = expected_lengths
        self.errors = 0

    def reset(self):
        """Discard partial data after link loss."""
        self.buffer.clear()

    def feed(self, data):
        """Return all complete CRC-checked frames in the supplied chunk."""
        self.buffer.extend(data)
        if len(self.buffer) > 16384:
            del self.buffer[:-16384]
            self.errors += 1
        frames = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(USB_HEAD)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xA5 else b''
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            size, command = self.buffer[2:4]
            if (
                self.expected_lengths is not None
                and self.expected_lengths.get(command) != size
            ):
                del self.buffer[0]
                self.errors += 1
                continue
            end = size + 7
            if len(self.buffer) < end:
                break
            candidate = self.buffer[:end]
            crc = (candidate[-3] << 8) | candidate[-2]
            if candidate[-1] != USB_TAIL or crc16_modbus(candidate[:-3]) != crc:
                del self.buffer[0]
                self.errors += 1
                continue
            frames.append(Frame(command, bytes(candidate[4:-3])))
            del self.buffer[:end]
        return frames


def _floats(payload, offset, count=3):
    values = list(struct.unpack_from('<' + 'f' * count, payload, offset))
    if not all(isfinite(value) for value in values):
        raise ValueError('non-finite firmware telemetry')
    return values


def decode_feedback(frame):
    """Decode exact firmware layouts; reject pre-FLU robot protocol versions."""
    p, cmd = frame.payload, frame.command
    if RESPONSE_LENGTHS.get(cmd) != len(p):
        raise ValueError('unexpected feedback length')
    if cmd == Command.ROBOT:
        if p[0] != 4:
            raise ValueError(f'FLU protocol v4 required, received v{p[0]}')
        return {
            'protocol_version': p[0], 'source': p[1], 'enabled': p[2],
            'executing': p[3], 'errors': p[4], 'online': p[5],
            'rx_count': struct.unpack_from('<I', p, 8)[0],
            'nav': _floats(p, 32), 'world_velocity': _floats(p, 44),
            'odom': _floats(p, 56), 'commanded_velocity': _floats(p, 68),
            'arm_error_deg': _floats(p, 80), 'position_state': p[94],
            'timeouts': p[95], 'climb_state': p[97],
            'climb_enabled': bool(p[98]), 'climb_auto': bool(p[99]),
            'climb_done': bool(p[100]), 'climb_errors': p[101],
            'climb_pending': bool(p[102] or p[103]),
            'climb_test_active': bool(p[107] & 3),
            'laser_valid': p[116], 'laser_online': p[117],
            'laser_mm': list(struct.unpack_from('<3i', p, 128)),
            'yaw_tune_state': p[140],
            'target_velocity': _floats(p, 160),
            'target_delta': _floats(p, 172),
            'arm_target_mm': _floats(p, 184),
            'arm_target_deg': _floats(p, 196),
            'arm_actual_deg': _floats(p, 208),
            'source_stale': bool(p[239]),
        }
    if cmd == Command.CHASSIS:
        return {
            'mode': p[0], 'position_state': p[1], 'stopped': bool(p[2]),
            'velocity': _floats(p, 4), 'odom': _floats(p, 16),
            'limits': _floats(p, 28), 'progress': _floats(p, 40, 1)[0],
            'position_error': _floats(p, 44), 'imu_online': bool(p[84]),
            'flags': p[86], 'errors': p[87],
            'target_velocity': _floats(p, 88), 'target_delta': _floats(p, 100),
        }
    if cmd == Command.ARM:
        return {
            'has_valid': bool(p[0]), 'ik_status': p[1], 'action': p[2],
            'target_deg': _floats(p, 16), 'actual_deg': _floats(p, 28),
            'requested_mm': _floats(p, 40), 'reachable': bool(p[52]),
            'safe': bool(p[53]), 'unsafe_reason': p[54],
            'actual_valid': bool(p[55]), 'motor_online': p[56],
            'flags': p[57], 'errors': p[58],
            'max_error_deg': _floats(p, 60, 1)[0],
        }
    if cmd == Command.TOOL:
        return {
            'selected': p[0], 'gripper_open': bool(p[1]),
            'gripper_running': p[2], 'gripper_at_position': bool(p[3]),
            'suction_on': bool(p[8]), 'suction_running': p[9],
            'suction_at_position': bool(p[10]), 'source': p[11],
            'flags': p[24], 'errors': p[25],
        }
    if cmd == Command.CLIMB:
        return {
            'state': p[0], 'enabled': bool(p[1]), 'auto': bool(p[2]),
            'done': bool(p[3]), 'errors': p[4], 'source': p[5],
            'motor_online': p[6], 'test_action': p[7],
            'elapsed_ms': struct.unpack_from('<I', p, 8)[0],
            'tick_ms': struct.unpack_from('<I', p, 12)[0],
            'leg_mm': _floats(p, 16, 4), 'leg_target_mm': _floats(p, 32, 4),
            'rear_drive_mm': _floats(p, 48, 2),
            'rear_drive_target_mm': _floats(p, 56, 2),
            'flow': p[64], 'flags': p[65], 'leg_reached': p[66],
            'rear_drive_reached': p[67],
        }
    if cmd == Command.IK:
        return {
            'ik_status': p[0], 'unsafe_reason': p[1], 'action': p[2],
            'reachable': bool(p[3]), 'safe': bool(p[4]), 'has_valid': bool(p[5]),
        }
    return {'raw': list(p)}
