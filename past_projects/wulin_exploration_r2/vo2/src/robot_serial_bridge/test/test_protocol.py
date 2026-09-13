"""Tests for STM32 USB CDC framing and telemetry decoding."""

from math import nan
import struct

import pytest

from robot_serial_bridge.protocol import ARM_SET_TARGET, Command, Frame
from robot_serial_bridge.protocol import crc16_modbus, decode_feedback
from robot_serial_bridge.protocol import pack_float_command, pack_usb_frame
from robot_serial_bridge.protocol import RESPONSE_LENGTHS, StreamParser


def test_crc16_modbus_reference_vector():
    assert crc16_modbus(b'123456789') == 0x4B37


def test_pack_empty_frame_layout():
    frame = pack_usb_frame(0x21)
    assert frame[:4] == bytes((0xA5, 0x5A, 0x00, 0x21))
    assert frame[-1] == 0xFF
    expected_crc = crc16_modbus(frame[:-3])
    assert frame[-3:-1] == bytes((expected_crc >> 8, expected_crc & 0xFF))


def test_pack_arm_target_uses_little_endian_float_payload():
    frame = pack_float_command(ARM_SET_TARGET, (100.0, -20.0, 300.0, 0.0))
    expected = bytes.fromhex(
        'a5 5a 10 23 00 00 c8 42 00 00 a0 c1 00 00 96 43 '
        '00 00 00 00 b9 e4 ff'
    )
    assert frame == expected
    assert frame[2] == 16
    assert frame[3] == ARM_SET_TARGET
    assert struct.unpack('<4f', frame[4:20]) == (100.0, -20.0, 300.0, 0.0)


def test_pack_float_command_rejects_non_finite_control_value():
    with pytest.raises(ValueError, match='finite'):
        pack_float_command(Command.VELOCITY, (0.2, nan, 0.0, 0.0))


def test_stream_parser_recovers_fragmented_frames_and_crc_noise():
    parser = StreamParser({int(Command.IK): RESPONSE_LENGTHS[Command.IK]})
    good = pack_usb_frame(Command.IK, bytes((0, 0, 0, 1, 1, 1)))
    corrupt = bytearray(good)
    corrupt[-2] ^= 0x40

    assert parser.feed(b'noise\xa5' + bytes(corrupt[:4])) == []
    assert parser.feed(bytes(corrupt[4:]) + good[:5]) == []
    frames = parser.feed(good[5:])

    assert frames == [Frame(Command.IK, bytes((0, 0, 0, 1, 1, 1)))]
    assert parser.errors >= 1


def test_robot_feedback_requires_v4_and_decodes_flu_odom():
    payload = bytearray(RESPONSE_LENGTHS[Command.ROBOT])
    payload[0] = 4
    payload[1] = 1
    payload[5] = 0x3D
    struct.pack_into('<3f', payload, 56, 1.25, -0.5, 0.75)
    struct.pack_into('<3f', payload, 68, 0.3, -0.1, 0.2)
    struct.pack_into('<3i', payload, 128, 101, 202, 303)

    decoded = decode_feedback(Frame(Command.ROBOT, bytes(payload)))

    assert decoded['source'] == 1
    assert decoded['odom'] == pytest.approx([1.25, -0.5, 0.75])
    assert decoded['commanded_velocity'] == pytest.approx([0.3, -0.1, 0.2])
    assert decoded['laser_mm'] == [101, 202, 303]

    payload[0] = 3
    with pytest.raises(ValueError, match='v4 required'):
        decode_feedback(Frame(Command.ROBOT, bytes(payload)))
