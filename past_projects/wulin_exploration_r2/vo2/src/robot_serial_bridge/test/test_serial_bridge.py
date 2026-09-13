"""ROS-level serial handshake test with a simulated protocol-v4 firmware."""

import struct
import sys
from types import SimpleNamespace

from msg_interface.srv import RobotCommand
import rclpy
from rclpy.parameter import Parameter

from robot_serial_bridge.protocol import Command, RESPONSE_LENGTHS
from robot_serial_bridge.protocol import pack_usb_frame
from robot_serial_bridge.serial_bridge import RobotSerialBridge


class FakeFirmwareSerial:
    """Reply to status requests and apply source-switch commands."""

    def __init__(self):
        self.rx = bytearray()
        self.source = 0
        self.commands = []

    @property
    def in_waiting(self):
        """Return bytes waiting for the bridge."""
        return len(self.rx)

    def read(self, size):
        """Read one simulated CDC chunk."""
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, frame):
        """Accept one command frame and queue matching firmware feedback."""
        command = frame[3]
        self.commands.append(command)
        if command == Command.SOURCE:
            self.source = int(struct.unpack_from('<f', frame, 4)[0])
            self._queue_robot_status()
        elif command == Command.ROBOT:
            self._queue_robot_status()
        elif command in RESPONSE_LENGTHS:
            self.rx.extend(pack_usb_frame(
                command, bytes(RESPONSE_LENGTHS[command])
            ))
        return len(frame)

    def _queue_robot_status(self):
        payload = bytearray(RESPONSE_LENGTHS[Command.ROBOT])
        payload[0] = 4
        payload[1] = self.source
        self.rx.extend(pack_usb_frame(Command.ROBOT, payload))

    def flush(self):
        """Match the pyserial API."""

    def close(self):
        """Match the pyserial API."""


def test_bridge_handshake_and_claim_are_bidirectional(monkeypatch):
    fake = FakeFirmwareSerial()
    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(
        serial_for_url=lambda *args, **kwargs: fake
    ))
    rclpy.init()
    node = RobotSerialBridge(parameter_overrides=[
        Parameter('send_enabled', value=True),
        Parameter('port', value='fake://firmware'),
    ])

    try:
        node._io_cycle()
        node._io_cycle()
        assert node.controller.ready
        assert node.controller.status[Command.ROBOT]['source'] == 0

        request = RobotCommand.Request()
        request.command = 'claim'
        response = node._command_callback(request, RobotCommand.Response())
        assert response.accepted

        node._io_cycle()
        node._io_cycle()
        assert fake.source == 1
        assert Command.SOURCE in fake.commands
        assert node.controller.status[Command.ROBOT]['source'] == 1
        assert 'system' not in node.controller.operations
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
