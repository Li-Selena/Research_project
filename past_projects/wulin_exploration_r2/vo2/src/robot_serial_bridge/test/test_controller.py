"""Behavior tests for feedback-confirmed robot operations."""

from math import pi
import struct

import pytest

from robot_serial_bridge.controller import Controller
from robot_serial_bridge.protocol import Command, Frame


class Clock:
    """Controllable monotonic clock for watchdog and lease tests."""

    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        """Advance monotonic time."""
        self.value += seconds


def robot_frame(source=1, odom=(0.0, 0.0, 0.0), executing=0,
                climb_state=0):
    """Build the protocol-v4 summary fields used by the controller."""
    payload = bytearray(240)
    payload[0] = 4
    payload[1] = source
    payload[3] = executing
    payload[97] = climb_state
    struct.pack_into('<3f', payload, 56, *odom)
    return Frame(Command.ROBOT, bytes(payload))


def chassis_frame(mode=1, position_state=0, stopped=False,
                  odom=(0.0, 0.0, 0.0), target=(0.0, 0.0, 0.0)):
    """Build chassis feedback with odometry and target fields."""
    payload = bytearray(112)
    payload[0] = mode
    payload[1] = position_state
    payload[2] = int(stopped)
    struct.pack_into('<3f', payload, 16, *odom)
    struct.pack_into('<3f', payload, 88, *target)
    return Frame(Command.CHASSIS, bytes(payload))


def arm_frame(requested, status=0, safe=True, actual_valid=True,
              max_error=0.5):
    """Build mechanical-arm IK and encoder feedback."""
    payload = bytearray(64)
    payload[0] = 1
    payload[1] = status
    struct.pack_into('<3f', payload, 40, *requested)
    payload[52] = int(status == 0)
    payload[53] = int(safe)
    payload[55] = int(actual_valid)
    struct.pack_into('<f', payload, 60, max_error)
    return Frame(Command.ARM, bytes(payload))


def tool_frame(selected, gripper_open=False, suction_on=False):
    """Build enabled, idle and position-safe tool feedback."""
    payload = bytearray(32)
    payload[0] = selected
    payload[1] = int(gripper_open)
    payload[3] = 1
    payload[8] = int(suction_on)
    payload[10] = 1
    payload[11] = 1
    payload[24] = 0x41
    return Frame(Command.TOOL, bytes(payload))


def climb_frame(state, done=False, auto=False, errors=0):
    """Build climb state-machine feedback."""
    payload = bytearray(68)
    payload[0] = state
    payload[1] = 1
    payload[2] = int(auto)
    payload[3] = int(done)
    payload[4] = errors
    return Frame(Command.CLIMB, bytes(payload))


def make_ready(source=1, odom=(0.0, 0.0, 0.0)):
    """Create a controller with fresh protocol-v4 feedback."""
    clock = Clock()
    controller = Controller(clock=clock)
    controller.connect()
    controller.ingest(robot_frame(source=source, odom=odom))
    return controller, clock


def sent_commands(controller):
    """Read command bytes from complete outgoing frames."""
    return [frame[3] for frame in controller.outgoing]


def test_claim_waits_for_firmware_source_feedback():
    controller, _ = make_ready(source=0)

    request_id = controller.submit('claim')

    assert Command.SOURCE in sent_commands(controller)
    assert controller.operations['system'].request_id == request_id
    controller.ingest(robot_frame(source=1))
    assert 'system' not in controller.operations
    assert controller.events[-1]['state'] == 'succeeded'


def test_timed_velocity_is_renewed_then_stopped_and_confirmed():
    controller, clock = make_ready()
    values = [0.4, -0.2, 0.1]
    controller.submit('velocity', values, duration=0.2)
    controller.outgoing.clear()

    clock.advance(0.05)
    controller.tick()
    assert Command.VELOCITY in sent_commands(controller)
    controller.ingest(chassis_frame(mode=1, target=values))

    controller.outgoing.clear()
    clock.advance(0.16)
    controller.tick()
    assert Command.CHASSIS_STOP in sent_commands(controller)
    controller.ingest(chassis_frame(mode=1, stopped=True, target=values))

    assert 'chassis' not in controller.operations
    assert controller.events[-1]['state'] == 'succeeded'


def test_arm_ik_rejection_requests_hold_and_fails_operation():
    controller, _ = make_ready()
    target = [350.0, 0.0, 400.0]
    controller.submit('arm_move', target)
    controller.outgoing.clear()

    controller.ingest(arm_frame(target, status=2, safe=False))

    assert Command.ARM_STOP in sent_commands(controller)
    assert controller.events[-1]['state'] == 'failed'
    assert 'IK rejected' in controller.events[-1]['detail']


def test_arm_move_finishes_only_after_encoder_error_is_small():
    controller, _ = make_ready()
    target = [350.0, 0.0, 400.0]
    controller.submit('arm_move', target)

    controller.ingest(arm_frame(target, max_error=4.0))
    assert 'arm' in controller.operations
    controller.ingest(arm_frame(target, max_error=1.5))
    assert controller.events[-1]['state'] == 'succeeded'


@pytest.mark.parametrize(
    'command,value,selected,gripper_open,suction_on',
    [
        ('gripper', 1.0, 0, True, False),
        ('suction', 1.0, 1, False, True),
    ],
)
def test_gripper_and_suction_commands_confirm_output_state(
        command, value, selected, gripper_open, suction_on):
    controller, _ = make_ready()

    controller.submit(command, [value])
    action = next(
        frame for frame in controller.outgoing if frame[3] == Command.TOOL_STATE
    )
    assert struct.unpack('<2f', action[4:12]) == (float(selected), value)
    controller.ingest(tool_frame(
        selected, gripper_open=gripper_open, suction_on=suction_on
    ))
    assert controller.events[-1]['state'] == 'succeeded'


def test_tool_rotation_waits_for_selected_safe_position():
    controller, _ = make_ready()
    controller.submit('tool_select', [1.0])

    controller.ingest(tool_frame(selected=0))
    assert 'tool' in controller.operations
    controller.ingest(tool_frame(selected=1))
    assert controller.events[-1]['state'] == 'succeeded'


def test_move_to_uses_world_delta_and_finishes_from_odometry():
    controller, _ = make_ready(odom=(1.0, 2.0, pi / 2))
    goal = [2.0, 4.0, pi]

    controller.submit('move_to', goal)

    position = next(
        frame for frame in controller.outgoing if frame[3] == Command.POSITION
    )
    delta = struct.unpack('<3f', position[4:16])
    assert delta == pytest.approx((1.0, 2.0, pi / 2))

    controller.ingest(chassis_frame(
        mode=7, position_state=2, odom=goal
    ))
    assert controller.events[-1]['state'] == 'succeeded'


def test_climb_owns_chassis_until_sequence_reports_done():
    controller, _ = make_ready()
    controller.submit('climb_up')

    with pytest.raises(ValueError, match='owned'):
        controller.stream_velocity([0.1, 0.0, 0.0])

    controller.ingest(climb_frame(state=1, auto=True))
    controller.ingest(climb_frame(state=22, done=True))
    assert controller.events[-1]['state'] == 'succeeded'


def test_system_stop_waits_for_climb_execution_bit_to_clear():
    controller, _ = make_ready()
    controller.submit('stop')

    controller.ingest(robot_frame(executing=0x10, climb_state=0))
    assert 'system' in controller.operations
    controller.ingest(robot_frame(executing=0, climb_state=0))
    assert controller.events[-1]['state'] == 'succeeded'


def test_velocity_and_vision_streams_expire_independently():
    controller, clock = make_ready()
    controller.stream_velocity([0.1, 0.0, 0.0], lease=0.2)
    controller.stream_vision([300.0, 0.0, 400.0], lease=0.3)
    controller.outgoing.clear()
    controller.tick()
    commands = sent_commands(controller)
    assert Command.VELOCITY in commands
    assert Command.ARM_TARGET in commands

    controller.outgoing.clear()
    clock.advance(0.21)
    controller.tick()
    assert Command.CHASSIS_STOP in sent_commands(controller)
    assert 'velocity' not in controller.streams
    assert 'vision' in controller.streams
