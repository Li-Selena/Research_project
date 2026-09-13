"""Deterministic control engine with leases, feedback and motion arbitration."""

from dataclasses import dataclass, field
from math import atan2, cos, hypot, isfinite, sin
import time

from .protocol import Command as C
from .protocol import decode_feedback, pack_float_command, pack_usb_frame


CLIMB_COMMANDS = {
    'climb_up': C.CLIMB_UP, 'climb_down': C.CLIMB_DOWN,
    'climb_step_up': C.CLIMB_STEP_UP, 'climb_step_down': C.CLIMB_STEP_DOWN,
    'climb_up_pause': C.CLIMB_UP_PAUSE, 'climb_down_pause': C.CLIMB_DOWN_PAUSE,
    'climb_gate_up': C.CLIMB_GATE_UP, 'climb_gate_down': C.CLIMB_GATE_DOWN,
    'climb_resume': C.CLIMB_RESUME, 'climb_test': C.CLIMB_TEST,
}
STOP_COMMANDS = {
    'stop': ('system', C.STOP), 'arm_stop': ('arm', C.ARM_STOP),
    'tool_stop': ('tool', C.TOOL_STOP),
    'chassis_stop': ('chassis', C.CHASSIS_STOP),
    'climb_stop': ('climb', C.CLIMB_STOP),
}
GROUPS = {
    'claim': 'system', 'release': 'system', 'arm_move': 'arm',
    'tool_select': 'tool', 'gripper': 'tool', 'suction': 'tool',
    'velocity': 'chassis', 'move_relative': 'chassis', 'move_to': 'chassis',
}
GROUPS.update({name: 'climb' for name in CLIMB_COMMANDS})
GROUPS.update({name: group for name, (group, _) in STOP_COMMANDS.items()})
COUNTS = {
    'arm_move': 3, 'tool_select': 1, 'gripper': 1, 'suction': 1,
    'velocity': 3, 'move_relative': 3, 'move_to': 3, 'climb_test': 1,
}


def angle_difference(target, actual):
    """Return the shortest angular difference in radians."""
    return atan2(sin(target - actual), cos(target - actual))


def close_values(left, right, tolerance):
    """Compare finite coordinate arrays with a scalar tolerance."""
    return len(left) == len(right) and all(
        abs(a - b) <= tolerance for a, b in zip(left, right)
    )


@dataclass
class Operation:
    """A locally identified request tracked until observed completion."""

    request_id: int
    command: str
    group: str
    values: list
    start: float
    deadline: float
    duration: float = 0.0
    next_send: float = 0.0
    goal: list = field(default_factory=list)
    observed: bool = False
    stage: str = 'running'
    initial_climb_state: int = 0


class Controller:
    """Own command scheduling; all methods run on one executor thread."""

    def __init__(self, clock=time.monotonic, feedback_timeout=0.6):
        self.clock = clock
        self.feedback_timeout = feedback_timeout
        self.connected = False
        self.status = {}
        self.received = {}
        self.operations = {}
        self.outgoing = []
        self.events = []
        self.next_id = 1
        self.next_poll = 0.0
        self.next_detail = 0.0
        self.detail_index = 0
        self.streams = {}
        self.problem = 'not connected'

    @property
    def ready(self):
        """Require a recently received v4 robot status, not just an open port."""
        return (
            self.connected and C.ROBOT in self.received
            and self.clock() - self.received[C.ROBOT] <= self.feedback_timeout
            and self.status[C.ROBOT]['protocol_version'] == 4
        )

    def connect(self):
        """Begin a fresh handshake without replaying previous requests."""
        self.status.clear()
        self.received.clear()
        self.outgoing.clear()
        self.connected = True
        self.next_poll = 0.0
        self.next_detail = 0.0
        self.problem = 'waiting for protocol v4 feedback'

    def disconnect(self, reason):
        """Fail pending operations and clear all queued motion."""
        for op in list(self.operations.values()):
            self._finish(op, 'failed', reason)
        self.streams.clear()
        self.outgoing.clear()
        self.status.clear()
        self.received.clear()
        self.connected = False
        self.problem = reason

    def _send(self, command, values=None):
        if values is None:
            self.outgoing.append(pack_usb_frame(command))
        else:
            vals = list(values)
            self.outgoing.append(pack_float_command(
                command, vals + [0.0] * (4 - len(vals))
            ))

    def _event(self, op, state, detail):
        self.events.append({
            'request_id': op.request_id, 'command': op.command,
            'state': state, 'detail': detail,
        })

    def _finish(self, op, state, detail):
        if self.operations.get(op.group) is op:
            del self.operations[op.group]
        self._event(op, state, detail)

    def _require_usb(self):
        if not self.ready:
            raise ValueError('no fresh protocol v4 feedback')
        if self.status[C.ROBOT]['source'] != 1:
            raise ValueError('USB control is not selected; issue claim first')

    def _climb_owns_chassis(self):
        robot = self.status.get(C.ROBOT, {})
        return (
            'climb' in self.operations
            or robot.get('climb_test_active', False)
            or robot.get('climb_pending', False)
            or (
                robot.get('climb_enabled', False)
                and robot.get('climb_state', 0) not in (0, 22, 23)
            )
        )

    def _validate(self, command, values, duration, timeout):
        if command not in GROUPS:
            raise ValueError('unknown command: ' + command)
        if len(values) != COUNTS.get(command, 0):
            raise ValueError(f'{command} expects {COUNTS.get(command, 0)} values')
        if not all(isfinite(v) for v in values + [duration, timeout]):
            raise ValueError('all parameters must be finite')
        if duration < 0 or timeout < 0 or timeout > 600:
            raise ValueError('invalid duration/timeout (timeout maximum is 600 s)')
        if command in ('tool_select', 'gripper', 'suction'):
            if values[0] not in (0.0, 1.0):
                raise ValueError('tool selection/state must be 0 or 1')
        if command == 'climb_test':
            if values[0] != int(values[0]) or not 1 <= values[0] <= 38:
                raise ValueError('climb test action must be an integer in 1..38')
        if command == 'arm_move':
            if any(abs(v) > 2000 for v in values):
                raise ValueError('arm coordinates exceed ±2000 mm sanity limit')
            if values[2] <= 250 and abs(values[1]) > 5:
                raise ValueError('firmware requires |Y| <= 5 mm at Z <= 250 mm')
        if command == 'velocity':
            if not 0 < duration <= 120:
                raise ValueError('velocity duration must be in (0, 120] seconds')
            self._validate_velocity(values)
        if command in ('move_relative', 'move_to'):
            if any(abs(v) > 100 for v in values[:2]) or abs(values[2]) > 20:
                raise ValueError('chassis goal exceeds 100 m / 20 rad bound')

    @staticmethod
    def _validate_velocity(values):
        if len(values) != 3 or not all(isfinite(v) for v in values):
            raise ValueError('velocity requires three finite values')
        if abs(values[0]) > 2 or abs(values[1]) > 2 or abs(values[2]) > 0.6283185:
            raise ValueError('firmware limits: |vx/vy| <= 2 m/s, |wz| <= 0.6283185')

    def submit(self, command, values=(), duration=0.0, timeout=0.0):
        """Validate and queue a request; completion arrives separately in events."""
        values = [float(v) for v in values]
        self._validate(command, values, duration, timeout)
        if not self.ready:
            raise ValueError('waiting for fresh protocol v4 robot feedback')
        group = GROUPS[command]
        if command not in ('claim', 'release'):
            self._require_usb()
        if command in STOP_COMMANDS or command == 'release':
            self._cancel_for_stop(group if command != 'release' else 'system')
        elif self.operations.get('system') or group in self.operations:
            raise ValueError(f'{group} is busy; stop or await the active request')
        elif group == 'system' and (self.operations or self.streams):
            raise ValueError('stop current motions before claiming control')
        elif group == 'chassis' and self._climb_owns_chassis():
            raise ValueError('climbing owns the chassis; stop climbing first')
        elif group == 'climb' and (
            'chassis' in self.operations or 'velocity' in self.streams
        ):
            raise ValueError('chassis is busy; stop chassis before climbing')

        now = self.clock()
        default_timeout = (
            duration + 3 if command == 'velocity'
            else 180 if group == 'climb'
            else 30 if group == 'chassis' else 10
        )
        op = Operation(
            self.next_id, command, group, values, now,
            now + (timeout or default_timeout), duration, now,
        )
        self.next_id += 1
        # Compute the physical goal before starting or replacing a motion.
        if command in ('move_relative', 'move_to'):
            current = self.status[C.ROBOT]['odom']
            if command == 'move_to':
                op.goal = values[:]
                op.values = [
                    values[0] - current[0], values[1] - current[1],
                    angle_difference(values[2], current[2]),
                ]
            else:
                c, s = cos(current[2]), sin(current[2])
                op.goal = [
                    current[0] + c * values[0] - s * values[1],
                    current[1] + s * values[0] + c * values[1],
                    current[2] + values[2],
                ]
        if group == 'climb':
            op.initial_climb_state = self.status[C.ROBOT]['climb_state']
        self.operations[group] = op
        self._event(op, 'running', 'queued; waiting for firmware feedback')
        self._start(op)
        self.next_detail = 0.0
        return op.request_id

    def _cancel_for_stop(self, group):
        groups = {group}
        if group == 'system':
            groups = set(self.operations)
        elif group in ('chassis', 'climb'):
            groups |= {'chassis', 'climb'}
        for key in groups:
            if key in self.operations:
                self._finish(self.operations[key], 'canceled', 'stop requested')
        if group == 'system':
            self.streams.clear()
        if group in ('chassis', 'climb'):
            self.streams.pop('velocity', None)
        if group == 'arm':
            self.streams.pop('vision', None)
        # A pending keepalive must never follow a stop in the same I/O cycle.
        self.outgoing = [
            frame for frame in self.outgoing
            if frame[3] in (C.ROBOT, C.CHASSIS, C.ARM, C.TOOL, C.CLIMB)
        ]

    def _start(self, op):
        name = op.command
        if name in STOP_COMMANDS:
            if op.group in ('chassis', 'climb'):
                self._send(C.CLIMB_STOP)
                self._send(C.CHASSIS_STOP)
            else:
                self._send(STOP_COMMANDS[name][1])
        elif name == 'claim':
            self._send(C.SOURCE, [1])
        elif name == 'release':
            if self.status[C.ROBOT]['source'] == 1:
                self._send(C.STOP)
            self._send(C.SOURCE, [0])
        elif op.group == 'arm':
            self.streams.pop('vision', None)
            self._send(C.ARM_ENABLE)
            self._send(C.ARM_TARGET, op.values)
            self._send(C.ARM)
            op.next_send = self.clock() + 0.1
        elif op.group == 'tool':
            self._send(C.TOOL_ENABLE)
            if name == 'tool_select':
                self._send(C.TOOL_SELECT, op.values)
            else:
                self._send(C.TOOL_STATE, [int(name == 'suction'), op.values[0]])
            self._send(C.TOOL)
        elif op.group == 'chassis':
            self.streams.pop('velocity', None)
            self._send(C.CHASSIS_ENABLE)
            mode = 1 if name == 'velocity' else 7 if name == 'move_to' else 5
            self._send(C.MODE, [mode])
            self._send(C.VELOCITY if name == 'velocity' else C.POSITION, op.values)
            op.next_send = self.clock() + 0.04
            self._send(C.CHASSIS)
        elif op.group == 'climb':
            self._send(C.CLIMB_ENABLE)
            self._send(
                CLIMB_COMMANDS[name], op.values if name == 'climb_test' else None
            )
            self._send(C.CLIMB)

    def stream_velocity(self, values, lease=0.2):
        """Renew a cmd_vel lease; expiration sends an explicit chassis stop."""
        self._require_usb()
        values = list(values)
        self._validate_velocity(values)
        if (
            'system' in self.operations or 'chassis' in self.operations
            or self._climb_owns_chassis()
        ):
            raise ValueError('chassis is owned by another operation')
        if 'velocity' not in self.streams:
            self._send(C.CHASSIS_ENABLE)
            self._send(C.MODE, [1])
        self.streams['velocity'] = {
            'values': values, 'deadline': self.clock() + lease, 'next': 0.0,
        }

    def stream_vision(self, values, lease=0.25):
        """Renew a measured target lease only when manual arm control is idle."""
        self._require_usb()
        values = list(values)
        self._validate('arm_move', values, 0.0, 0.0)
        if 'system' in self.operations or 'arm' in self.operations:
            raise ValueError('manual arm control is active')
        if 'vision' not in self.streams:
            self._send(C.ARM_ENABLE)
        self.streams['vision'] = {
            'values': values, 'deadline': self.clock() + lease, 'next': 0.0,
        }

    def stop_stream(self, kind=None):
        """Cancel one or all active streams and hold/stop their actuators."""
        kinds = list(self.streams) if kind is None else [kind]
        for stream_kind in kinds:
            if self.streams.pop(stream_kind, None) is not None:
                self._send(
                    C.ARM_STOP if stream_kind == 'vision' else C.CHASSIS_STOP
                )

    def ingest(self, frame):
        """Update telemetry and resolve requests using observed state."""
        now = self.clock()
        decoded = decode_feedback(frame)
        self.status[frame.command] = decoded
        self.received[frame.command] = now
        if frame.command == C.ROBOT:
            self.problem = ''
            if decoded['source'] != 1:
                self.streams.clear()
                for op in list(self.operations.values()):
                    if op.group != 'system':
                        self._finish(op, 'failed', 'USB control source was lost')
        for op in list(self.operations.values()):
            self._observe(op, frame.command, decoded)

    def _observe(self, op, cmd, data):
        name = op.command
        if name in ('claim', 'release') and cmd == C.ROBOT:
            if data['source'] == (1 if name == 'claim' else 0):
                self._finish(op, 'succeeded', 'control source confirmed')
        elif name in STOP_COMMANDS and cmd == C.ROBOT:
            stopped = {
                'arm': not data['executing'] & 4,
                'tool': not data['executing'] & 8,
                'chassis': not data['executing'] & 3 and data['climb_state'] == 0,
                'climb': data['climb_state'] == 0 and not data['executing'] & 3,
                'system': not data['executing'] & 0x3F and data['climb_state'] == 0,
            }
            if stopped[op.group]:
                self._finish(op, 'succeeded', 'firmware reports motion stopped/held')
        elif name == 'arm_move' and cmd == C.ARM:
            if close_values(data['requested_mm'], op.values, 0.1):
                if data['ik_status'] or not data['safe']:
                    self._send(C.ARM_STOP)
                    self._finish(
                        op, 'failed',
                        f"IK rejected: status={data['ik_status']}, "
                        f"reason={data['unsafe_reason']}",
                    )
                elif data['actual_valid'] and data['max_error_deg'] <= 2:
                    self._finish(op, 'succeeded', 'arm joint feedback is within 2 deg')
        elif op.group == 'tool' and cmd == C.TOOL:
            if data['errors'] & 7:
                self._finish(op, 'failed', 'tool reports fault/timeout')
            elif name == 'tool_select':
                dev = int(op.values[0])
                prefix = 'suction' if dev else 'gripper'
                if (
                    data['selected'] == dev and data[prefix + '_at_position']
                    and data[prefix + '_running'] == 0
                ):
                    self._finish(op, 'succeeded', 'tool rotation position confirmed')
            elif name in ('gripper', 'suction'):
                key = 'gripper_open' if name == 'gripper' else 'suction_on'
                if data[key] == bool(op.values[0]) and data['flags'] & 1:
                    self._finish(op, 'succeeded', 'actuator output state confirmed')
        elif op.group == 'chassis' and cmd == C.CHASSIS:
            self._observe_chassis(op, data)
        elif op.group == 'climb' and cmd == C.CLIMB:
            self._observe_climb(op, data)

    def _observe_chassis(self, op, data):
        if op.command == 'velocity':
            if data['mode'] == 1 and close_values(
                data['target_velocity'], op.values, 1e-4
            ):
                op.observed = True
            if op.stage == 'stopping' and data['stopped']:
                state = 'succeeded' if op.observed else 'failed'
                self._finish(op, state, 'timed velocity ended; stop confirmed')
        elif op.command in ('move_relative', 'move_to'):
            x, y, yaw = data['odom']
            if (
                data['position_state'] == 2
                and hypot(x - op.goal[0], y - op.goal[1]) <= 0.02
                and abs(angle_difference(op.goal[2], yaw)) <= 0.035
            ):
                self._finish(op, 'succeeded', 'odometry confirms requested pose')

    def _observe_climb(self, op, data):
        if data['errors'] or data['state'] == 23:
            self._send(C.CLIMB_STOP)
            self._send(C.CHASSIS_STOP)
            self._finish(op, 'failed', f"climb error flags={data['errors']}")
            return
        if data['state'] not in (0, 22, 23):
            if data['state'] != op.initial_climb_state or not data['done']:
                op.observed = True
        if op.command == 'climb_test':
            if data['test_action'] == int(op.values[0]):
                op.observed = True
            if op.observed and data['flags'] & 0x80:
                self._finish(op, 'succeeded', 'test action feedback is ready')
        elif op.observed:
            if data['state'] == 22 and data['done']:
                self._finish(op, 'succeeded', 'climb sequence completed')
            elif data['done'] and not data['auto']:
                if op.command in ('climb_up_pause', 'climb_down_pause'):
                    self._finish(op, 'paused', 'preset pause reached; use climb_resume')
                elif 'step' in op.command or 'gate' in op.command:
                    self._finish(op, 'succeeded', 'step/gate finished')

    def tick(self):
        """Poll status and renew only bounded, still-owned motion commands."""
        now = self.clock()
        if not self.connected:
            return
        if C.ROBOT in self.received and not self.ready:
            self._send(C.STOP)
            for op in list(self.operations.values()):
                self._finish(op, 'failed', 'robot feedback timed out')
            self.stop_stream()
            self.status.clear()
            self.received.clear()
            self.next_poll = 0.0
            self.problem = 'robot feedback timed out'
            return
        if now >= self.next_poll:
            self._send(C.ROBOT)
            self.next_poll = now + 0.1
        if self.ready and now >= self.next_detail:
            details = [C.CHASSIS, C.ARM, C.TOOL, C.CLIMB]
            self._send(details[self.detail_index % len(details)])
            self.detail_index += 1
            self.next_detail = now + 0.025

        for op in list(self.operations.values()):
            if now >= op.deadline:
                self._send({
                    'arm': C.ARM_STOP, 'tool': C.TOOL_STOP,
                    'chassis': C.CHASSIS_STOP, 'climb': C.CLIMB_STOP,
                    'system': C.STOP,
                }[op.group])
                if op.group == 'climb':
                    self._send(C.CHASSIS_STOP)
                self._finish(op, 'failed', 'operation timeout; stop requested')
                continue
            if op.command == 'arm_move' and now >= op.next_send:
                self._send(C.ARM_TARGET, op.values)
                op.next_send = now + 0.1
            if op.command == 'velocity':
                if now >= op.start + op.duration and op.stage != 'stopping':
                    op.stage = 'stopping'
                    self._send(C.CHASSIS_STOP)
                elif op.stage == 'running' and now >= op.next_send:
                    self._send(C.VELOCITY, op.values)
                    op.next_send = now + 0.04
        for kind, stream in list(self.streams.items()):
            if now >= stream['deadline']:
                self.stop_stream(kind)
            elif now >= stream['next']:
                self._send(
                    C.VELOCITY if kind == 'velocity' else C.ARM_TARGET,
                    stream['values'],
                )
                stream['next'] = now + (0.04 if kind == 'velocity' else 0.1)

    def snapshot(self):
        """Return JSON-compatible feedback with freshness and active requests."""
        now = self.clock()
        names = {
            C.ROBOT: 'robot', C.CHASSIS: 'chassis', C.ARM: 'arm',
            C.TOOL: 'tool', C.CLIMB: 'climb', C.IK: 'ik',
        }
        return {
            'connected': self.connected, 'ready': self.ready,
            'problem': self.problem,
            'age_s': {
                names[k]: max(0.0, now - value)
                for k, value in self.received.items() if k in names
            },
            'feedback': {
                names[k]: value for k, value in self.status.items() if k in names
            },
            'active_requests': [
                {'request_id': op.request_id, 'command': op.command}
                for op in self.operations.values()
            ],
        }
