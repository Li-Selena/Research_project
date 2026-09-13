"""Bidirectional ROS 2 bridge for the STM32 robot controller."""

import json
from math import cos, sin
import time

from geometry_msgs.msg import TransformStamped, Twist
from msg_interface.msg import PnpResult, RobotCommandResult
from msg_interface.srv import RobotCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .controller import Controller
from .protocol import Command, RESPONSE_LENGTHS, StreamParser
from .protocol import pack_usb_frame


ALIASES = {
    'gripper_open': ('gripper', [1.0]),
    'gripper_close': ('gripper', [0.0]),
    'suction_on': ('suction', [1.0]),
    'suction_off': ('suction', [0.0]),
    'select_gripper': ('tool_select', [0.0]),
    'select_suction': ('tool_select', [1.0]),
}


class RobotSerialBridge(Node):
    """Expose feedback-confirmed robot control over ROS 2."""

    def __init__(self, **node_kwargs):
        super().__init__('robot_serial_bridge_node', **node_kwargs)
        self.declare_parameter('send_enabled', False)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout_s', 0.0)
        self.declare_parameter('reconnect_interval_s', 1.0)
        self.declare_parameter('claim_usb_source', False)
        self.declare_parameter('enable_arm_on_start', False)
        self.declare_parameter('target_topic', 'pnp_result')
        self.declare_parameter('vision_control_enabled', True)
        self.declare_parameter('expected_frame_id', 'arm_base_link')
        self.declare_parameter('send_predicted_targets', False)
        self.declare_parameter('maximum_abs_coordinate_mm', 2000.0)
        self.declare_parameter('cmd_vel_enabled', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('command_service', 'robot/command')
        self.declare_parameter('status_topic', 'robot/status')
        self.declare_parameter('result_topic', 'robot/command_result')
        self.declare_parameter('odom_topic', 'robot/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('pose_xy_variance', 0.01)
        self.declare_parameter('pose_yaw_variance', 0.02)
        self.declare_parameter('twist_xy_variance', 0.04)
        self.declare_parameter('twist_yaw_variance', 0.04)

        self.enabled = bool(self.get_parameter('send_enabled').value)
        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.serial_timeout = float(
            self.get_parameter('serial_timeout_s').value
        )
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_s').value
        )
        self.claim_on_start = bool(
            self.get_parameter('claim_usb_source').value
        )
        self.enable_arm_on_start = bool(
            self.get_parameter('enable_arm_on_start').value
        )
        self.vision_enabled = bool(
            self.get_parameter('vision_control_enabled').value
        )
        self.expected_frame = str(
            self.get_parameter('expected_frame_id').value
        )
        self.send_predictions = bool(
            self.get_parameter('send_predicted_targets').value
        )
        self.maximum_arm_mm = float(
            self.get_parameter('maximum_abs_coordinate_mm').value
        )
        self.cmd_vel_enabled = bool(
            self.get_parameter('cmd_vel_enabled').value
        )
        self.publish_odom_tf = bool(
            self.get_parameter('publish_odom_tf').value
        )
        self.pose_xy_variance = float(
            self.get_parameter('pose_xy_variance').value
        )
        self.pose_yaw_variance = float(
            self.get_parameter('pose_yaw_variance').value
        )
        self.twist_xy_variance = float(
            self.get_parameter('twist_xy_variance').value
        )
        self.twist_yaw_variance = float(
            self.get_parameter('twist_yaw_variance').value
        )
        if self.reconnect_interval <= 0 or self.maximum_arm_mm <= 0:
            raise ValueError('reconnect interval and arm limit must be positive')
        if min(
            self.pose_xy_variance,
            self.pose_yaw_variance,
            self.twist_xy_variance,
            self.twist_yaw_variance,
        ) < 0:
            raise ValueError('odometry variances cannot be negative')

        expected = {int(key): value for key, value in RESPONSE_LENGTHS.items()}
        self.parser = StreamParser(expected)
        self.controller = Controller()
        self.serial = None
        self.next_reconnect = 0.0
        self.next_publish = 0.0
        self.startup_claim_id = None
        self.startup_arm_enabled = False
        self.last_callback_warning = 0.0

        status_topic = str(self.get_parameter('status_topic').value)
        result_topic = str(self.get_parameter('result_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        command_service = str(self.get_parameter('command_service').value)
        target_topic = str(self.get_parameter('target_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.result_publisher = self.create_publisher(
            RobotCommandResult, result_topic, 20
        )
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_odom_tf else None
        )
        self.command_service = self.create_service(
            RobotCommand, command_service, self._command_callback
        )
        self.target_subscription = self.create_subscription(
            PnpResult, target_topic, self._target_callback, 10
        )
        self.cmd_vel_subscription = self.create_subscription(
            Twist, cmd_vel_topic, self._cmd_vel_callback, 10
        )
        self.timer = self.create_timer(0.01, self._io_cycle)

        if self.enabled:
            self._open_serial()
        else:
            self.get_logger().info(
                'serial is disabled; set send_enabled:=true to connect'
            )

    def _open_serial(self):
        try:
            import serial

            self.serial = serial.serial_for_url(
                self.port,
                baudrate=self.baud,
                timeout=self.serial_timeout,
                write_timeout=max(self.serial_timeout, 0.05),
            )
            self.parser.reset()
            self.controller.connect()
            self.startup_claim_id = None
            self.startup_arm_enabled = False
            self.get_logger().info(
                f'opened {self.port} at {self.baud}; waiting for protocol v4'
            )
        except Exception as error:
            self.serial = None
            self.controller.disconnect(f'open failed: {error}')
            self.next_reconnect = time.monotonic() + self.reconnect_interval
            self._warn(f'cannot open {self.port}: {error}')

    def _link_failed(self, error):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.parser.reset()
        self.controller.disconnect(f'serial link failed: {error}')
        self.next_reconnect = time.monotonic() + self.reconnect_interval
        self._warn(f'serial link failed: {error}')

    def _warn(self, message):
        now = time.monotonic()
        if now - self.last_callback_warning >= 2.0:
            self.get_logger().warning(message)
            self.last_callback_warning = now

    def _command_callback(self, request, response):
        command = request.command.strip().lower()
        values = list(request.values)
        if command in ALIASES:
            command, alias_values = ALIASES[command]
            if values:
                response.message = (
                    request.command + ' does not accept explicit values'
                )
                return response
            values = alias_values
        try:
            request_id = self.controller.submit(
                command, values, request.duration_s, request.timeout_s
            )
            response.accepted = True
            response.request_id = request_id
            response.message = (
                'accepted; observe robot/command_result for completion'
            )
        except ValueError as error:
            response.accepted = False
            response.request_id = 0
            response.message = str(error)
        return response

    def _target_callback(self, message):
        if not self.enabled or not self.vision_enabled:
            return
        if message.header.frame_id != self.expected_frame:
            self._warn(
                f'ignored vision frame {message.header.frame_id!r}; '
                f'expected {self.expected_frame!r}'
            )
            return
        if not message.valid or (
            message.predicted and not self.send_predictions
        ):
            return
        point_mm = [
            message.x * 1000.0,
            message.y * 1000.0,
            message.z * 1000.0,
        ]
        if any(abs(value) > self.maximum_arm_mm for value in point_mm):
            self._warn('ignored vision target outside configured arm limit')
            return
        try:
            self.controller.stream_vision(point_mm)
        except ValueError as error:
            self._warn(f'vision target not sent: {error}')

    def _cmd_vel_callback(self, message):
        if not self.enabled or not self.cmd_vel_enabled:
            return
        values = [message.linear.x, message.linear.y, message.angular.z]
        try:
            self.controller.stream_velocity(values)
        except ValueError as error:
            self._warn(f'cmd_vel not sent: {error}')

    def _io_cycle(self):
        if not self.enabled:
            self._publish_if_due()
            return
        if self.serial is None:
            if time.monotonic() >= self.next_reconnect:
                self._open_serial()
            self._publish_events()
            self._publish_if_due()
            return
        try:
            waiting = min(int(getattr(self.serial, 'in_waiting', 0)), 4096)
            if waiting:
                for frame in self.parser.feed(self.serial.read(waiting)):
                    try:
                        self.controller.ingest(frame)
                    except ValueError as error:
                        self._warn(f'rejected firmware feedback: {error}')
            self.controller.tick()
            self._handle_startup()
            frames = self.controller.outgoing[:]
            self.controller.outgoing.clear()
            for frame in frames:
                if self.serial.write(frame) != len(frame):
                    raise IOError('partial serial write')
        except Exception as error:
            self._link_failed(error)

        self._publish_events()

        self._publish_if_due()

    def _publish_if_due(self):
        """Publish link and robot state at 10 Hz, including while disconnected."""
        now = time.monotonic()
        if now >= self.next_publish:
            self._publish_status()
            self.next_publish = now + 0.1

    def _handle_startup(self):
        if not self.controller.ready:
            return
        robot = self.controller.status[Command.ROBOT]
        if self.claim_on_start and robot['source'] != 1:
            if self.startup_claim_id is None:
                try:
                    self.startup_claim_id = self.controller.submit('claim')
                except ValueError as error:
                    self._warn(f'cannot claim USB control: {error}')
            return
        if (
            self.enable_arm_on_start
            and robot['source'] == 1
            and not self.startup_arm_enabled
        ):
            self.controller.outgoing.append(
                pack_usb_frame(Command.ARM_ENABLE)
            )
            self.startup_arm_enabled = True

    def _publish_events(self):
        while self.controller.events:
            event = self.controller.events.pop(0)
            if (
                event['request_id'] == self.startup_claim_id
                and event['state'] != 'running'
            ):
                self.startup_claim_id = None
            message = RobotCommandResult()
            message.header.stamp = self.get_clock().now().to_msg()
            message.request_id = event['request_id']
            message.command = event['command']
            message.state = event['state']
            message.detail = event['detail']
            self.result_publisher.publish(message)

    def _publish_status(self):
        snapshot = self.controller.snapshot()
        snapshot['serial'] = {
            'enabled': self.enabled,
            'port': self.port,
            'parser_errors': self.parser.errors,
        }
        message = String()
        message.data = json.dumps(
            snapshot, ensure_ascii=False, allow_nan=False
        )
        self.status_publisher.publish(message)
        robot = snapshot['feedback'].get('robot')
        if robot is None:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = str(
            self.get_parameter('odom_frame_id').value
        )
        odom.child_frame_id = str(
            self.get_parameter('base_frame_id').value
        )
        x, y, yaw = robot['odom']
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = sin(yaw * 0.5)
        odom.pose.pose.orientation.w = cos(yaw * 0.5)
        odom.pose.covariance[0] = self.pose_xy_variance
        odom.pose.covariance[7] = self.pose_xy_variance
        odom.pose.covariance[14] = 1.0e6
        odom.pose.covariance[21] = 1.0e6
        odom.pose.covariance[28] = 1.0e6
        odom.pose.covariance[35] = self.pose_yaw_variance
        vx, vy, wz = robot['commanded_velocity']
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = self.twist_xy_variance
        odom.twist.covariance[7] = self.twist_xy_variance
        odom.twist.covariance[14] = 1.0e6
        odom.twist.covariance[21] = 1.0e6
        odom.twist.covariance[28] = 1.0e6
        odom.twist.covariance[35] = self.twist_yaw_variance
        self.odom_publisher.publish(odom)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def close(self):
        """Request a controlled stop and close the serial device."""
        if self.serial is not None:
            try:
                self.serial.write(pack_usb_frame(Command.STOP))
                self.serial.flush()
            except Exception:
                pass
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.controller.disconnect('node shutdown')


def main(args=None):
    """Run the bidirectional serial bridge node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RobotSerialBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
