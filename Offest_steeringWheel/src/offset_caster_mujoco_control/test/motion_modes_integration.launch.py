import math
import time
import unittest

import launch
import launch_testing
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from offset_caster_interfaces.msg import MotionCommand, MotionStatus
from sensor_msgs.msg import Imu


@pytest.mark.launch_test
def generate_test_description():
    launch_file = (
        get_package_share_directory("offset_caster_mujoco_control")
        + "/launch/simulation_control.launch.py"
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "enable_viewer": "false",
            "enable_dashboard": "false",
            "enable_monitor": "false",
        }.items(),
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
            simulation,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class MotionModesIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("motion_modes_integration_test")
        self.publisher = self.node.create_publisher(
            MotionCommand, "/offset_caster/motion_command", 10
        )
        self.twist_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = None
        self.imu = None
        self.status = None
        self.node.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.node.create_subscription(
            Imu, "/imu/data", self._imu_callback, qos_profile_sensor_data
        )
        self.node.create_subscription(
            MotionStatus, "/offset_caster/motion_status", self._status_callback, 10
        )
        self.command_id = 0
        self._wait_for_state()

    def tearDown(self):
        self._send_twist(duration=0.1)
        self._send(MotionCommand.STOP, duration=0.2)
        self.node.destroy_node()

    def _odom_callback(self, message):
        self.odom = message

    def _imu_callback(self, message):
        self.imu = message

    def _status_callback(self, message):
        self.status = message

    def _spin(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _wait_for_state(self):
        end = time.monotonic() + 5.0
        while time.monotonic() < end and (self.odom is None or self.imu is None):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNotNone(self.odom)
        self.assertIsNotNone(self.imu)

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )

    @staticmethod
    def _angle_error(target, actual):
        return math.atan2(math.sin(target - actual), math.cos(target - actual))

    def _send(self, mode, x=0.0, y=0.0, yaw=0.0, yaw_rate=0.0, duration=0.5):
        self.command_id += 1
        message = MotionCommand()
        message.command_id = self.command_id
        message.mode = mode
        message.x = x
        message.y = y
        message.target_yaw = yaw
        message.yaw_rate = yaw_rate
        end = time.monotonic() + duration
        while time.monotonic() < end:
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.03)
        return message

    def _send_twist(self, x=0.0, y=0.0, yaw_rate=0.0, duration=0.5):
        message = Twist()
        message.linear.x = x
        message.linear.y = y
        message.angular.z = yaw_rate
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.twist_publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.03)

    def _send_until_reached(self, mode, x, y, yaw, timeout=12.0):
        self.command_id += 1
        message = MotionCommand()
        message.command_id = self.command_id
        message.mode = mode
        message.x = x
        message.y = y
        message.target_yaw = yaw
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.status is not None
                and self.status.command_id == message.command_id
                and self.status.state == MotionStatus.REACHED
            ):
                return
            time.sleep(0.03)
        status_text = "no status"
        if self.status is not None:
            status_text = (
                f"state={self.status.state}, detail={self.status.detail}, "
                f"position_error={self.status.position_error:.4f}, "
                f"yaw_error={self.status.yaw_error:.4f}"
            )
        pose_text = "no odometry"
        if self.odom is not None:
            pose_text = (
                f"pose=({self.odom.pose.pose.position.x:.4f}, "
                f"{self.odom.pose.pose.position.y:.4f})"
            )
        self.fail(f"Mode {mode} did not reach its target: {status_text}, {pose_text}")

    def test_keyboard_velocity_and_ui_position_modes(self):
        # Keyboard path: /cmd_vel must move and rotate the chassis simultaneously.
        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        start_yaw = self._yaw(self.imu.orientation)
        self._send_twist(x=0.12, y=0.04, yaw_rate=0.20, duration=1.5)
        self._send_twist(duration=0.8)
        displacement = math.hypot(
            self.odom.pose.pose.position.x - start_x,
            self.odom.pose.pose.position.y - start_y,
        )
        yaw_change = abs(
            self._angle_error(self._yaw(self.imu.orientation), start_yaw)
        )
        self.assertGreater(displacement, 0.04)
        self.assertGreater(yaw_change, 0.08)

        # UI path: fixed-yaw and dynamic-yaw world-position commands must reach.
        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        yaw = self._yaw(self.imu.orientation)
        self._send_until_reached(
            MotionCommand.WORLD_POSITION_FIXED_YAW,
            current_x + 0.12,
            current_y,
            yaw,
        )
        self.assertLess(
            math.hypot(
                self.odom.pose.pose.position.x - (current_x + 0.12),
                self.odom.pose.pose.position.y - current_y,
            ),
            0.05,
        )

        current_x = self.odom.pose.pose.position.x
        current_y = self.odom.pose.pose.position.y
        target_yaw = self._yaw(self.imu.orientation) + 0.35
        self._send_until_reached(
            MotionCommand.WORLD_POSITION_DYNAMIC_YAW,
            current_x,
            current_y + 0.12,
            target_yaw,
        )
        self.assertLess(
            math.hypot(
                self.odom.pose.pose.position.x - current_x,
                self.odom.pose.pose.position.y - (current_y + 0.12),
            ),
            0.05,
        )
        self.assertLess(
            abs(self._angle_error(target_yaw, self._yaw(self.imu.orientation))), 0.05
        )
