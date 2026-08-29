import math
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from geometry_msgs.msg import Quaternion, Twist
from offset_caster_interfaces.msg import MotionCommand, MotionStatus
from python_qt_binding.QtWidgets import QApplication

from offset_caster_dashboard.dashboard import DashboardWindow
from offset_caster_dashboard.dashboard_model import field_configuration, quaternion_to_rpy


class FakeBridge:
    def __init__(self):
        self.odom = None
        self.imu = None
        self.status = None
        self.cmd_vel = None
        self.cmd_vel_time = 0.0
        self.last_activation = None
        self.stopped = False

    def activate(self, *values):
        self.last_activation = values

    def stop(self):
        self.stopped = True

    def publish_heartbeat(self):
        pass


def test_all_six_mode_field_configurations():
    fixed_modes = [
        MotionCommand.WORLD_VELOCITY_FIXED_YAW,
        MotionCommand.BODY_VELOCITY_FIXED_YAW,
        MotionCommand.WORLD_POSITION_FIXED_YAW,
        MotionCommand.WORLD_POSITION_DYNAMIC_YAW,
    ]
    dynamic_velocity_modes = [
        MotionCommand.WORLD_VELOCITY_DYNAMIC_YAW,
        MotionCommand.BODY_VELOCITY_DYNAMIC_YAW,
    ]
    for mode in fixed_modes:
        config = field_configuration(mode)
        assert config["yaw_enabled"]
        assert not config["yaw_rate_enabled"]
    for mode in dynamic_velocity_modes:
        config = field_configuration(mode)
        assert not config["yaw_enabled"]
        assert config["yaw_rate_enabled"]
    assert "目标" in field_configuration(MotionCommand.WORLD_POSITION_FIXED_YAW)["x_label"]


def test_quaternion_to_rpy_yaw():
    quaternion = Quaternion()
    quaternion.w = math.cos(math.pi / 4.0)
    quaternion.z = math.sin(math.pi / 4.0)
    roll, pitch, yaw = quaternion_to_rpy(quaternion)
    assert abs(roll) < 1.0e-12
    assert abs(pitch) < 1.0e-12
    assert abs(yaw - math.pi / 2.0) < 1.0e-12


def test_dashboard_offscreen_smoke_and_command_mapping():
    application = QApplication.instance() or QApplication([])
    bridge = FakeBridge()
    window = DashboardWindow(bridge)
    assert window.mode.count() == 2
    window.mode.setCurrentIndex(1)
    window.x.setValue(1.2)
    window.y.setValue(-0.4)
    window.yaw.setValue(0.3)
    window.send_command()
    assert bridge.last_activation[0] == MotionCommand.WORLD_POSITION_DYNAMIC_YAW
    assert bridge.last_activation[1:] == (1.2, -0.4, 0.3, 0.0)
    bridge.cmd_vel = Twist()
    bridge.cmd_vel.linear.x = 0.25
    bridge.cmd_vel.angular.z = -0.2
    bridge.cmd_vel_time = time.monotonic()
    bridge.status = MotionStatus()
    bridge.status.state = MotionStatus.IDLE
    window.refresh()
    assert "vx=+0.250" in window.low_command_value.text()
    assert window.state_badge.text() == "键盘速度"
    window.stop_command()
    assert bridge.stopped
    window.close()
    application.processEvents()
