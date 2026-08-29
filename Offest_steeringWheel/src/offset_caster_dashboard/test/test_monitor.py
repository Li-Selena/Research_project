import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from offset_caster_interfaces.msg import MotionStatus
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import JointState

from offset_caster_dashboard.monitor import CurveMonitorWindow


class FakeMonitorBridge:
    def __init__(self):
        self.status = MotionStatus()
        self.status.position_error = 0.4
        self.status.yaw_error = -0.2
        self.joint_state = JointState()
        self.joint_state.name = [
            "steering_joint_1",
            "steering_joint_2",
            "steering_joint_3",
            "steering_joint_4",
            "wheel_joint_1",
            "wheel_joint_2",
            "wheel_joint_3",
            "wheel_joint_4",
        ]
        self.joint_state.position = [0.1, 0.2, 0.3, 0.4, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0]
        self.last_data_time = time.monotonic()


def test_curve_monitor_offscreen_sampling_pause_and_clear():
    application = QApplication.instance() or QApplication([])
    bridge = FakeMonitorBridge()
    window = CurveMonitorWindow(bridge)
    window.refresh()
    assert window.connection_badge.text() == "数据正常"
    assert window.error_plot.history["位置误差"][-1][1] == 0.4
    assert window.error_plot.history["Yaw误差"][-1][1] == 0.2
    assert window.steering_plot.history["舵角 4"][-1][1] == 0.4
    assert window.wheel_plot.history["轮速 4"][-1][1] == 4.0

    sample_count = len(window.error_plot.history["位置误差"])
    window.toggle_pause()
    window.refresh()
    assert window.pause_button.text() == "继续"
    assert len(window.error_plot.history["位置误差"]) == sample_count

    window.clear_plots()
    assert not window.error_plot.history["位置误差"]
    assert not window.steering_plot.history["舵角 1"]
    assert not window.wheel_plot.history["轮速 1"]
    window.close()
    application.processEvents()
