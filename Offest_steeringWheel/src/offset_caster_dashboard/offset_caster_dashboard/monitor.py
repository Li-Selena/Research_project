import sys
import time

import rclpy
from offset_caster_interfaces.msg import MotionStatus
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .plot_widgets import TimeSeriesPlot


class CurveMonitorBridge(Node):
    def __init__(self):
        super().__init__("offset_caster_curve_monitor")
        self.status = None
        self.joint_state = None
        self.last_data_time = 0.0
        self.create_subscription(
            MotionStatus, "/offset_caster/motion_status", self._status_callback, 10
        )
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)

    def _status_callback(self, message):
        self.status = message
        self.last_data_time = time.monotonic()

    def _joint_callback(self, message):
        self.joint_state = message
        self.last_data_time = time.monotonic()


class CurveMonitorWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.paused = False
        self.setWindowTitle("偏置主动脚轮 · 独立曲线监测")
        self.resize(1260, 980)
        self._build_ui()
        self._apply_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(10)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(50)

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        header = QHBoxLayout()
        title = QLabel("实时运动曲线  /  MOTION CURVE MONITOR")
        title.setObjectName("title")
        self.connection_badge = QLabel("等待数据")
        self.connection_badge.setObjectName("badge")
        self.pause_button = QPushButton("暂停")
        self.clear_button = QPushButton("清空")
        self.pause_button.clicked.connect(self.toggle_pause)
        self.clear_button.clicked.connect(self.clear_plots)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.connection_badge)
        header.addWidget(self.pause_button)
        header.addWidget(self.clear_button)
        outer.addLayout(header)

        colors = ["#38bdf8", "#34d399", "#fbbf24", "#fb7185"]
        self.error_plot = TimeSeriesPlot(
            "轨迹误差", "位置 m / Yaw rad",
            {"位置误差": "#38bdf8", "Yaw误差": "#fb7185"},
        )
        self.steering_plot = TimeSeriesPlot(
            "四路舵角", "rad",
            {f"舵角 {index + 1}": color for index, color in enumerate(colors)},
        )
        self.wheel_plot = TimeSeriesPlot(
            "四路轮速", "rad/s",
            {f"轮速 {index + 1}": color for index, color in enumerate(colors)},
        )
        plots = QGridLayout()
        plots.addWidget(self.error_plot, 0, 0)
        plots.addWidget(self.steering_plot, 1, 0)
        plots.addWidget(self.wheel_plot, 2, 0)
        outer.addLayout(plots, 1)

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #0b111b; color: #dbeafe; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #f8fafc; padding: 8px; }
            QLabel#badge { background: #334155; border-radius: 12px; padding: 7px 14px; font-weight: 700; }
            QPushButton { background: #1e3a5f; color: white; border: 0; border-radius: 7px; padding: 9px 16px; font-weight: 700; }
            QPushButton:hover { background: #2563eb; }
            """
        )

    def _spin_ros(self):
        try:
            rclpy.spin_once(self.bridge, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self.close()

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("继续" if self.paused else "暂停")

    def clear_plots(self):
        self.error_plot.clear()
        self.steering_plot.clear()
        self.wheel_plot.clear()

    def refresh(self):
        connected = time.monotonic() - self.bridge.last_data_time < 0.5
        self.connection_badge.setText("数据正常" if connected else "等待数据")
        self.connection_badge.setStyleSheet(
            "background: #047857; border-radius: 12px; padding: 7px 14px; font-weight: 700;"
            if connected
            else "background: #334155; border-radius: 12px; padding: 7px 14px; font-weight: 700;"
        )
        if self.paused:
            return
        if self.bridge.status is not None:
            self.error_plot.add_sample(
                {
                    "位置误差": self.bridge.status.position_error,
                    "Yaw误差": abs(self.bridge.status.yaw_error),
                }
            )
        joint_state = self.bridge.joint_state
        if joint_state is None:
            return
        steering_values = {}
        wheel_values = {}
        for caster_index in range(4):
            steering_name = f"steering_joint_{caster_index + 1}"
            wheel_name = f"wheel_joint_{caster_index + 1}"
            if steering_name in joint_state.name:
                message_index = joint_state.name.index(steering_name)
                if message_index < len(joint_state.position):
                    steering_values[f"舵角 {caster_index + 1}"] = joint_state.position[
                        message_index
                    ]
            if wheel_name in joint_state.name:
                message_index = joint_state.name.index(wheel_name)
                if message_index < len(joint_state.velocity):
                    wheel_values[f"轮速 {caster_index + 1}"] = joint_state.velocity[
                        message_index
                    ]
        self.steering_plot.add_sample(steering_values)
        self.wheel_plot.add_sample(wheel_values)


def main(args=None):
    rclpy.init(args=args)
    bridge = CurveMonitorBridge()
    application = QApplication.instance() or QApplication(sys.argv)
    application.setFont(QFont("Noto Sans CJK SC", 10))
    window = CurveMonitorWindow(bridge)
    window.show()
    try:
        exit_code = application.exec_()
    except KeyboardInterrupt:
        exit_code = 0
    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
