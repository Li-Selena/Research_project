import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from offset_caster_interfaces.msg import MotionCommand, MotionStatus
from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu

from .dashboard_model import (
    MODE_NAMES,
    POSITION_MODE_NAMES,
    field_configuration,
    quaternion_to_rpy,
)


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("offset_caster_dashboard")
        self.publisher = self.create_publisher(
            MotionCommand, "/offset_caster/motion_command", 10
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(
            Imu, "/imu/data", self._imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            MotionStatus, "/offset_caster/motion_status", self._status_callback, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.odom = None
        self.imu = None
        self.status = None
        self.cmd_vel = None
        self.cmd_vel_time = 0.0
        self.command = None
        self.command_id = 0
        self.active = False

    def _odom_callback(self, message):
        self.odom = message

    def _imu_callback(self, message):
        self.imu = message

    def _status_callback(self, message):
        self.status = message

    def _cmd_vel_callback(self, message):
        self.cmd_vel = message
        self.cmd_vel_time = time.monotonic()

    def activate(self, mode, x, y, target_yaw, yaw_rate):
        self.command_id += 1
        message = MotionCommand()
        message.command_id = self.command_id
        message.mode = mode
        message.x = x
        message.y = y
        message.target_yaw = target_yaw
        message.yaw_rate = yaw_rate
        self.command = message
        self.active = True
        self.publish_heartbeat()

    def stop(self):
        self.command_id += 1
        message = MotionCommand()
        message.command_id = self.command_id
        message.mode = MotionCommand.STOP
        self.command = message
        self.active = False
        self.publish_heartbeat(force=True)

    def publish_heartbeat(self, force=False):
        if self.command is None or (not self.active and not force):
            return
        self.command.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.command)


class CoordinateWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.setMinimumSize(330, 330)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_yaw(self, yaw):
        self.yaw = yaw
        self.update()

    @staticmethod
    def _arrow(painter, center, angle, length, color, label):
        import math

        end = QPointF(
            center.x() + length * math.cos(angle),
            center.y() - length * math.sin(angle),
        )
        painter.setPen(QPen(color, 4, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(center, end)
        left = QPointF(
            end.x() - 12 * math.cos(angle - 0.45),
            end.y() + 12 * math.sin(angle - 0.45),
        )
        right = QPointF(
            end.x() - 12 * math.cos(angle + 0.45),
            end.y() + 12 * math.sin(angle + 0.45),
        )
        painter.drawLine(end, left)
        painter.drawLine(end, right)
        painter.setPen(color)
        painter.drawText(QRectF(end.x() - 20, end.y() - 28, 40, 24), Qt.AlignCenter, label)

    def paintEvent(self, _event):
        import math

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101722"))
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) * 0.36
        painter.setPen(QPen(QColor("#2b3b50"), 2))
        painter.drawEllipse(center, radius, radius)
        painter.setFont(QFont("Sans", 10, QFont.Bold))
        self._arrow(painter, center, 0.0, radius, QColor("#38bdf8"), "世界 X")
        self._arrow(painter, center, math.pi / 2.0, radius, QColor("#34d399"), "世界 Y")
        self._arrow(painter, center, self.yaw, radius * 0.50, QColor("#fb7185"), "机器人 x")
        self._arrow(
            painter,
            center,
            self.yaw + math.pi / 2.0,
            radius * 0.50,
            QColor("#fbbf24"),
            "机器人 y",
        )
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(
            QRectF(0, self.height() - 32, self.width(), 24),
            Qt.AlignCenter,
            f"机器人相对世界朝向  {self.yaw:+.3f} rad",
        )


class DashboardWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("偏置主动脚轮 · 运动控制台")
        self.resize(1180, 760)
        self._build_ui()
        self._apply_style()
        self.mode_changed()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(10)
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(self.bridge.publish_heartbeat)
        self.heartbeat_timer.start(50)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh)
        self.refresh_timer.start(50)

    def _spin_ros(self):
        try:
            rclpy.spin_once(self.bridge, timeout_sec=0.0)
        except (KeyboardInterrupt, ExternalShutdownException):
            self.close()

    @staticmethod
    def _value_box(minimum, maximum, decimals=3):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setDecimals(decimals)
        box.setSingleStep(0.05)
        return box

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        title_row = QHBoxLayout()
        title = QLabel("偏置主动脚轮  /  POSITION & KEYBOARD CONSOLE")
        title.setObjectName("title")
        self.state_badge = QLabel("等待数据")
        self.state_badge.setObjectName("badge")
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self.state_badge)
        outer.addLayout(title_row)

        content = QGridLayout()
        outer.addLayout(content, 1)

        command_group = QGroupBox("运动命令")
        command_form = QFormLayout(command_group)
        self.mode = QComboBox()
        for value, name in POSITION_MODE_NAMES.items():
            self.mode.addItem(name, value)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.x = self._value_box(-20.0, 20.0)
        self.y = self._value_box(-20.0, 20.0)
        self.yaw = self._value_box(-3.14159, 3.14159)
        self.x_label = QLabel()
        self.y_label = QLabel()
        command_form.addRow("控制模式", self.mode)
        command_form.addRow(self.x_label, self.x)
        command_form.addRow(self.y_label, self.y)
        command_form.addRow("目标 Yaw (rad)", self.yaw)
        keyboard_hint = QLabel(
            "速度控制由键盘终端下发。切换到键盘前请先点击“停止”。"
        )
        keyboard_hint.setWordWrap(True)
        keyboard_hint.setStyleSheet("color: #94a3b8; padding: 6px 0;")
        command_form.addRow(keyboard_hint)
        button_row = QHBoxLayout()
        self.send_button = QPushButton("发送 / 保持")
        self.send_button.setObjectName("send")
        self.stop_button = QPushButton("停止")
        self.stop_button.setObjectName("stop")
        self.send_button.clicked.connect(self.send_command)
        self.stop_button.clicked.connect(self.stop_command)
        button_row.addWidget(self.send_button)
        button_row.addWidget(self.stop_button)
        command_form.addRow(button_row)

        attitude_group = QGroupBox("机器人姿态与世界坐标")
        attitude_form = QFormLayout(attitude_group)
        self.roll_value = QLabel("—")
        self.pitch_value = QLabel("—")
        self.yaw_value = QLabel("—")
        self.position_value = QLabel("—")
        attitude_form.addRow("Roll", self.roll_value)
        attitude_form.addRow("Pitch", self.pitch_value)
        attitude_form.addRow("Yaw", self.yaw_value)
        attitude_form.addRow("世界坐标 X / Y / Z", self.position_value)

        left = QVBoxLayout()
        left.addWidget(command_group)
        left.addWidget(attitude_group)
        left.addStretch()
        left_frame = QFrame()
        left_frame.setLayout(left)

        self.coordinate = CoordinateWidget()

        telemetry_group = QGroupBox("实时控制状态")
        telemetry_form = QFormLayout(telemetry_group)
        self.active_mode_value = QLabel("—")
        self.high_command_value = QLabel("—")
        self.low_command_value = QLabel("—")
        self.error_value = QLabel("—")
        self.detail_value = QLabel("—")
        self.high_command_value.setWordWrap(True)
        telemetry_form.addRow("当前模式", self.active_mode_value)
        telemetry_form.addRow("高层命令", self.high_command_value)
        telemetry_form.addRow("实时 /cmd_vel（键盘/位置）", self.low_command_value)
        telemetry_form.addRow("位置 / Yaw 误差", self.error_value)
        telemetry_form.addRow("控制器信息", self.detail_value)

        content.addWidget(left_frame, 0, 0)
        content.addWidget(self.coordinate, 0, 1)
        content.addWidget(telemetry_group, 0, 2)
        content.setColumnStretch(0, 3)
        content.setColumnStretch(1, 4)
        content.setColumnStretch(2, 4)


    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #0b111b; color: #dbeafe; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #f8fafc; padding: 8px; }
            QLabel#badge { background: #334155; border-radius: 12px; padding: 7px 14px; font-weight: 700; }
            QGroupBox { border: 1px solid #26364b; border-radius: 10px; margin-top: 14px; padding: 14px; font-weight: 700; color: #93c5fd; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }
            QComboBox, QDoubleSpinBox { background: #111c2d; border: 1px solid #334d6b; border-radius: 6px; padding: 7px; min-height: 22px; color: #f8fafc; }
            QPushButton { border: 0; border-radius: 7px; padding: 10px 16px; font-weight: 700; }
            QPushButton#send { background: #0284c7; color: white; }
            QPushButton#send:hover { background: #0ea5e9; }
            QPushButton#stop { background: #be123c; color: white; }
            QPushButton#stop:hover { background: #e11d48; }
            """
        )

    def mode_changed(self):
        config = field_configuration(self.mode.currentData())
        self.x_label.setText(config["x_label"])
        self.y_label.setText(config["y_label"])
        self.yaw.setEnabled(config["yaw_enabled"])

    def send_command(self):
        self.bridge.activate(
            self.mode.currentData(),
            self.x.value(),
            self.y.value(),
            self.yaw.value(),
            0.0,
        )

    def stop_command(self):
        self.bridge.stop()

    def refresh(self):
        if self.bridge.imu is not None:
            roll, pitch, yaw = quaternion_to_rpy(self.bridge.imu.orientation)
            self.roll_value.setText(f"{roll:+.4f} rad")
            self.pitch_value.setText(f"{pitch:+.4f} rad")
            self.yaw_value.setText(f"{yaw:+.4f} rad")
            self.coordinate.set_yaw(yaw)
        if self.bridge.odom is not None:
            position = self.bridge.odom.pose.pose.position
            self.position_value.setText(
                f"{position.x:+.3f}  /  {position.y:+.3f}  /  {position.z:+.3f} m"
            )
        live_twist = self.bridge.cmd_vel
        if live_twist is not None:
            self.low_command_value.setText(
                f"vx={live_twist.linear.x:+.3f} m/s   "
                f"vy={live_twist.linear.y:+.3f} m/s   "
                f"wz={live_twist.angular.z:+.3f} rad/s"
            )
        status = self.bridge.status
        if status is None:
            return
        names = {
            MotionStatus.IDLE: ("空闲", "#334155"),
            MotionStatus.ACTIVE: ("运行中", "#0369a1"),
            MotionStatus.REACHED: ("已到达", "#047857"),
            MotionStatus.FAULT: ("故障", "#be123c"),
        }
        text, color = names.get(status.state, ("未知", "#334155"))
        self.state_badge.setText(text)
        self.state_badge.setStyleSheet(
            f"background: {color}; border-radius: 12px; padding: 7px 14px; font-weight: 700;"
        )
        self.active_mode_value.setText(MODE_NAMES.get(status.mode, "停止"))
        command = status.active_command
        self.high_command_value.setText(
            f"ID {command.command_id}  |  x={command.x:+.3f}, y={command.y:+.3f}, "
            f"yaw={command.target_yaw:+.3f}, wz={command.yaw_rate:+.3f}"
        )
        keyboard_command_is_live = (
            live_twist is not None
            and time.monotonic() - self.bridge.cmd_vel_time < 0.5
            and (
                abs(live_twist.linear.x) > 1.0e-5
                or abs(live_twist.linear.y) > 1.0e-5
                or abs(live_twist.angular.z) > 1.0e-5
            )
            and status.state != MotionStatus.ACTIVE
        )
        if keyboard_command_is_live:
            self.state_badge.setText("键盘速度")
            self.state_badge.setStyleSheet(
                "background: #7c3aed; border-radius: 12px; padding: 7px 14px; "
                "font-weight: 700;"
            )
        self.error_value.setText(
            f"{status.position_error:.4f} m  /  {status.yaw_error:+.4f} rad"
        )
        self.detail_value.setText(status.detail)

    def closeEvent(self, event):
        try:
            self.bridge.stop()
        except Exception:  # ROS may already be shutting down after SIGINT.
            pass
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    bridge = DashboardBridge()
    application = QApplication.instance() or QApplication(sys.argv)
    application.setFont(QFont("Noto Sans CJK SC", 10))
    window = DashboardWindow(bridge)
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
