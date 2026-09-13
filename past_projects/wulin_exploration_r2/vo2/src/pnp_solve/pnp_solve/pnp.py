"""ROS 2 node that estimates a detected target position with solvePnP."""

from math import sqrt
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from msg_interface.msg import PnpResult, YoloResult
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .calibration import load_camera_calibration
from .geometry import bbox_midpoints, camera_point_to_base_flu
from .geometry import planar_target_points
from .kalman import PositionVelocityKalman
from .solver import solve_target_translation


class PnpSolveNode(Node):
    """Convert the best YOLO box into a filtered FLU-frame target point."""

    def __init__(self, **node_kwargs):
        super().__init__('pnp_solve_node', **node_kwargs)

        default_calibration = (
            Path(get_package_share_directory('pnp_solve'))
            / 'config'
            / 'calibration.example.yaml'
        )
        self.declare_parameter('calibration_file', str(default_calibration))
        self.declare_parameter('allow_example_calibration', False)
        self.declare_parameter('target_radius_m', 0.12)
        self.declare_parameter('target_class_name', '')
        self.declare_parameter('minimum_confidence', 0.25)
        self.declare_parameter('detection_topic', 'yolo_result')
        self.declare_parameter('result_topic', 'pnp_result')
        self.declare_parameter('base_frame_id', 'arm_base_link')
        self.declare_parameter('camera_to_base_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_mount_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('distance_scale', 1.0)
        self.declare_parameter('distance_offset_m', 0.0)
        self.declare_parameter('nominal_dt', 1.0 / 30.0)
        self.declare_parameter('max_prediction_frames', 10)
        self.declare_parameter('process_noise', 0.5)
        self.declare_parameter('measurement_noise', 0.01)

        calibration_file = str(self.get_parameter('calibration_file').value)
        allow_example = bool(
            self.get_parameter('allow_example_calibration').value
        )
        self.camera_matrix, self.distortion, is_example = (
            load_camera_calibration(calibration_file)
        )
        if is_example and not allow_example:
            raise RuntimeError(
                'example camera calibration is disabled; calibrate the camera and '
                'set calibration_file, or enable allow_example_calibration for a '
                'non-accurate smoke test'
            )

        self.target_radius_m = float(self.get_parameter('target_radius_m').value)
        self.target_class_name = str(
            self.get_parameter('target_class_name').value
        )
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.camera_to_base_xyz_m = list(
            self.get_parameter('camera_to_base_xyz_m').value
        )
        self.camera_mount_rpy_deg = list(
            self.get_parameter('camera_mount_rpy_deg').value
        )
        self.distance_scale = float(self.get_parameter('distance_scale').value)
        self.distance_offset_m = float(
            self.get_parameter('distance_offset_m').value
        )
        self.nominal_dt = float(self.get_parameter('nominal_dt').value)
        self.max_prediction_frames = int(
            self.get_parameter('max_prediction_frames').value
        )

        if self.target_radius_m <= 0.0:
            raise ValueError('target_radius_m must be positive')
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in [0, 1]')
        if len(self.camera_to_base_xyz_m) != 3:
            raise ValueError('camera_to_base_xyz_m must contain three values')
        if len(self.camera_mount_rpy_deg) != 3:
            raise ValueError('camera_mount_rpy_deg must contain three values')
        if self.distance_scale <= 0.0:
            raise ValueError('distance_scale must be positive')
        if self.nominal_dt <= 0.0:
            raise ValueError('nominal_dt must be positive')
        if self.max_prediction_frames < 0:
            raise ValueError('max_prediction_frames cannot be negative')

        process_noise = float(self.get_parameter('process_noise').value)
        measurement_noise = float(self.get_parameter('measurement_noise').value)
        self.filters = [
            PositionVelocityKalman(process_noise, measurement_noise)
            for _ in range(3)
        ]
        self.object_points = planar_target_points(self.target_radius_m)
        self.missed_frames = 0
        self.last_stamp_seconds = None
        self.last_warning_ns = -5_000_000_000

        result_topic = str(self.get_parameter('result_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)
        self.publisher = self.create_publisher(PnpResult, result_topic, 10)
        self.subscription = self.create_subscription(
            YoloResult, detection_topic, self._detection_callback, 10
        )
        self.get_logger().info(
            f'PnP ready: radius={self.target_radius_m:.3f} m, '
            f'calibration={calibration_file}, output_frame={self.base_frame_id}'
        )

    def _message_time_seconds(self, message):
        stamp = message.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return float(self.get_clock().now().nanoseconds) * 1.0e-9

    def _frame_dt(self, message):
        current = self._message_time_seconds(message)
        if self.last_stamp_seconds is None:
            dt = self.nominal_dt
        else:
            dt = current - self.last_stamp_seconds
            if dt <= 0.0:
                dt = self.nominal_dt
            dt = min(max(dt, 1.0e-3), 1.0)
        self.last_stamp_seconds = current
        return dt

    def _select_box(self, message):
        candidates = [
            box for box in message.boxes
            if box.confidence >= self.minimum_confidence
        ]
        if self.target_class_name:
            candidates = [
                box for box in candidates
                if box.class_name == self.target_class_name
            ]
        return max(candidates, key=lambda box: box.confidence) if candidates else None

    def _detection_callback(self, message):
        dt = self._frame_dt(message)
        box = self._select_box(message) if message.target else None
        if box is None:
            self._handle_miss(message, dt)
            return

        try:
            image_points = bbox_midpoints(box.x1, box.y1, box.x2, box.y2)
            point_camera = solve_target_translation(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.distortion,
                self.distance_scale,
                self.distance_offset_m,
            )
            point_base = camera_point_to_base_flu(
                point_camera,
                self.camera_to_base_xyz_m,
                self.camera_mount_rpy_deg,
            )
        except (ValueError, cv2.error) as error:
            self._warn_throttled(f'PnP rejected a detection: {error}')
            self._handle_miss(message, dt)
            return

        filtered = np.asarray(
            [flt.update(value, dt) for flt, value in zip(self.filters, point_base)],
            dtype=np.float64,
        )
        self.missed_frames = 0
        self._publish(message, filtered, valid=True, predicted=False)

    def _handle_miss(self, source_message, dt):
        self.missed_frames += 1
        initialized = all(flt.initialized for flt in self.filters)
        if initialized and self.missed_frames <= self.max_prediction_frames:
            predicted = np.asarray(
                [flt.predict(dt) for flt in self.filters], dtype=np.float64
            )
            self._publish(source_message, predicted, valid=True, predicted=True)
            return

        if self.missed_frames == self.max_prediction_frames + 1:
            for flt in self.filters:
                flt.reset()
        self._publish(
            source_message,
            np.zeros(3, dtype=np.float64),
            valid=False,
            predicted=False,
        )

    def _publish(self, source_message, point, valid, predicted):
        result = PnpResult()
        result.header.stamp = source_message.header.stamp
        result.header.frame_id = self.base_frame_id
        result.valid = bool(valid)
        result.predicted = bool(predicted)
        result.x, result.y, result.z = (float(value) for value in point)
        result.distance = (
            sqrt(result.x * result.x + result.y * result.y + result.z * result.z)
            if valid else -1.0
        )
        self.publisher.publish(result)

    def _warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 5_000_000_000:
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns


def main(args=None):
    """Run the PnP node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PnpSolveNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
