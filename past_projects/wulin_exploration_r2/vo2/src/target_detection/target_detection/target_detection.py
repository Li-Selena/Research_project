"""ROS 2 node that publishes YOLO detections from a local camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from msg_interface.msg import YoloBox, YoloResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ultralytics import YOLO


class DetectionNode(Node):
    """Capture camera frames, run YOLO and publish every detected box."""

    def __init__(self):
        super().__init__('yolo_detection_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('device', '')
        self.declare_parameter('use_half', False)
        self.declare_parameter('show_image', True)
        self.declare_parameter('camera_frame_id', 'camera_optical_frame')
        self.declare_parameter('result_topic', 'yolo_result')

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.device = str(self.get_parameter('device').value)
        self.use_half = bool(self.get_parameter('use_half').value)
        self.show_image = bool(self.get_parameter('show_image').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)
        result_topic = str(self.get_parameter('result_topic').value)

        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError('width, height and fps must be positive')
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in [0, 1]')

        model_path = str(self.get_parameter('model_path').value)
        self.model_path = Path(model_path) if model_path else self._default_model_path()
        if not self.model_path.is_file() or self.model_path.stat().st_size == 0:
            raise FileNotFoundError(f'YOLO model is missing or empty: {self.model_path}')

        self.publisher = self.create_publisher(YoloResult, result_topic, 10)
        self.model = YOLO(str(self.model_path))
        self.cap = cv2.VideoCapture(self.camera_id)
        try:
            self._configure_camera()
        except Exception:
            self.cap.release()
            raise
        self.last_read_warning_ns = -5_000_000_000
        self.last_inference_warning_ns = -5_000_000_000
        self.timer = self.create_timer(1.0 / self.fps, self._process_frame)

        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'camera={self.camera_id} size={actual_width}x{actual_height} '
            f'fps={actual_fps:.1f} model={self.model_path}'
        )

    @staticmethod
    def _default_model_path():
        share = Path(get_package_share_directory('target_detection'))
        return share / 'models' / 'yolov8n_test.pt'

    def _configure_camera(self):
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open camera {self.camera_id}')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def _process_frame(self):
        ok, image = self.cap.read()
        if not ok or image is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_read_warning_ns >= 5_000_000_000:
                self.get_logger().warning(f'failed to read camera {self.camera_id}')
                self.last_read_warning_ns = now_ns
            self.publisher.publish(self._new_result_message())
            return

        predict_args = {
            'source': image,
            'verbose': False,
            'conf': self.confidence_threshold,
        }
        if self.device:
            predict_args['device'] = self.device
        if self.use_half:
            predict_args['half'] = True

        try:
            result = self.model.predict(**predict_args)[0]
        except Exception as error:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_inference_warning_ns >= 5_000_000_000:
                self.get_logger().error(f'YOLO inference failed: {error}')
                self.last_inference_warning_ns = now_ns
            self.publisher.publish(self._new_result_message())
            return
        message = self._to_message(result)
        self.publisher.publish(message)

        if self.show_image:
            annotated = result.plot()
            cv2.imshow('vo2 target detection', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

    def _to_message(self, result):
        message = self._new_result_message()

        if result.boxes is None:
            message.target = False
            return message

        coordinates = result.boxes.xyxy.detach().cpu().numpy()
        confidences = result.boxes.conf.detach().cpu().numpy()
        class_ids = result.boxes.cls.detach().cpu().numpy().astype(int)
        names = result.names

        for xyxy, confidence, class_id in zip(
            coordinates, confidences, class_ids
        ):
            box = YoloBox()
            box.x1, box.y1, box.x2, box.y2 = (float(value) for value in xyxy)
            box.confidence = float(confidence)
            box.class_id = int(class_id)
            if isinstance(names, dict):
                box.class_name = str(names.get(int(class_id), int(class_id)))
            else:
                box.class_name = str(names[int(class_id)])
            message.boxes.append(box)

        message.target = bool(message.boxes)
        return message

    def _new_result_message(self):
        message = YoloResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.camera_frame_id
        message.target = False
        return message

    def close(self):
        """Release camera and GUI resources."""
        if hasattr(self, 'cap'):
            self.cap.release()
        if self.show_image:
            cv2.destroyAllWindows()


def main(args=None):
    """Run the detection node."""
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionNode()
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
