"""ROS-level test for the detection-to-PnP message pipeline."""

from time import monotonic

from msg_interface.msg import PnpResult, YoloBox, YoloResult
from pnp_solve.pnp import PnpSolveNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def test_detection_message_produces_flu_target():
    rclpy.init()
    pnp_node = PnpSolveNode(parameter_overrides=[
        Parameter('allow_example_calibration', value=True),
    ])
    test_node = Node('pnp_pipeline_test_node')
    received = []
    publisher = test_node.create_publisher(YoloResult, 'yolo_result', 10)
    test_node.create_subscription(PnpResult, 'pnp_result', received.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(pnp_node)
    executor.add_node(test_node)

    try:
        detection = YoloResult()
        detection.header.stamp = test_node.get_clock().now().to_msg()
        detection.header.frame_id = 'camera_optical_frame'
        detection.target = True
        box = YoloBox()
        # With fx=fy=600, radius=0.12 m and Z=2 m, radius is 36 pixels.
        box.x1, box.y1, box.x2, box.y2 = 284.0, 204.0, 356.0, 276.0
        box.confidence = 1.0
        box.class_id = 0
        box.class_name = 'item'
        detection.boxes.append(box)

        deadline = monotonic() + 3.0
        while not received and monotonic() < deadline:
            publisher.publish(detection)
            executor.spin_once(timeout_sec=0.1)

        assert received, 'PnP node did not publish a result'
        result = received[-1]
        assert result.valid is True
        assert result.predicted is False
        assert result.header.frame_id == 'arm_base_link'
        assert result.x == pytest.approx(2.0, abs=1.0e-4)
        assert result.y == pytest.approx(0.0, abs=1.0e-4)
        assert result.z == pytest.approx(0.0, abs=1.0e-4)
        assert result.distance == pytest.approx(2.0, abs=1.0e-4)
    finally:
        executor.shutdown()
        pnp_node.destroy_node()
        test_node.destroy_node()
        rclpy.shutdown()
