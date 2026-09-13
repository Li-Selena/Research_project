"""Tests for camera-to-robot coordinate conversion."""

import numpy as np
from pnp_solve.geometry import bbox_midpoints, camera_point_to_base_flu
from pnp_solve.geometry import planar_target_points
import pytest


def test_bbox_and_object_points_use_the_same_order():
    image = bbox_midpoints(10.0, 20.0, 30.0, 60.0)
    target = planar_target_points(0.12)
    assert image.tolist() == [
        [20.0, 20.0],
        [20.0, 60.0],
        [10.0, 40.0],
        [30.0, 40.0],
    ]
    assert np.allclose(target, [
        [0.0, -0.12, 0.0],
        [0.0, 0.12, 0.0],
        [-0.12, 0.0, 0.0],
        [0.12, 0.0, 0.0],
    ])


def test_optical_axes_map_to_robot_flu():
    # OpenCV optical: right, down, forward -> robot: forward, left, up.
    result = camera_point_to_base_flu(
        [1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    )
    assert np.allclose(result, [3.0, -1.0, -2.0])


def test_mounting_transform_is_applied_in_flu():
    result = camera_point_to_base_flu(
        [0.0, 0.0, 1.0], [0.1, 0.2, 0.3], [0.0, 0.0, 90.0]
    )
    assert np.allclose(result, [0.1, 1.2, 0.3], atol=1.0e-9)


def test_invalid_bbox_is_rejected():
    with pytest.raises(ValueError):
        bbox_midpoints(10.0, 20.0, 10.0, 60.0)
