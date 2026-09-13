"""Geometry helpers for converting image detections into the robot frame."""

from math import cos, radians, sin

import numpy as np


def bbox_midpoints(x1, y1, x2, y2):
    """Return top, bottom, left and right midpoint pixels for a valid box."""
    if x2 <= x1 or y2 <= y1:
        raise ValueError('bounding box must have positive width and height')
    mid_x = (x1 + x2) * 0.5
    mid_y = (y1 + y2) * 0.5
    return np.asarray(
        [
            [mid_x, y1],
            [mid_x, y2],
            [x1, mid_y],
            [x2, mid_y],
        ],
        dtype=np.float32,
    )


def planar_target_points(radius_m):
    """Return target points matching top, bottom, left and right image points."""
    if radius_m <= 0.0:
        raise ValueError('target radius must be positive')
    return np.asarray(
        [
            [0.0, -radius_m, 0.0],
            [0.0, radius_m, 0.0],
            [-radius_m, 0.0, 0.0],
            [radius_m, 0.0, 0.0],
        ],
        dtype=np.float32,
    )


def rotation_matrix_rpy(rpy_deg):
    """Build a FLU-frame Rz(yaw) Ry(pitch) Rx(roll) rotation matrix."""
    if len(rpy_deg) != 3:
        raise ValueError('roll, pitch and yaw must contain three values')
    roll, pitch, yaw = (radians(float(value)) for value in rpy_deg)
    rx = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, cos(roll), -sin(roll)],
         [0.0, sin(roll), cos(roll)]],
        dtype=np.float64,
    )
    ry = np.asarray(
        [[cos(pitch), 0.0, sin(pitch)], [0.0, 1.0, 0.0],
         [-sin(pitch), 0.0, cos(pitch)]],
        dtype=np.float64,
    )
    rz = np.asarray(
        [[cos(yaw), -sin(yaw), 0.0], [sin(yaw), cos(yaw), 0.0],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rz @ ry @ rx


def optical_to_flu(point_camera):
    """Convert OpenCV optical axes (right, down, forward) to FLU axes."""
    point = np.asarray(point_camera, dtype=np.float64).reshape(3)
    return np.asarray([point[2], -point[0], -point[1]], dtype=np.float64)


def camera_point_to_base_flu(point_camera, translation_m, mounting_rpy_deg):
    """Transform an optical-frame point into the robot base FLU frame."""
    translation = np.asarray(translation_m, dtype=np.float64).reshape(3)
    rotation = rotation_matrix_rpy(mounting_rpy_deg)
    return rotation @ optical_to_flu(point_camera) + translation
