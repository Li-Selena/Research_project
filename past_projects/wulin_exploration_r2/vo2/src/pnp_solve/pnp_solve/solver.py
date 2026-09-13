"""Validated OpenCV PnP calculation helpers."""

import cv2
import numpy as np


def solve_target_translation(
    object_points,
    image_points,
    camera_matrix,
    distortion,
    distance_scale=1.0,
    distance_offset_m=0.0,
):
    """Return the corrected target-origin translation in optical coordinates."""
    if distance_scale <= 0.0:
        raise ValueError('distance_scale must be positive')
    ok, _rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise ValueError('cv2.solvePnP did not find a solution')

    point_camera = translation_vector.reshape(3).astype(np.float64)
    raw_distance = float(np.linalg.norm(point_camera))
    if raw_distance <= 1.0e-9:
        raise ValueError('solvePnP returned a zero-length translation')
    corrected_distance = raw_distance * distance_scale + distance_offset_m
    if corrected_distance <= 0.0:
        raise ValueError('distance correction produced a non-positive range')
    return point_camera * (corrected_distance / raw_distance)
