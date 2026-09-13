"""Synthetic projection tests for the PnP calculation."""

import cv2
import numpy as np
from pnp_solve.geometry import planar_target_points
from pnp_solve.solver import solve_target_translation


def test_solve_pnp_recovers_synthetic_translation():
    camera_matrix = np.asarray(
        [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros((5, 1), dtype=np.float64)
    object_points = planar_target_points(0.12)
    expected = np.asarray([0.1, -0.05, 2.0], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        object_points,
        np.zeros(3, dtype=np.float64),
        expected,
        camera_matrix,
        distortion,
    )

    actual = solve_target_translation(
        object_points,
        image_points.reshape(-1, 2),
        camera_matrix,
        distortion,
    )
    assert np.allclose(actual, expected, atol=1.0e-5)


def test_distance_correction_scales_the_translation_vector():
    camera_matrix = np.asarray(
        [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros((5, 1), dtype=np.float64)
    object_points = planar_target_points(0.12)
    expected = np.asarray([0.0, 0.0, 2.0], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        object_points,
        np.zeros(3, dtype=np.float64),
        expected,
        camera_matrix,
        distortion,
    )

    actual = solve_target_translation(
        object_points,
        image_points.reshape(-1, 2),
        camera_matrix,
        distortion,
        distance_scale=1.1,
        distance_offset_m=0.1,
    )
    assert np.allclose(actual, [0.0, 0.0, 2.3], atol=1.0e-5)
