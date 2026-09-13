"""Tests for camera calibration validation."""

from pathlib import Path

from pnp_solve.calibration import load_camera_calibration
import pytest


def test_example_calibration_loads():
    path = Path(__file__).parents[1] / 'config' / 'calibration.example.yaml'
    camera_matrix, distortion, is_example = load_camera_calibration(path)
    assert camera_matrix.shape == (3, 3)
    assert distortion.shape == (5, 1)
    assert is_example is True


def test_missing_calibration_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_camera_calibration(tmp_path / 'missing.yaml')
