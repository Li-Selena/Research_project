"""Read and validate ROS-style camera calibration YAML files."""

from pathlib import Path

import numpy as np
import yaml


def _matrix_data(document, key):
    value = document.get(key)
    if isinstance(value, dict):
        value = value.get('data')
    if not isinstance(value, list):
        raise ValueError(f'{key}.data must be a list')
    return value


def load_camera_calibration(path):
    """Return camera matrix, distortion coefficients and example flag."""
    calibration_path = Path(path).expanduser()
    if not calibration_path.is_file():
        raise FileNotFoundError(f'camera calibration file not found: {path}')

    with calibration_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('camera calibration must be a YAML mapping')

    camera_values = _matrix_data(document, 'camera_matrix')
    distortion_values = _matrix_data(document, 'distortion_coefficients')
    if len(camera_values) != 9:
        raise ValueError('camera_matrix must contain exactly 9 values')
    if len(distortion_values) < 4:
        raise ValueError('distortion_coefficients must contain at least 4 values')

    camera_matrix = np.asarray(camera_values, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion_values, dtype=np.float64).reshape(-1, 1)
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise ValueError('camera focal lengths fx and fy must be positive')
    return camera_matrix, distortion, bool(document.get('example', False))
