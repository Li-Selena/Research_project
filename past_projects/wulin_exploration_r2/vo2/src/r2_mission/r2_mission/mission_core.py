"""Pure helpers for target stability and arm waypoint generation."""

from collections import deque
from math import dist, isfinite


class TargetStabilizer:
    """Accept a target after consecutive measured points form a tight cluster."""

    def __init__(self, sample_count, tolerance_m):
        if not 1 <= sample_count <= 100:
            raise ValueError('stable_samples must be in 1..100')
        if not isfinite(tolerance_m) or not 0 < tolerance_m <= 0.5:
            raise ValueError('stable_tolerance_m must be in (0, 0.5]')
        self.samples = deque(maxlen=sample_count)
        self.sample_count = sample_count
        self.tolerance = tolerance_m

    def add(self, point):
        """Add a finite XYZ point and return its stable centroid when ready."""
        values = tuple(float(value) for value in point)
        if len(values) != 3 or not all(isfinite(value) for value in values):
            return None
        self.samples.append(values)
        if len(self.samples) < self.sample_count:
            return None
        center = tuple(
            sum(sample[axis] for sample in self.samples) / len(self.samples)
            for axis in range(3)
        )
        if max(dist(sample, center) for sample in self.samples) > self.tolerance:
            return None
        return center


def arm_waypoints(target_m, pregrasp_backoff_m, pregrasp_lift_m,
                  grasp_forward_offset_m, retreat_backoff_m,
                  retreat_lift_m):
    """Build pre-grasp, grasp and retreat points in arm-base FLU metres."""
    values = [
        *target_m,
        pregrasp_backoff_m,
        pregrasp_lift_m,
        grasp_forward_offset_m,
        retreat_backoff_m,
        retreat_lift_m,
    ]
    if not all(isfinite(value) for value in values):
        raise ValueError('target and offsets must be finite')
    if any(value < 0 or value > 1 for value in (
        pregrasp_backoff_m, pregrasp_lift_m,
        retreat_backoff_m, retreat_lift_m,
    )):
        raise ValueError('backoff/lift offsets must be in [0, 1] m')
    if abs(grasp_forward_offset_m) > 0.5:
        raise ValueError('grasp_forward_offset_m must be within ±0.5 m')

    x, y, z = (float(value) for value in target_m)
    grasp = (x + grasp_forward_offset_m, y, z)
    pregrasp = (
        grasp[0] - pregrasp_backoff_m,
        grasp[1],
        grasp[2] + pregrasp_lift_m,
    )
    retreat = (
        grasp[0] - retreat_backoff_m,
        grasp[1],
        grasp[2] + retreat_lift_m,
    )
    return pregrasp, grasp, retreat
