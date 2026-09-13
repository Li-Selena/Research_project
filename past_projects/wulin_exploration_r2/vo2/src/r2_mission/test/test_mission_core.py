"""Tests for target stability and generated arm motion points."""

import pytest

from r2_mission.mission_core import arm_waypoints, TargetStabilizer


def test_target_requires_a_tight_consecutive_cluster():
    stabilizer = TargetStabilizer(3, 0.02)

    assert stabilizer.add((0.40, 0.00, 0.30)) is None
    assert stabilizer.add((0.60, 0.00, 0.30)) is None
    assert stabilizer.add((0.41, 0.00, 0.30)) is None
    assert stabilizer.add((0.405, 0.005, 0.295)) is None
    center = stabilizer.add((0.395, -0.005, 0.305))

    assert center == pytest.approx((0.403333, 0.0, 0.3), abs=1e-6)


def test_arm_waypoints_follow_flu_backoff_and_lift():
    points = arm_waypoints(
        (0.50, -0.10, 0.30),
        pregrasp_backoff_m=0.08,
        pregrasp_lift_m=0.05,
        grasp_forward_offset_m=0.01,
        retreat_backoff_m=0.12,
        retreat_lift_m=0.10,
    )

    assert points[0] == pytest.approx((0.43, -0.10, 0.35))
    assert points[1] == pytest.approx((0.51, -0.10, 0.30))
    assert points[2] == pytest.approx((0.39, -0.10, 0.40))


def test_unstable_or_non_finite_settings_are_rejected():
    with pytest.raises(ValueError, match='stable_samples'):
        TargetStabilizer(0, 0.02)
    with pytest.raises(ValueError, match='offsets'):
        arm_waypoints((0.5, 0.0, 0.3), -0.1, 0.0, 0.0, 0.1, 0.1)
