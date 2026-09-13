"""Behavior tests for the position/velocity Kalman filter."""

from pnp_solve.kalman import PositionVelocityKalman
import pytest


def test_filter_tracks_constant_velocity_and_predicts_forward():
    flt = PositionVelocityKalman(process_noise=0.5, measurement_noise=0.001)
    for step in range(11):
        estimate = flt.update(step * 0.1, 0.1)
    assert estimate == pytest.approx(1.0, abs=0.02)
    assert flt.state[1, 0] == pytest.approx(1.0, abs=0.1)
    assert flt.predict(0.1) == pytest.approx(1.1, abs=0.03)


def test_reset_requires_a_new_measurement_before_prediction():
    flt = PositionVelocityKalman()
    flt.update(2.0, 0.1)
    flt.reset()
    with pytest.raises(RuntimeError):
        flt.predict(0.1)
