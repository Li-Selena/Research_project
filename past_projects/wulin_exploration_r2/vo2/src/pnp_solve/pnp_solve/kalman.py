"""Small constant-velocity Kalman filter used by the PnP node."""

import numpy as np


class PositionVelocityKalman:
    """Estimate one-dimensional position and velocity."""

    def __init__(self, process_noise=0.5, measurement_noise=0.01):
        if process_noise <= 0.0 or measurement_noise <= 0.0:
            raise ValueError('noise values must be positive')
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.state = np.zeros((2, 1), dtype=np.float64)
        self.covariance = np.eye(2, dtype=np.float64)
        self.initialized = False

    def reset(self):
        """Clear the current estimate."""
        self.state.fill(0.0)
        self.covariance = np.eye(2, dtype=np.float64)
        self.initialized = False

    @staticmethod
    def _validate_dt(dt):
        value = float(dt)
        if value <= 0.0:
            raise ValueError('dt must be positive')
        return value

    def predict(self, dt):
        """Advance the state by dt seconds and return predicted position."""
        if not self.initialized:
            raise RuntimeError('filter has no measurement')
        dt = self._validate_dt(dt)
        transition = np.asarray([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        acceleration = np.asarray([[0.5 * dt * dt], [dt]], dtype=np.float64)
        process_covariance = (
            acceleration @ acceleration.T * self.process_noise
        )
        self.state = transition @ self.state
        self.covariance = (
            transition @ self.covariance @ transition.T + process_covariance
        )
        return float(self.state[0, 0])

    def update(self, measurement, dt):
        """Predict, apply a position measurement and return filtered position."""
        value = float(measurement)
        if not self.initialized:
            self.state[0, 0] = value
            self.state[1, 0] = 0.0
            self.initialized = True
            return value

        self.predict(dt)
        observation = np.asarray([[1.0, 0.0]], dtype=np.float64)
        innovation = value - float((observation @ self.state)[0, 0])
        innovation_covariance = float(
            (observation @ self.covariance @ observation.T)[0, 0]
        ) + self.measurement_noise
        gain = self.covariance @ observation.T / innovation_covariance
        self.state = self.state + gain * innovation
        identity = np.eye(2, dtype=np.float64)
        self.covariance = (identity - gain @ observation) @ self.covariance
        return float(self.state[0, 0])
