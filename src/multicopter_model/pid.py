"""
PID controller primitive.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import numpy as np


class PID:
    def __init__(self):
        self._k_p = 1
        self._k_i = 0
        self._k_d = 0.1
        self._integral = 0
        self._last_error = 0
        self._integral_limit = 300
        self._min_value = -1
        self._max_value = 1

    def set_pid_gains(self, k_p: float, k_i: float, k_d: float) -> None:
        self._k_p = k_p
        self._k_i = k_i
        self._k_d = k_d

    @property
    def integral_limit(self):
        return self._integral_limit

    @integral_limit.setter
    def integral_limit(self, limit: float):
        self._integral_limit = limit

    def set_saturation_limit(self, min: float, max: float) -> None:
        self._min_value = min
        self._max_value = max

    def update(self, input_val: float, target_val: float, dt: float) -> float:
        # Error calculation
        error = target_val - input_val
        # Integral update
        if dt <= 0:
            dt = 1e-6
        self._integral += error * dt
        # Saturate integral
        self._integral = np.clip(
            self._integral, -self._integral_limit, self._integral_limit
        )
        # PID components
        p_term = self._k_p * error
        i_term = self._k_i * self._integral
        d_term = self._k_d * (error - self._last_error) / dt
        self._last_error = error
        output = p_term + i_term + d_term
        # Output saturation
        output = np.clip(output, self._min_value, self._max_value)
        return output
