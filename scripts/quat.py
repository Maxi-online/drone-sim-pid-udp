#!/usr/bin/env python3
"""
Quaternion utilities used by the VisPy visualizer.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

License: CC BY-NC 4.0 (see repository LICENSE file).

Design goals:
- Full-featured quaternion math (normalize, inverse, matrix, SLERP, rotate vectors)
- NumPy-friendly, well-tested behavior
- Backward compatibility with the existing visualizer via `TQuat.rotate_vector_angles(...)`
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Union

import numpy as np

ArrayLike = Union[np.ndarray, Iterable[float]]


def _as_vec3(v: ArrayLike) -> np.ndarray:
    arr = np.asarray(v, dtype=float).reshape(-1)
    if arr.size != 3:
        raise ValueError(f"Expected a 3D vector, got shape {np.asarray(v).shape}")
    return arr


def _safe_norm(v: np.ndarray) -> float:
    n = float(np.linalg.norm(v))
    return n


@dataclass(frozen=True)
class Quaternion:
    """Quaternion in (w, x, y, z) convention."""

    w: float
    x: float
    y: float
    z: float

    def as_np(self) -> np.ndarray:
        return np.array([self.w, self.x, self.y, self.z], dtype=float)

    def norm(self) -> float:
        return float(
            np.sqrt(
                self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
            )
        )

    def normalized(self) -> Quaternion:
        n = self.norm()
        if n <= 0.0:
            raise ValueError("Cannot normalize a zero-length quaternion")
        inv = 1.0 / n
        return Quaternion(self.w * inv, self.x * inv, self.y * inv, self.z * inv)

    def conjugate(self) -> Quaternion:
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def inverse(self) -> Quaternion:
        n2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
        if n2 <= 0.0:
            raise ValueError("Cannot invert a zero-length quaternion")
        c = self.conjugate()
        inv = 1.0 / float(n2)
        return Quaternion(c.w * inv, c.x * inv, c.y * inv, c.z * inv)

    def __mul__(self, other: Quaternion) -> Quaternion:
        """Hamilton product."""
        a1, b1, c1, d1 = self.w, self.x, self.y, self.z
        a2, b2, c2, d2 = other.w, other.x, other.y, other.z
        return Quaternion(
            a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
            a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
            a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2,
            a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2,
        )

    def to_rotation_matrix(self) -> np.ndarray:
        """Return a 3x3 rotation matrix."""
        q = self.normalized()
        w, x, y, z = q.w, q.x, q.y, q.z
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=float,
        )

    def rotate_vector(self, v: ArrayLike) -> np.ndarray:
        """Rotate a 3D vector by this quaternion."""
        vec = _as_vec3(v)
        R = self.to_rotation_matrix()
        return (R @ vec.reshape(3, 1)).reshape(3)

    @staticmethod
    def from_axis_angle(axis: ArrayLike, angle_rad: float) -> Quaternion:
        axis_v = _as_vec3(axis)
        n = _safe_norm(axis_v)
        if n <= 0.0:
            raise ValueError("Axis must be non-zero")
        axis_v = axis_v / n
        half = float(angle_rad) * 0.5
        s = float(np.sin(half))
        return Quaternion(
            float(np.cos(half)),
            float(axis_v[0] * s),
            float(axis_v[1] * s),
            float(axis_v[2] * s),
        )

    @staticmethod
    def from_euler_zyx(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Create quaternion from roll/pitch/yaw using ZYX order (yaw -> pitch -> roll).
        This matches the visualizer usage pattern.
        """
        q_yaw = Quaternion.from_axis_angle([0.0, 0.0, 1.0], yaw)
        q_pitch = Quaternion.from_axis_angle([0.0, 1.0, 0.0], pitch)
        q_roll = Quaternion.from_axis_angle([1.0, 0.0, 0.0], roll)
        return (q_yaw * q_pitch * q_roll).normalized()

    def to_euler_zyx(self) -> tuple[float, float, float]:
        """Return (roll, pitch, yaw) for ZYX convention."""
        q = self.normalized()
        w, x, y, z = q.w, q.x, q.y, q.z

        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = float(np.copysign(np.pi / 2.0, sinp))
        else:
            pitch = float(np.arcsin(sinp))

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        return roll, pitch, yaw

    @staticmethod
    def slerp(q0: Quaternion, q1: Quaternion, t: float) -> Quaternion:
        """Spherical linear interpolation (t in [0,1])."""
        t = float(np.clip(t, 0.0, 1.0))
        a = q0.normalized().as_np()
        b = q1.normalized().as_np()
        dot = float(np.dot(a, b))

        # If dot < 0, slerp the long way around unless we flip one quaternion.
        if dot < 0.0:
            b = -b
            dot = -dot

        # If very close, fall back to lerp.
        if dot > 0.9995:
            out = a + t * (b - a)
            out /= float(np.linalg.norm(out))
            return Quaternion(
                float(out[0]), float(out[1]), float(out[2]), float(out[3])
            )

        theta_0 = float(np.arccos(np.clip(dot, -1.0, 1.0)))
        sin_theta_0 = float(np.sin(theta_0))
        theta = theta_0 * t
        sin_theta = float(np.sin(theta))

        s0 = float(np.sin(theta_0 - theta) / sin_theta_0)
        s1 = float(sin_theta / sin_theta_0)
        out = s0 * a + s1 * b
        return Quaternion(
            float(out[0]), float(out[1]), float(out[2]), float(out[3])
        ).normalized()


class TQuat:
    """
    Backward-compatible wrapper used by `scripts/visualizer.py`.

    The original visualizer expects:
    - `TQuat.euler_to_quat(roll, pitch, yaw)`
    - `TQuat.rotate_vector_angles(roll, pitch, yaw, vector)`
    """

    def __init__(self, rotate_angle: float = 0.0, vector: ArrayLike = (0.0, 0.0, 1.0)):
        q = Quaternion.from_axis_angle(vector, rotate_angle)
        self.w, self.x, self.y, self.z = q.w, q.x, q.y, q.z

    @staticmethod
    def euler_to_quat(
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
    ) -> TQuat:
        q = Quaternion.from_euler_zyx(roll, pitch, yaw)
        out = TQuat(0.0, (0.0, 0.0, 1.0))
        out.w, out.x, out.y, out.z = q.w, q.x, q.y, q.z
        return out

    @staticmethod
    def quat_to_euler(quat: TQuat) -> tuple[float, float, float]:
        q = Quaternion(quat.w, quat.x, quat.y, quat.z)
        return q.to_euler_zyx()

    @staticmethod
    def rotate_vector_angles(
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        vector: ArrayLike = (0.0, 0.0, 0.0),
    ) -> np.ndarray:
        q = Quaternion.from_euler_zyx(roll, pitch, yaw)
        return q.rotate_vector(vector)

    # Advanced API (optional usage)
    @staticmethod
    def slerp(q0: TQuat, q1: TQuat, t: float) -> TQuat:
        qa = Quaternion(q0.w, q0.x, q0.y, q0.z)
        qb = Quaternion(q1.w, q1.x, q1.y, q1.z)
        q = Quaternion.slerp(qa, qb, t)
        out = TQuat(0.0, (0.0, 0.0, 1.0))
        out.w, out.x, out.y, out.z = q.w, q.x, q.y, q.z
        return out
