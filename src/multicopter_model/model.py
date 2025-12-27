"""
Quadcopter dynamics model.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import numpy as np

import multicopter_model.constants as cs


class QuadCopterModel:
    def __init__(
        self,
        inertia,
        mass: float,
        trust_coef: float,
        drag_coef: float,
        arm_length: float,
    ):
        # Model constants
        self._inertia = inertia
        self._inertia_inv = np.linalg.inv(inertia)
        self._mass = mass
        self._trust_coef = trust_coef
        self._drag_coef = drag_coef
        self._arm_length = arm_length
        # Gravity vector (world frame)
        self._g = np.array([[0.0], [0.0], [-9.81]])
        self._motor_trust = np.array([[0.0], [0.0], [0.0]])
        self._motor_moments = np.array([[0.0], [0.0], [0.0]])
        self._rotor_omega = np.zeros(4, dtype=float)

        # State vector initial values (all zeros)
        self._state_vector = np.array(
            [
                [0.0],  # pose X
                [0.0],  # pose Y
                [0.0],  # pose Z
                [0.0],  # roll
                [0.0],  # pitch
                [0.0],  # yaw
                [0.0],  # velocity X
                [0.0],  # velocity Y
                [0.0],  # velocity Z
                [0.0],  # roll rate
                [0.0],  # pitch rate
                [0.0],
            ]
        )  # yaw rate

    @property
    def state_vector(self):
        return self._state_vector

    @property
    def rotor_omega(self):
        return self._rotor_omega.copy()

    def update_state(self, u, dt: float) -> None:
        # Compute linear and angular accelerations
        lin_acc, ang_acc = self._func_right(u, dt)
        # Integrate and update state vector
        self._integrate(lin_acc, ang_acc, dt)

    def _integrate(self, linear_acceleration, angular_acceleration, dt: float):
        # Integrate linear acceleration -> velocity
        self.state_vector[6:9] += linear_acceleration * dt
        # Integrate velocity -> position
        self.state_vector[0:3] += self.state_vector[6:9] * dt
        # Integrate angular acceleration -> body angular rate
        self.state_vector[9:12] += angular_acceleration * dt
        # Integrate body angular rate -> Euler angles
        self.state_vector[3:6] += self.state_vector[9:12] * dt
        # Angle normalization to [-pi, pi]
        for i in range(3, 6):
            angle = float(self.state_vector[i])
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            self.state_vector[i] = angle
        # Ground contact (simple clamp)
        if float(self.state_vector[2]) < 0.0:
            self.state_vector[2] = 0.0
            if float(self.state_vector[8]) < 0.0:
                self.state_vector[8] = 0.0

    def _func_right(self, u, dt):
        # u: motor commands in [0..1]; first-order motor dynamics
        u = np.squeeze(np.array(u)).astype(float)
        u = np.clip(u, 0.0, 1.0)
        omega_cmd = cs.min_rotors_rpm + u * (cs.max_rotors_rpm - cs.min_rotors_rpm)
        alpha = float(np.clip(dt / max(cs.motor_time_constant, 1e-3), 0.0, 1.0))
        self._rotor_omega = (1.0 - alpha) * self._rotor_omega + alpha * omega_cmd
        rotor_omega = self._rotor_omega
        # Rotor thrust: T_i = b * omega_i^2
        T1 = self._trust_coef * rotor_omega[0] ** 2
        T2 = self._trust_coef * rotor_omega[1] ** 2
        T3 = self._trust_coef * rotor_omega[2] ** 2
        T4 = self._trust_coef * rotor_omega[3] ** 2
        # Total thrust in body frame is along +Z
        T_total_body = np.array([[0.0], [0.0], [T1 + T2 + T3 + T4]])
        # Transform thrust to world frame via body->world rotation matrix
        roll = float(self._state_vector[3])
        pitch = float(self._state_vector[4])
        yaw = float(self._state_vector[5])
        R = self._rotation_matrix_3d(pitch, roll, yaw)
        thrust_world = R @ T_total_body
        # Linear acceleration: a = (thrust_world + drag) / m + g
        vel_world = self._state_vector[6:9].reshape(3, 1)
        # Simple wind model: relative velocity = v - v_wind
        v_rel = vel_world - cs.wind_velocity_world
        drag_world = -cs.linear_drag_coeff * v_rel
        linear_acceleration = (thrust_world + drag_world) / self._mass + self._g
        # Rotor moments in the body frame ('+' configuration)
        arm_length = self._arm_length
        # Assume rotor directions: 1(+), 2(-), 3(+), 4(-) -> alternating yaw moment
        M_roll = arm_length * (T2 - T4)
        M_pitch = arm_length * (T3 - T1)
        M_yaw = self._drag_coef * (
            rotor_omega[0] ** 2
            - rotor_omega[1] ** 2
            + rotor_omega[2] ** 2
            - rotor_omega[3] ** 2
        )
        motor_moment_body = np.array([[M_roll], [M_pitch], [M_yaw]])
        # Angular acceleration: I^{-1} (M - ω × Iω)
        omega_body = self._state_vector[9:12]
        omega_body = omega_body.reshape((3, 1))
        gyro = omega_body
        # Additional rotational damping
        angular_damping = -cs.angular_damping_coeff.reshape(3, 1) * gyro
        ang_acc = self._inertia_inv @ (
            motor_moment_body
            + angular_damping
            - np.cross(gyro.flatten(), (self._inertia @ gyro).flatten()).reshape(3, 1)
        )
        angular_acceleration = ang_acc

        return linear_acceleration, angular_acceleration

    def _rotation_matrix_3d(self, pitch, roll, yaw):
        # ZYX rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cr = np.cos(roll)
        sr = np.sin(roll)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )
