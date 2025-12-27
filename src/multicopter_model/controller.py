"""
Cascaded PID controller for the quadcopter model.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import numpy as np

import multicopter_model.constants as cs
from multicopter_model.constants import States
from multicopter_model.pid import PID


class QuadCopterController:
    def __init__(self):
        self.target_x = 0
        self.target_y = 0
        self.target_z = 10
        self.target_yaw = 0

        self.position_controller_x = PID()
        self.position_controller_y = PID()
        self.position_controller_z = PID()

        self.velocity_controller_x = PID()
        self.velocity_controller_y = PID()
        self.velocity_controller_z = PID()

        self.roll_controller = PID()
        self.pitch_controller = PID()
        self.yaw_controller = PID()

        self.roll_rate_controller = PID()
        self.pitch_rate_controller = PID()
        self.yaw_rate_controller = PID()

        # Example mission (x, y, z, yaw)
        self._mission = [
            [0.0, 0.0, 2.0, 0.0],
            [1.0, 0.0, 2.0, 0.0],
            [1.0, 1.0, 2.5, 0.5],
        ]
        self._current_mission_index = 0

        # Hover thrust estimate in [0..1]
        omega_hover = np.sqrt(cs.quadcopter_mass * 9.81 / (4.0 * cs.trust_coef))
        self._hover_u = float(
            np.clip(
                (omega_hover - cs.min_rotors_rpm)
                / (cs.max_rotors_rpm - cs.min_rotors_rpm),
                0.0,
                1.0,
            )
        )

        # Last axis commands (for AirSim/logging)
        self._last_axes = {
            "thrust": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    def set_target_position(self, x, y, z, yaw):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw

    def update(self, state_vector, dt) -> np.ndarray:
        # 1) Update mission target
        if self._current_mission_index < len(self._mission):
            tx, ty, tz, tyaw = self._mission[self._current_mission_index]
            self.set_target_position(tx, ty, tz, tyaw)
            # radius check
            dx = float(self.target_x - state_vector[States.X])
            dy = float(self.target_y - state_vector[States.Y])
            dz = float(self.target_z - state_vector[States.Z])
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < 0.2:
                self._current_mission_index += 1

        # 2) Position -> desired velocity (world frame)
        vx_des = self.position_controller_x.update(
            float(state_vector[States.X]), float(self.target_x), dt
        )
        vy_des = self.position_controller_y.update(
            float(state_vector[States.Y]), float(self.target_y), dt
        )
        vz_des = self.position_controller_z.update(
            float(state_vector[States.Z]), float(self.target_z), dt
        )

        # 3) Velocity (world) -> velocity (body X/Y) using current yaw
        yaw = float(state_vector[States.YAW])
        R_yaw = self._rotation2d(yaw)
        vel_des_world = np.array([[vx_des], [vy_des]])
        vel_des_body_xy = R_yaw.T @ vel_des_world
        vx_des_body = float(vel_des_body_xy[0])
        vy_des_body = float(vel_des_body_xy[1])
        vel_meas_world = np.array(
            [[float(state_vector[States.VX])], [float(state_vector[States.VY])]]
        )
        vel_meas_body_xy = R_yaw.T @ vel_meas_world
        vx_body = float(vel_meas_body_xy[0])
        vy_body = float(vel_meas_body_xy[1])

        # 4) Velocity -> attitude angles (via body-velocity PID).
        # Small-angle approximation: ax≈g*theta, ay≈-g*phi
        g = 9.81
        ax_cmd = self.velocity_controller_x.update(vx_body, vx_des_body, dt)
        ay_cmd = self.velocity_controller_y.update(vy_body, vy_des_body, dt)
        target_roll = np.clip(-ax_cmd / g, -0.6, 0.6)
        target_pitch = np.clip(ay_cmd / g, -0.6, 0.6)

        # 5) Vertical thrust: gravity compensation (hover) + Vz correction
        vz_cmd = self.velocity_controller_z.update(
            float(state_vector[States.VZ]), float(vz_des), dt
        )
        thrust_cmd = float(np.clip(self._hover_u + vz_cmd, 0.0, 1.0))

        # 6) Attitude -> angular rates
        target_roll_rate = self.roll_controller.update(
            float(state_vector[States.ROLL]), float(target_roll), dt
        )
        target_pitch_rate = self.pitch_controller.update(
            float(state_vector[States.PITCH]), float(target_pitch), dt
        )
        target_yaw_rate = self.yaw_controller.update(
            float(state_vector[States.YAW]), float(self.target_yaw), dt
        )

        # 7) Rate loop -> torque commands
        cmd_roll = self.roll_rate_controller.update(
            float(state_vector[States.ROLL_RATE]), float(target_roll_rate), dt
        )
        cmd_pitch = self.pitch_rate_controller.update(
            float(state_vector[States.PITCH_RATE]), float(target_pitch_rate), dt
        )
        cmd_yaw = self.yaw_rate_controller.update(
            float(state_vector[States.YAW_RATE]), float(target_yaw_rate), dt
        )

        # Save axis commands
        self._last_axes = {
            "thrust": float(thrust_cmd),
            "roll": float(cmd_roll),
            "pitch": float(cmd_pitch),
            "yaw": float(cmd_yaw),
        }

        u = self._mixer(thrust_cmd, cmd_roll, cmd_pitch, cmd_yaw)
        return u

    def _mixer(self, cmd_trust, cmd_roll, cmd_pitch, cmd_yaw) -> np.ndarray:
        # '+' configuration: m1(front), m2(right), m3(back), m4(left)
        # Linearly normalize commands into [0, 1]
        m1 = cmd_trust + cmd_pitch - cmd_yaw
        m2 = cmd_trust - cmd_roll + cmd_yaw
        m3 = cmd_trust - cmd_pitch - cmd_yaw
        m4 = cmd_trust + cmd_roll + cmd_yaw
        u = np.array([m1, m2, m3, m4], dtype=float)
        # Clip into [0, 1]
        u = np.clip(u, 0.0, 1.0)
        return u

    @property
    def last_axes(self):
        return self._last_axes.copy()

    def _rotation2d(self, theta):
        return np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
