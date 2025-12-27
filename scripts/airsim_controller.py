#!/usr/bin/env python3

"""
AirSim -> local controller -> UDP telemetry bridge.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This script connects to a running AirSim Multirotor instance, reads the current
state estimate, feeds it into the local cascaded controller (`QuadCopterController`),
applies controller commands back into AirSim, and streams telemetry over UDP in the
same 18-double format expected by `scripts/visualizer.py`.

Notes / limitations:
- AirSim does not provide per-rotor omega in this simple bridge, so rotor omegas
  are sent as zeros (visualizer propeller animation may not spin).
- The mapping from controller thrust to AirSim's `moveByAngleRatesZAsync` is a
  rough approximation and is meant for demonstration.
"""

import math
import socket
import struct
import time

import numpy as np

try:
    import airsim
except ImportError:
    airsim = None

from multicopter_model.controller import QuadCopterController


class AirSimControllerBridge:
    def __init__(self, udp_host="127.0.0.1", udp_port=12346, dt=0.01):
        """
        Create a bridge instance.

        Args:
            udp_host: UDP destination host for visualization/plotter receivers.
            udp_port: UDP destination port (default matches visualizer: 12346).
            dt: control loop timestep (seconds).
        """
        if airsim is None:
            raise RuntimeError(
                "AirSim is not installed. Install it separately (recommended after NumPy): "
                "pip install airsim"
            )
        self.dt = dt
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (udp_host, udp_port)

        self.controller = QuadCopterController()

        self.time = 0.0

    def _get_state(self):
        """
        Read the current AirSim vehicle state and convert to the local controller
        state vector format.

        Returns:
            numpy.ndarray of shape (12, 1) with:
            [x, y, z, roll, pitch, yaw, vx, vy, vz, p, q, r]^T

        Frames:
        - Position/velocity are in AirSim's world frame.
        - Orientation is converted from quaternion to Euler (ZYX).
        - Angular velocity is taken from AirSim kinematics.
        """
        kin = self.client.getMultirotorState().kinematics_estimated
        pos = self.client.getMultirotorState().position
        # position (m)
        x = pos.x_val
        y = pos.y_val
        z = pos.z_val
        # orientation (quaternion to euler ZYX)
        q = kin.orientation
        w, xi, yj, zk = q.w_val, q.x_val, q.y_val, q.z_val
        # yaw-pitch-roll
        yaw = math.atan2(2 * (w * zk + xi * yj), 1 - 2 * (yj * yj + zk * zk))
        sinp = 2 * (w * yj - zk * xi)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        roll = math.atan2(2 * (w * xi + yj * zk), 1 - 2 * (xi * xi + yj * yj))
        # linear velocity (world)
        vel = self.client.getMultirotorState().kinematics_estimated.linear_velocity
        vx, vy, vz = vel.x_val, vel.y_val, vel.z_val
        # angular rates (body) approximate from kinematics
        ang = self.client.getMultirotorState().kinematics_estimated.angular_velocity
        p, q_, r = ang.x_val, ang.y_val, ang.z_val
        return np.array(
            [[x], [y], [z], [roll], [pitch], [yaw], [vx], [vy], [vz], [p], [q_], [r]],
            dtype=float,
        )

    def _send_udp(self, state):
        """
        Send telemetry over UDP in the 18-double format consumed by the visualizer.

        Payload order (18 doubles):
        0..2   : x, y, z
        3..5   : roll, pitch, yaw
        6..8   : vx, vy, vz
        9..11  : p, q, r
        12     : timestamp (seconds)
        13..16 : rotor omega (w1..w4) (not available here -> zeros)
        17     : timestamp duplicate (used by some receivers as last element)
        """
        # no rotor omegas from AirSim; fill zeros; no explicit target: zeros
        data = bytearray(
            struct.pack(
                "dddddddddddddddddd",
                float(state[0]),
                float(state[1]),
                float(state[2]),
                float(state[3]),
                float(state[4]),
                float(state[5]),
                float(state[6]),
                float(state[7]),
                float(state[8]),
                float(state[9]),
                float(state[10]),
                float(state[11]),
                float(self.time),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        )
        self.udp_socket.sendto(data, self.addr)

    def _apply_controls(self, axes):
        """
        Apply controller outputs to AirSim.

        Args:
            axes: dict with normalized control channels:
                - thrust: [0..1]
                - roll, pitch, yaw: controller outputs (treated as angle rates here)

        Implementation detail:
        Uses AirSim `moveByAngleRatesZAsync` for simplicity. AirSim uses NED by
        convention in many APIs (down is positive), so we apply a sign flip on Z.
        """
        if not self.client:
            return
        # Map our axes to AirSim API (normalized):
        # thrust in [0..1] -> z velocity or climb rate; roll/pitch/yaw -> angle rates/cmd
        # Use moveByAngleRatesZ API for simplicity
        self.client.moveByAngleRatesZAsync(
            roll_rate=axes["roll"],
            pitch_rate=axes["pitch"],
            yaw_rate=axes["yaw"],
            z=float(-axes["thrust"] * 2.0),  # rough mapping (down positive in AirSim)
            duration=self.dt,
        )

    def run(self):
        """
        Run an infinite control loop:
        - read AirSim state
        - update local controller
        - apply controls to AirSim
        - send UDP telemetry
        """
        while True:
            state = self._get_state()
            self.controller.update(state, self.dt)
            self._apply_controls(self.controller.last_axes)
            self._send_udp(state)
            self.time += self.dt
            time.sleep(self.dt)


if __name__ == "__main__":
    bridge = AirSimControllerBridge()
    bridge.run()
