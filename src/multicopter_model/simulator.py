"""
Main simulator loop and UDP telemetry sender for the multicopter model.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import socket
import struct
import time as tm

from multicopter_model.controller import QuadCopterController
from multicopter_model.model import QuadCopterModel


class Simulator:
    def __init__(self, controller: QuadCopterController, model: QuadCopterModel, dt):
        self.controller = controller
        self.model = model
        self.dt = dt
        self.time = 0
        self._host = "127.0.0.1"
        self._port = 12346
        self.addr = (self._host, self._port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Target buffer for telemetry
        self._target = (0.0, 0.0, 0.0)

    def run(self):
        while True:
            # Get state vector from model
            state = self.model.state_vector
            # Compute control vector based on current state
            u = self.controller.update(state, self.dt)
            # Update model state
            self.model.update_state(u, self.dt)
            self._target = (
                self.controller.target_x,
                self.controller.target_y,
                self.controller.target_z,
            )
            self._send_pose_data(state)
            self.time += self.dt
            tm.sleep(self.dt)

    def _send_pose_data(self, state):
        # Pack and send state over UDP for visualization.
        # Order:
        # X,Y,Z, roll,pitch,yaw, Vx,Vy,Vz, rollRate,pitchRate,yawRate, timeStamp,
        # ω1,ω2,ω3,ω4, targetX,targetY,targetZ
        w = self.model.rotor_omega
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
                float(w[0]),
                float(w[1]),
                float(w[2]),
                float(w[3]),
                float(self._target[0]),
                float(self._target[1]),
                float(self._target[2]),
            )
        )
        self.udp_socket.sendto(data, self.addr)
