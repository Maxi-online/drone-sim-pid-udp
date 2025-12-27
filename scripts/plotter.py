"""
Real-time plotter (UDP receiver) for the quadcopter simulator telemetry.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

UAV state vector (plotter):

Position in world frame:
0 - X
1 - Y
2 - Z

Velocity in world frame:
3 - VelX
4 - VelY
5 - VelZ

Attitude:
6 - Pitch
7 - Roll
8 - Yaw

Angular rates:
9  - PitchRate
10 - RollRate
11 - YawRate

Simulation timestamp:
12 - timeStamp
"""

import socket
import struct
import sys
import threading

import numpy as np
from vispy import app, scene

# Select which component of the state vector to plot
########################################################
index = 0  #
########################################################


class plotter:

    def __init__(self):
        self.data = []
        self.time = []
        self.canvas = scene.SceneCanvas(keys="interactive", show=True)
        self.grid = self.canvas.central_widget.add_grid(spacing=0)
        self.viewbox = self.grid.add_view(row=0, col=1, camera="panzoom")

        # add axes
        x_axis = scene.AxisWidget(orientation="bottom")
        x_axis.stretch = (1, 0.1)
        self.grid.add_widget(x_axis, row=1, col=1)
        x_axis.link_view(self.viewbox)
        y_axis = scene.AxisWidget(orientation="left")
        y_axis.stretch = (0.1, 1)
        self.grid.add_widget(y_axis, row=0, col=0)
        y_axis.link_view(self.viewbox)
        self.viewbox.camera.set_range((0, 100), (-20, 20))

        self.host = "127.0.0.1"
        self.port = 12346
        # Extended telemetry: 20 doubles = 160 bytes
        self.msgLen = 160
        self.addr = (self.host, self.port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.udp_socket.bind(self.addr)
        self.currentDataIndex = index
        pos = np.array([[0, 0], [1, 1], [2, 2]])
        self.LastTime = 0
        self.line = scene.Line(pos, color=(0, 1, 1, 1), parent=self.viewbox.scene)

        self.grid = scene.visuals.GridLines(
            scale=(1, 1), color="w", parent=self.viewbox.scene
        )

        self.stateVector = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rcv_thread = threading.Thread(target=self._receiveData, name="rcv_thread")

        rcv_thread.start()

    def _receiveData(self):
        while True:
            # recvfrom receives UDP packets
            conn, addr = self.udp_socket.recvfrom(self.msgLen)
            self.stateVector = self._convertMessage(conn)
            if self.stateVector[-1] < self.LastTime:
                self.data = []
                self.time = []
                self.data.append(self.stateVector)
                self.time.append(self.stateVector[-1])

            self.data.append(self.stateVector)
            self.time.append(self.stateVector[-1])
            self.LastTime = self.stateVector[-1]

    def _convertMessage(self, data):
        revData = bytearray(reversed(data))
        decodeData = list(reversed(struct.unpack("!dddddddddddddddddd", revData)))
        return decodeData

    def startPlotter(self):
        timer = app.Timer()
        timer.connect(self._updatePlot)
        timer.start()
        app.run()

    def _updatePlot(self, ev):
        self.currData = [i[self.currentDataIndex] for i in self.data]

        dataPlot = np.array([self.time, self.currData]).T

        self.line.set_data(pos=dataPlot, color=(0, 1, 1, 1))


if __name__ == "__main__" and sys.flags.interactive == 0:
    plot = plotter()
    plot.startPlotter()
