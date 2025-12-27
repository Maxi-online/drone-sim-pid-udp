"""
Real-time 3D visualizer (UDP receiver) for the quadcopter simulator telemetry.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import os
import socket
import struct
import threading

import numpy as np
from quat import TQuat
from vispy import app, geometry, scene
from vispy.io import read_png
from vispy.visuals.transforms import MatrixTransform, STTransform

radToDeg = 57.29577951308232
drawTrajLine = True

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class uavDynamicVisualizer:
    def __init__(self):
        self.canvas = scene.SceneCanvas(keys="interactive", bgcolor="grey")
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = "turntable"
        # HUD text
        self.hud = scene.Label(
            text="", color="white", font_size=12, anchor_x="left", anchor_y="top"
        )
        self.hud.pos = (10, 10)
        self.view.add(self.hud)
        # 3D axis
        self.axis = scene.visuals.XYZAxis(parent=self.view.scene)

        xax = scene.Axis(
            pos=[[0, 0], [5, 0]],
            tick_direction=(0, -1),
            axis_color="r",
            tick_color="r",
            text_color="r",
            font_size=16,
            parent=self.view.scene,
            domain=(0.0, 5.0),
        )
        # Bind axis objects to the visualizer instance (by analogy with zax).
        self.xax = xax
        self.x_axis = xax  # alias

        yax = scene.Axis(
            pos=[[0, 0], [0, 5]],
            tick_direction=(-1, 0),
            axis_color="g",
            tick_color="g",
            text_color="g",
            font_size=16,
            parent=self.view.scene,
            domain=(0.0, 5.0),
        )
        self.yax = yax
        self.y_axis = yax  # alias

        zax = scene.Axis(
            pos=[[0, 0], [-5, 0]],
            tick_direction=(0, -1),
            axis_color="b",
            tick_color="b",
            text_color="b",
            font_size=16,
            parent=self.view.scene,
            domain=(0.0, 5.0),
        )

        # # its acutally an inverted xaxis
        zax.transform = scene.transforms.MatrixTransform()
        zax.transform.rotate(90, (0, 1, 0))  # rotate cw around yaxis
        zax.transform.rotate(-45, (0, 0, 1))  # tick direction towards (-1,-1)
        self.zax = zax
        self.z_axis = zax  # alias

        img_data = read_png(SCRIPT_DIR + "/textures/grass.png")
        interpolation = "nearest"
        self.image = scene.visuals.Image(
            img_data, interpolation=interpolation, parent=self.view.scene, method="auto"
        )
        imageScale = 0.01
        self.image.transform = STTransform(
            translate=[
                -self.image.size[0] * imageScale / 2,
                -self.image.size[1] * imageScale / 2,
                -0.1,
            ],
            scale=(imageScale, imageScale),
        )
        self._initUavGeometry()

        self.server_address = ("127.0.0.1", 12346)
        # 20 double = 160 bytes
        self.msgLen = 160
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind(self.server_address)

        # 0..12 state + 13..16 ω1..ω4 + 17..19 target xyz
        self.stateVector = [0] * 20
        rcv_thread = threading.Thread(target=self._receiveData)
        rcv_thread.start()
        # Average rotor angular speed (rad/s)
        self.markers = scene.visuals.GridLines()
        self.dataX = [0]
        self.dataY = [0]
        self.dataZ = [0]
        self.dataPos = []

        pos = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]])
        self.line = scene.Line(pos, color=(0, 1, 1, 1), width=3)
        self.view.add(self.line)
        self.scatter = scene.visuals.Markers(
            edge_color=(1, 1, 1, 1), face_color=(0, 0, 1, 1), size=15
        )

    def _addArrow(self, lenth, radius):
        meshData = geometry.generation.create_arrow(
            rows=100, cols=100, radius=0.1, length=1, cone_radius=None, cone_length=None
        )
        uavTrustArrow = scene.visuals.Mesh(mode="triangles", meshdata=meshData)

        return uavTrustArrow

    def _addPropVisualization(self, pose, poseOffset):
        """
        :param pose: DESCRIPTION
        :type pose: TYPE
        :param poseOffset: DESCRIPTION
        :type poseOffset: TYPE
        :return: DESCRIPTION
        :rtype: TYPE

        """
        rows = 100
        cols = 100
        meshData = geometry.generation.create_cylinder(
            rows, cols, radius=[0.05, 0.5], length=0.1, offset=False
        )

        uavProp = scene.visuals.Mesh(mode="triangles", meshdata=meshData)
        self.propPoseBodyList.append(pose + poseOffset)
        self.propVisList.append(uavProp)
        self.view.add(uavProp)

    def _initUavGeometry(self):
        self.bodyBoxHeight = 1
        self.bodyBoxWidth = 1
        self.bodyBoxDepth = 1
        self.armLenth = 1
        self.armWidth = 0.1
        self.armCount = 4
        self.armBodyHeight = 0
        self.armPoseBodyList = []
        self.armVisList = []
        self.propPoseBodyList = []
        self.propVisList = []
        self.arrowVisList = []
        self.armAngleBody = 0
        self.armOrientationBodyList = []
        self.uavBody = scene.visuals.Box(
            width=self.bodyBoxWidth,
            height=self.bodyBoxHeight,
            depth=self.bodyBoxDepth,
            color=(0, 0, 1, 1),
            edge_color="green",
        )
        self.view.add(self.uavBody)
        # Add body-frame axes
        posAxes = np.array(
            [[0, 0, 0], [3, 0, 0], [0, 0, 0], [0, 3, 0], [0, 0, 0], [0, 0, 3]]
        )
        self.axisBody = scene.visuals.XYZAxis(pos=posAxes, width=5)
        self.view.add(self.axisBody)

        armAngleStep = 2 * np.pi / self.armCount
        currentAngle = self.armAngleBody
        # Add arm + propeller
        armPoseX = self.bodyBoxWidth / 2 + self.armLenth / 2
        armPoseY = 0
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(
            width=self.armLenth,
            height=self.armWidth,
            depth=self.armWidth,
            color=(0, 1, 1, 1),
            edge_color="green",
        )

        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([self.armLenth + self.bodyBoxWidth / 2 - 1, 0, 0.05])
        self._addPropVisualization(pose, poseOffset)
        # Add arm + propeller
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(
            width=self.armWidth,
            height=self.armWidth,
            depth=self.armLenth,
            color=(1, 0, 1, 1),
            edge_color="green",
        )
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([0, self.armLenth + self.bodyBoxWidth / 2 - 1, 0.05])
        self._addPropVisualization(pose, poseOffset)
        # Add arm + propeller
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(
            width=self.armLenth,
            height=self.armWidth,
            depth=self.armWidth,
            color=(0, 1, 1, 1),
            edge_color="green",
        )
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        currentAngle += armAngleStep
        poseOffset = np.array([-(self.armLenth + self.bodyBoxWidth / 2 - 1), 0, 0.05])
        self._addPropVisualization(pose, poseOffset)
        # Add arm + propeller
        armAngle = currentAngle
        armBodyX = armPoseX * np.cos(armAngle) - armPoseY * np.sin(armAngle)
        armBodyY = armPoseX * np.sin(armAngle) + armPoseY * np.cos(armAngle)
        armBodyZ = self.armBodyHeight
        pose = np.array([armBodyX, armBodyY, armBodyZ])
        self.armPoseBodyList.append(pose)
        self.armOrientationBodyList.append(np.array([0, 0, armAngle]))
        self.uavArm = scene.visuals.Box(
            width=self.armWidth,
            height=self.armWidth,
            depth=self.armLenth,
            color=(0, 1, 1, 1),
            edge_color="green",
        )
        self.armVisList.append(self.uavArm)
        self.view.add(self.uavArm)
        poseOffset = np.array([0, -(self.armLenth + self.bodyBoxWidth / 2 - 1), 0.05])
        self._addPropVisualization(pose, poseOffset)

    def startVisualization(self):
        """

        :return: DESCRIPTION
        :rtype: TYPE

        """
        # Add timer and start the app
        timer = app.Timer()
        timer.connect(self._updatePlot)
        # timer.start(step, totalTime)
        timer.start()
        self.canvas.show()
        app.run()

    def _convertMessage(self, data):
        revData = bytearray(reversed(data))
        decodeData = list(reversed(struct.unpack("!dddddddddddddddddd", revData)))
        return decodeData

    def _receiveData(self):
        while True:
            # recvfrom receives UDP packets
            conn, addr = self.udp_socket.recvfrom(self.msgLen)
            self.stateVector = self._convertMessage(conn)
            self.dataPos.append(
                [self.stateVector[0], self.stateVector[1], self.stateVector[2]]
            )
            if drawTrajLine:
                self.dataX.append(self.stateVector[0])
                self.dataY.append(self.stateVector[1])
                self.dataZ.append(self.stateVector[2])

    def _transformObjectPosition(self, bodyOrientation, poseBody, visualObj):
        tr = MatrixTransform()
        tr.rotate(radToDeg * (bodyOrientation[0]), (1, 0, 0))
        tr.rotate(radToDeg * (bodyOrientation[1]), (0, 1, 0))
        tr.rotate(radToDeg * (bodyOrientation[2]), (0, 0, 1))

        worldPose = TQuat.rotate_vector_angles(
            bodyOrientation[0], bodyOrientation[1], bodyOrientation[2], poseBody
        )
        tr.translate(
            (
                worldPose[0] + self.stateVector[0],
                worldPose[1] + self.stateVector[1],
                worldPose[2] + self.stateVector[2],
            )
        )
        visualObj.transform = tr

    def _updatePlot(self, ev):
        tr = MatrixTransform()
        tr.rotate(radToDeg * self.stateVector[3], (1, 0, 0))
        tr.rotate(radToDeg * self.stateVector[4], (0, 1, 0))
        tr.rotate(radToDeg * self.stateVector[5], (0, 0, 1))
        tr.translate((self.stateVector[0], self.stateVector[1], self.stateVector[2]))
        self.uavBody.transform = tr
        self.axisBody.transform = tr

        bodyOrientation = np.array(
            [self.stateVector[3], self.stateVector[4], self.stateVector[5]]
        )

        if drawTrajLine:
            dataPlot = np.array([self.dataX, self.dataY, self.dataZ]).T
            # print(dataPlot)
            self.line.set_data(pos=dataPlot, color=(0, 1, 1, 1))

        for i in range(len(self.armVisList)):

            self._transformObjectPosition(
                bodyOrientation, self.armPoseBodyList[i], self.armVisList[i]
            )

        # Propeller rotation from omega telemetry
        w1 = self.stateVector[13]
        w2 = self.stateVector[14]
        w3 = self.stateVector[15]
        w4 = self.stateVector[16]
        w_list = [w1, w2, w3, w4]
        for i in range(len(self.propVisList)):
            prop = self.propVisList[i]
            # Base transform to propeller location
            self._transformObjectPosition(
                bodyOrientation, self.propPoseBodyList[i], prop
            )
            # Apply local rotation around body Z
            local = prop.transform
            extra = MatrixTransform()
            extra.rotate(radToDeg * (w_list[i] * 0.02), (0, 0, 1))
            prop.transform = local * extra

        # Trajectory and target marker
        tx, ty, tz = self.stateVector[17], self.stateVector[18], self.stateVector[19]
        self.scatter.set_data(np.array([[tx, ty, tz]]))
        self.view.add(self.scatter)

        # Follow camera
        self.view.camera.center = (
            self.stateVector[0],
            self.stateVector[1],
            self.stateVector[2],
        )

        # HUD update
        speed = np.linalg.norm(
            [self.stateVector[6], self.stateVector[7], self.stateVector[8]]
        )
        txt = f"Z: {self.stateVector[2]:.2f} m  |  V: {speed:.2f} m/s  |  yaw: {self.stateVector[5]:.2f} rad"
        self.hud.text = txt


if __name__ == "__main__":
    visualizer = uavDynamicVisualizer()
    visualizer.startVisualization()
