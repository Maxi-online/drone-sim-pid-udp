#!/usr/bin/env python3
"""
Professional robot simulator (PyBullet) – lightweight GitHub-ready demo.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This module intentionally keeps the API small and stable for integration:
- create_realistic_environment()
- create_professional_drone()
- autonomous_flight_controller()
- apply_drone_physics()
- simulate_sensors()
- export_telemetry()
- close()
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Optional

import numpy as np

try:
    import pybullet as p
    import pybullet_data
except Exception as e:  # pragma: no cover
    p = None
    pybullet_data = None
    _PYBULLET_IMPORT_ERROR = e


@dataclass
class DroneControl:
    """Simple control outputs for the physics step."""

    force_world: np.ndarray  # shape (3,)


class ProfessionalRobotSimulator:
    def __init__(self, gui_mode: bool = True, real_time: bool = True):
        if p is None:
            raise RuntimeError(
                "PyBullet is required for ProfessionalRobotSimulator. "
                "Install it with: pip install pybullet"
            ) from _PYBULLET_IMPORT_ERROR

        self._real_time = bool(real_time)
        self._gui_mode = bool(gui_mode)

        self.physics_client = p.connect(p.GUI if gui_mode else p.DIRECT)
        if self.physics_client < 0:
            raise RuntimeError("Failed to connect to PyBullet")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(1 if self._real_time else 0)

        self.drones: list[dict[str, Any]] = []
        self.obstacles: list[int] = []
        self.telemetry_data: list[dict[str, Any]] = []

        self._start_time = time.time()

    def create_realistic_environment(self, environment_type: str = "urban") -> None:
        """Create a simple demo environment."""
        p.loadURDF("plane.urdf")

        # Keep environments intentionally simple and deterministic.
        if environment_type == "urban":
            self._spawn_boxes_grid(size=4, spacing=6.0, height_range=(2.0, 8.0))
        elif environment_type == "forest":
            self._spawn_cylinders(count=25, area=30.0)
        elif environment_type == "industrial":
            self._spawn_boxes_grid(size=6, spacing=5.0, height_range=(1.0, 4.0))
        else:
            self._spawn_boxes_grid(size=3, spacing=8.0, height_range=(1.0, 3.0))

    def _spawn_boxes_grid(
        self, *, size: int, spacing: float, height_range: tuple[float, float]
    ) -> None:
        low_h, high_h = height_range
        half = (size - 1) / 2.0
        for ix in range(size):
            for iy in range(size):
                if ix == int(half) and iy == int(half):
                    continue
                x = (ix - half) * spacing
                y = (iy - half) * spacing
                h = float(
                    low_h + (high_h - low_h) * ((ix + iy) % size) / max(size - 1, 1)
                )
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.0, 1.0, h / 2])
                vis = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[1.0, 1.0, h / 2],
                    rgbaColor=[0.6, 0.6, 0.7, 1],
                )
                body_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=col,
                    baseVisualShapeIndex=vis,
                    basePosition=[x, y, h / 2],
                )
                self.obstacles.append(body_id)

    def _spawn_cylinders(self, *, count: int, area: float) -> None:
        rng = np.random.default_rng(123)
        for _ in range(count):
            x = float(rng.uniform(-area, area))
            y = float(rng.uniform(-area, area))
            h = float(rng.uniform(2.0, 8.0))
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.35, height=h)
            vis = p.createVisualShape(
                p.GEOM_CYLINDER, radius=0.35, length=h, rgbaColor=[0.2, 0.6, 0.2, 1]
            )
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=[x, y, h / 2],
            )
            self.obstacles.append(body_id)

    def create_professional_drone(
        self, position: Optional[list[float]] = None, drone_type: str = "quadcopter"
    ) -> dict[str, Any]:
        """Create a simple rigid-body drone."""
        if position is None:
            position = [0.0, 0.0, 2.0]

        # Body geometry
        half_extents = [0.25, 0.25, 0.08]
        mass = 1.5
        if drone_type == "scout_quadcopter":
            mass = 1.2
            half_extents = [0.22, 0.22, 0.07]
        elif drone_type == "heavy_lift_hexacopter":
            mass = 3.0
            half_extents = [0.35, 0.35, 0.12]
        elif drone_type == "surveillance_octocopter":
            mass = 2.5
            half_extents = [0.30, 0.30, 0.10]

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.2, 0.4, 0.8, 1]
        )
        drone_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=position,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        )

        p.changeDynamics(drone_id, -1, linearDamping=0.05, angularDamping=0.05)

        drone: dict[str, Any] = {
            "id": drone_id,
            "type": drone_type,
            "mass": float(mass),
            "current_target": list(position),
            "telemetry": [],
        }
        self.drones.append(drone)
        return drone

    def autonomous_flight_controller(
        self, drone: dict[str, Any], target_position: list[float]
    ) -> DroneControl:
        """
        A minimal position-hold controller that outputs a force vector.

        This is intentionally simple and stable for demonstration purposes.
        """
        pos, orn = p.getBasePositionAndOrientation(drone["id"])
        vel_lin, vel_ang = p.getBaseVelocity(drone["id"])

        pos = np.array(pos, dtype=float)
        vel_lin = np.array(vel_lin, dtype=float)
        target = np.array(target_position, dtype=float)

        # PD on position with gravity compensation.
        kp = np.array([2.0, 2.0, 6.0], dtype=float)
        kd = np.array([1.2, 1.2, 2.5], dtype=float)
        acc_cmd = kp * (target - pos) - kd * vel_lin
        acc_cmd[2] += 9.81

        force_world = drone["mass"] * acc_cmd
        force_world = np.clip(force_world, [-20, -20, 0], [20, 20, 60]).astype(float)
        return DroneControl(force_world=force_world)

    def apply_drone_physics(self, drone: dict[str, Any], control: DroneControl) -> None:
        """Apply external forces and record telemetry."""
        drone_id = drone["id"]
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        vel_lin, vel_ang = p.getBaseVelocity(drone_id)

        p.applyExternalForce(
            objectUniqueId=drone_id,
            linkIndex=-1,
            forceObj=control.force_world.tolist(),
            posObj=pos,
            flags=p.WORLD_FRAME,
        )

        t = time.time() - self._start_time
        entry = {
            "timestamp": float(t),
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "orientation_quat": [
                float(orn[0]),
                float(orn[1]),
                float(orn[2]),
                float(orn[3]),
            ],
            "linear_velocity": [
                float(vel_lin[0]),
                float(vel_lin[1]),
                float(vel_lin[2]),
            ],
            "angular_velocity": [
                float(vel_ang[0]),
                float(vel_ang[1]),
                float(vel_ang[2]),
            ],
            "target": [
                float(drone["current_target"][0]),
                float(drone["current_target"][1]),
                float(drone["current_target"][2]),
            ],
        }
        drone["telemetry"].append(entry)
        self.telemetry_data.append({"drone_id": int(drone_id), **entry})

    def simulate_sensors(self, drone: dict[str, Any]) -> dict[str, Any]:
        """Return a small, stable sensor snapshot for integration."""
        pos, orn = p.getBasePositionAndOrientation(drone["id"])
        vel_lin, vel_ang = p.getBaseVelocity(drone["id"])
        return {
            "gps": {"position": [float(pos[0]), float(pos[1]), float(pos[2])]},
            "imu": {
                "orientation_quat": [float(x) for x in orn],
                "angular_velocity": [float(x) for x in vel_ang],
            },
            "velocity": {"linear": [float(x) for x in vel_lin]},
            "type": drone.get("type", "unknown"),
        }

    def export_telemetry(self, filename: Optional[str] = None) -> str:
        """Export recorded telemetry to a JSON file."""
        if filename is None:
            filename = (
                f"robot_sim_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.telemetry_data, f, indent=2)
        return filename

    def close(self) -> None:
        """Close the simulator and disconnect PyBullet."""
        p.disconnect()


def _demo() -> None:  # pragma: no cover
    sim = ProfessionalRobotSimulator(gui_mode=True, real_time=True)
    sim.create_realistic_environment("urban")
    drone = sim.create_professional_drone(position=[0, 0, 6], drone_type="quadcopter")
    drone["current_target"] = [5, 5, 8]

    for _ in range(600):
        ctrl = sim.autonomous_flight_controller(drone, drone["current_target"])
        sim.apply_drone_physics(drone, ctrl)
        p.stepSimulation()
        time.sleep(1 / 240)

    out = sim.export_telemetry()
    print(f"Telemetry exported: {out}")
    sim.close()


if __name__ == "__main__":  # pragma: no cover
    _demo()
