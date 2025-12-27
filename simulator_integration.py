#!/usr/bin/env python3
"""
High-level integration wrapper around the local PyBullet simulator.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This module provides a production-oriented, lightweight integration layer:
- No ROS2 / Gazebo dependency in this wrapper
- Thread-based simulation loop suitable for embedding into larger systems
- Stable API for external integrations and CI-friendly demos
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any, Callable, Optional

import pybullet as p

from professional_robot_simulator import ProfessionalRobotSimulator


class SimulatorIntegration:
    def __init__(self, gui_mode: bool = True, callback_func: Optional[Callable] = None):
        self.simulator = ProfessionalRobotSimulator(gui_mode=gui_mode, real_time=False)
        self.callback_func = callback_func

        self.command_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self.running = False
        self.sim_thread: Optional[threading.Thread] = None

        self.managed_drones: dict[str, dict[str, Any]] = {}

    def start_simulation(self, environment: str = "urban") -> None:
        """Create the environment and start the background simulation loop."""
        self.simulator.create_realistic_environment(environment)
        self.running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()

    def add_drone(
        self,
        drone_id: str,
        position: Optional[list[float]] = None,
        drone_type: str = "quadcopter",
    ) -> dict[str, Any]:
        drone = self.simulator.create_professional_drone(
            position=position, drone_type=drone_type
        )
        self.managed_drones[drone_id] = drone
        return drone

    def move_drone_to(
        self, drone_id: str, target_position: list[float], hover_time: float = 2.0
    ) -> None:
        self.command_queue.put(
            {
                "type": "move_to",
                "drone_id": drone_id,
                "target": target_position,
                "hover_time": hover_time,
            }
        )

    def execute_drone_mission(
        self, drone_id: str, waypoints: list[list[float]], hover_time: float = 2.0
    ) -> None:
        self.command_queue.put(
            {
                "type": "mission",
                "drone_id": drone_id,
                "waypoints": waypoints,
                "hover_time": hover_time,
            }
        )

    def get_drone_telemetry(self, drone_id: str) -> Optional[dict[str, Any]]:
        drone = self.managed_drones.get(drone_id)
        if not drone:
            return None
        return self.simulator.simulate_sensors(drone)

    def get_all_telemetry(self) -> dict[str, dict[str, Any]]:
        return {
            drone_id: self.simulator.simulate_sensors(drone)
            for drone_id, drone in self.managed_drones.items()
        }

    def set_callback(self, callback_func: Optional[Callable]) -> None:
        self.callback_func = callback_func

    def export_flight_data(self, filename: Optional[str] = None) -> str:
        return self.simulator.export_telemetry(filename=filename)

    def stop_simulation(self) -> None:
        self.running = False
        if self.sim_thread:
            self.sim_thread.join(timeout=3)
        self.simulator.close()

    def _simulation_loop(self) -> None:
        hz = 240.0
        dt = 1.0 / hz

        # Mission state per drone
        mission_state: dict[str, dict[str, Any]] = {}

        while self.running:
            # Consume commands without blocking the physics loop.
            try:
                while True:
                    cmd = self.command_queue.get_nowait()
                    drone_id = cmd["drone_id"]
                    if drone_id not in self.managed_drones:
                        continue

                    drone = self.managed_drones[drone_id]
                    if cmd["type"] == "move_to":
                        drone["current_target"] = list(cmd["target"])
                        mission_state.pop(drone_id, None)
                    elif cmd["type"] == "mission":
                        mission_state[drone_id] = {
                            "waypoints": [list(w) for w in cmd["waypoints"]],
                            "idx": 0,
                            "hover_time": float(cmd.get("hover_time", 2.0)),
                            "hover_until": None,
                        }
            except queue.Empty:
                pass

            # Step each drone towards its current target (or active mission waypoint).
            for drone_id, drone in self.managed_drones.items():
                if drone_id in mission_state:
                    st = mission_state[drone_id]
                    waypoints = st["waypoints"]
                    idx = int(st["idx"])
                    if idx < len(waypoints):
                        target = waypoints[idx]
                        drone["current_target"] = target
                        pos, _ = p.getBasePositionAndOrientation(drone["id"])
                        dist = (
                            (pos[0] - target[0]) ** 2
                            + (pos[1] - target[1]) ** 2
                            + (pos[2] - target[2]) ** 2
                        ) ** 0.5
                        if dist < 0.5:
                            if st["hover_until"] is None:
                                st["hover_until"] = time.time() + float(
                                    st["hover_time"]
                                )
                            elif time.time() >= float(st["hover_until"]):
                                st["idx"] = idx + 1
                                st["hover_until"] = None
                    else:
                        # Mission completed
                        mission_state.pop(drone_id, None)

                ctrl = self.simulator.autonomous_flight_controller(
                    drone, drone["current_target"]
                )
                self.simulator.apply_drone_physics(drone, ctrl)

            p.stepSimulation()

            if self.callback_func:
                try:
                    self.callback_func(self.get_all_telemetry())
                except Exception:
                    # Callback errors must not kill the simulation loop.
                    pass

            time.sleep(dt)
