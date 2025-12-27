#!/usr/bin/env python3
"""
Advanced mission controller (optional DroneKit demo).

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This script is NOT required for the core UDP simulator in `src/` and `scripts/`.
It demonstrates how you could plan and execute higher-level missions using DroneKit.

Dependencies (optional):
- dronekit
- pymavlink

Example install:
pip install dronekit pymavlink dronekit-sitl
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

try:
    from dronekit import LocationGlobalRelative, connect
except Exception as e:  # pragma: no cover
    LocationGlobalRelative = None  # type: ignore
    connect = None  # type: ignore
    _DRONEKIT_IMPORT_ERROR = e


@dataclass
class Detection:
    timestamp: str
    lat: float
    lon: float
    confidence: float
    description: str


class AdvancedMissionController:
    def __init__(self, connection_string: str):
        if connect is None or LocationGlobalRelative is None:
            raise RuntimeError(
                "DroneKit is required for this demo. Install it first: pip install dronekit pymavlink"
            ) from _DRONEKIT_IMPORT_ERROR

        self.vehicle = connect(connection_string, wait_ready=True)
        self.mission_log: list[dict] = []

    def plan_survey_mission(
        self,
        area_corners: list[tuple[float, float]],
        altitude_m: float = 30.0,
        overlap: float = 0.7,
    ) -> list:
        """
        Plan a simple serpentine survey mission over a bounding box.

        area_corners: list of (lat, lon) corners (any order)
        altitude_m: survey altitude
        overlap: photo overlap (0.7 = 70%)
        """
        lats = [c[0] for c in area_corners]
        lons = [c[1] for c in area_corners]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        # Very rough camera footprint approximation.
        ground_coverage_m = altitude_m * 1.67
        step_m = ground_coverage_m * (1 - overlap)
        lat_step = step_m / 111_320.0

        waypoints = []
        direction = 1
        current_lat = min_lat
        while current_lat <= max_lat:
            if direction == 1:
                waypoints.append(
                    LocationGlobalRelative(current_lat, min_lon, altitude_m)
                )
                waypoints.append(
                    LocationGlobalRelative(current_lat, max_lon, altitude_m)
                )
            else:
                waypoints.append(
                    LocationGlobalRelative(current_lat, max_lon, altitude_m)
                )
                waypoints.append(
                    LocationGlobalRelative(current_lat, min_lon, altitude_m)
                )
            direction *= -1
            current_lat += lat_step
        return waypoints

    def goto_waypoint(self, waypoint, timeout_s: float = 60.0) -> None:
        """Send the vehicle to a waypoint and wait until timeout."""
        self.vehicle.simple_goto(waypoint)
        # Minimal wait loop. Real systems should use distance + EKF checks.
        import time

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            time.sleep(1)

    def save_log(self, filename: Optional[str] = None) -> str:
        if filename is None:
            filename = f"mission_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.mission_log, f, indent=2)
        return filename

    def close(self) -> None:
        self.vehicle.close()


if __name__ == "__main__":  # pragma: no cover
    connection = "tcp:127.0.0.1:5760"
    ctrl = AdvancedMissionController(connection)
    area = [(55.7540, 37.6209), (55.7542, 37.6212), (55.7538, 37.6211)]
    wps = ctrl.plan_survey_mission(area, altitude_m=30.0, overlap=0.7)
    print(f"Planned {len(wps)} waypoints.")
    ctrl.close()
