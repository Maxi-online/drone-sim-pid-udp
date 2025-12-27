#!/usr/bin/env python3
"""
Drone telemetry analyzer (optional DroneKit demo).

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This script is NOT required for the core UDP simulator in `src/` and `scripts/`.
It demonstrates how to collect basic telemetry from a DroneKit vehicle and export it.

Dependencies (optional):
- dronekit
- pymavlink

Example install:
pip install dronekit pymavlink dronekit-sitl
"""

from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Optional

try:
    from dronekit import connect
except Exception as e:  # pragma: no cover
    connect = None  # type: ignore
    _DRONEKIT_IMPORT_ERROR = e


@dataclass
class TelemetryPoint:
    timestamp_s: float
    lat: float
    lon: float
    alt_m: float
    groundspeed_mps: float
    airspeed_mps: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    mode: str
    battery_v: float
    gps_sats: int


class DroneTelemetryAnalyzer:
    def __init__(self, connection_string: str):
        if connect is None:
            raise RuntimeError(
                "DroneKit is required for this demo. Install it first: pip install dronekit pymavlink"
            ) from _DRONEKIT_IMPORT_ERROR

        self.vehicle = connect(connection_string, wait_ready=True)
        self._t0 = time.time()
        self.data: list[TelemetryPoint] = []

    def collect(self, duration_s: float = 30.0, sample_period_s: float = 0.5) -> None:
        """Collect telemetry for a fixed duration."""
        end_t = time.time() + float(duration_s)
        while time.time() < end_t:
            t = time.time() - self._t0

            loc = self.vehicle.location.global_relative_frame
            att = self.vehicle.attitude
            bat = self.vehicle.battery
            gps = self.vehicle.gps_0

            pt = TelemetryPoint(
                timestamp_s=float(t),
                lat=float(getattr(loc, "lat", 0.0) or 0.0),
                lon=float(getattr(loc, "lon", 0.0) or 0.0),
                alt_m=float(getattr(loc, "alt", 0.0) or 0.0),
                groundspeed_mps=float(getattr(self.vehicle, "groundspeed", 0.0) or 0.0),
                airspeed_mps=float(getattr(self.vehicle, "airspeed", 0.0) or 0.0),
                roll_deg=float(math.degrees(getattr(att, "roll", 0.0) or 0.0)),
                pitch_deg=float(math.degrees(getattr(att, "pitch", 0.0) or 0.0)),
                yaw_deg=float(math.degrees(getattr(att, "yaw", 0.0) or 0.0)),
                mode=str(getattr(self.vehicle.mode, "name", "UNKNOWN")),
                battery_v=float(getattr(bat, "voltage", 0.0) or 0.0),
                gps_sats=int(getattr(gps, "satellites_visible", 0) or 0),
            )
            self.data.append(pt)
            time.sleep(float(sample_period_s))

    def export_json(self, filename: Optional[str] = None) -> str:
        if filename is None:
            filename = (
                f"drone_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            )
        with open(filename, "w", encoding="utf-8") as f:
            json.dump([asdict(x) for x in self.data], f, indent=2)
        return filename

    def export_csv(self, filename: Optional[str] = None) -> str:
        if filename is None:
            filename = f"drone_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        if not self.data:
            with open(filename, "w", newline="", encoding="utf-8") as f:
                f.write("")
            return filename

        with open(filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(asdict(self.data[0]).keys()))
            writer.writeheader()
            for row in self.data:
                writer.writerow(asdict(row))
        return filename

    def close(self) -> None:
        self.vehicle.close()


if __name__ == "__main__":  # pragma: no cover
    analyzer = DroneTelemetryAnalyzer("tcp:127.0.0.1:5760")
    analyzer.collect(duration_s=10.0, sample_period_s=0.5)
    out_json = analyzer.export_json()
    out_csv = analyzer.export_csv()
    print(f"Saved: {out_json}, {out_csv}")
    analyzer.close()
