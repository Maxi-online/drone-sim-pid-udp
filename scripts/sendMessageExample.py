#!/usr/bin/env python3
"""
Advanced UDP telemetry generator (debug tool).

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

License: CC BY-NC 4.0 (see repository LICENSE file).

This script is useful for validating the UDP receiver stack (visualizer/plotter)
without running the full dynamics simulator.

Important: the current `scripts/visualizer.py` expects **18 doubles** and unpacks:

- indices 0..2: position (x, y, z)
- indices 3..5: roll, pitch, yaw (radians)
- indices 6..8: linear velocity (vx, vy, vz)
- indices 9..11: angular rates (p, q, r)
- index 12: timestamp
- indices 13..16: rotor omega (w1..w4) (rad/s) (used by propeller animation)
- index 17: timestamp duplicate (used by plotter reset logic via stateVector[-1])
"""

from __future__ import annotations

import argparse
import socket
import struct
import time

import numpy as np

PACK_FMT_18 = "dddddddddddddddddd"  # 18 doubles


def _pack_telemetry_18(values: np.ndarray) -> bytes:
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size != 18:
        raise ValueError(f"Expected 18 values, got {values.size}")
    return struct.pack(PACK_FMT_18, *[float(x) for x in values])


def _trajectory(
    t: float, mode: str, radius: float, speed: float, z0: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (pos, vel) in world frame.
    """
    omega = float(speed)
    if mode == "hover":
        pos = np.array([0.0, 0.0, z0], dtype=float)
        vel = np.array([0.0, 0.0, 0.0], dtype=float)
        return pos, vel

    if mode == "circle":
        x = radius * np.cos(omega * t)
        y = radius * np.sin(omega * t)
        vx = -radius * omega * np.sin(omega * t)
        vy = radius * omega * np.cos(omega * t)
        pos = np.array([x, y, z0], dtype=float)
        vel = np.array([vx, vy, 0.0], dtype=float)
        return pos, vel

    if mode == "figure8":
        # Lemniscate-like (simple parametric)
        x = radius * np.sin(omega * t)
        y = radius * np.sin(omega * t) * np.cos(omega * t)
        vx = radius * omega * np.cos(omega * t)
        vy = radius * omega * (np.cos(2 * omega * t) - np.sin(2 * omega * t)) * 0.5
        pos = np.array([x, y, z0], dtype=float)
        vel = np.array([vx, vy, 0.0], dtype=float)
        return pos, vel

    raise ValueError(f"Unknown trajectory mode: {mode}")


def _attitude(t: float, mode: str, yaw_rate: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (rpy, rpy_rates) in radians and rad/s.
    """
    if mode == "stable":
        roll = 0.0
        pitch = 0.0
        yaw = yaw_rate * t
        return np.array([roll, pitch, yaw], dtype=float), np.array(
            [0.0, 0.0, yaw_rate], dtype=float
        )

    if mode == "wobble":
        roll = 0.15 * np.sin(1.2 * t)
        pitch = 0.12 * np.sin(1.0 * t + 0.5)
        yaw = yaw_rate * t + 0.2 * np.sin(0.6 * t)
        roll_rate = 0.15 * 1.2 * np.cos(1.2 * t)
        pitch_rate = 0.12 * 1.0 * np.cos(1.0 * t + 0.5)
        yaw_rate_out = yaw_rate + 0.2 * 0.6 * np.cos(0.6 * t)
        return (
            np.array([roll, pitch, yaw], dtype=float),
            np.array([roll_rate, pitch_rate, yaw_rate_out], dtype=float),
        )

    raise ValueError(f"Unknown attitude mode: {mode}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Advanced UDP telemetry generator")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=12346)
    ap.add_argument("--hz", type=float, default=100.0, help="Send rate (Hz)")
    ap.add_argument(
        "--duration", type=float, default=20.0, help="Seconds to run (0 = infinite)"
    )
    ap.add_argument("--traj", choices=["hover", "circle", "figure8"], default="circle")
    ap.add_argument("--att", choices=["stable", "wobble"], default="stable")
    ap.add_argument("--radius", type=float, default=5.0)
    ap.add_argument(
        "--speed", type=float, default=0.6, help="Trajectory angular speed (rad/s)"
    )
    ap.add_argument("--z", type=float, default=2.0, help="Base altitude (m)")
    ap.add_argument("--yaw-rate", type=float, default=0.2, help="Base yaw rate (rad/s)")
    ap.add_argument(
        "--omega", type=float, default=800.0, help="Base rotor omega (rad/s)"
    )
    ap.add_argument(
        "--omega-delta",
        type=float,
        default=150.0,
        help="Rotor omega modulation (rad/s)",
    )
    ap.add_argument(
        "--noise",
        type=float,
        default=0.0,
        help="Gaussian noise sigma (applied to pos/vel/rates)",
    )
    args = ap.parse_args()

    addr = (args.host, int(args.port))
    dt = 1.0 / float(max(args.hz, 1e-6))
    rng = np.random.default_rng(12345)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    t0 = time.time()
    last = t0

    try:
        while True:
            now = time.time()
            t = now - t0
            if args.duration > 0 and t >= args.duration:
                break

            # rate control
            sleep_for = dt - (now - last)
            if sleep_for > 0:
                time.sleep(sleep_for)
                now = time.time()
                t = now - t0
            last = now

            pos, vel = _trajectory(t, args.traj, args.radius, args.speed, args.z)
            rpy, rpy_rates = _attitude(t, args.att, args.yaw_rate)

            # Rotor omega pattern (gives visible propeller motion)
            w_base = float(args.omega)
            w_amp = float(args.omega_delta)
            w1 = w_base + w_amp * np.sin(2.0 * t)
            w2 = w_base + w_amp * np.sin(2.0 * t + np.pi / 2.0)
            w3 = w_base + w_amp * np.sin(2.0 * t + np.pi)
            w4 = w_base + w_amp * np.sin(2.0 * t + 3.0 * np.pi / 2.0)

            if args.noise > 0:
                pos = pos + rng.normal(0.0, args.noise, size=3)
                vel = vel + rng.normal(0.0, args.noise, size=3)
                rpy_rates = rpy_rates + rng.normal(0.0, args.noise, size=3)

            # Build 18-double payload (see module docstring).
            payload = np.zeros(18, dtype=float)
            payload[0:3] = pos
            payload[3:6] = rpy
            payload[6:9] = vel
            payload[9:12] = rpy_rates
            payload[12] = t
            payload[13] = w1
            payload[14] = w2
            payload[15] = w3
            payload[16] = w4
            payload[17] = t  # duplicate timestamp for plotter reset logic

            sock.sendto(_pack_telemetry_18(payload), addr)

    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
