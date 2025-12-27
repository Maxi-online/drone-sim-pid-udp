#!/usr/bin/env python3
"""
Quick start: simulator integration.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This is a minimal example showing how to start the local PyBullet simulator,
add drones, move them, collect telemetry, and export a dataset.
"""

import time

from simulator_integration import SimulatorIntegration


def main():
    print("Quick start: simulator integration")
    print("=" * 50)
    print()

    # Create simulator
    sim = SimulatorIntegration(gui_mode=True)

    # Start simulation with an environment preset
    sim.start_simulation("urban")

    # Add drones
    sim.add_drone("Drone1", position=[0, 0, 8])
    sim.add_drone("Drone2", position=[5, 5, 6])

    # Send move commands
    sim.move_drone_to("Drone1", [10, 0, 10])
    sim.move_drone_to("Drone2", [-5, -5, 8])

    print("Simulation running.")
    print()
    print("Demo: 20 seconds...")

    time.sleep(20)

    # Collect telemetry snapshot
    telemetry = sim.get_all_telemetry()
    print(f"Received telemetry from {len(telemetry)} drones")

    # Export dataset
    data_file = sim.export_flight_data()
    print(f"Telemetry exported: {data_file}")

    # Stop simulation
    sim.stop_simulation()

    print()
    print("Done.")


if __name__ == "__main__":
    main()
