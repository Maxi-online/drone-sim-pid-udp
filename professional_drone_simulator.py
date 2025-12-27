#!/usr/bin/env python3
"""
Professional drone simulator demo.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.

This script uses DroneKit-SITL + ArduCopter SITL to run a MAVLink-compatible
software-in-the-loop simulation and execute a simple waypoint mission.
"""

import math
import time

try:
    from dronekit import APIException, LocationGlobalRelative, VehicleMode, connect
    from dronekit_sitl import SITL
except Exception as e:  # pragma: no cover
    APIException = Exception  # type: ignore
    LocationGlobalRelative = None  # type: ignore
    VehicleMode = None  # type: ignore
    connect = None  # type: ignore
    SITL = None  # type: ignore
    _DRONEKIT_IMPORT_ERROR = e


class ProfessionalDroneSimulator:
    def __init__(self):
        if (
            SITL is None
            or connect is None
            or VehicleMode is None
            or LocationGlobalRelative is None
        ):
            raise RuntimeError(
                "This demo requires DroneKit and dronekit-sitl. "
                "Install them first (example): pip install dronekit dronekit-sitl pymavlink"
            ) from _DRONEKIT_IMPORT_ERROR

        print("Professional drone simulator")
        print("=" * 50)
        print("DroneKit-SITL + ArduCopter SITL")
        print("MAVLink-compatible simulation")
        print("=" * 50)

        self.sitl = None
        self.vehicle = None

    def start_simulator(self):
        """Start the SITL simulator."""
        print("Starting ArduCopter SITL...")

        # Create SITL instance
        self.sitl = SITL()
        self.sitl.download("copter", "4.0.7", verbose=True)

        # Simulator config
        sitl_args = [
            "--model",
            "quad",  # Quadcopter
            "--home",
            "55.753930,37.620795,584,353",  # Moscow, Red Square
        ]

        # Launch SITL
        self.sitl.launch(sitl_args, await_ready=True, restart=True)
        tcp, ip, port = self.sitl.connection_string().split(":")
        port = int(port)

        print(f"SITL started on {ip}:{port}")
        return f"tcp:{ip}:{port}"

    def connect_vehicle(self, connection_string):
        """Connect to the simulated vehicle."""
        print("Connecting to simulator...")

        try:
            # Connect with timeout
            self.vehicle = connect(connection_string, wait_ready=True, timeout=60)
            print("Connected.")

            # Vehicle info
            print(f"Vehicle type: {self.vehicle._vehicle_type}")
            print(f"Autopilot version: {self.vehicle.version}")
            print(f"GPS status: {self.vehicle.gps_0}")
            print(f"Mode: {self.vehicle.mode.name}")
            print(f"Battery: {self.vehicle.battery}")

            return True

        except APIException as e:
            print(f"Connection error: {e}")
            return False

    def arm_and_takeoff(self, target_altitude=10):
        """Arm and take off."""
        print(f"Preparing for takeoff to {target_altitude}m...")

        # Safety checks
        print("Waiting until vehicle is armable...")
        while not self.vehicle.is_armable:
            print("Waiting for system readiness...")
            time.sleep(1)

        print("System is ready.")

        # Arm
        print("Arming motors...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Motors armed.")

        # Takeoff
        print(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        # Wait until reaching target altitude
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {current_altitude:.2f}m / {target_altitude}m")

            if current_altitude >= target_altitude * 0.95:
                print("Target altitude reached.")
                break
            time.sleep(1)

    def execute_mission(self):
        """Execute a simple demo mission."""
        print("Starting demo mission")
        print("=" * 40)

        # Square route
        waypoints = [
            LocationGlobalRelative(55.754030, 37.621000, 10),  # Point 1
            LocationGlobalRelative(55.754130, 37.621000, 10),  # Point 2
            LocationGlobalRelative(55.754130, 37.620900, 10),  # Point 3
            LocationGlobalRelative(55.754030, 37.620900, 10),  # Point 4
        ]

        for i, waypoint in enumerate(waypoints, 1):
            print(f"Going to waypoint {i}: {waypoint.lat:.6f}, {waypoint.lon:.6f}")
            self.vehicle.simple_goto(waypoint)

            # Wait for arrival
            while True:
                current_location = self.vehicle.location.global_relative_frame
                distance = self.get_distance_metres(current_location, waypoint)

                print(f"Distance to target: {distance:.2f}m")

                if distance < 2.0:  # Arrived within 2m
                    print(f"Waypoint {i} reached.")
                    time.sleep(2)  # Hover at waypoint
                    break

                time.sleep(1)

    def return_and_land(self):
        """Return to launch and land."""
        print("Return to launch and land")
        print("=" * 30)

        print("Switching to RTL (Return To Launch)...")
        self.vehicle.mode = VehicleMode("RTL")

        # Wait for landing
        print("Waiting for landing...")
        while self.vehicle.armed:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {current_altitude:.2f}m")
            time.sleep(2)

        print("Landed.")

    def get_distance_metres(self, location1, location2):
        """Distance between two locations (approx)."""
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def run_full_demo(self):
        """Run the full demo."""
        try:
            # Start simulator
            connection_string = self.start_simulator()

            # Connect
            if self.connect_vehicle(connection_string):

                # Demo flight
                self.arm_and_takeoff(10)
                time.sleep(5)

                self.execute_mission()
                time.sleep(3)

                self.return_and_land()

            print("Demo completed successfully.")

        except Exception as e:
            print(f"Error: {e}")

        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        print("Cleaning up...")

        if self.vehicle:
            self.vehicle.close()

        if self.sitl:
            self.sitl.stop()

        print("Resources released.")


if __name__ == "__main__":
    simulator = ProfessionalDroneSimulator()
    simulator.run_full_demo()
