"""
UDP demo entry point for the multicopter model simulator.

Copyright (c) 2025 Maxim Strekolovskiy
Licensed under CC BY-NC 4.0. See LICENSE.
"""

import multicopter_model.constants as cs
from multicopter_model.controller import QuadCopterController
from multicopter_model.model import QuadCopterModel
from multicopter_model.simulator import Simulator


def main():
    # Create controller instance
    controller = QuadCopterController()
    # Initialize controller gains (tune as needed).

    # POSITION CONTROLLER
    controller.position_controller_x.set_pid_gains(0.8, 0.0, 0.1)
    controller.position_controller_x.set_saturation_limit(-2.0, 2.0)
    controller.position_controller_x.integral_limit = 0.0

    controller.position_controller_y.set_pid_gains(0.8, 0.0, 0.1)
    controller.position_controller_y.set_saturation_limit(-2.0, 2.0)
    controller.position_controller_y.integral_limit = 0.0

    controller.position_controller_z.set_pid_gains(2.0, 0.0, 0.3)
    controller.position_controller_z.set_saturation_limit(-2.0, 2.0)
    controller.position_controller_z.integral_limit = 0.0

    # VELOCITY CONTROLLER
    controller.velocity_controller_x.set_pid_gains(1.2, 0.0, 0.1)
    controller.velocity_controller_x.set_saturation_limit(-0.7, 0.7)
    controller.velocity_controller_x.integral_limit = 0.0

    controller.velocity_controller_y.set_pid_gains(1.2, 0.0, 0.1)
    controller.velocity_controller_y.set_saturation_limit(-0.7, 0.7)
    controller.velocity_controller_y.integral_limit = 0.0

    controller.velocity_controller_z.set_pid_gains(2.0, 0.2, 0.4)
    controller.velocity_controller_z.set_saturation_limit(-1.0, 1.0)
    controller.velocity_controller_z.integral_limit = 0.5

    # ANGLE CONTROLLER
    controller.pitch_controller.set_pid_gains(4.0, 0.0, 0.2)
    controller.pitch_controller.set_saturation_limit(-0.7, 0.7)
    controller.pitch_controller.integral_limit = 0.0

    controller.roll_controller.set_pid_gains(4.0, 0.0, 0.2)
    controller.roll_controller.set_saturation_limit(-0.7, 0.7)
    controller.roll_controller.integral_limit = 0.0

    controller.yaw_controller.set_pid_gains(2.0, 0.0, 0.1)
    controller.yaw_controller.set_saturation_limit(-0.8, 0.8)
    controller.yaw_controller.integral_limit = 0.0

    # ANGULAR RATE CONTROLLER
    controller.pitch_rate_controller.set_pid_gains(6.0, 0.15, 0.05)
    controller.pitch_rate_controller.set_saturation_limit(-0.5, 0.5)
    controller.pitch_rate_controller.integral_limit = 0.0

    controller.roll_rate_controller.set_pid_gains(6.0, 0.15, 0.05)
    controller.roll_rate_controller.set_saturation_limit(-0.5, 0.5)
    controller.roll_rate_controller.integral_limit = 0.0

    controller.yaw_rate_controller.set_pid_gains(4.0, 0.1, 0.05)
    controller.yaw_rate_controller.set_saturation_limit(-0.5, 0.5)
    controller.yaw_rate_controller.integral_limit = 0.0

    # Create model and start simulation
    model = QuadCopterModel(
        cs.quadcopter_inertia,
        cs.quadcopter_mass,
        cs.trust_coef,
        cs.drag_coef,
        cs.arm_length,
    )

    sim = Simulator(controller, model, 0.01)
    sim.run()


if __name__ == "__main__":
    main()
