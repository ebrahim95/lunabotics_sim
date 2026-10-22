#!/usr/bin/env python3
"""
Live table of the motors' torque, speed and load, from /joint_states.

`ros2 topic echo /joint_states --field effort` prints 37 bare numbers (31 of
them nan, for joints with no motor). This shows only the motors, by name,
with units and how close each is to its limit:

    Motor               Torque/force    Peak        Speed     Load
    Wheel front-left       4.21 N·m     4.60 N·m    6.1 rpm   ##........  21%
    ...

Each refresh averages the samples since the last one (so the tiny +/- jitter
of a wheel held at standstill reads as ~0) and shows the largest |value| as
the peak. Limits come from fix_kinematics.py, so they always match the model.

    ros2 run lunabotics_model motor_monitor.py
"""
import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from fix_kinematics import (LIFT_EFFORT, LIFT_VELOCITY, ROLLER_EFFORT,  # noqa: E402
                            ROLLER_VELOCITY, WHEEL_EFFORT, WHEEL_VELOCITY)

REFRESH = 0.5  # s
BAR = 20

# joint: (label, torque/force limit, speed limit, linear?)
MOTORS = {
    "revolute_1_2": ("Wheel front-left", WHEEL_EFFORT, WHEEL_VELOCITY, False),
    "revolute_1_3": ("Wheel front-right", WHEEL_EFFORT, WHEEL_VELOCITY, False),
    "revolute_1_1": ("Wheel rear-left", WHEEL_EFFORT, WHEEL_VELOCITY, False),
    "revolute_1": ("Wheel rear-right", WHEEL_EFFORT, WHEEL_VELOCITY, False),
    "drive_roller_joint": ("Belt motor", ROLLER_EFFORT, ROLLER_VELOCITY, False),
    "excavator_lift": ("Lift (2 actuators)", LIFT_EFFORT, LIFT_VELOCITY, True),
}

CLEAR = "\033[H\033[J"
BOLD, DIM, RESET = "\033[1m", "\033[2m", "\033[0m"
GREEN, YELLOW, RED = "\033[32m", "\033[33m", "\033[31m"


def colour(frac):
    return GREEN if frac < 0.6 else YELLOW if frac < 0.9 else RED


class MotorMonitor(Node):
    def __init__(self):
        super().__init__("motor_monitor")
        self.samples = {j: [] for j in MOTORS}
        self.speed = {j: 0.0 for j in MOTORS}
        self.count = 0
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_timer(REFRESH, self.draw)

    def on_state(self, msg):
        self.count += 1
        for i, name in enumerate(msg.name):
            if name not in MOTORS:
                continue
            if i < len(msg.effort) and not math.isnan(msg.effort[i]):
                self.samples[name].append(msg.effort[i])
            if i < len(msg.velocity) and not math.isnan(msg.velocity[i]):
                self.speed[name] = msg.velocity[i]

    def draw(self):
        out = [CLEAR, f"{BOLD}{'Motor':<20}{'Torque/force':>14}{'Peak':>12}"
                      f"{'Speed':>13}   Load (of limit){RESET}"]
        if self.count == 0:
            out.append(f"{DIM}waiting for /joint_states ... is the simulation running?{RESET}")
        for joint, (label, limit, vmax, linear) in MOTORS.items():
            s = self.samples[joint]
            if not s:
                out.append(f"{label:<20}{DIM}{'no data':>14}{RESET}")
                continue
            mean = sum(s) / len(s)
            peak = max(s, key=abs)
            unit = "N" if linear else "N·m"
            v = self.speed[joint]
            speed = f"{v * 1000:+7.1f} mm/s" if linear else f"{v * 60 / (2 * math.pi):+7.1f} rpm "
            frac = min(abs(mean) / limit, 1.0)
            n = round(frac * BAR)
            bar = f"{colour(frac)}{'#' * n}{RESET}{DIM}{'.' * (BAR - n)}{RESET}"
            vlim = f"{vmax * 1000:.0f} mm/s" if linear else f"{vmax * 60 / (2 * math.pi):.0f} rpm"
            out.append(f"{label:<20}{mean:>+9.2f} {unit:<4}{peak:>+8.2f} {unit:<4}"
                       f"{speed:>13}   {bar} {frac * 100:3.0f}%  "
                       f"{DIM}limit {limit:.0f} {unit}, {vlim}{RESET}")
            self.samples[joint] = []
        out.append(f"\n{DIM}Average over {REFRESH} s; + = wheel driving forward, belt running "
                   f"forward, lift pushing the ladder up. Ctrl+C to quit.{RESET}")
        print("\n".join(out), flush=True)


def main():
    rclpy.init()
    node = MotorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
