#!/usr/bin/env python3
"""
Keyboard teleop for the whole robot: driving, excavator belt and ladder lift.

Driving uses the same keys as teleop_twist_keyboard (i , j l u o m . k,
q/z w/x e/c) and publishes TwistStamped on /cmd_vel for
diff_drive_controller. The excavator keys publish Float64MultiArray on
/belt_velocity_controller/commands (belt speed, rad/s) and
/excavator_lift_controller/commands (ladder position on its rails, m).

    ros2 run lunabotics_model teleop_keyboard.py

Runs on sim time (use_sim_time is forced on), so the /cmd_vel stamps match
the controller's clock. Belt and lift limits come from fix_kinematics.py.
"""
import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from fix_kinematics import LIFT_LOWER, LIFT_UPPER, ROLLER_VELOCITY  # noqa: E402

# key: (forward, turn) direction, as in teleop_twist_keyboard
DRIVE_KEYS = {
    "i": (1, 0), ",": (-1, 0), "j": (0, 1), "l": (0, -1),
    "u": (1, 1), "o": (1, -1), "m": (-1, -1), ".": (-1, 1), "k": (0, 0),
}
# key: (forward speed factor, turn speed factor)
SPEED_KEYS = {
    "q": (1.1, 1.1), "z": (0.9, 0.9),
    "w": (1.1, 1.0), "x": (0.9, 1.0),
    "e": (1.0, 1.1), "c": (1.0, 0.9),
}
START_SPEED = 0.5   # m/s
START_TURN = 1.0    # rad/s
BELT_STEP = 1.0     # rad/s per press
LIFT_STEP = 0.0254  # m per press (1")

HELP = f"""
Lunabotics teleop
-----------------
Driving (keeps going until you stop it):
   u    i    o       forward-left   forward   forward-right
   j    k    l       spin left      STOP      spin right
   m    ,    .       back-left      reverse   back-right

   q/z  all speeds +/- 10%     w/x  forward speed only     e/c  turning speed only

Excavator belt:     r faster   f slower / reverse   v stop     (max {ROLLER_VELOCITY:.2f} rad/s)
Ladder lift:        t raise 1"   g lower 1"   y fully raised   h fully lowered
Space: stop driving and the belt (the ladder stays where it is).   Ctrl+C: stop and quit
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Teleop(Node):
    def __init__(self):
        super().__init__("lunabotics_teleop",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.belt_pub = self.create_publisher(Float64MultiArray, "/belt_velocity_controller/commands", 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, "/excavator_lift_controller/commands", 10)
        self.speed, self.turn = START_SPEED, START_TURN
        self.drive = (0, 0)
        self.belt = 0.0
        self.lift = LIFT_UPPER

    def send_drive(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self.drive[0] * self.speed
        msg.twist.angular.z = self.drive[1] * self.turn
        self.cmd_pub.publish(msg)

    def send_belt(self):
        self.belt_pub.publish(Float64MultiArray(data=[self.belt]))

    def send_lift(self):
        self.lift_pub.publish(Float64MultiArray(data=[self.lift]))

    def on_key(self, key):
        """Apply one key press; return False to quit."""
        if key == "\x03":
            return False
        if key in DRIVE_KEYS:
            self.drive = DRIVE_KEYS[key]
            self.send_drive()
        elif key in SPEED_KEYS:
            fs, ft = SPEED_KEYS[key]
            self.speed *= fs
            self.turn *= ft
        elif key in "rfv":
            step = {"r": BELT_STEP, "f": -BELT_STEP, "v": None}[key]
            self.belt = 0.0 if step is None else round(
                clamp(self.belt + step, -ROLLER_VELOCITY, ROLLER_VELOCITY), 2)
            self.send_belt()
        elif key in "tgyh":
            self.lift = {"t": self.lift + LIFT_STEP, "g": self.lift - LIFT_STEP,
                         "y": LIFT_UPPER, "h": LIFT_LOWER}[key]
            self.lift = clamp(self.lift, LIFT_LOWER, LIFT_UPPER)
            self.send_lift()
        else:  # space or any other key: stop driving and the belt
            self.drive = (0, 0)
            self.send_drive()
            if key == " ":
                self.belt = 0.0
                self.send_belt()
        self.status()
        return True

    def status(self):
        lowered = (LIFT_UPPER - self.lift) / 0.0254
        sys.stdout.write(
            f"\r\033[Kdrive {self.drive[0] * self.speed:+.2f} m/s {self.drive[1] * self.turn:+.2f} rad/s"
            f" (speed {self.speed:.2f}, turn {self.turn:.2f})  |  "
            f"belt {self.belt:+.2f} rad/s ({self.belt * 60 / (2 * math.pi):+.0f} rpm)  |  "
            f"ladder {self.lift:+.3f} m ({lowered:.1f}\" down)")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Teleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.status()
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)  # keeps the sim clock current
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                ok = node.on_key(key)
                tty.setraw(sys.stdin.fileno())
                if not ok:
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.drive, node.belt = (0, 0), 0.0  # leave the robot stopped
        node.send_drive()
        node.send_belt()
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
