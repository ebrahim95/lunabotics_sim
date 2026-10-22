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

Below the key help, a status block redraws in place five times a second:
what you've commanded next to what the robot is actually doing, read from
/joint_states (wheel and belt speed, ladder position).
"""
import math
import os
import select
import shutil
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
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
REDRAW = 0.2        # s between status redraws
LEFT_WHEELS = ("revolute_1_1", "revolute_1_2")
RIGHT_WHEELS = ("revolute_1", "revolute_1_3")

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


def rpm(rad_s):
    return rad_s * 60 / (2 * math.pi)


def inches_down(lift):
    return (LIFT_UPPER - lift) / 0.0254


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
        self.joints = {}  # joint name -> (position, velocity), from /joint_states
        self.drawn = 0    # lines of the status block currently on screen
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def on_joints(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else math.nan
            vel = msg.velocity[i] if i < len(msg.velocity) else math.nan
            self.joints[name] = (pos, vel)

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

    def actual(self, joints, field):
        vals = [self.joints[j][field] for j in joints if j in self.joints]
        return sum(vals) / len(vals) if vals else None

    def status(self):
        """Redraw the status block in place (cursor up over the old one)."""
        left = self.actual(LEFT_WHEELS, 1)
        right = self.actual(RIGHT_WHEELS, 1)
        belt = self.actual(["drive_roller_joint"], 1)
        lift = self.actual(["excavator_lift"], 0)
        def show(v, fmt):
            return fmt(v) if v is not None and not math.isnan(v) else "--"
        rows = [
            ("", "commanded", "actual"),
            ("Drive", f"{self.drive[0] * self.speed:+.2f} m/s  {self.drive[1] * self.turn:+.2f} rad/s",
             f"wheels L {show(left, lambda v: f'{rpm(v):+5.1f}')}  R {show(right, lambda v: f'{rpm(v):+5.1f}')} rpm"),
            ("Belt", f"{self.belt:+.2f} rad/s ({rpm(self.belt):+3.0f} rpm)",
             show(belt, lambda v: f"{rpm(v):+3.0f} rpm")),
            ("Ladder", f'{inches_down(self.lift):.1f}" down',
             show(lift, lambda v: f'{inches_down(v):.1f}" down')),
        ]
        lines = [f"{name:<8}{cmd:<28}{act}" for name, cmd, act in rows]
        lines.append(f"Key speeds: forward {self.speed:.2f} m/s, turn {self.turn:.2f} rad/s")
        width = max(20, shutil.get_terminal_size().columns - 1)
        out = f"\033[{self.drawn}A" if self.drawn else ""
        out += "".join(f"\r\033[K{line[:width]}\r\n" for line in lines)
        self.drawn = len(lines)
        sys.stdout.write(out)
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Teleop()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.status()
    # ROS callbacks (sim clock, /joint_states) run in the background;
    # this thread reads keys and redraws the status block.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], REDRAW)[0]:
                if not node.on_key(sys.stdin.read(1)):
                    break
            else:
                node.status()
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
