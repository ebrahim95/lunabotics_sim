#!/usr/bin/env python3
"""
Patches the repaired URDF (output of fix_kinematics.py) with the
<ros2_control> and <gazebo> plugin blocks needed to drive the wheels, the
excavator drive roller, the excavator lift and the bucket track joints
from ROS 2 / Gazebo, without
hand-editing a multi-thousand-line file.

Usage:
    ros2 run lunabotics_model add_ros2_control.py \
        --input urdf/assembly_1_fixed.urdf \
        --output urdf/assembly_1_sim.urdf

Run this after fix_kinematics.py whenever you re-export from Onshape.
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fix_kinematics import TOTAL_MASS, loop_pose  # noqa: E402

# Wheel slip on loose regolith (Gazebo WheelSlip system). Steady-state slip
# ~= compliance * (traction force / wheel load). ESTIMATE for BP-1-like loose
# sand: ~20% slip while pushing at ~0.3x the wheel load -> ~0.6. No measured
# value for these wheels on BP-1; tune against traction tests if you have them.
SLIP_COMPLIANCE_LONGITUDINAL = 0.6
SLIP_COMPLIANCE_LATERAL = 0.6

WHEEL_JOINTS = ["revolute_1", "revolute_1_1", "revolute_1_2", "revolute_1_3"]

# (joint, command interface or None for a passive joint that only reports state)
EXCAVATOR_JOINTS = [
    ("drive_roller_joint", "velocity"),
    ("excavator_lift", "position"),
    ("idler_roller_joint", None),
]

ROS2_CONTROL_BLOCK = """
    <ros2_control name="lunabotics_wheel_drive" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>
{wheel_joints}
    </ros2_control>

    <ros2_control name="lunabotics_excavator" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>
{excavator_joints}
    </ros2_control>

    <ros2_control name="lunabotics_belt" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>
{belt_joints}
    </ros2_control>

    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>$(find lunabotics_model)/config/controllers.yaml</parameters>
        </plugin>
        <plugin filename="gz-sim-wheel-slip-system" name="gz::sim::systems::WheelSlip">
{wheel_slip}
        </plugin>
    </gazebo>
"""


def joint_xml(name, command="velocity", initial=None):
    lines = [f'        <joint name="{name}">']
    if command:
        lines.append(f'            <command_interface name="{command}"/>')
    if initial is None:
        lines.append('            <state_interface name="position"/>')
    else:
        lines.append('            <state_interface name="position">')
        lines.append(f'                <param name="initial_value">{initial:.9g}</param>')
        lines.append('            </state_interface>')
    lines.append('            <state_interface name="velocity"/>')
    lines.append("        </joint>")
    return "\n".join(lines)


def wheel_slip_xml(urdf_text):
    """One <wheel> per drive wheel. Gazebo merges each wheel into the flange
    its joint drives, so the flange is the link that touches the ground."""
    robot = ET.fromstring(urdf_text)
    flanges = [robot.find(f"joint[@name='{j}']").find("child").get("link") for j in WHEEL_JOINTS]
    radius = float(robot.find("link[@name='wheel']").find("collision/geometry/cylinder").get("radius"))
    load = TOTAL_MASS * 9.81 / len(flanges)
    return "\n".join(
        f'            <wheel link_name="{f}">\n'
        f'                <slip_compliance_longitudinal>{SLIP_COMPLIANCE_LONGITUDINAL}</slip_compliance_longitudinal>\n'
        f'                <slip_compliance_lateral>{SLIP_COMPLIANCE_LATERAL}</slip_compliance_lateral>\n'
        f'                <wheel_normal_force>{load:.1f}</wheel_normal_force>\n'
        f'                <wheel_radius>{radius:.4f}</wheel_radius>\n'
        f'            </wheel>' for f in flanges)


def build_block(belt_joints, belt_start, wheel_slip):
    return ROS2_CONTROL_BLOCK.format(
        wheel_slip=wheel_slip,
        wheel_joints="\n".join(joint_xml(j) for j in WHEEL_JOINTS),
        excavator_joints="\n".join(joint_xml(j, c) for j, c in EXCAVATOR_JOINTS),
        belt_joints="\n".join(
            joint_xml(j, "position", belt_start.get(j)) for j in belt_joints),
    )


def belt_start_positions(config_path):
    """Joint values that put every bucket at its CAD pose, so Gazebo spawns
    them on the belt instead of stacked on the bottom shaft."""
    with open(config_path) as f:
        p = yaml.safe_load(f)["belt_animator"]["ros__parameters"]
    q = {}
    for b, s0 in zip(p["buckets"], p["start_positions"]):
        u, v, th = loop_pose(s0, p["shaft_distance"], p["pitch_radius"])
        q.update({f"{b}_belt_u": u, f"{b}_belt_v": v, f"{b}_belt_rot": th})
    return q


def find_belt_joints(urdf_text):
    """Bucket track joints added by fix_kinematics.py (<bucket>_belt_u/v/rot)."""
    robot = ET.fromstring(urdf_text)
    names = [j.get("name") for j in robot.findall("joint")]
    joints = [n for n in names if re.fullmatch(r"bucket_\d+_belt_(u|v|rot)", n)]
    if not joints:
        raise ValueError("No <bucket>_belt_* joints found -- run fix_kinematics.py first.")
    return joints


def patch(input_path, output_path, belt_config):
    with open(input_path, "r") as f:
        content = f.read()

    if "<ros2_control" in content:
        print("File already contains a <ros2_control> block -- skipping insertion.")
        with open(output_path, "w") as f:
            f.write(content)
        return

    belt_joints = find_belt_joints(content)
    belt_start = belt_start_positions(belt_config)
    missing = set(belt_joints) - set(belt_start)
    if missing:
        raise ValueError(f"{belt_config} has no start position for {sorted(missing)} -- "
                         "re-run fix_kinematics.py")
    block = build_block(belt_joints, belt_start, wheel_slip_xml(content))

    # Insert right before the closing </robot> tag
    if "</robot>" not in content:
        raise ValueError("No </robot> closing tag found -- is this a valid URDF?")

    patched = re.sub(r"</robot>\s*$", block + "\n</robot>\n", content)

    with open(output_path, "w") as f:
        f.write(patched)

    print(f"Wrote patched URDF to {output_path}")
    print(f"Added ros2_control interfaces for wheel joints: {WHEEL_JOINTS}")
    print(f"Added ros2_control interfaces for excavator joints: {EXCAVATOR_JOINTS}")
    print(f"Added position interfaces for {len(belt_joints)} bucket track joints")
    print(f"Added wheel slip (compliance {SLIP_COMPLIANCE_LONGITUDINAL} long., "
          f"{SLIP_COMPLIANCE_LATERAL} lat.) on the 4 drive wheels")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Path to the repaired URDF from fix_kinematics.py")
    parser.add_argument("--output", required=True, help="Path to write the patched URDF")
    parser.add_argument("--belt-config", default=None,
                        help="belt_animator.yaml from fix_kinematics.py "
                             "(default: ../config/belt_animator.yaml next to --input)")
    args = parser.parse_args()
    belt_config = args.belt_config or os.path.join(
        os.path.dirname(os.path.abspath(args.input)), "..", "config", "belt_animator.yaml")
    patch(args.input, args.output, belt_config)


if __name__ == "__main__":
    main()
