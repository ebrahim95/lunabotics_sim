#!/usr/bin/env python3
"""Generate a driveable Gazebo model from the visual CAD URDF.

All fixed CAD parts become visuals on one physical chassis.  The four wheel
meshes stay separate, driven links so that they rotate with Gazebo's DiffDrive
system.  The CAD's native forward axis is rotated onto Gazebo +X.
"""

import math
import os
import sys
import xml.etree.ElementTree as ET


WHEELS = {
    "wheel": ("rear_left_wheel", "wheel.stl", "left"),
    "wheel_1": ("front_left_wheel", "wheel.stl", "left"),
    "wheel_mirrored": ("rear_right_wheel", "wheel_Mirrored.stl", "right"),
    "wheel_mirrored_1": ("front_right_wheel", "wheel_Mirrored.stl", "right"),
}
MODEL_YAW = -math.pi / 2.0
WHEEL_RADIUS = 0.1805
WHEEL_WIDTH = 0.10
# The wheel-link origin in the CAD is z=-0.127393 m.  This brings the
# measured 0.1805 m tire radius exactly onto the ground plane.
GROUND_CLEARANCE = 0.307893


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
            [-sp, cp * sr, cp * cr, 0], [0, 0, 0, 1]]


def transform(origin):
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    result = rpy_matrix(*rpy)
    result[0][3], result[1][3], result[2][3] = xyz
    return result


def pose_from_matrix(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    roll = math.atan2(matrix[2][1], matrix[2][2])
    yaw = math.atan2(matrix[1][0], matrix[0][0])
    return (matrix[0][3], matrix[1][3], matrix[2][3], roll, pitch, yaw)


def fmt_pose(matrix, raise_z=False):
    x, y, z, roll, pitch, yaw = pose_from_matrix(matrix)
    if raise_z:
        z += GROUND_CLEARANCE
    return " ".join(f"{value:.9g}" for value in (x, y, z, roll, pitch, yaw))


def rotated(matrix):
    return matmul(rpy_matrix(0, 0, MODEL_YAW), matrix)


def normalized(vector):
    magnitude = math.sqrt(sum(value * value for value in vector))
    return [value / magnitude for value in vector]


def main():
    robot = ET.parse(sys.argv[1]).getroot()
    joints = {}
    parents = {}
    for joint in robot.findall("joint"):
        child = joint.find("child").attrib["link"]
        parent = joint.find("parent").attrib["link"]
        joints[child] = joint
        parents[child] = (parent, transform(joint.find("origin")))

    def absolute_transform(link_name):
        result = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        while link_name in parents:
            link_name, joint_transform = parents[link_name]
            result = matmul(joint_transform, result)
        return rotated(result)

    out = []
    add = out.append
    add('<?xml version="1.0" ?>')
    add('<sdf version="1.9"><model name="lunabotics_rover"><self_collide>false</self_collide>')
    add('<link name="base_link"><pose>0 0 0.38 0 0 0</pose>')
    add('<inertial><mass>55</mass><inertia><ixx>5</ixx><iyy>7</iyy><izz>8</izz></inertia></inertial>')
    add('<collision name="chassis_collision"><geometry><box><size>1.15 0.72 0.24</size></box></geometry></collision>')

    for link in robot.findall("link"):
        link_name = link.attrib["name"]
        if link_name in WHEELS:
            continue
        link_transform = absolute_transform(link_name)
        for index, visual in enumerate(link.findall("visual")):
            visual_transform = matmul(link_transform, transform(visual.find("origin")))
            mesh = os.path.basename(visual.find("geometry/mesh").attrib["filename"])
            color = visual.find("material/color")
            rgba = color.attrib["rgba"] if color is not None else "0.7 0.7 0.7 1"
            add(f'<visual name="cad_{link_name}_{index}"><pose>{fmt_pose(visual_transform)}</pose>'
                f'<geometry><mesh><uri>model://lunabotics_rover/meshes/{mesh}</uri></mesh></geometry>'
                f'<material><diffuse>{rgba}</diffuse></material></visual>')
    add('</link>')

    # All four exported wheels share one physical axle line.  The mirrored
    # CAD links use the opposite *direction* for that line, but the same line;
    # force one common joint-axis direction so wheel commands do not fight.
    axle_transform = absolute_transform("wheel")
    axle_axis = normalized([axle_transform[0][1], axle_transform[1][1], axle_transform[2][1]])
    axle_axis_text = " ".join(f"{value:.9g}" for value in axle_axis)

    left_joints, right_joints = [], []
    for urdf_name, (sim_name, mesh, side) in WHEELS.items():
        wheel_transform = absolute_transform(urdf_name)
        pose = fmt_pose(wheel_transform, raise_z=True)
        mass_node = next(link for link in robot.findall("link") if link.attrib["name"] == urdf_name).find("inertial/mass")
        mass = mass_node.attrib["value"]
        add(f'<link name="{sim_name}"><pose>{pose}</pose><inertial><mass>{mass}</mass>'
            '<inertia><ixx>0.0797664</ixx><iyy>0.146428</iyy><izz>0.0797664</izz></inertia></inertial>'
            f'<collision name="collision"><pose>0 0 0 1.5708 0 0</pose><geometry><cylinder><radius>{WHEEL_RADIUS}</radius><length>{WHEEL_WIDTH}</length></cylinder></geometry>'
            '<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>'
            f'<visual name="visual"><pose>0 0.027 0 0 0 0</pose><geometry><mesh><uri>model://lunabotics_rover/meshes/{mesh}</uri></mesh></geometry><material><diffuse>0.231373 0.380392 0.705882 1</diffuse></material></visual></link>')
        # The wheel mesh and collision both rotate around the URDF's common
        # axle line. expressed_in="__model__" prevents mirrored links from
        # being interpreted as opposite steering angles.
        joint_name = sim_name + "_joint"
        add(f'<joint name="{joint_name}" type="revolute"><parent>base_link</parent><child>{sim_name}</child>'
            f'<axis><xyz expressed_in="__model__">{axle_axis_text}</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>80</effort><velocity>25</velocity></limit></axis></joint>')
        (left_joints if side == "left" else right_joints).append(joint_name)

    add('<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">')
    for name in left_joints:
        add(f'<left_joint>{name}</left_joint>')
    for name in right_joints:
        add(f'<right_joint>{name}</right_joint>')
    add(f'<wheel_separation>0.55</wheel_separation><wheel_radius>{WHEEL_RADIUS}</wheel_radius>'
        '<topic>cmd_vel</topic><odom_topic>odom</odom_topic><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>'
        '</plugin></model></sdf>')
    print("\n".join(out))


if __name__ == "__main__":
    main()
