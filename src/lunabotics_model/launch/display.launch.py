import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabotics_model")
    urdf_path = os.path.join(pkg_share, "urdf", "assembly_1_fixed.urdf")
    belt_animator_config = os.path.join(pkg_share, "config", "belt_animator.yaml")

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    robot_description = {"robot_description": robot_desc}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            # Bucket joints come from belt_animator (slaved to the
            # drive_roller_joint slider) rather than their own sliders.
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"source_list": ["/bucket_joint_states"]}],
                output="screen",
            ),
            Node(
                package="lunabotics_model",
                executable="belt_animator.py",
                parameters=[belt_animator_config],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
            ),
        ]
    )
