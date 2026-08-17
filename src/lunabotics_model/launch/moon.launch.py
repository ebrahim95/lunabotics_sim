"""Launch the rover on a lightweight, native Gazebo Moon heightmap."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabotics_model")
    gz_share = get_package_share_directory("ros_gz_sim")
    models_path = os.path.join(pkg_share, "models")
    existing_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = models_path if not existing_paths else models_path + os.pathsep + existing_paths

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": "-r " + os.path.join(pkg_share, "worlds", "moon_surface.world.sdf"),
            }.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
            output="screen",
        ),
    ])
