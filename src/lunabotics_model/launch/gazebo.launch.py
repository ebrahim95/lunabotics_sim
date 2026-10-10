import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabotics_model")

    # Use the patched URDF (with <ros2_control> + <gazebo> plugin blocks).
    # Regenerate it after every Onshape export with:
    #   ros2 run lunabotics_model fix_kinematics.py \
    #       --input urdf/assembly_1.urdf --output urdf/assembly_1_fixed.urdf
    #   ros2 run lunabotics_model add_ros2_control.py \
    #       --input urdf/assembly_1_fixed.urdf --output urdf/assembly_1_sim.urdf
    urdf_path = os.path.join(pkg_share, "urdf", "assembly_1_sim.urdf")
    controllers_config = os.path.join(pkg_share, "config", "controllers.yaml")
    belt_animator_config = os.path.join(pkg_share, "config", "belt_animator.yaml")
    bucket_controller_config = os.path.join(pkg_share, "config", "bucket_controller.yaml")

    # The URDF is read as plain text, not through xacro, so expand the
    # $(find ...) in the gz_ros2_control <parameters> tag ourselves --
    # otherwise the controller manager never sees controllers.yaml.
    with open(urdf_path, "r") as f:
        urdf = f.read().replace("$(find lunabotics_model)", pkg_share)
    robot_description = {"robot_description": urdf}

    # Let Gazebo resolve package://lunabotics_model/meshes/... URIs.
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(pkg_share)
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        # Wheels are the lowest point, 0.308 m below the model origin; drop
        # the robot from just above that so it lands on them.
        arguments=["-topic", "robot_description", "-name", "lunabotics", "-z", "0.32"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Recent controller_manager releases no longer hand the plugin's
    # controllers.yaml to each controller, so the spawner passes it along.
    # One spawner for all of them avoids racing retries against a slow sim.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "wheel_velocity_controller",
            "belt_velocity_controller",
            "excavator_lift_controller",
            "--param-file", controllers_config,
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    bucket_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bucket_position_controller",
            "--param-file", bucket_controller_config,
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    belt_animator = Node(
        package="lunabotics_model",
        executable="belt_animator.py",
        parameters=[belt_animator_config],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    # Stagger controller spawners so they come up after Gazebo has
    # actually loaded the model and controller_manager is available.
    return LaunchDescription(
        [
            gz_resource_path,
            gz_sim,
            robot_state_publisher,
            clock_bridge,
            spawn_robot,
            TimerAction(period=5.0, actions=[controller_spawner]),
            TimerAction(period=5.0, actions=[bucket_controller_spawner]),
            TimerAction(period=7.0, actions=[belt_animator]),
            rviz,
        ]
    )
