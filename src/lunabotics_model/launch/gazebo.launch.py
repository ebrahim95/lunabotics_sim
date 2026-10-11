import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where to put the robot in this package's worlds (x, y, yaw), unless the
# x/y/yaw launch arguments say otherwise.
WORLD_SPAWN = {
    # Artemis Arena: middle of the 2 x 2 m starting zone, facing down the arena.
    "artemis_arena": ("1.0", "1.0", "0.0"),
}


def world_path(pkg_share, world):
    """`world:=artemis_arena` -> this package's worlds/artemis_arena.world.sdf;
    anything else (empty.sdf, an absolute path, ...) goes to Gazebo as is."""
    for candidate in (world, world + ".world.sdf"):
        path = os.path.join(pkg_share, "worlds", candidate)
        if os.path.isfile(path):
            return path
    return world


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

    gpu_arg = DeclareLaunchArgument(
        "gpu",
        default_value="false",
        description="Render on the NVIDIA GPU on hybrid-graphics laptops "
                    "(PRIME on-demand). Only affects rendering; physics is CPU-only.",
    )
    # PRIME render offload: route GLX and EGL (Gazebo uses both) to NVIDIA.
    nvidia_env = [
        SetEnvironmentVariable(name, value, condition=IfCondition(LaunchConfiguration("gpu")))
        for name, value in (
            ("__NV_PRIME_RENDER_OFFLOAD", "1"),
            ("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            ("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
        )
    ]

    world_args = [
        DeclareLaunchArgument("world", default_value="empty.sdf",
                              description="Gazebo world: a name from this package's worlds/ "
                                          "(e.g. artemis_arena) or any world Gazebo can find."),
        DeclareLaunchArgument("x", default_value="", description="spawn x (m)"),
        DeclareLaunchArgument("y", default_value="", description="spawn y (m)"),
        DeclareLaunchArgument("yaw", default_value="", description="spawn heading (rad)"),
    ]

    def launch_world(context):
        world = LaunchConfiguration("world").perform(context)
        name = os.path.basename(world).replace(".world.sdf", "").replace(".sdf", "")
        x, y, yaw = WORLD_SPAWN.get(name, ("0.0", "0.0", "0.0"))
        x = LaunchConfiguration("x").perform(context) or x
        y = LaunchConfiguration("y").perform(context) or y
        yaw = LaunchConfiguration("yaw").perform(context) or yaw
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": world_path(pkg_share, world) + " -r"}.items(),
        )
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            # base_link sits on the ground under the wheels; drop the robot
            # from just above it so it lands on them.
            arguments=["-topic", "robot_description", "-name", "lunabotics",
                       "-x", x, "-y", y, "-z", "0.01", "-Y", yaw],
            output="screen",
        )
        return [gz_sim, spawn_robot]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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
            "diff_drive_controller",
            "belt_velocity_controller",
            "excavator_lift_controller",
            "--param-file", controllers_config,
            "--controller-manager-timeout", "60",
            # Plain /cmd_vel, like teleop_twist_keyboard's default topic.
            "--controller-ros-args", "-r /diff_drive_controller/cmd_vel:=/cmd_vel",
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
            gpu_arg,
            *nvidia_env,
            *world_args,
            gz_resource_path,
            OpaqueFunction(function=launch_world),
            robot_state_publisher,
            clock_bridge,
            TimerAction(period=5.0, actions=[controller_spawner]),
            TimerAction(period=5.0, actions=[bucket_controller_spawner]),
            TimerAction(period=7.0, actions=[belt_animator]),
            rviz,
        ]
    )
