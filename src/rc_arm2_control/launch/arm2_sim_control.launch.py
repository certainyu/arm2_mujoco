from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("rc_arm2_control"),
        "config",
        "arm2_control.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_viewer",
            default_value="true",
            description="Open the MuJoCo viewer window when true.",
        ),
        Node(
            package="rc_arm2_control",
            executable="arm2_mujoco_sim.py",
            name="arm2_mujoco_sim",
            output="screen",
            parameters=[config, {"enable_viewer": LaunchConfiguration("enable_viewer")}],
        ),
        Node(
            package="rc_arm2_control",
            executable="arm2_torque_trajectory_controller",
            name="arm2_torque_trajectory_controller",
            output="screen",
            parameters=[config],
        ),
    ])
