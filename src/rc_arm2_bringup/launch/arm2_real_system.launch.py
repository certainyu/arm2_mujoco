from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _control_config_path() -> Path:
    return Path(get_package_share_directory("rc_arm2_control")) / "config" / "arm2_control.yaml"


def _usb2canfd_params_from_controller() -> dict:
    config = yaml.safe_load(_control_config_path().read_text())
    controller_params = config["arm2_torque_trajectory_controller"]["ros__parameters"]
    real_params = config["arm2_mujoco_real"]["ros__parameters"]
    return {
        "command_topic": controller_params["command_topic"],
        "joint_state_topic": controller_params["joint_state_topic"],
        "payload_active_topic": controller_params["payload_active_topic"],
        "joint_names": controller_params["joint_names"],
        "joint_directions": real_params["joint_directions"],
        "vacuum_activate_topic": real_params["vacuum_activate_topic"],
        "kp_unloaded": controller_params["kp_unloaded"],
        "kd_unloaded": controller_params["kd_unloaded"],
        "kp_loaded": controller_params["kp_loaded"],
        "kd_loaded": controller_params["kd_loaded"],
    }


def generate_launch_description():
    control_config = str(_control_config_path())
    usb2canfd_params = _usb2canfd_params_from_controller()
    start_moveit = LaunchConfiguration("start_moveit")
    start_task_goal_server = LaunchConfiguration("start_task_goal_server")
    start_payload_scene_sync = LaunchConfiguration("start_payload_scene_sync")
    start_rviz = LaunchConfiguration("start_rviz")

    move_group_launch = PathJoinSubstitution([
        FindPackageShare("arm2_moveit_config"),
        "launch",
        "move_group.launch.py",
    ])
    rviz_launch = PathJoinSubstitution([
        FindPackageShare("arm2_moveit_config"),
        "launch",
        "moveit_rviz.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_moveit",
            default_value="true",
            description="Start move_group and robot_state_publisher.",
        ),
        DeclareLaunchArgument(
            "start_task_goal_server",
            default_value="true",
            description="Start the persistent /arm2_task_goal action server.",
        ),
        DeclareLaunchArgument(
            "start_payload_scene_sync",
            default_value="true",
            description="Start payload collision-object synchronization for MoveIt.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz with MoveIt parameters.",
        ),
        Node(
            package="rc_arm2_control",
            executable="arm2_torque_trajectory_controller",
            name="arm2_torque_trajectory_controller",
            output="screen",
            parameters=[control_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_group_launch),
            condition=IfCondition(start_moveit),
        ),
        Node(
            package="rc_arm2_moveit_client",
            executable="arm2_plan_to_task_goal",
            name="arm2_plan_to_task_goal",
            output="screen",
            condition=IfCondition(start_task_goal_server),
        ),
        Node(
            package="rc_arm2_moveit_client",
            executable="arm2_payload_scene_sync",
            name="arm2_payload_scene_sync",
            output="screen",
            parameters=[control_config],
            condition=IfCondition(start_payload_scene_sync),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(start_rviz),
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="dmbot_serial",
                    executable="usb2canfd_dm_node_cpp",
                    name="usb2canfd_dm_node",
                    output="screen",
                    parameters=[usb2canfd_params],
                ),
            ],
        ),
    ])
