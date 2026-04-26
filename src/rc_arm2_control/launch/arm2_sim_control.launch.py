from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _control_config_path() -> Path:
    return Path(get_package_share_directory("rc_arm2_control")) / "config" / "arm2_control.yaml"


def _controller_params(config: dict) -> dict:
    return config["arm2_torque_trajectory_controller"]["ros__parameters"]


def _sim_unique_params(config: dict) -> dict:
    return config["arm2_mujoco_sim"]["ros__parameters"]


def _sim_params() -> dict:
    config = yaml.safe_load(_control_config_path().read_text())
    controller_params = _controller_params(config)
    sim_unique_params = _sim_unique_params(config)
    return {
        "command_topic": controller_params["command_topic"],
        "joint_state_topic": controller_params["joint_state_topic"],
        "payload_command_topic": controller_params["payload_command_topic"],
        "payload_active_topic": controller_params["payload_active_topic"],
        "joint_names": controller_params["joint_names"],
        "effort_limits": controller_params["effort_limits"],
        "kp": controller_params["kp"],
        "kd": controller_params["kd"],
        "payload_mass": controller_params["payload_mass"],
        "payload_cube_side": controller_params["payload_cube_side"],
        "tool0_frame_name": controller_params["tool0_frame_name"],
        **sim_unique_params,
    }


def generate_launch_description():
    config = str(_control_config_path())
    sim_params = _sim_params()

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
            parameters=[sim_params, {"enable_viewer": LaunchConfiguration("enable_viewer")}],
        ),
        Node(
            package="rc_arm2_control",
            executable="arm2_torque_trajectory_controller",
            name="arm2_torque_trajectory_controller",
            output="screen",
            parameters=[config],
        ),
    ])
