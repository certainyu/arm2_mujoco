from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = Path(get_package_share_directory("rc_arm2_description"))
    moveit_config = (
        MoveItConfigsBuilder("arm2", package_name="arm2_moveit_config")
        .robot_description(file_path=str(description_share / "urdf" / "arm2.urdf"))
        .robot_description_semantic(file_path="config/arm2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False)
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])
