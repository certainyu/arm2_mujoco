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
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
    ])
