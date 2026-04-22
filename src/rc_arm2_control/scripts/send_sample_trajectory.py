#!/usr/bin/env python3
"""Send a small 4-joint trajectory to the custom arm2 action controller."""

from __future__ import annotations

import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class SampleTrajectoryClient(Node):
    def __init__(self) -> None:
        super().__init__("arm2_sample_trajectory_client")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm2_controller/follow_joint_trajectory",
        )

    def send(self) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["j1", "j2", "j3", "j4"]
        goal.trajectory.points = [
            self._point([0.0, 0.0, 0.0, 0.0], 0.0),
            self._point([0.25, -0.25, 0.2, 0.35], 2.0),
            self._point([0.0, 0.0, 0.0, 0.0], 4.0),
        ]

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"Result error_code={result.error_code} error_string='{result.error_string}'"
        )

    def _point(self, positions, time_from_start: float) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0]
        sec = int(math.floor(time_from_start))
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((time_from_start - sec) * 1.0e9)
        return point


def main() -> None:
    rclpy.init()
    node = SampleTrajectoryClient()
    try:
        node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
