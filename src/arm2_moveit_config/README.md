# arm2_moveit_config

Minimal MoveIt 2 configuration for the 4-DOF arm2 manipulator.

This package does not use `ros2_control`. MoveIt sends planned trajectories to the existing custom controller through:

```text
/arm2_controller/follow_joint_trajectory
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rc_arm2_description rc_arm2_control arm2_moveit_config rc_arm2_moveit_client
source install/setup.bash
```

## Launch

Start MuJoCo and the torque controller first:

```bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py
```

Start MoveIt:

```bash
ros2 launch arm2_moveit_config move_group.launch.py
```

Optionally start RViz:

```bash
ros2 launch arm2_moveit_config moveit_rviz.launch.py
```

## Configuration Notes

- Planning group: `arm`
- Joints: `j1`, `j2`, `j3`, `j4`
- End frame: `tool0`
- Planner: `ompl_interface/OMPLPlanner`
- Default planner config: `geometric::RRTConnect`
- Time parameterization: `AddTimeOptimalParameterization`
- Controller manager: `moveit_simple_controller_manager`

The package intentionally leaves `kinematics.yaml` empty because 6D pose IK is not used in phase 1. The 4D task target is solved by `rc_arm2_moveit_client`, then MoveIt plans in joint space.
