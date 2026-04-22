# rc_arm2_bringup

Bringup package for launching the complete arm2 phase-1 stack.

## Full System

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_full_system.launch.py
```

This starts:

- MuJoCo simulation and viewer
- Custom torque trajectory controller
- MoveIt `move_group`
- Persistent `/arm2_task_goal` action server
- Payload planning-scene synchronization

## Headless

```bash
ros2 launch rc_arm2_bringup arm2_full_system.launch.py enable_viewer:=false
```

## With RViz

```bash
ros2 launch rc_arm2_bringup arm2_full_system.launch.py start_rviz:=true
```

## Send A 4D Goal

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, execute: true}"
```

## Launch Arguments

- `enable_viewer`: default `true`; opens the MuJoCo viewer.
- `start_moveit`: default `true`; starts `move_group`.
- `start_task_goal_server`: default `true`; starts `/arm2_task_goal`.
- `start_payload_scene_sync`: default `true`; mirrors `/arm2/payload_active` into MoveIt.
- `start_rviz`: default `false`; starts RViz.
