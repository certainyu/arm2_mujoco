# rc_arm2_bringup

Bringup package for launching the complete arm2 phase-1 stack.

## Sim System

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py
```

This starts:

- MuJoCo simulation and viewer
- Custom torque trajectory controller
- MoveIt `move_group`
- Persistent `/arm2_task_goal` action server
- Payload planning-scene synchronization

## Real System

```bash
ros2 launch rc_arm2_bringup arm2_real_system.launch.py
```

This starts:

- Custom torque trajectory controller
- USB-CAN-FD motor driver node
- MoveIt `move_group`
- Persistent `/arm2_task_goal` action server
- Payload planning-scene synchronization

## Headless Sim

```bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py enable_viewer:=false
```

## Sim With RViz

```bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py start_rviz:=true
```

## Send A 4D Goal

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: true}"
```

## Launch Arguments

- `enable_viewer`: sim launch only, default `true`; opens the MuJoCo viewer.
- `start_moveit`: default `true`; starts `move_group`.
- `start_task_goal_server`: default `true`; starts `/arm2_task_goal`.
- `start_payload_scene_sync`: default `true`; mirrors `/arm2/payload_active` into MoveIt.
- `start_rviz`: default `false`; starts RViz.
