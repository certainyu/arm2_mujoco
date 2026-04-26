# rc_arm2_moveit_client

MoveIt client utilities for arm2.

## 4D Goal Planning

`arm2_plan_to_task_goal` converts a 4D task target into a joint-space target:

```text
target = [tool0_x, tool0_y, tool0_z, tool0_spin]
```

`target_xyz` is expressed in `base_link`, in meters. `target_spin` is the rotation of `tool0` around its own `+Z` axis, in radians. `planning_time` is the MoveIt planning time for this goal, in seconds; use `0.0` to keep the node default.

`arm2_plan_to_task_goal` is a persistent action server. Start it once:

```bash
ros2 run rc_arm2_moveit_client arm2_plan_to_task_goal
```

Then send as many goals as needed:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: true}"
```

Planning only:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: false}"
```

The action interface is:

```text
Goal:   target_xyz[3], target_spin, planning_time, execute
Result: success, error_code, message
Feedback: state, q_goal[4]
```

The node refuses to execute if the MoveIt trajectory does not contain positions, velocities, and accelerations.

## Payload Planning Scene Sync

`arm2_payload_scene_sync` mirrors `/arm2/payload_active` into the MoveIt planning scene. When payload is active, it attaches a cube collision object to `tool0`; when inactive, it removes that object.

```bash
ros2 run rc_arm2_moveit_client arm2_payload_scene_sync
```

The cube geometry is configured by parameters:

- `payload_cube_side`, default `0.08`
- `base_frame`, default `base_link`
- `tool_frame`, default `tool0`
