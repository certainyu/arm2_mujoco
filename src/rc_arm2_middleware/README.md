# rc_arm2_middleware

`arm2_middleware` is a YAML-driven action-set executor for arm2. It subscribes to:

- target points from `/arm2/middleware/target_point`
- controller state from `/arm2_controller/state`
- payload model state from `/arm2/payload_active`

It coordinates those inputs with:

- `/arm2_task_goal`
- `/arm2/payload_attached`
- `/arm2/vacuum_activate`

## Startup

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rc_arm2_middleware arm2_middleware \
  --ros-args \
  --params-file src/rc_arm2_control/config/arm2_control.yaml
```

The action-set file is configured by `arm2_middleware.ros__parameters.action_sets_config_path`.
Relative paths are resolved against the `rc_arm2_middleware` package share directory.

## Command Interface

- topic: `/arm2/middleware/command`
- type: `rc_arm2_msgs/msg/Arm2MiddlewareCommand`

```text
int32 action_set_id
```

Example:

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{action_set_id: 1}"
```

## State Interface

- topic: `/arm2/middleware/state`
- type: `rc_arm2_msgs/msg/Arm2MiddlewareState`

The state message reports the current executor state, active action set and step, cached target point,
mirrored controller state, payload status, and the last move/action-set result.

## Target Point Input

- topic: `/arm2/middleware/target_point`
- type: `geometry_msgs/msg/Point`

Example:

```bash
ros2 topic pub --once /arm2/middleware/target_point geometry_msgs/msg/Point \
  "{x: 0.30, y: -0.05, z: 0.20}"
```

## YAML Schema

Action sets live in `config/action_sets.yaml`:

```yaml
action_sets:
  - id: 1
    name: pick_from_target
    steps:
      - type: set_vacuum
        label: vacuum_on
        enabled: true
      - type: move_target_offset
        label: approach_target
        offset_xyz: [0.0, 0.0, 0.05]
        target_spin: 0.0
        planning_time: 5.0
      - type: set_payload
        label: attach_payload
        attached: true
```

Supported step types:

- `move_target_offset`
  - required: `offset_xyz`, `target_spin`
  - optional: `planning_time`
- `move_fixed_pose`
  - required: `xyz`, `target_spin`
  - optional: `planning_time`
- `set_vacuum`
  - required: `enabled`
- `set_payload`
  - required: `attached`

Validation rules:

- `action_sets` must be a non-empty list
- each set needs a unique integer `id`
- each set needs a non-empty `name`
- each set needs a non-empty `steps`
- vector fields must contain exactly 3 floats
- `planning_time`, if present, must be positive

## Runtime Behavior

- Only one action set can run at a time.
- A second command received while busy is rejected with `ERROR_BUSY_REJECTED`.
- `move_target_offset` uses the latest cached target point when that step begins.
- Move steps advance when the action returns, even if `success=false`.
- `set_payload` waits until `/arm2/payload_active` matches the requested value.
- The current action set aborts immediately on controller `FAULT`, action timeouts, goal rejection,
  missing target point, or invalid action-set selection.
