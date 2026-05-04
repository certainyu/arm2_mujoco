# rc_arm2_middleware

`arm2_middleware` 是一个业务层状态机节点，用来把：

- 目标点输入
- 抓取 / 放置 / 回默认命令
- `/arm2_task_goal` action
- `/arm2_controller/state`
- `/arm2/payload_active`

串起来，执行完整的宏动作流程。

## 依赖

启动前需要先有这些节点在线：

- `arm2_torque_trajectory_controller`
- `arm2_plan_to_task_goal`
- `move_group`

也就是说，通常先启动整套 bringup，再单独启动 middleware。

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rc_arm2_middleware arm2_middleware \
  --ros-args \
  --params-file src/rc_arm2_control/config/arm2_control.yaml
```

## 输入接口

目标点：

- topic: `/arm2/middleware/target_point`
- type: `geometry_msgs/msg/Point`

命令：

- topic: `/arm2/middleware/command`
- type: `rc_arm2_msgs/msg/Arm2MiddlewareCommand`

命令模式：

- `0`: `NONE`
- `1`: `PICK`
- `2`: `PLACE`
- `3`: `DEFAULT`

## 输出接口

状态：

- topic: `/arm2/middleware/state`
- type: `rc_arm2_msgs/msg/Arm2MiddlewareState`

middleware 还会主动使用这些底层接口：

- action: `/arm2_task_goal`
- topic: `/arm2/payload_attached`
- topic: `/arm2/vacuum_activate`

## 常用命令

发送目标点：

```bash
ros2 topic pub --once /arm2/middleware/target_point geometry_msgs/msg/Point \
  "{x: 0.30, y: -0.05, z: 0.20}"
```

抓取：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{mode: 1, warehouse_index: -1, target_spin: 0.0}"
```

放置到仓库点 `0`：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{mode: 2, warehouse_index: 0, target_spin: 0.0}"
```

回默认点：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{mode: 3, warehouse_index: -1, target_spin: 0.0}"
```

查看状态：

```bash
ros2 topic echo /arm2/middleware/state
```

## 参数

middleware 参数写在：

- `src/rc_arm2_control/config/arm2_control.yaml`
- 节点段：`arm2_middleware.ros__parameters`

重点参数：

- `planning_time`
- `action_server_timeout_sec`
- `action_result_timeout_sec`
- `payload_wait_timeout_sec`
- `staging_pose`
- `default_pose`
- `warehouse_poses`

位姿格式统一为：

```text
[x, y, z, spin]
```

其中位置单位为米，`spin` 单位为弧度。

## 行为说明

- `PICK` 前必须先收到目标点，否则会进入错误恢复。
- `PLACE` 的 `warehouse_index` 使用 `0-based`。
- 切到带载、切回卸载，都会等待 `/arm2/payload_active` 确认。
- 发生流程错误时，middleware 会先尝试回默认点，再自动回空闲。
