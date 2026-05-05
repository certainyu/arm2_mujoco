# arm2_mujoco

4 自由度 `arm2` 机械臂工作区，包含：

- MuJoCo 仿真
- 自定义力矩轨迹控制器
- MoveIt 关节空间规划
- 4D 任务目标 action
- YAML 动作集合 middleware

当前方案不使用 `ros2_control`。MoveIt 直接把 `FollowJointTrajectory` 发给自定义控制器。

## 系统链路

### 直接任务目标链路

```text
/arm2_task_goal
  ->
arm2_plan_to_task_goal
  ->
move_group
  ->
/arm2_controller/follow_joint_trajectory
  ->
arm2_torque_trajectory_controller
  ->
/arm2/command/effort
  ->
arm2_mujoco_sim.py / 真机驱动
```

### 动作集合链路

```text
/arm2/middleware/target_point
/arm2/middleware/command
  ->
arm2_middleware
  ->
/arm2_task_goal
/arm2/payload_attached
/arm2/vacuum_activate
```

## 主要包

- `rc_arm2_description`: URDF 和 mesh
- `rc_arm2_control`: 力矩控制器与 MuJoCo 仿真
- `arm2_moveit_config`: MoveIt 配置
- `rc_arm2_moveit_client`: `/arm2_task_goal` action server 与 payload scene sync
- `rc_arm2_middleware`: YAML 动作集合执行器
- `rc_arm2_bringup`: sim / real 启动入口
- `rc_arm2_msgs`: 自定义消息

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 推荐启动

如果局域网里有别的 ROS 2 设备，建议每个终端先设置：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

### 仿真

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py
```

### 真机

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_real_system.launch.py
```

### 无界面仿真

```bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py enable_viewer:=false
```

## 直接发送 4D 任务目标

启动 bringup 后，可以直接调用 `/arm2_task_goal`：

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: true}"
```

目标含义：

- `target_xyz`: `base_link` 坐标系下的 `[x, y, z]`，单位米
- `target_spin`: `tool0` 绕自身 `+Z` 旋转，单位弧度
- `planning_time`: MoveIt 规划时间
- `execute`: `true` 执行，`false` 只规划

## 使用 middleware

`arm2_middleware` 不会默认随 bringup 启动，需要单独运行：

```bash
ros2 run rc_arm2_middleware arm2_middleware \
  --ros-args \
  --params-file src/rc_arm2_control/config/arm2_control.yaml
```

### 输入

目标点：

```bash
ros2 topic pub --once /arm2/middleware/target_point geometry_msgs/msg/Point \
  "{x: 0.30, y: -0.05, z: 0.20}"
```

执行动作集合 `1`：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{action_set_id: 1}"
```

执行动作集合 `2`：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{action_set_id: 2}"
```

查看 middleware 状态：

```bash
ros2 topic echo /arm2/middleware/state
```

说明：

- `Arm2MiddlewareCommand` 现在只接受 `action_set_id`
- 动作集合定义在 `src/rc_arm2_middleware/config/action_sets.yaml`
- `move_target_offset` 步骤会在该步骤开始时读取最新目标点，再叠加 `offset_xyz`
- `set_payload` 会等待 `/arm2/payload_active` 与请求值一致
- 动作步里的移动 action 即使 `success=false` 也会继续执行下一步，但会把失败写入状态消息
- 如果 middleware 正忙、新命令会被拒绝并上报 `ERROR_BUSY_REJECTED`
- 如果控制器进入 `FAULT`，当前动作集合会立即中止并进入 `ABORTED`

### YAML 动作集合

middleware 通过独立 YAML 文件描述动作集合，默认路径由
`src/rc_arm2_control/config/arm2_control.yaml` 中的
`arm2_middleware.ros__parameters.action_sets_config_path` 指定。

示例：

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

支持的步骤类型：

- `move_target_offset`: 使用最新目标点加 `offset_xyz`
- `move_fixed_pose`: 直接使用给定 `xyz`
- `set_vacuum`: 发布 `/arm2/vacuum_activate`
- `set_payload`: 发布 `/arm2/payload_attached` 并等待 `/arm2/payload_active`

## Payload 与吸盘

手动切换动力学载荷：

```bash
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: false}"
```

手动开关吸盘：

```bash
ros2 topic pub /arm2/vacuum_activate std_msgs/msg/Bool "{data: true}" -1
ros2 topic pub /arm2/vacuum_activate std_msgs/msg/Bool "{data: false}" -1
```

## 关键接口

- `tool0`: 末端执行器 frame
- `/arm2_task_goal`: 4D 任务目标 action
- `/arm2_controller/follow_joint_trajectory`: 轨迹 action
- `/arm2_controller/state`: 控制器状态
- `/arm2/command/effort`: 力矩命令
- `/joint_states`: 关节状态
- `/arm2/payload_attached`: 请求切换载荷模型
- `/arm2/payload_active`: 实际生效的载荷状态
- `/arm2/vacuum_activate`: 吸盘控制

## 配置文件

主要配置：

- `src/rc_arm2_control/config/arm2_control.yaml`

MoveIt 配置：

- `src/arm2_moveit_config/config/`

middleware 的点位、超时和命令话题也写在 `arm2_control.yaml` 里。
动作集合内容本身默认写在 `src/rc_arm2_middleware/config/action_sets.yaml`。
