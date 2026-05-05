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

## 使用 middleware

`arm2_middleware` 不会默认随 bringup 启动，需要单独运行：

```bash
ros2 run rc_arm2_middleware arm2_middleware \
  --ros-args \
  --params-file src/rc_arm2_control/config/arm2_control.yaml
```

middleware 当前是“YAML 动作集合执行器”，它的工作方式是：

1. 在 `src/rc_arm2_middleware/config/action_sets.yaml` 里定义若干动作集合
2. 上层通过 `/arm2/middleware/command` 发送 `action_set_id`
3. middleware 顺序执行该集合中的每个动作单元
4. 动作单元内部会调用 `/arm2_task_goal`、`/arm2/payload_attached`、`/arm2/vacuum_activate`

### 配置文件

middleware 运行依赖两份配置：

- 主参数文件：`src/rc_arm2_control/config/arm2_control.yaml`
- 动作集合文件：`src/rc_arm2_middleware/config/action_sets.yaml`

其中 `arm2_control.yaml` 里的这一项决定动作集合文件位置：

```yaml
arm2_middleware:
  ros__parameters:
    action_sets_config_path: config/action_sets.yaml
```

`action_sets_config_path` 写相对路径时，会相对于 `rc_arm2_middleware` 包的 share 目录解析。

### 输入

目标点：

```bash
ros2 topic pub --once /arm2/middleware/target_point geometry_msgs/msg/Point \
  "{x: 0.30, y: -0.05, z: 0.20}"
```

发送动作集合命令：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{action_set_id: 1}"
```

例如执行动作集合 `2`：

```bash
ros2 topic pub --once /arm2/middleware/command rc_arm2_msgs/msg/Arm2MiddlewareCommand \
  "{action_set_id: 2}"
```

查看 middleware 状态：

```bash
ros2 topic echo /arm2/middleware/state
```

### 动作集合 YAML 示例

```yaml
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
      - type: move_target_offset
        label: descend_to_target
        offset_xyz: [0.0, 0.0, 0.0]
        target_spin: 0.0
      - type: set_payload
        label: attach_payload
        attached: true
      - type: move_fixed_pose
        label: lift_to_safe_pose
        xyz: [0.25, 0.0, 0.24]
        target_spin: 0.0
```

支持的动作单元：

- `move_target_offset`
  - 必填：`offset_xyz`、`target_spin`
  - 可选：`planning_time`
- `move_fixed_pose`
  - 必填：`xyz`、`target_spin`
  - 可选：`planning_time`
- `set_vacuum`
  - 必填：`enabled`
- `set_payload`
  - 必填：`attached`

### 使用说明

- `Arm2MiddlewareCommand` 现在只接受 `action_set_id`
- 动作集合定义在 `src/rc_arm2_middleware/config/action_sets.yaml`
- `move_target_offset` 步骤会在该步骤开始时读取最新目标点，再叠加 `offset_xyz`
- `set_payload` 会等待 `/arm2/payload_active` 与请求值一致
- 动作步里的移动 action 即使 `success=false` 也会继续执行下一步，但会把失败写入状态消息
- 如果 middleware 正忙、新命令会被拒绝并上报 `ERROR_BUSY_REJECTED`
- 如果控制器进入 `FAULT`，当前动作集合会立即中止并进入 `ABORTED`

推荐的实际使用顺序：

1. 启动 sim 或 real bringup，确保 `/arm2_task_goal`、控制器和 payload 相关 topic 都在线
2. 单独启动 `arm2_middleware`
3. 如果动作集合里有 `move_target_offset`，先发布一次 `/arm2/middleware/target_point`
4. 发布 `/arm2/middleware/command`，携带目标 `action_set_id`
5. 用 `ros2 topic echo /arm2/middleware/state` 观察当前执行到哪一步，以及是否成功




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
