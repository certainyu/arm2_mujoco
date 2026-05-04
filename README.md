# arm2_mujoco

4 自由度 `arm2` 机械臂工作区，包含：

- MuJoCo 仿真
- 自定义力矩轨迹控制器
- MoveIt 关节空间规划
- 4D 任务目标 action
- 宏动作 middleware

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

### 宏动作链路

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
- `rc_arm2_middleware`: 抓取/放置/默认位宏动作状态机
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

查看 middleware 状态：

```bash
ros2 topic echo /arm2/middleware/state
```

说明：

- `mode: 1` 是 `PICK`
- `mode: 2` 是 `PLACE`
- `mode: 3` 是 `DEFAULT`
- `warehouse_index` 是 `0-based`
- `PICK` 前必须先有目标点
- middleware 出现流程错误时会先尝试回默认点

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
