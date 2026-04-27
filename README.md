# arm2_mujoco

4-DOF arm2 manipulator simulation, torque trajectory control, and MoveIt/OMPL planning integration.

This workspace intentionally does not use `ros2_control` in the current phase. MoveIt sends `FollowJointTrajectory` goals directly to the custom torque controller.

## System Chain

The normal execution chain is:

```text
4D task goal
  target_xyz + target_spin
        |
        v
rc_arm2_moveit_client / arm2_plan_to_task_goal
  custom Pinocchio 4D IK: [x, y, z, spin] -> q_goal
        |
        v
MoveIt move_group
  OMPL joint-space planning + time parameterization
        |
        v
/arm2_controller/follow_joint_trajectory
  FollowJointTrajectory action
        |
        v
rc_arm2_control / arm2_torque_trajectory_controller
  tau = RNEA(qd, dqd, ddqd) + Kp(qd - q) + Kd(dqd - dq)
        |
        v
/arm2/command/effort
        |
        v
rc_arm2_control / arm2_mujoco_sim.py
  MuJoCo torque simulation and viewer
        |
        v
/joint_states
```

Payload state is synchronized separately:

```text
/arm2/payload_attached
  external request
        |
        v
arm2_torque_trajectory_controller
  switches unloaded/loaded dynamics only in HOLDING
        |
        v
/arm2/payload_active
        |
        +--> arm2_mujoco_sim.py
        |     enables/disables MuJoCo weld to tool0
        |
        +--> arm2_payload_scene_sync
              attaches/removes cube in MoveIt planning scene
```

## Packages

- `rc_arm2_description`: URDF and meshes. The URDF defines `tool0` as a fixed link under `l4`.
- `rc_arm2_control`: custom torque trajectory controller and MuJoCo simulator.
- `arm2_moveit_config`: minimal MoveIt configuration, including SRDF, OMPL, joint limits, controller mapping, and launch files.
- `rc_arm2_moveit_client`: task-level clients for 4D goal planning and payload planning-scene synchronization.

## Important Frames And Interfaces

- Planning group: `arm`
- Controlled joints: `j1`, `j2`, `j3`, `j4`
- End frame: `tool0`
- Trajectory action: `/arm2_controller/follow_joint_trajectory`
- Torque command topic: `/arm2/command/effort`
- Joint state topic: `/joint_states`
- Payload request topic: `/arm2/payload_attached`
- Payload active topic: `/arm2/payload_active`

`tool0` geometry is defined only in the URDF. Controller and MuJoCo read `tool0` from the model instead of using duplicated `tool0_xyz/tool0_rpy` YAML parameters.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  rc_arm2_description \
  rc_arm2_control \
  arm2_moveit_config \
  rc_arm2_moveit_client \
  rc_arm2_bringup
source install/setup.bash
```

## Start The System

If your LAN contains other ROS 2 machines publishing the same topic names such as `/joint_states`, run this workspace in localhost-only mode so discovery and traffic stay on the current machine:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

Use the same two environment variables in every terminal before launching nodes or using `ros2 topic/ros2 action` commands.

Recommended sim bringup:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py
```

Real hardware bringup:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_real_system.launch.py
```

Headless sim:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py enable_viewer:=false
```

Sim with RViz:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_bringup arm2_sim_system.launch.py start_rviz:=true
```

Manual startup for debugging:

Terminal 1: start MuJoCo simulation and the custom torque controller.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py
```

For headless checks without the MuJoCo viewer:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py enable_viewer:=false
```

Terminal 2: start MoveIt.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm2_moveit_config move_group.launch.py
```

Terminal 3: start the persistent 4D task goal action server.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rc_arm2_moveit_client arm2_plan_to_task_goal
```

Terminal 4: synchronize payload collision geometry into the MoveIt planning scene.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rc_arm2_moveit_client arm2_payload_scene_sync
```

Optional: start RViz.

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm2_moveit_config moveit_rviz.launch.py
```

## Plan A 4D Task Goal

The supported task target is:

```text
target_xyz:  tool0 position in base_link, meters
target_spin: tool0 rotation around its own +Z axis, radians
planning_time: MoveIt planning time for this goal, seconds; use 0.0 to keep the node default
```

After `arm2_plan_to_task_goal` is running, send task goals with the `/arm2_task_goal` action.

Planning only:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: false}"
```

Plan and execute:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: true}"
```

You can send another goal after the previous one finishes:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.12, 0.02, 0.22], target_spin: 0.4, planning_time: 3.0, execute: true}"
```

The client refuses to execute if the MoveIt trajectory does not contain positions, velocities, and accelerations, because the torque controller requires complete `q`, `dq`, and `ddq`.

## Payload Attach And Detach

Request payload attach:

```bash
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: true}"
```

Request payload detach:

```bash
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: false}"
```

The controller only applies payload model switching while in `HOLDING`. If a request arrives during trajectory execution, it is stored as pending and applied after the controller returns to `HOLDING`.

When active:

- Pinocchio uses the loaded dynamics model.
- MuJoCo welds the cube to `tool0`.
- MoveIt planning scene attaches a cube collision object to `tool0`.

## Configuration Files

Controller and simulator:

- `src/rc_arm2_control/config/arm2_control.yaml`

MoveIt:

- `src/arm2_moveit_config/config/arm2.srdf`
- `src/arm2_moveit_config/config/joint_limits.yaml`
- `src/arm2_moveit_config/config/ompl_planning.yaml`
- `src/arm2_moveit_config/config/moveit_controllers.yaml`
- `src/arm2_moveit_config/config/kinematics.yaml`

`kinematics.yaml` is intentionally empty in this phase. The arm is 4-DOF, so ordinary 6D MoveIt pose IK is not used. The project uses a custom 4D IK in `arm2_plan_to_task_goal`, then asks MoveIt to plan in joint space.

## Quick Validation

Build selected packages:

```bash
colcon build --packages-select rc_arm2_description rc_arm2_control arm2_moveit_config rc_arm2_moveit_client
```

Run a headless simulation/controller smoke test:

```bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py enable_viewer:=false
```

In another terminal, send the old sample trajectory directly to the controller:

```bash
ros2 run rc_arm2_control send_sample_trajectory.py
```

Start the persistent 4D task goal server:

```bash
ros2 run rc_arm2_moveit_client arm2_plan_to_task_goal
```

Run MoveIt planning without execution:

```bash
ros2 action send_goal /arm2_task_goal rc_arm2_moveit_client/action/PlanToTaskGoal \
  "{target_xyz: [0.10, -0.04, 0.24], target_spin: 0.0, planning_time: 5.0, execute: false}"
```

## Expected MoveIt Warnings

MoveIt may print:

```text
No kinematics plugins defined. Fill and load kinematics.yaml!
```

This is expected in the current phase. We are not using MoveIt 6D pose IK; the 4D IK is implemented in `rc_arm2_moveit_client`.

MoveIt may also print an Octomap sensor warning if no 3D sensor plugin is configured. This does not block joint-space planning.
