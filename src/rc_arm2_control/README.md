# rc_arm2_control

Custom ROS 2 Humble torque control and MuJoCo simulation for the 4-DOF arm2 manipulator.

This package intentionally does not use `ros2_control` in phase 1. MoveIt can connect later through the standard `FollowJointTrajectory` action exposed by the controller.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rc_arm2_description rc_arm2_control
source install/setup.bash
```

The MuJoCo simulator uses the Python `mujoco` package and opens a viewer window by default.

## Start Simulation And Controller

```bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py
```

For headless checks:

```bash
ros2 launch rc_arm2_control arm2_sim_control.launch.py enable_viewer:=false
```

Nodes and interfaces:

- `arm2_mujoco_sim.py` subscribes `/arm2/command/effort` as `sensor_msgs/JointState` and publishes `/joint_states`.
- Command `JointState` fields are `position=qd`, `velocity=dqd`, and `effort=tau_ff`.
- `arm2_torque_trajectory_controller` exposes `/arm2_controller/follow_joint_trajectory`.
- Publish `/arm2/payload_attached` as `std_msgs/Bool` to request unloaded/loaded switching.
- The controller publishes `/arm2/payload_active` only when the dynamic model has actually switched; the simulator listens to that by default so execution-time payload requests are delayed until `HOLDING`.

## Send A Test Trajectory

```bash
ros2 run rc_arm2_control send_sample_trajectory.py
```

Attach and detach the cube payload:

```bash
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: false}"
```

## Controller Behavior

- Control rate: `250 Hz`.
- Controller command: `tau_ff = rnea(qd, dqd, ddqd)`.
- Simulator control law: `tau = tau_ff + Kp * (qd - q) + Kd * (dqd - dq)`.
- Required trajectory fields: position, velocity, and acceleration for `j1,j2,j3,j4`.
- Saturation policy: `tau_ff` and simulator output torque are clamped to effort limits; continuous `tau_ff` saturation for `saturation_abort_sec` aborts the active trajectory.
- `STARTUP` is initialization only. The node enters `HOLDING` after loading parameters, Pinocchio models, publishers, subscribers, and action server.

## Payload Model

The payload is a cube attached at the face center:

- `payload_mass`: default `0.2 kg`.
- `payload_cube_side`: default `0.08 m`.
- `tool0_frame_name`: default `tool0`; this frame must exist in the URDF.

Pinocchio keeps unloaded and loaded models. The loaded model reads the `tool0` frame from the URDF and appends the cube mass and inertia at `tool0 + payload_cube_side / 2` along `tool0` +Z. MuJoCo keeps the cube as a free body and enables a weld constraint to the URDF `tool0` body when `/arm2/payload_active` becomes true.

## Later MoveIt/OMPL Integration

Create a separate `arm2_moveit_config` package:

- Define one planning group containing `j1,j2,j3,j4`.
- Use the same `tool0/suction_frame` convention for the end effector.
- Configure MoveIt controllers to send `FollowJointTrajectory` goals to `/arm2_controller/follow_joint_trajectory`.
- Use OMPL for path planning and a time parameterizer such as TOTG or Ruckig that emits complete `q,dq,ddq`.
- Start order: MuJoCo sim, this controller, `move_group`, then RViz or a MoveIt client.
