#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{
// 固定自由度 4
constexpr std::size_t kDof = 4;
// 定义了一个非常小的正数，
// 计算两个轨迹点的时间间隔 b.t - a.t 时，如果间隔非常小甚至为 0
// 就用 1e-9 代替，防止 (elapsed - a.t) / span 出现除以 0。
constexpr double kMinPositive = 1.0e-9;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

/**
 * @brief 将参数中的 vector<double> 转换为固定长度 4 的数组(array<double, kDof>)。
 * @brief 在构造函数declaere_parameter时使用。
 *
 * @param values 从 ROS 参数读取到的数组；为空时使用 defaults。
 * @param defaults 默认数组。
 * @param name 参数名，仅用于报错信息。
 * @return std::array<double, kDof> 长度固定为 4 的数组。
 *
 * @throw std::runtime_error 当 values 非空但长度不是 4 时抛出。
 */
std::array<double, kDof> to_array4(
  const std::vector<double> & values,
  const std::array<double, kDof> & defaults,
  const std::string & name)
{
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != kDof) {
    throw std::runtime_error(name + " must contain exactly 4 values");
  }
  std::array<double, kDof> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

/**
 * @brief 将 ROS Duration 消息转换为秒。
 *
 * @param duration ROS 内置 Duration，包含 sec 和 nanosec。
 * @return double 以秒为单位的浮点时间。
 */
double duration_to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

/**
 * @brief 将关节名数组拼接为逗号分隔的字符串。
 *
 * @param names 关节名数组。
 * @return std::string 形如 "j1, j2, j3, j4" 的字符串。
 */
std::string join_names(const std::vector<std::string> & names)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << names[i];
  }
  return stream.str();
}
}  // namespace








class Arm2TorqueTrajectoryController : public rclcpp::Node
{
public:
  /**
   * @brief 构造机械臂力矩轨迹控制器节点。
   *
   * 构造过程中完成参数读取、Pinocchio 空载/负载模型加载、ROS topic 创建、
   * FollowJointTrajectory action server 创建以及 250 Hz 控制定时器创建。
   * 初始化期间状态为 STARTUP，全部完成后进入 HOLDING。
   */
  Arm2TorqueTrajectoryController()
  : Node("arm2_torque_trajectory_controller")
  {
    state_ = State::STARTUP;
    
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"j1", "j2", "j3", "j4"});
    if (joint_names_.size() != kDof) {
      throw std::runtime_error("joint_names must contain exactly 4 names");
    }
    // action_name_ 是 FollowJointTrajectory action server 的名字
    // 后续会由moveit生成的action client连接到这个名字发送目标
    action_name_ = declare_parameter<std::string>(
      "action_name", "/arm2_controller/follow_joint_trajectory");
    // 发布 MIT 目标命令的 topic，类型为 sensor_msgs::msg::JointState：
    // position=期望位置，velocity=期望速度，effort=期望前馈力矩。
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm2/command/effort");
    // 订阅关节状态的 topic，类型为 sensor_msgs::msg::JointState，包含所有关节的状态，但只使用 j1-j4 的位置和速度
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    // 订阅负载吸附状态的 topic，类型为 std_msgs::msg::Bool，true 表示负载已吸附，false 表示负载未吸附
    payload_command_topic_ = declare_parameter<std::string>(
      "payload_command_topic", "/arm2/payload_attached");
    // 发布负载吸附状态的 topic，类型为 std_msgs::msg::Bool，目前由控制器发布当前使用的模型是带负载还是空载
    payload_active_topic_ = declare_parameter<std::string>(
      "payload_active_topic", "/arm2/payload_active");
    // 发布控制器状态的 topic，类型为 std_msgs::msg::String，
    // 包含 "initialized"、"goal accepted"、"payload switch pending"、"holding torque saturation fault" 等状态信息
    state_topic_ = declare_parameter<std::string>("state_topic", "/arm2_controller/state");
    
    // 控制周期频率，单位 Hz，默认 250 Hz
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 250.0);
    // 每秒向 FollowJointTrajectory action client 反馈 20 次状态，避免 feedback 消息太频繁。
    feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 20.0);
    // 如果连续的力矩饱和时间超过 saturation_abort_sec_，则认为执行失败并进入 FAULT 状态。
    saturation_abort_sec_ = declare_parameter<double>("saturation_abort_sec", 0.25);
    // 执行轨迹时，如果最终位置与目标位置的误差小于 goal_tolerance_rad_，
    // 且持续时间超过 settle_time_sec_，则认为执行成功并进入 HOLDING 状态。
    goal_tolerance_rad_ = declare_parameter<double>("goal_tolerance_rad", 0.02);
    settle_time_sec_ = declare_parameter<double>("settle_time_sec", 0.5);
    // 负载相关参数。tool0_frame_name_ 必须存在于 URDF/Pinocchio 模型中；
    // 负载会固定在 tool0 的 +Z 方向半个正方体边长处。
    tool0_frame_name_ = declare_parameter<std::string>("tool0_frame_name", "tool0");
    // 默认负载质量 0.2 kg，边长 0.08 m，用户可以根据实际负载调整这两个参数以获得更准确的动力学补偿。
    payload_mass_ = declare_parameter<double>("payload_mass", 0.2);
    payload_cube_side_ = declare_parameter<double>("payload_cube_side", 0.08);

    // 默认的 effort_limits 和 MIT 闭环参数。
    // 控制器本体不直接使用 Kp/Kd，但仍声明 loaded/unloaded 两套参数，
    // 便于仿真节点和真机电机驱动共享同一份 YAML 配置。
    const auto effort_defaults_limits = std::array<double, kDof>{10.0, 10.0, 10.0, 10.0};
    const auto kp_defaults = std::array<double, kDof>{20.0, 20.0, 15.0, 5.0};
    const auto kd_defaults = std::array<double, kDof>{1.5, 1.5, 1.0, 0.25};

    effort_limits_ = to_array4(
      declare_parameter<std::vector<double>>("effort_limits", std::vector<double>{
        effort_defaults_limits.begin(), effort_defaults_limits.end()}),
      effort_defaults_limits, 
      "effort_limits");
    kp_unloaded_ = to_array4(
      declare_parameter<std::vector<double>>("kp_unloaded", std::vector<double>{
        kp_defaults.begin(), kp_defaults.end()}),
      kp_defaults,
      "kp_unloaded");
    kd_unloaded_ = to_array4(
      declare_parameter<std::vector<double>>("kd_unloaded", std::vector<double>{
        kd_defaults.begin(), kd_defaults.end()}),
      kd_defaults,
      "kd_unloaded");
    kp_loaded_ = to_array4(
      declare_parameter<std::vector<double>>("kp_loaded", std::vector<double>{
        kp_defaults.begin(), kp_defaults.end()}),
      kp_defaults,
      "kp_loaded");
    kd_loaded_ = to_array4(
      declare_parameter<std::vector<double>>("kd_loaded", std::vector<double>{
        kd_defaults.begin(), kd_defaults.end()}),
      kd_defaults,
      "kd_loaded");

    // 从参数服务器读取 urdf_path 参数，如果用户没有提供，就使用 rc_arm2_description 包中的arm2.urdf
    auto urdf_path = declare_parameter<std::string>("urdf_path", "");
    if (urdf_path.empty()) {
      urdf_path = ament_index_cpp::get_package_share_directory("rc_arm2_description") +
        "/urdf/arm2.urdf";
    }
    load_dynamics_models(urdf_path);

    // 创建 ROS 发布者和订阅者，以及 FollowJointTrajectory action server。
    // effort_pub_ 发布关节 MIT 目标命令
    // "/arm2/command/effort"
    effort_pub_ = create_publisher<sensor_msgs::msg::JointState>(command_topic_, 10);
    // state_pub_ 发布控制器状态
    // "/arm2_controller/state"
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    // joint_state_sub_ 订阅关节状态
    // "/joint_states"
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20,
      std::bind(&Arm2TorqueTrajectoryController::joint_state_callback, this, std::placeholders::_1));
    
    // 负载切换流程：
    // 外部节点发布/arm2/payload_attached -> 控制器订阅/arm2/payload_attached，回调payload_callback()
    // 控制器切换switch_payload_locked成功 -> 发布/arm2/payload_active -> MuJoCo 启用/关闭 weld
    // payload_sub_ 订阅负载吸附状态，请求外部结点发布
    // "/arm2/payload_attached"
    payload_sub_ = create_subscription<std_msgs::msg::Bool>(
      payload_command_topic_, 10,
      std::bind(&Arm2TorqueTrajectoryController::payload_callback, this, std::placeholders::_1));
    // payload_active_pub_ 发布到mujoco，当前使用的模型是带负载还是空载
    payload_active_pub_ = create_publisher<std_msgs::msg::Bool>(payload_active_topic_, 10);

    // FollowJointTrajectory action server
    // 名字由 action_name_ 参数指定，默认 "/arm2_controller/follow_joint_trajectory"
    // 收到轨迹目标 -> handle_goal
    // 收到取消请求 -> handle_cancel
    // 目标被接受后 -> handle_accepted
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      action_name_,
      std::bind(
        &Arm2TorqueTrajectoryController::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &Arm2TorqueTrajectoryController::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &Arm2TorqueTrajectoryController::handle_accepted,
        this,
        std::placeholders::_1));

    // 创建 250 Hz 的控制定时器，周期由 control_rate_hz_ 参数指定。
    if (control_rate_hz_ <= 0.0) {
      throw std::runtime_error("control_rate_hz must be positive");
    }
    control_period_sec_ = 1.0 / control_rate_hz_;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_period_sec_));
    // control_tick() 是控制循环的入口函数，每个周期调用一次，根据当前状态执行相应的控制逻辑。
    control_timer_ = create_wall_timer(
      period,
      std::bind(&Arm2TorqueTrajectoryController::control_tick, this));
    // 初始化完成，进入 HOLDING 状态，等待接收轨迹目标。
    state_ = State::HOLDING;
    publish_payload_active();
    publish_state("initialized");
    RCLCPP_INFO(
      get_logger(),
      "arm2 torque controller ready. joints=[%s], action=%s, control_rate=%.1f Hz",
      join_names(joint_names_).c_str(), action_name_.c_str(), control_rate_hz_);
  }

private:
  // 控制器状态机的状态枚举，包含 STARTUP、HOLDING、EXECUTING 和 FAULT 四个状态。
  enum class State { STARTUP, HOLDING, EXECUTING, FAULT };

  // TargetSample 结构体表示轨迹上的一个采样点，包含时间 t、关节位置 q、关节速度 dq 和关节加速度 ddq。
  struct TargetSample
  {
    double t{0.0};
    std::array<double, kDof> q{};
    std::array<double, kDof> dq{};
    std::array<double, kDof> ddq{};
  };

  /**
   * @brief 从 URDF 构建 Pinocchio 空载模型和负载模型。
   *
   * @param urdf_path URDF 文件路径。
   *
   * 函数会校验四个受控关节是否存在，并缓存每个关节在 Pinocchio
   * q/v 向量中的索引，之后 RNEA 计算会使用这些索引填充状态量。
   *
   * @throw std::runtime_error 当 URDF 不包含足够自由度、缺少关节、
   * 或关节不是单自由度关节时抛出。
   */
  void load_dynamics_models(const std::string & urdf_path)
  {
    pinocchio::urdf::buildModel(urdf_path, unloaded_model_);
    if (unloaded_model_.nq < static_cast<int>(kDof) ||
      unloaded_model_.nv < static_cast<int>(kDof))
    {
      throw std::runtime_error("Pinocchio model does not contain enough DoFs");
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      if (!unloaded_model_.existJointName(joint_names_[i])) {
        throw std::runtime_error("URDF is missing joint " + joint_names_[i]);
      }
      const pinocchio::JointIndex joint_id = unloaded_model_.getJointId(joint_names_[i]);
      const auto & joint = unloaded_model_.joints[joint_id];
      if (joint.nq() != 1 || joint.nv() != 1) {
        throw std::runtime_error("Only single-DoF revolute joints are supported: " + joint_names_[i]);
      }
      q_indices_[i] = joint.idx_q();
      v_indices_[i] = joint.idx_v();
    }

    loaded_model_ = unloaded_model_;
    add_payload_to_loaded_model();

    unloaded_data_ = std::make_unique<pinocchio::Data>(unloaded_model_);
    loaded_data_ = std::make_unique<pinocchio::Data>(loaded_model_);
  }

  /**
   * @brief 在 loaded_model_ 上追加正方体负载惯量。
   *
   * 正方体通过吸盘吸附在一个面的面心，因此质心位于 tool0 坐标系
   * +Z 方向 payload_cube_side / 2 的位置。惯量按薄壁正方体表面均匀分布计算，
   * 忽略厚度时 Ixx = Iyy = Izz = (5/18) * m * a^2。
   *
   * @throw std::runtime_error 当 URDF 中不存在 tool0 frame、负载质量非法、
   * 或正方体边长非法时抛出。
   */
  void add_payload_to_loaded_model()
  {
    // tool0 是吸盘坐标系的唯一几何来源；如果 URDF 里没有它，直接失败，避免静默使用过期配置。
    if (!loaded_model_.existFrame(tool0_frame_name_)) {
      throw std::runtime_error("tool0 frame does not exist in URDF: " + tool0_frame_name_);
    }
    if (payload_mass_ <= 0.0) {
      throw std::runtime_error("payload_mass must be positive");
    }
    if (payload_cube_side_ <= 0.0) {
      throw std::runtime_error("payload_cube_side must be positive");
    }

    const double side = payload_cube_side_;
    // 薄壁正方体表面均匀分布、厚度可忽略时，三主惯量相同：
    // Ixx = Iyy = Izz = (5/18) * m * a^2。
    const double inertia_diag = (5.0 / 18.0) * payload_mass_ * side * side;
    const Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Identity() * inertia_diag;
    const pinocchio::Inertia payload_inertia(payload_mass_, Eigen::Vector3d::Zero(), inertia_com);

    const pinocchio::FrameIndex tool_frame_id = loaded_model_.getFrameId(tool0_frame_name_);
    const auto & tool_frame = loaded_model_.frames[tool_frame_id];
    const pinocchio::JointIndex payload_joint_id = tool_frame.parentJoint;
    const pinocchio::SE3 tool_placement = tool_frame.placement;

    const pinocchio::SE3 cube_center_from_tool(
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(0.0, -side * 0.5, 0.0));
      
    loaded_model_.appendBodyToJoint(
      payload_joint_id,
      payload_inertia,
      tool_placement * cube_center_from_tool);
  }

  /**
   * @brief 处理 FollowJointTrajectory action 的新目标请求。
   *
   * @param goal_uuid action 目标 UUID；当前实现不使用该值。
   * @param goal 待执行的轨迹目标。
   * @return rclcpp_action::GoalResponse ACCEPT_AND_EXECUTE 表示接受并执行；
   * REJECT 表示拒绝该目标。
   *
   * 只有在 HOLDING 状态且没有正在执行的目标时才接受新目标。
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::HOLDING || active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory goal because controller is not holding");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::string error;
    if (!validate_goal(*goal, error)) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory goal: %s", error.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief 处理 FollowJointTrajectory action 的取消请求。
   *
   * @param goal_handle 被请求取消的目标句柄；当前实现不需要读取其内容。
   * @return rclcpp_action::CancelResponse ACCEPT 表示接受取消请求。
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory>)
  {
    RCLCPP_INFO(get_logger(), "Trajectory cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief 处理已经被 action server 接受的目标。
   *
   * @param goal_handle 已接受目标的句柄。
   *
   * 函数会把 ROS 轨迹转换为控制器内部固定关节顺序的 trajectory_，
   * 然后记录起始时间并进入 EXECUTING 状态。
   */
  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string error;
    if (!prepare_trajectory(*goal_handle->get_goal(), error)) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
      result->error_string = error;
      goal_handle->abort(result);
      return;
    }

    active_goal_ = goal_handle;
    trajectory_start_time_ = now();
    last_feedback_time_ = trajectory_start_time_;
    saturation_time_.fill(0.0);
    state_ = State::EXECUTING;
    publish_state("goal accepted");
  }

  /**
   * @brief 校验 FollowJointTrajectory 目标是否满足控制器要求。
   *
   * @param goal 待校验的 action 目标。
   * @param error 输出参数；校验失败时写入失败原因。
   * @return bool true 表示目标合法，false 表示目标不合法。
   *
   * 本控制器要求轨迹包含 j1-j4，且每个轨迹点都必须提供位置、
   * 速度和加速度，因为 Pinocchio RNEA 需要 q、dq、ddq。
   */
  bool validate_goal(const FollowJointTrajectory::Goal & goal, std::string & error) const
  {
    const auto & trajectory = goal.trajectory;

    // 轨迹必须非空。
    if (trajectory.points.empty()) {
      error = "trajectory contains no points";
      return false;
    }

    // 轨迹必须包含 j1-j4 四个关节，且可以包含额外的关节，但不允许缺少受控关节。
    std::unordered_map<std::string, std::size_t> goal_joint_index;
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      goal_joint_index[trajectory.joint_names[i]] = i;
    }
    for (const auto & name : joint_names_) {
      if (goal_joint_index.find(name) == goal_joint_index.end()) {
        error = "trajectory is missing joint " + name;
        return false;
      }
    }

    double previous_t = -1.0;
    for (std::size_t p = 0; p < trajectory.points.size(); ++p) {
      const auto & point = trajectory.points[p];
      // 每个轨迹点必须包含与 joint_names 同样数量的 position、velocity 和 acceleration，
      if (point.positions.size() != trajectory.joint_names.size() ||
        point.velocities.size() != trajectory.joint_names.size() ||
        point.accelerations.size() != trajectory.joint_names.size())
      {
        error = "each point must contain position, velocity, and acceleration for every joint";
        return false;
      }
      const double t = duration_to_seconds(point.time_from_start);
      // 允许非常小的时间间隔，但不允许时间倒退或完全重复。
      if (t < previous_t) {
        error = "trajectory time_from_start must be monotonic";
        return false;
      }
      previous_t = t;
    }
    return true;
  }

  /**
   * @brief 将 ROS 轨迹转换为控制器内部轨迹格式。
   *
   * @param goal 已通过基本校验的 action 目标。
   * @param error 输出参数；转换失败时写入失败原因。
   * @return bool true 表示转换成功，false 表示转换失败。
   *
   * 转换时会根据 joint_names 做重排，确保内部 trajectory_ 始终按
   * j1、j2、j3、j4 的顺序存储。
   */
  bool prepare_trajectory(const FollowJointTrajectory::Goal & goal, std::string & error)
  {
    // 校验轨迹合法性
    if (!validate_goal(goal, error)) {
      return false;
    }

    // 建立轨迹关节名到索引的映射，方便后续重排。轨迹中的关节顺序可能不固定，甚至包含额外关节，
    std::unordered_map<std::string, std::size_t> goal_joint_index;
    for (std::size_t i = 0; i < goal.trajectory.joint_names.size(); ++i) {
      goal_joint_index[goal.trajectory.joint_names[i]] = i;
    }

    trajectory_.clear();
    trajectory_.reserve(goal.trajectory.points.size());
    for (const auto & point : goal.trajectory.points) {
      TargetSample sample;
      sample.t = duration_to_seconds(point.time_from_start);
      for (std::size_t j = 0; j < kDof; ++j) {
        const auto src = goal_joint_index[joint_names_[j]];
        sample.q[j] = point.positions[src];
        sample.dq[j] = point.velocities[src];
        sample.ddq[j] = point.accelerations[src];
      }
      trajectory_.push_back(sample);
    }
    trajectory_duration_sec_ = trajectory_.back().t;
    return true;
  }

  /**
   * @brief 处理 /joint_states，更新当前关节位置和速度。
   *
   * @param msg sensor_msgs::msg::JointState 消息。
   *
   * /joint_states 可能包含额外关节，且顺序不固定，因此这里会按
   * joint_names_ 查找 j1-j4 并重排到 current_q_ / current_dq_。
   * 根据关节名字符串重新排序
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, kDof> q{};
    std::array<double, kDof> dq{};

    std::unordered_map<std::string, std::size_t> state_index;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      state_index[msg->name[i]] = i;
    }
    for (std::size_t j = 0; j < kDof; ++j) {
      const auto it = state_index.find(joint_names_[j]);
      if (it == state_index.end() || it->second >= msg->position.size()) {
        return;
      }
      q[j] = msg->position[it->second];
      dq[j] = it->second < msg->velocity.size() ? msg->velocity[it->second] : 0.0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    current_q_ = q;
    current_dq_ = dq;
    have_joint_state_ = true;
    if (!hold_initialized_) {
      hold_q_ = q;
      hold_initialized_ = true;
      publish_state("first joint state received");
    }
  }

  /**
   * @brief 处理负载吸附状态请求。
   *
   * @param msg std_msgs::msg::Bool；true 表示请求切换到带负载模型，
   * false 表示请求切换到空载模型。
   * 
   * 外部节点发布/arm2/payload_attached -> 控制器订阅/arm2/payload_attached，回调payload_callback()
   * 控制器切换switch_payload_locked成功 -> 发布/arm2/payload_active -> MuJoCo 启用/关闭 weld
   *
   * 如果当前处于 HOLDING，立即切换模型；如果正在 EXECUTING，
   * 只记录 pending_payload_switch_，等回到 HOLDING 后再切换。
   */
  void payload_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_payload_attached_ = msg->data;
    // 如果当前请求的模型和正在使用的模型已经一致，就不需要切换了。
    if (desired_payload_attached_ == active_payload_attached_) {
      pending_payload_switch_ = false;
      return;
    }
    // 如果当前正在保持状态，直接切换模型；如果正在执行轨迹，先记录切换请求，等执行完回到保持状态时再切换。
    if (state_ == State::HOLDING) {
      switch_payload_locked();
    } else {
      pending_payload_switch_ = true;
      publish_state("payload switch pending");
    }
  }

  /**
   * @brief 250 Hz 控制循环入口。可在arm2_control.yaml中修改
   *
   * 该函数由 wall timer 周期调用。根据当前状态选择执行轨迹跟踪
   * 或静态保持，并在合适时处理延迟的负载模型切换。
   */
  void control_tick()
  {
    // 加锁，防止这个函数和 joint_state_callback()、payload_callback()、action 回调同时修改共享变量。
    std::lock_guard<std::mutex> lock(mutex_);
    // 没有首次关节状态时无法执行控制，直接返回。
    if (!have_joint_state_) {
      return;
    }
    // 如果当前是保持状态，并且之前有负载切换请求，就在这里切换动力学模型。比如从空载模型切到带负载模型。
    if (state_ == State::HOLDING && pending_payload_switch_) {
      switch_payload_locked();
    }

    TargetSample target;
    if (state_ == State::EXECUTING && active_goal_) {
      if (active_goal_->is_canceling()) {
        finish_cancel_locked("cancel requested");
        return;
      }
      const double elapsed = (now() - trajectory_start_time_).seconds();
      target = sample_trajectory(elapsed);
      // 根据目标点计算前馈力矩，并以 JointState 发布 MIT 目标命令到：/arm2/command/effort
      const bool command_ok = compute_and_publish_torque(target);
      // 向 action client 发布反馈，告诉它当前 desired、actual、error。
      publish_feedback_locked(target);
      if (!command_ok) {
        finish_abort_locked(
          control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED,
          "feedforward torque saturation exceeded allowed duration");
        return;
      }

      if (elapsed >= trajectory_duration_sec_ + settle_time_sec_) {
        // 轨迹执行时间已超过最后一个点的时间加上稳定时间，如果此时位置误差在容忍范围内，就认为执行成功。
        // 转到 HOLDING 状态，继续保持在目标位置，等待下一个目标。
        finish_goal_if_tolerant_locked(target);
      }
      return;
    }

    if (state_ == State::HOLDING) {
      target = hold_target();
      const bool command_ok = compute_and_publish_torque(target);
      if (!command_ok) {
        state_ = State::FAULT;
        publish_state("holding torque saturation fault");
      }
    }
  }

  /**
   * @brief 生成静态保持目标。
   *
   * @return TargetSample 位置为 hold_q_，速度和加速度为 0 的目标点。
   */
  TargetSample hold_target() const
  {
    TargetSample target;
    target.q = hold_q_;
    target.dq.fill(0.0);
    target.ddq.fill(0.0);
    return target;
  }

  /**
   * @brief 根据执行时间从内部轨迹中采样目标点。
   *
   * @param elapsed 从当前 action 目标开始执行到现在的时间，单位秒。
   * @return TargetSample 当前时刻对应的 q、dq、ddq 目标。
   *
   * 轨迹点之间使用线性插值。轨迹的平滑性主要依赖上游
   * MoveIt/时间参数化模块生成连续的 q、dq、ddq。
   */
  TargetSample sample_trajectory(double elapsed) const
  {
    // 如果当前没有轨迹，就返回保持目标，也就是让机械臂保持在 hold_q_ 的位置，速度和加速度为 0。
    if (trajectory_.empty()) {
      return hold_target();
    }
    // 时间误差的修正
    // 如果当前时间还没到第一个轨迹点，就直接返回第一个点。
    if (elapsed <= trajectory_.front().t) {
      return trajectory_.front();
    }
    // 如果当前时间已经超过最后一个轨迹点，就返回最后一个点。
    if (elapsed >= trajectory_.back().t) {
      return trajectory_.back();
    }
    // 在 trajectory_ 里找第一个时间 sample.t 大于 elapsed 的轨迹点。
    // 轨迹时间点 0.0s, 1.0s, 2.0s, 3.0s
    // elapsed = 1.4 那么 upper 会指向 2.0s 那个点。
    auto upper = std::upper_bound(
      trajectory_.begin(), trajectory_.end(), elapsed,
      [](double t, const TargetSample & sample) {return t < sample.t;});
    // 现在位置在 upper 前一个点和 upper 之间，做线性插值。
    const auto & b = *upper;
    const auto & a = *(upper - 1);
    // 计算时间比例，作为路程比例alpha
    const double span = std::max(b.t - a.t, kMinPositive);
    const double alpha = std::clamp((elapsed - a.t) / span, 0.0, 1.0);

    // 对每个关节的 q、dq、ddq 做线性插值，得到当前时刻的目标点。
    TargetSample out;
    out.t = elapsed;
    for (std::size_t i = 0; i < kDof; ++i) {
      out.q[i] = a.q[i] + alpha * (b.q[i] - a.q[i]);
      out.dq[i] = a.dq[i] + alpha * (b.dq[i] - a.dq[i]);
      out.ddq[i] = a.ddq[i] + alpha * (b.ddq[i] - a.ddq[i]);
    }
    return out;
  }

  /**
   * @brief 计算并发布当前目标点对应的 MIT 目标命令。
   *
   * @param target 当前控制周期的目标 q、dq、ddq。
   * @return bool true 表示前馈力矩计算和发布正常；false 表示出现非有限值
   * 或连续前馈力矩饱和时间超过阈值。
   *
   * 这里只计算前馈力矩 tau_ff = RNEA(qd, dqd, ddqd)，并通过 JointState 发布：
   * name=joint_names，position=qd，velocity=dqd，effort=tau_ff。
   * MIT 的 Kp/Kd 误差反馈在 MuJoCo 仿真节点中完成。
   */
  bool compute_and_publish_torque(const TargetSample & target)
  {
    // 根据当前是否带负载选择对应的 Pinocchio 模型和数据。
    const pinocchio::Model & model = active_payload_attached_ ? loaded_model_ : unloaded_model_;
    pinocchio::Data & data = active_payload_attached_ ? *loaded_data_ : *unloaded_data_;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    for (std::size_t i = 0; i < kDof; ++i) {
      q[q_indices_[i]] = target.q[i];
      v[v_indices_[i]] = target.dq[i];
      a[v_indices_[i]] = target.ddq[i];
    }

    // 前馈力矩计算
    const Eigen::VectorXd feedforward = pinocchio::rnea(model, data, q, v, a);
    std::array<double, kDof> command{};
    bool saturated = false;
    for (std::size_t i = 0; i < kDof; ++i) {
      double tau = feedforward[v_indices_[i]];

      if (!std::isfinite(tau)) {
        state_ = State::FAULT;
        publish_state("non-finite feedforward torque");
        return false;
      }

      const double limit = std::abs(effort_limits_[i]);
      if (limit > 0.0 && std::abs(tau) > limit) {
        tau = std::clamp(tau, -limit, limit);
        saturation_time_[i] += control_period_sec_;
        saturated = true;
      } else {
        saturation_time_[i] = 0.0;
      }
      command[i] = tau;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.assign(target.q.begin(), target.q.end());
    msg.velocity.assign(target.dq.begin(), target.dq.end());
    msg.effort.assign(command.begin(), command.end());
    effort_pub_->publish(msg);

    if (saturated) {
      for (double t : saturation_time_) {
        if (t >= saturation_abort_sec_) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * @brief 发布 FollowJointTrajectory action feedback。
   *
   * @param desired 当前控制周期的期望目标点。
   *
   * feedback 中包含 desired、actual 和 error 三组关节状态。
   * 发布频率由 feedback_rate_hz_ 控制，避免每个控制周期都发布。
   */
  void publish_feedback_locked(const TargetSample & desired)
  {
    if (!active_goal_) {
      return;
    }
    const double feedback_period = feedback_rate_hz_ > 0.0 ? 1.0 / feedback_rate_hz_ : 0.05;
    if ((now() - last_feedback_time_).seconds() < feedback_period) {
      return;
    }
    last_feedback_time_ = now();

    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->header.stamp = last_feedback_time_;
    feedback->joint_names = joint_names_;
    // 填入目标desire
    fill_point(feedback->desired, desired);
    
    for (std::size_t i = 0; i < kDof; ++i) {
      // 填入实际
      feedback->actual.positions.push_back(current_q_[i]);
      feedback->actual.velocities.push_back(current_dq_[i]);
      feedback->actual.accelerations.push_back(0.0);
      // 填入误差
      feedback->error.positions.push_back(desired.q[i] - current_q_[i]);
      feedback->error.velocities.push_back(desired.dq[i] - current_dq_[i]);
      // 由于没有直接测量加速度，这里暂时把加速度误差设为 0，实际应用中可以考虑使用数值微分或其他方法估计。
      feedback->error.accelerations.push_back(0.0);
    }
    // 发布 feedback 给 action client，帮助上游监控执行状态和误差。
    active_goal_->publish_feedback(feedback);
  }

  /**
   * @brief 将内部 TargetSample 填入 ROS JointTrajectoryPoint。
   * @brief 用于反馈给action
   *
   * @param point 输出参数；被写入 positions、velocities、accelerations。
   * @param sample 内部目标点。
   */
  void fill_point(JointTrajectoryPoint & point, const TargetSample & sample) const
  {
    point.positions.assign(sample.q.begin(), sample.q.end());
    point.velocities.assign(sample.dq.begin(), sample.dq.end());
    point.accelerations.assign(sample.ddq.begin(), sample.ddq.end());
  }

  /**
   * @brief 判断轨迹结束后是否满足目标误差，并结束 action。
   *
   * @param final_target 轨迹末端目标点。
   *
   * 如果最大关节误差小于 goal_tolerance_rad_，则 action succeed；
   * 否则 action abort。
   * 无论成功与否，hold_q_ 都会设置为末端目标位置, 进入 HOLDING 状态。
   */
  void finish_goal_if_tolerant_locked(const TargetSample & final_target)
  {
    hold_q_ = final_target.q;
    double max_error = 0.0;
    std::ostringstream error_details;
    for (std::size_t i = 0; i < kDof; ++i) {
      const double joint_error = final_target.q[i] - current_q_[i];
      max_error = std::max(max_error, std::abs(joint_error));
      if (i != 0) {
        error_details << ", ";
      }
      error_details << joint_names_[i] << "=" << joint_error;
    }
    // 如果最大位置误差在容忍范围内，认为目标达成，succeed action；
    if (max_error <= goal_tolerance_rad_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
      result->error_string = "goal reached";
      active_goal_->succeed(result);
      // 清除 active_goal_，表示当前没有正在执行的目标了。
      active_goal_.reset();
      // 成功结束后进入 HOLDING 状态，继续保持在目标位置，等待下一个目标。
      state_ = State::HOLDING;
      publish_state("goal succeeded");
    } 
    // 否则认为执行失败，abort action。
    else {
      std::ostringstream stream;
      stream << "goal tolerance violated, max error=" << max_error <<
        ", joint_errors=[" << error_details.str() << "]";
      RCLCPP_WARN(get_logger(), "%s", stream.str().c_str());
      finish_abort_locked(
        control_msgs::action::FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED,
        stream.str());
    }
  }

  /**
   * @brief 中止当前 action 目标并回到 HOLDING。
   *
   * @param error_code FollowJointTrajectory result 的错误码。
   * @param error 错误描述字符串。
   *
   * 中止后 hold_q_ 会设置为当前实测关节位置，避免继续追踪失败目标。
   */
  void finish_abort_locked(int32_t error_code, const std::string & error)
  {
    if (active_goal_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = error_code;
      result->error_string = error;
      active_goal_->abort(result);
      active_goal_.reset();
    }
    hold_q_ = current_q_;
    state_ = State::HOLDING;
    saturation_time_.fill(0.0);
    publish_state(error);
  }

  /**
   * @brief 取消当前 action 目标并回到 HOLDING。
   *
   * @param reason 取消原因描述。
   *
   * 取消后 hold_q_ 会设置为当前实测关节位置。
   */
  void finish_cancel_locked(const std::string & reason)
  {
    if (active_goal_) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
      result->error_string = reason;
      active_goal_->canceled(result);
      active_goal_.reset();
    }
    hold_q_ = current_q_;
    state_ = State::HOLDING;
    saturation_time_.fill(0.0);
    publish_state(reason);
  }

  /**
   * @brief 执行负载模型切换。
   *
   * 这是唯一修改 active_payload_attached_ 的函数。切换后会发布
   * /arm2/payload_active，MuJoCo 仿真节点据此启用或关闭正方体 weld。
   * 
   * 外部节点发布/arm2/payload_attached -> 控制器订阅/arm2/payload_attached，回调payload_callback()
   * 控制器切换switch_payload_locked成功 -> publish_payload_active发布/arm2/payload_active 
   * -> MuJoCo 启用/关闭 weld
   */
  void switch_payload_locked()
  {
    active_payload_attached_ = desired_payload_attached_;
    pending_payload_switch_ = false;
    saturation_time_.fill(0.0);
    publish_payload_active();
    publish_state(active_payload_attached_ ? "payload attached" : "payload detached");
  }

  /**
   * @brief 发布当前实际生效的负载状态。
   *
   * 发布到 payload_active_topic_，用于通知仿真或其他节点当前控制器
   * 实际使用的是空载模型还是带负载模型。
   * 
   * 外部节点发布/arm2/payload_attached -> 控制器订阅/arm2/payload_attached，回调payload_callback()
   * 控制器切换switch_payload_locked成功 -> publish_payload_active发布/arm2/payload_active 
   * -> MuJoCo 启用/关闭 weld
   */
  void publish_payload_active()
  {
    std_msgs::msg::Bool msg;
    msg.data = active_payload_attached_;
    payload_active_pub_->publish(msg);
  }

  /**
   * @brief 发布控制器状态字符串。
   *
   * @param detail 附加说明，例如初始化完成、目标接受、负载切换等。
   */
  void publish_state(const std::string & detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream stream;
    stream << "state=" << state_to_string(state_)
           << " payload=" << (active_payload_attached_ ? "attached" : "detached")
           << " pending_payload=" << (pending_payload_switch_ ? "true" : "false")
           << " detail=\"" << detail << "\"";
    msg.data = stream.str();
    state_pub_->publish(msg);
  }

  /**
   * @brief 将状态枚举转换为可读字符串。
   *
   * @param state 控制器状态枚举。
   * @return const char* 状态名称字符串。
   */
  const char * state_to_string(State state) const
  {
    switch (state) {
      case State::STARTUP:
        return "STARTUP";
      case State::HOLDING:
        return "HOLDING";
      case State::EXECUTING:
        return "EXECUTING";
      case State::FAULT:
        return "FAULT";
    }
    return "UNKNOWN";
  }

  std::mutex mutex_;
  // 默认启动后处于 STARTUP 状态，等待首次关节状态消息后切换到 HOLDING。
  State state_{State::STARTUP};
  // 受控关节名称列表，必须与 URDF 中的关节名一致，且必须包含 j1-j4。
  std::vector<std::string> joint_names_;
  // ROS 话题和 action 名称配置参数。
  std::string action_name_;
  std::string command_topic_;
  std::string joint_state_topic_;
  // 控制器订阅外部负载状态请求的 topic， /arm2/payload_attached。
  std::string payload_command_topic_;
  // 控制器发布（给mujoco）当前实际负载状态的 topic， /arm2/payload_active。
  std::string payload_active_topic_;
  // 发布控制器状态的 topic，类型为 std_msgs::msg::String，
  // 包含 "initialized"、"goal accepted"、"payload switch pending"、"holding torque saturation fault" 等状态信息
  std::string state_topic_;
  // URDF 中吸盘工具坐标系名称；负载位置只从该 frame 读取，不再从 YAML 配置几何偏移。
  std::string tool0_frame_name_{"tool0"};

  // 控制参数配置，均可通过 YAML 文件覆盖。
  // 控制周期频率，单位 Hz，默认 250 Hz
  double control_rate_hz_{250.0};
  // 控制周期时间，单位秒，由 control_rate_hz_ 计算得出，默认 0.004 秒。
  double control_period_sec_{0.004};
  // 发布 action feedback 的频率，单位 Hz，默认 20 Hz。
  double feedback_rate_hz_{20.0};
  // 力矩饱和持续时间阈值，单位秒，默认 0.25 秒。如果连续超过这个时间，认为控制失败。
  double saturation_abort_sec_{0.25};
  // 轨迹末端位置误差容忍度，单位弧度，默认 0.02 rad（约 1.15 度）。如果轨迹执行结束时最大位置误差超过这个值，认为执行失败。
  double goal_tolerance_rad_{0.02};
  // 轨迹末端稳定时间，单位秒，默认 0.5 秒，执行时间=轨迹时间+稳定时间，结束后判断是否满足误差容忍度。
  double settle_time_sec_{0.5};
  // 负载参数，必须与仿真中正方体的质量和尺寸一致，才能保证动力学计算正确。
  double payload_mass_{0.2};
  // 正方体边长，单位米，默认 0.08 m（8 cm）。根据质量和尺寸计算惯量。
  double payload_cube_side_{0.08};
  std::array<double, kDof> effort_limits_{10.0, 10.0, 10.0, 10.0};
  std::array<double, kDof> kp_unloaded_{20.0, 20.0, 15.0, 5.0};
  std::array<double, kDof> kd_unloaded_{1.5, 1.5, 1.0, 0.25};
  std::array<double, kDof> kp_loaded_{20.0, 20.0, 15.0, 5.0};
  std::array<double, kDof> kd_loaded_{1.5, 1.5, 1.0, 0.25};
  // 每个关节的力矩饱和持续时间，初始为 0。如果某个关节的饱和时间超过 saturation_abort_sec_，就认为控制失败
  std::array<double, kDof> saturation_time_{0.0, 0.0, 0.0, 0.0};

  pinocchio::Model unloaded_model_;
  pinocchio::Model loaded_model_;
  std::unique_ptr<pinocchio::Data> unloaded_data_;
  std::unique_ptr<pinocchio::Data> loaded_data_;
  std::array<int, kDof> q_indices_{};
  std::array<int, kDof> v_indices_{};

  bool have_joint_state_{false};
  bool hold_initialized_{false};
  bool active_payload_attached_{false};
  bool desired_payload_attached_{false};
  bool pending_payload_switch_{false};
  std::array<double, kDof> current_q_{0.0, 0.0, 0.0, 0.0};
  std::array<double, kDof> current_dq_{0.0, 0.0, 0.0, 0.0};
  std::array<double, kDof> hold_q_{0.0, 0.0, 0.0, 0.0};

  std::vector<TargetSample> trajectory_;
  double trajectory_duration_sec_{0.0};
  rclcpp::Time trajectory_start_time_;
  rclcpp::Time last_feedback_time_;
  // 当前正在执行的 action 目标句柄；如果没有正在执行的目标，则为 nullptr。
  std::shared_ptr<GoalHandleFollowJointTrajectory> active_goal_;

  // 发布 MIT 目标命令的 publisher，发布到 command_topic_，
  // 消息类型为 sensor_msgs::msg::JointState：
  // name=关节名，position=期望位置，velocity=期望速度，effort=期望前馈力矩。
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr effort_pub_;
  // 发布当前负载状态的 publisher，发布到 payload_active_topic_，消息类型为 std_msgs::msg::Bool。
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_active_pub_;
  // 发布控制器状态的 publisher，发布到 state_topic_，消息类型为 std_msgs::msg::String。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  // 订阅关节状态的 subscriber，订阅 joint_state_topic_，消息类型为 sensor_msgs::msg::JointState。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  // 订阅负载状态请求的 subscriber，订阅 payload_command_topic_，消息类型为 std_msgs::msg::Bool。
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_sub_;
  // FollowJointTrajectory action server，服务器名称为 action_name_。
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  // 控制循环的 wall timer，周期由 control_rate_hz_ 决定。
  rclcpp::TimerBase::SharedPtr control_timer_;
};

/**
 * @brief 控制器节点入口函数。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return int 0 表示正常退出，1 表示启动或运行时发生异常。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Arm2TorqueTrajectoryController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("arm2_torque_trajectory_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
