#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rc_arm2_moveit_client/action/plan_to_task_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{
// 机械臂当前规划/控制的自由度数量。
constexpr std::size_t kDof = 4;
// MoveIt joint target 和控制器轨迹都使用同一组关节名。
const std::array<std::string, kDof> kJointNames = {"j1", "j2", "j3", "j4"};

using PlanToTaskGoal = rc_arm2_moveit_client::action::PlanToTaskGoal;
using GoalHandlePlanToTaskGoal = rclcpp_action::ServerGoalHandle<PlanToTaskGoal>;

/**
 * @brief 读取文本文件的全部内容。
 *
 * @param path 文件路径，通常是 URDF 或 SRDF 的绝对路径。
 * @return std::string 文件完整文本内容。
 *
 * @throw std::runtime_error 当文件无法打开时抛出。
 */
std::string read_text_file(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Failed to open file: " + path);
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

/**
 * @brief 获取 ROS 参数；如果参数不存在则声明并使用默认值。
 *
 * @tparam T 参数类型，例如 std::string、double、bool、std::vector<double>。
 * @param node 当前 ROS 2 节点。
 * @param name 参数名。
 * @param default_value 参数不存在时使用的默认值。
 * @return T 参数值。
 */
template<typename T>
T get_or_declare(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  T value{};
  node->get_parameter(name, value);
  return value;
}

/**
 * @brief 设置字符串参数；如果参数还不存在则先声明。
 *
 * @param node 当前 ROS 2 节点。
 * @param name 参数名。
 * @param value 要写入的字符串值。
 * @return void 无返回值。
 *
 * 这里用于把 URDF/SRDF 文本写入 `robot_description`
 * 和 `robot_description_semantic`，供 MoveGroupInterface 在本节点内加载机器人模型。
 */
void set_or_declare(const rclcpp::Node::SharedPtr & node, const std::string & name, const std::string & value)
{
  if (node->has_parameter(name)) {
    node->set_parameter(rclcpp::Parameter(name, value));
  } else {
    node->declare_parameter<std::string>(name, value);
  }
}

/**
 * @brief 将角度归一化到 [-pi, pi]。
 *
 * @param angle 输入角度，单位 rad。
 * @return double 归一化后的角度，单位 rad。
 *
 * 自转角 spin 是周期量，直接相减可能出现 2*pi 跳变，因此误差需要归一化。
 */
double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief 将关节向量限制在 Pinocchio 模型的关节上下限内。
 *
 * @param model Pinocchio 机器人模型，内部包含 lowerPositionLimit 和 upperPositionLimit。
 * @param q 输入关节向量，会按值传入并在函数内修正。
 * @return Eigen::VectorXd 限幅后的关节向量。
 */
Eigen::VectorXd clamp_to_limits(const pinocchio::Model & model, Eigen::VectorXd q)
{
  for (int i = 0; i < q.size(); ++i) {
    q[i] = std::clamp(q[i], model.lowerPositionLimit[i], model.upperPositionLimit[i]);
  }
  return q;
}

/**
 * @brief 检查 MoveIt 规划出的轨迹是否包含速度和加速度。
 *
 * @param trajectory MoveIt 返回的机器人轨迹。
 * @return bool true 表示每个轨迹点都包含至少 4 个位置、速度、加速度；false 表示不完整。
 *
 * 自定义力矩控制器需要 q、dq、ddq 来做 RNEA 前馈，因此缺少速度或加速度时不能执行。
 */
bool trajectory_has_velocity_and_acceleration(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  if (trajectory.joint_trajectory.points.empty()) {
    return false;
  }
  for (const auto & point : trajectory.joint_trajectory.points) {
    if (point.positions.size() < kDof ||
      point.velocities.size() < kDof ||
      point.accelerations.size() < kDof)
    {
      return false;
    }
  }
  return true;
}
}  // namespace

class Arm2TaskGoalPlanner
{
public:
  /**
   * @brief 构造 4D 任务目标规划器。
   *
   * @param node ROS 2 节点，用于读取参数、写入 robot_description、创建 MoveGroupInterface 和输出日志。
   *
   * 构造过程中会加载 URDF/SRDF、建立 Pinocchio 模型、检查 tool0 frame，
   * 并创建 MoveIt 的 MoveGroupInterface。具体目标由 `/arm2_task_goal` action 提供。
   *
   * @throw std::runtime_error 当 URDF/SRDF 无法读取，或 URDF 中不存在 tool0 frame 时抛出。
   */
  explicit Arm2TaskGoalPlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    const auto description_share = ament_index_cpp::get_package_share_directory("rc_arm2_description");
    const auto moveit_share = ament_index_cpp::get_package_share_directory("arm2_moveit_config");
    urdf_path_ = get_or_declare<std::string>(
      node_, "urdf_path", description_share + "/urdf/arm2.urdf");
    srdf_path_ = get_or_declare<std::string>(
      node_, "srdf_path", moveit_share + "/config/arm2.srdf");
    group_name_ = get_or_declare<std::string>(node_, "group_name", "arm");
    tool0_frame_name_ = get_or_declare<std::string>(node_, "tool0_frame_name", "tool0");
    action_name_ = get_or_declare<std::string>(node_, "action_name", "/arm2_task_goal");
    planning_time_ = get_or_declare<double>(node_, "planning_time", 5.0);
    num_ik_seed_attempts_ = get_or_declare<int>(node_, "num_ik_seed_attempts", 10);
    max_ik_iterations_ = get_or_declare<int>(node_, "max_ik_iterations", 100);
    position_tolerance_ = get_or_declare<double>(node_, "position_tolerance", 0.002);
    spin_tolerance_ = get_or_declare<double>(node_, "spin_tolerance", 0.02);
    damping_ = get_or_declare<double>(node_, "ik_damping", 1.0e-3);
    finite_difference_eps_ = get_or_declare<double>(node_, "finite_difference_eps", 1.0e-6);
    max_step_ = get_or_declare<double>(node_, "max_ik_step", 0.15);

    set_or_declare(node_, "robot_description", read_text_file(urdf_path_));
    set_or_declare(node_, "robot_description_semantic", read_text_file(srdf_path_));

    pinocchio::urdf::buildModel(urdf_path_, model_);
    if (!model_.existFrame(tool0_frame_name_)) {
      throw std::runtime_error("URDF/Pinocchio model does not contain frame: " + tool0_frame_name_);
    }
    tool0_frame_id_ = model_.getFrameId(tool0_frame_name_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
    move_group_->startStateMonitor(2.0);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setStartStateToCurrentState();
  }

  /**
   * @brief 启动常驻 action server，用于连续接收多个 4D 任务目标。
   *
   * @return void 无返回值。
   *
   * action 名称由参数 `action_name` 指定，默认 `/arm2_task_goal`。
   * 外部可以反复发送目标，每个目标都会触发一次 IK、MoveIt 规划和可选执行。
   */
  void start_action_server()
  {
    action_server_ = rclcpp_action::create_server<PlanToTaskGoal>(
      node_,
      action_name_,
      std::bind(&Arm2TaskGoalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Arm2TaskGoalPlanner::handle_cancel, this, std::placeholders::_1),
      std::bind(&Arm2TaskGoalPlanner::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "arm2 task goal action server ready: %s",
      action_name_.c_str());
  }

private:
  struct PlanOutcome
  {
    bool success{false};
    int error_code{1};
    std::string message;
  };

  /**
   * @brief 执行一次 4D IK、MoveIt 规划和可选执行。
   *
   * @param target_xyz 目标 tool0 位置，单位 m，坐标系为 base_link，长度必须为 3。
   * @param target_spin 目标 tool0 绕自身 +Z 轴的自转角，单位 rad。
   * @param execute true 表示规划成功后执行轨迹；false 表示只规划不执行。
   * @param feedback 可选 action feedback 指针；非空时会发布求得的 q_goal 和当前阶段文字。
   * @return PlanOutcome 包含是否成功、错误码和描述信息。
   */
  PlanOutcome plan_and_execute(
    const std::vector<double> & target_xyz,
    double target_spin,
    bool execute,
    const std::shared_ptr<PlanToTaskGoal::Feedback> & feedback)
  {
    if (target_xyz.size() != 3) {
      return {false, 1, "target_xyz must contain exactly 3 values"};
    }

    const Eigen::Vector4d target_task(target_xyz[0], target_xyz[1], target_xyz[2], target_spin);
    const auto seed = current_joint_seed();

    Eigen::VectorXd q_goal;
    if (!solve_ik(target_task, seed, q_goal)) {
      std::ostringstream message;
      message << "4D IK failed for target_xyz=[" << target_xyz[0] << " " << target_xyz[1] << " " <<
        target_xyz[2] << "], target_spin=" << target_spin;
      RCLCPP_ERROR(node_->get_logger(), "%s", message.str().c_str());
      return {false, 2, message.str()};
    }

    std::map<std::string, double> joint_target;
    for (std::size_t i = 0; i < kDof; ++i) {
      joint_target[kJointNames[i]] = q_goal[static_cast<int>(i)];
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "IK q_goal=[%.4f %.4f %.4f %.4f]",
      q_goal[0], q_goal[1], q_goal[2], q_goal[3]);
    publish_feedback(feedback, "IK solved", q_goal);

    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(joint_target)) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt rejected joint target");
      return {false, 3, "MoveIt rejected joint target"};
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    publish_feedback(feedback, "planning", q_goal);
    const auto plan_result = move_group_->plan(plan);
    if (!plan_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed. error_code=%d", plan_result.val);
      return {false, 4, "MoveIt planning failed"};
    }

    if (!trajectory_has_velocity_and_acceleration(plan.trajectory_)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Planned trajectory is missing velocity or acceleration fields; refusing to execute");
      return {false, 5, "Planned trajectory is missing velocity or acceleration fields"};
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "MoveIt planning succeeded with %zu trajectory points. execute=%s",
      plan.trajectory_.joint_trajectory.points.size(), execute ? "true" : "false");
    publish_feedback(feedback, "planning succeeded", q_goal);

    if (!execute) {
      return {true, 0, "MoveIt planning succeeded"};
    }

    publish_feedback(feedback, "executing", q_goal);
    const auto exec_result = move_group_->execute(plan);
    if (!exec_result) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed. error_code=%d", exec_result.val);
      return {false, 6, "MoveIt execution failed"};
    }
    RCLCPP_INFO(node_->get_logger(), "MoveIt execution succeeded");
    publish_feedback(feedback, "execution succeeded", q_goal);
    return {true, 0, "MoveIt execution succeeded"};
  }

  /**
   * @brief 发送 action feedback。
   *
   * @param feedback action feedback 消息；为空时不发布。
   * @param state 当前阶段描述，例如 `planning` 或 `executing`。
   * @param q_goal 当前 IK 求得的关节目标。
   * @return void 无返回值。
   */
  void publish_feedback(
    const std::shared_ptr<PlanToTaskGoal::Feedback> & feedback,
    const std::string & state,
    const Eigen::VectorXd & q_goal) const
  {
    if (!feedback) {
      return;
    }
    feedback->state = state;
    for (std::size_t i = 0; i < kDof; ++i) {
      feedback->q_goal[i] = q_goal[static_cast<int>(i)];
    }
  }

  /**
   * @brief 处理新的 action goal 请求。
   *
   * @param uuid action goal UUID；当前只用于接口匹配，不参与逻辑。
   * @param goal 外部发送的 4D 任务目标。
   * @return rclcpp_action::GoalResponse ACCEPT_AND_EXECUTE 表示接受；REJECT 表示当前忙或目标非法。
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanToTaskGoal::Goal> goal)
  {
    (void)uuid;
    if (action_busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting task goal because another goal is still running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Accepted task goal target_xyz=[%.4f %.4f %.4f], target_spin=%.4f, execute=%s",
      goal->target_xyz[0], goal->target_xyz[1], goal->target_xyz[2], goal->target_spin,
      goal->execute ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief 处理 action cancel 请求。
   *
   * @param goal_handle 待取消的 goal handle。
   * @return rclcpp_action::CancelResponse ACCEPT 表示接受取消请求。
   *
   * MoveIt 执行阶段收到取消请求时调用 stop() 尝试停止当前运动。
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePlanToTaskGoal> goal_handle)
  {
    (void)goal_handle;
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief action goal 被接受后的入口。
   *
   * @param goal_handle 已接受的 goal handle。
   * @return void 无返回值。
   *
   * 为避免阻塞 action server 回调线程，这里为每个 goal 启动一个工作线程。
   */
  void handle_accepted(const std::shared_ptr<GoalHandlePlanToTaskGoal> goal_handle)
  {
    std::thread{std::bind(&Arm2TaskGoalPlanner::execute_action_goal, this, goal_handle)}.detach();
  }

  /**
   * @brief 执行一个 action goal 并写入 result。
   *
   * @param goal_handle 待执行的 goal handle。
   * @return void 无返回值。
   */
  void execute_action_goal(const std::shared_ptr<GoalHandlePlanToTaskGoal> goal_handle)
  {
    const auto clear_busy = [this]() {action_busy_ = false;};
    auto result = std::make_shared<PlanToTaskGoal::Result>();

    try {
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<PlanToTaskGoal::Feedback>();

      const std::vector<double> target_xyz{
        goal->target_xyz[0],
        goal->target_xyz[1],
        goal->target_xyz[2],
      };

      publish_feedback(feedback, "accepted", Eigen::VectorXd::Zero(model_.nq));
      goal_handle->publish_feedback(feedback);

      const auto outcome = plan_and_execute(target_xyz, goal->target_spin, goal->execute, feedback);
      result->success = outcome.success;
      result->error_code = outcome.error_code;
      result->message = outcome.message;

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_code = 7;
        result->message = "Task goal canceled";
        goal_handle->canceled(result);
        clear_busy();
        return;
      }

      if (outcome.success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
      clear_busy();
    } catch (const std::exception & error) {
      result->success = false;
      result->error_code = 8;
      result->message = error.what();
      RCLCPP_ERROR(node_->get_logger(), "Task goal execution threw exception: %s", error.what());
      goal_handle->abort(result);
      clear_busy();
    }
  }

  /**
   * @brief 从 MoveIt 当前状态读取 IK 初始种子。
   *
   * @return Eigen::VectorXd 当前关节角向量；如果 MoveIt 当前状态不可用，则返回零向量并按关节限位裁剪。
   *
   * 该种子通常来自 `/joint_states`，用于让 IK 优先从当前机械臂姿态附近开始求解。
   */
  Eigen::VectorXd current_joint_seed() const
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    const auto current = move_group_->getCurrentJointValues();
    if (current.size() >= kDof) {
      for (std::size_t i = 0; i < kDof; ++i) {
        q[static_cast<int>(i)] = current[i];
      }
    }
    return clamp_to_limits(model_, q);
  }

  /**
   * @brief 计算给定关节角对应的 4D 任务值。
   *
   * @param q 关节角向量，顺序为 j1、j2、j3、j4。
   * @return Eigen::Vector4d 当前任务值 `[tool0_x, tool0_y, tool0_z, spin]`。
   *
   * 位置来自 Pinocchio 正运动学中的 tool0 frame。
   * spin 表示 tool0 绕自身 +Z 轴的自转角：先将世界 +Z 投影到垂直于 tool0 +Z 的平面，
   * 再计算 tool0 +X 相对该参考方向的有符号角。
   */
  Eigen::Vector4d task_value(const Eigen::VectorXd & q)
  {
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    const auto & tool_pose = data_->oMf[tool0_frame_id_];

    const Eigen::Vector3d world_z = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d world_x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d tool_z = tool_pose.rotation() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d tool_x = tool_pose.rotation() * Eigen::Vector3d::UnitX();

    Eigen::Vector3d reference = world_z - world_z.dot(tool_z) * tool_z;
    if (reference.norm() < 1.0e-8) {
      reference = world_x - world_x.dot(tool_z) * tool_z;
    }
    reference.normalize();

    const Eigen::Vector3d x_projected = (tool_x - tool_x.dot(tool_z) * tool_z).normalized();
    const double spin = std::atan2(tool_z.dot(reference.cross(x_projected)), reference.dot(x_projected));
    return Eigen::Vector4d(
      tool_pose.translation().x(),
      tool_pose.translation().y(),
      tool_pose.translation().z(),
      spin);
  }

  /**
   * @brief 计算 4D 任务误差。
   *
   * @param target 目标任务值 `[x, y, z, spin]`。
   * @param q 当前关节角向量。
   * @return Eigen::Vector4d 任务误差，前三项为位置误差，第四项为归一化后的 spin 误差。
   */
  Eigen::Vector4d task_error(const Eigen::Vector4d & target, const Eigen::VectorXd & q)
  {
    const Eigen::Vector4d current = task_value(q);
    Eigen::Vector4d error = target - current;
    error[3] = wrap_to_pi(error[3]);
    return error;
  }

  /**
   * @brief 用有限差分计算 4D 任务对 4 个关节的数值雅可比。
   *
   * @param q 当前关节角向量。
   * @return Eigen::Matrix4d 4x4 雅可比矩阵，每一列表示一个关节扰动对 `[x,y,z,spin]` 的影响。
   *
   * 这里没有使用 Pinocchio 的解析雅可比，是为了直接匹配自定义 spin 任务定义。
   */
  Eigen::Matrix4d finite_difference_jacobian(const Eigen::VectorXd & q)
  {
    const Eigen::Vector4d base = task_value(q);
    Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();
    for (int i = 0; i < static_cast<int>(kDof); ++i) {
      Eigen::VectorXd q_perturbed = q;
      q_perturbed[i] += finite_difference_eps_;
      q_perturbed = clamp_to_limits(model_, q_perturbed);
      Eigen::Vector4d diff = task_value(q_perturbed) - base;
      diff[3] = wrap_to_pi(diff[3]);
      jacobian.col(i) = diff / finite_difference_eps_;
    }
    return jacobian;
  }

  /**
   * @brief 从单个初始种子开始求解一次阻尼最小二乘 IK。
   *
   * @param target 目标任务值 `[x, y, z, spin]`。
   * @param q_seed IK 初始关节角。
   * @param q_solution 输出参数；求解成功时写入满足容差的关节角。
   * @return bool true 表示在迭代次数内收敛；false 表示未收敛。
   *
   * 更新公式近似为：
   * `delta = J^T * (J*J^T + lambda^2 I)^-1 * error`。
   * 每步会限制最大关节增量，并强制裁剪到关节限位内。
   */
  bool solve_ik_once(const Eigen::Vector4d & target, Eigen::VectorXd q_seed, Eigen::VectorXd & q_solution)
  {
    Eigen::VectorXd q = clamp_to_limits(model_, q_seed);
    for (int iter = 0; iter < max_ik_iterations_; ++iter) {
      const Eigen::Vector4d error = task_error(target, q);
      if (error.head<3>().norm() <= position_tolerance_ && std::abs(error[3]) <= spin_tolerance_) {
        q_solution = q;
        return true;
      }

      const Eigen::Matrix4d jacobian = finite_difference_jacobian(q);
      const Eigen::Matrix4d regularized =
        jacobian * jacobian.transpose() + damping_ * damping_ * Eigen::Matrix4d::Identity();
      Eigen::VectorXd delta = jacobian.transpose() * regularized.ldlt().solve(error);
      const double max_abs_delta = delta.cwiseAbs().maxCoeff();
      if (max_abs_delta > max_step_) {
        delta *= max_step_ / max_abs_delta;
      }
      q = clamp_to_limits(model_, q + delta);
    }
    return false;
  }

  /**
   * @brief 多种子 IK 求解入口。
   *
   * @param target 目标任务值 `[x, y, z, spin]`。
   * @param current_seed 当前关节状态作为首个 IK 种子。
   * @param q_solution 输出参数；任意一次 IK 成功时写入求得的关节目标。
   * @return bool true 表示至少一个种子求解成功；false 表示全部尝试失败。
   *
   * 第一次尝试使用当前关节状态，后续尝试在关节限位内随机采样种子，
   * 用于降低局部初值不佳导致 IK 失败的概率。
   */
  bool solve_ik(const Eigen::Vector4d & target, const Eigen::VectorXd & current_seed, Eigen::VectorXd & q_solution)
  {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    double best_score = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_q = current_seed;
    const int attempts = std::max(1, num_ik_seed_attempts_);
    for (int attempt = 0; attempt < attempts; ++attempt) {
      Eigen::VectorXd seed = current_seed;
      if (attempt > 0) {
        for (int i = 0; i < model_.nq; ++i) {
          const double lower = model_.lowerPositionLimit[i];
          const double upper = model_.upperPositionLimit[i];
          seed[i] = lower + unit(rng) * (upper - lower);
        }
      }

      Eigen::VectorXd candidate;
      if (solve_ik_once(target, seed, candidate)) {
        q_solution = candidate;
        return true;
      }

      const Eigen::Vector4d error = task_error(target, seed);
      const double score = error.head<3>().norm() + std::abs(error[3]);
      if (score < best_score) {
        best_score = score;
        best_q = seed;
      }
    }

    const Eigen::Vector4d best_error = task_error(target, best_q);
    RCLCPP_WARN(
      node_->get_logger(),
      "Best IK seed error after %d attempts: position=%.5f spin=%.5f",
      attempts, best_error.head<3>().norm(), std::abs(best_error[3]));
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex tool0_frame_id_{0};

  std::string urdf_path_;
  std::string srdf_path_;
  std::string group_name_;
  std::string tool0_frame_name_;
  std::string action_name_{"/arm2_task_goal"};
  double planning_time_{5.0};
  int num_ik_seed_attempts_{10};
  int max_ik_iterations_{100};
  double position_tolerance_{0.002};
  double spin_tolerance_{0.02};
  double damping_{1.0e-3};
  double finite_difference_eps_{1.0e-6};
  double max_step_{0.15};
  std::atomic_bool action_busy_{false};
  rclcpp_action::Server<PlanToTaskGoal>::SharedPtr action_server_;
};

/**
 * @brief 程序入口。
 *
 * @param argc 命令行参数数量，由 ROS 2/rclcpp 使用。
 * @param argv 命令行参数数组，由 ROS 2/rclcpp 使用。
 * @return int 进程返回码；正常退出时为 0，初始化失败时为非 0。
 *
 * main 会创建 ROS 2 节点和多线程 executor，使 MoveGroupInterface 能接收状态/action 回调；
 * 随后创建 Arm2TaskGoalPlanner，并常驻提供 `/arm2_task_goal` action server。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "arm2_plan_to_task_goal",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int rc = 1;
  try {
    Arm2TaskGoalPlanner planner(node);
    planner.start_action_server();
    RCLCPP_INFO(node->get_logger(), "Waiting for /arm2_task_goal action goals.");
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rc = 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rc = 1;
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return rc;
}
