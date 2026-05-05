#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rc_arm2_moveit_client/action/plan_to_task_goal.hpp"
#include "rc_arm2_msgs/msg/arm2_controller_state.hpp"
#include "rc_arm2_msgs/msg/arm2_middleware_command.hpp"
#include "rc_arm2_msgs/msg/arm2_middleware_state.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
constexpr int kIdleStepIndex = -1;

using PlanToTaskGoal = rc_arm2_moveit_client::action::PlanToTaskGoal;
using GoalHandlePlanToTaskGoal = rclcpp_action::ClientGoalHandle<PlanToTaskGoal>;
using Arm2ControllerState = rc_arm2_msgs::msg::Arm2ControllerState;
using Arm2MiddlewareCommand = rc_arm2_msgs::msg::Arm2MiddlewareCommand;
using Arm2MiddlewareState = rc_arm2_msgs::msg::Arm2MiddlewareState;

// 枚举动作单元
enum class StepType
{
  NONE,
  // 移动到目标点+offest[x,y,z],指定target_spin:
  // 1目标点+offest[x,y,z],target_spin发布到命令动作接口 task_goal_action_name: /arm2_task_goal
  // 完成标志：收到action返回，无论bool success=true/false
  MOVE_TARGET_OFFSET,
  // 移动到自定点[x,y,z],指定target_spin
  // 1自定点[x,y,z],指定target_spin发布到命令动作接口 task_goal_action_name: /arm2_task_goal
  // 完成标志：收到action返回，无论bool success=true/false
  MOVE_FIXED_POSE,
  // 切换吸盘为指定状态
  // 发布吸盘状态切换ros2 topic pub /arm2/vacuum_activate std_msgs/msg/Bool "{data: true/falase}" -1
  // 完成标志：无
  SET_VACUUM,
  // 切换带载状态为指定状态
  // 发布动力学状态切换：ros2 topic pub --once /arm2/payload_attached std_msgs/msg/Bool "{data: true/false}"
  // 完成标志：订阅带载状态: /arm2/payload_active与发布状态一致
  SET_PAYLOAD,
};

enum class WaitState
{
  IDLE,
  WAITING_FOR_ACTION_SERVER,
  WAITING_FOR_ACTION_RESULT,
  WAITING_FOR_PAYLOAD_STATE,
};

struct TaskGoalRequest
{
  std::array<double, 3> target_xyz{0.0, 0.0, 0.0};
  double target_spin{0.0};
  double planning_time{5.0};
};

struct PendingTaskGoal
{
  TaskGoalRequest request{};
};

// 动作单元结构体
struct ActionStep
{
  StepType type{StepType::NONE};
  std::string label;
  // 目标点（若有）
  std::array<double, 3> vector3{0.0, 0.0, 0.0};
  double target_spin{0.0};
  // 轨迹预计算时间上限
  std::optional<double> planning_time;
  // 状态切换（若有）
  bool bool_value{false};
};
// 动作集结构体
struct ActionSet
{
  // 动作集序号
  int32_t id{0};
  std::string name;
  // 动作集内的动作单元排列
  std::vector<ActionStep> steps;
};

struct ControllerTopicConfig
{
  std::string controller_state_topic{"/arm2_controller/state"};
  std::string payload_active_topic{"/arm2/payload_active"};
  std::string payload_command_topic{"/arm2/payload_attached"};
  std::string vacuum_activate_topic{"/arm2/vacuum_activate"};
};

/**
 * @brief Find the parameter list for a specific node inside a parsed ROS parameter map.
 *
 * @param parameter_map Parsed parameter map loaded from a YAML file.
 * @param node_name Target node name to search for.
 * @return const std::vector<rclcpp::Parameter> & Parameters that belong to the requested node.
 */
const std::vector<rclcpp::Parameter> & find_node_parameters(
  const rclcpp::ParameterMap & parameter_map,
  const std::string & node_name)
{
  const auto match = std::find_if(
    parameter_map.begin(), parameter_map.end(),
    [&node_name](const auto & entry)
    {
      return entry.first == node_name || entry.first == ("/" + node_name) ||
             (entry.first.size() > node_name.size() &&
             entry.first.compare(entry.first.size() - node_name.size(), node_name.size(), node_name) == 0);
    });
  if (match == parameter_map.end()) {
    throw std::runtime_error("Could not find parameter section for node " + node_name);
  }
  return match->second;
}

/**
 * @brief Look up a required string parameter from a node parameter list.
 *
 * @param parameters Parameter list for one node.
 * @param name Name of the required string parameter.
 * @return std::string The parsed string parameter value.
 */
std::string lookup_string_parameter(
  const std::vector<rclcpp::Parameter> & parameters,
  const std::string & name)
{
  const auto match = std::find_if(
    parameters.begin(), parameters.end(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (match == parameters.end()) {
    throw std::runtime_error("Missing required controller parameter " + name);
  }
  return match->as_string();
}

/**
 * @brief Load controller-related topic names that middleware reuses from the shared control config.
 *
 * @param None
 * @return ControllerTopicConfig Topic names used for controller state, payload state, payload command,
 * and vacuum command.
 */
ControllerTopicConfig load_controller_topic_config()
{
  const auto config_path = ament_index_cpp::get_package_share_directory("rc_arm2_control") +
    "/config/arm2_control.yaml";
  const auto parameter_map = rclcpp::parameter_map_from_yaml_file(config_path);
  const auto & controller_parameters =
    find_node_parameters(parameter_map, "arm2_torque_trajectory_controller");
  const auto & real_parameters =
    find_node_parameters(parameter_map, "arm2_mujoco_real");

  ControllerTopicConfig config;
  config.controller_state_topic = lookup_string_parameter(controller_parameters, "state_topic");
  config.payload_active_topic = lookup_string_parameter(controller_parameters, "payload_active_topic");
  config.payload_command_topic = lookup_string_parameter(controller_parameters, "payload_command_topic");
  config.vacuum_activate_topic = lookup_string_parameter(real_parameters, "vacuum_activate_topic");
  return config;
}

/**
 * @brief Resolve a configured path against a package share directory when the path is relative.
 *
 * @param package_name Package whose share directory is used as the base for relative paths.
 * @param configured_path Absolute path or package-relative path from configuration.
 * @return std::filesystem::path Resolved filesystem path.
 */
std::filesystem::path resolve_package_relative_path(
  const std::string & package_name,
  const std::string & configured_path)
{
  if (configured_path.empty()) {
    throw std::runtime_error("Configured path must not be empty");
  }

  const std::filesystem::path path(configured_path);
  if (path.is_absolute()) {
    return path;
  }
  return std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) / path;
}

/**
 * @brief Fetch a required YAML map entry.
 *
 * @param node YAML map node that should contain the requested key.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return YAML::Node The requested YAML node.
 */
YAML::Node require_map_entry(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  const auto value = node[key];
  if (!value) {
    throw std::runtime_error(context + " is missing required key '" + key + "'");
  }
  return value;
}

/**
 * @brief Read a required non-empty string value from a YAML map.
 *
 * @param node YAML map node that contains the value.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return std::string Parsed non-empty string.
 */
std::string require_string(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  const auto value = require_map_entry(node, key, context);
  const auto parsed = value.as<std::string>();
  if (parsed.empty()) {
    throw std::runtime_error(context + " key '" + key + "' must not be empty");
  }
  return parsed;
}

/**
 * @brief Read a required int32 value from a YAML map.
 *
 * @param node YAML map node that contains the value.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return int32_t Parsed integer value.
 */
int32_t require_int32(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  return require_map_entry(node, key, context).as<int32_t>();
}

/**
 * @brief Read a required boolean value from a YAML map.
 *
 * @param node YAML map node that contains the value.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return bool Parsed boolean value.
 */
bool require_bool(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  return require_map_entry(node, key, context).as<bool>();
}

/**
 * @brief Read a required floating-point value from a YAML map.
 *
 * @param node YAML map node that contains the value.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return double Parsed floating-point value.
 */
double require_double(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  return require_map_entry(node, key, context).as<double>();
}

/**
 * @brief Read an optional positive floating-point value from a YAML map.
 *
 * @param node YAML map node that may contain the value.
 * @param key Optional key name.
 * @param context Human-readable path used in error messages.
 * @return std::optional<double> Parsed positive value when present, otherwise `std::nullopt`.
 */
std::optional<double> optional_positive_double(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  const auto value = node[key];
  if (!value) {
    return std::nullopt;
  }
  const double parsed = value.as<double>();
  if (parsed <= 0.0) {
    throw std::runtime_error(context + " key '" + key + "' must be positive");
  }
  return parsed;
}

/**
 * @brief Read a required 3-element vector from a YAML map.
 *
 * @param node YAML map node that contains the vector.
 * @param key Required key name.
 * @param context Human-readable path used in error messages.
 * @return std::array<double, 3> Parsed three-dimensional vector.
 */
std::array<double, 3> require_vector3(
  const YAML::Node & node,
  const char * key,
  const std::string & context)
{
  const auto value = require_map_entry(node, key, context);
  if (!value.IsSequence() || value.size() != 3U) {
    throw std::runtime_error(context + " key '" + key + "' must be a 3-element sequence");
  }
  return {value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}

/**
 * @brief Convert a YAML step type string into the internal step enum.
 *
 * @param type_name Step type string from the YAML config.
 * @return StepType Internal step type enum value.
 */
StepType parse_step_type(const std::string & type_name)
{
  if (type_name == "move_target_offset") {
    return StepType::MOVE_TARGET_OFFSET;
  }
  if (type_name == "move_fixed_pose") {
    return StepType::MOVE_FIXED_POSE;
  }
  if (type_name == "set_vacuum") {
    return StepType::SET_VACUUM;
  }
  if (type_name == "set_payload") {
    return StepType::SET_PAYLOAD;
  }
  throw std::runtime_error("Unsupported action step type '" + type_name + "'");
}

/**
 * @brief Convert an internal step type into the middleware state message constant.
 *
 * @param type Internal step type enum.
 * @return int32_t Matching `Arm2MiddlewareState::STEP_TYPE_*` constant.
 */
int32_t step_type_to_msg(const StepType type)
{
  switch (type) {
    case StepType::NONE:
      return Arm2MiddlewareState::STEP_TYPE_NONE;
    case StepType::MOVE_TARGET_OFFSET:
      return Arm2MiddlewareState::STEP_TYPE_MOVE_TARGET_OFFSET;
    case StepType::MOVE_FIXED_POSE:
      return Arm2MiddlewareState::STEP_TYPE_MOVE_FIXED_POSE;
    case StepType::SET_VACUUM:
      return Arm2MiddlewareState::STEP_TYPE_SET_VACUUM;
    case StepType::SET_PAYLOAD:
      return Arm2MiddlewareState::STEP_TYPE_SET_PAYLOAD;
  }
  return Arm2MiddlewareState::STEP_TYPE_NONE;
}

/**
 * @brief Load and validate all action sets from the middleware YAML config file.
 *
 * @param config_path Filesystem path to the action-set YAML file.
 * @return std::vector<ActionSet> Fully parsed action-set definitions.
 */
std::vector<ActionSet> load_action_sets_from_yaml(const std::filesystem::path & config_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path.string());
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "Failed to load action set config '" + config_path.string() + "': " + error.what());
  }

  const auto action_sets_node = root["action_sets"];
  if (!action_sets_node || !action_sets_node.IsSequence() || action_sets_node.size() == 0U) {
    throw std::runtime_error("action_sets must exist and contain at least one entry");
  }

  std::unordered_map<int32_t, bool> seen_ids;
  std::vector<ActionSet> action_sets;
  action_sets.reserve(action_sets_node.size());

  for (std::size_t set_index = 0; set_index < action_sets_node.size(); ++set_index) {
    const auto action_set_node = action_sets_node[set_index];
    const std::string set_context = "action_sets[" + std::to_string(set_index) + "]";

    ActionSet action_set;
    action_set.id = require_int32(action_set_node, "id", set_context);
    action_set.name = require_string(action_set_node, "name", set_context);
    if (seen_ids.count(action_set.id) != 0U) {
      throw std::runtime_error("Duplicate action set id " + std::to_string(action_set.id));
    }
    seen_ids[action_set.id] = true;

    const auto steps_node = require_map_entry(action_set_node, "steps", set_context);
    if (!steps_node.IsSequence() || steps_node.size() == 0U) {
      throw std::runtime_error(set_context + " steps must be a non-empty sequence");
    }

    action_set.steps.reserve(steps_node.size());
    for (std::size_t step_index = 0; step_index < steps_node.size(); ++step_index) {
      const auto step_node = steps_node[step_index];
      const std::string step_context =
        set_context + ".steps[" + std::to_string(step_index) + "]";

      ActionStep step;
      step.type = parse_step_type(require_string(step_node, "type", step_context));

      const auto label_node = step_node["label"];
      if (label_node) {
        step.label = label_node.as<std::string>();
      }

      switch (step.type) {
        case StepType::MOVE_TARGET_OFFSET:
          step.vector3 = require_vector3(step_node, "offset_xyz", step_context);
          step.target_spin = require_double(step_node, "target_spin", step_context);
          step.planning_time = optional_positive_double(step_node, "planning_time", step_context);
          break;
        case StepType::MOVE_FIXED_POSE:
          step.vector3 = require_vector3(step_node, "xyz", step_context);
          step.target_spin = require_double(step_node, "target_spin", step_context);
          step.planning_time = optional_positive_double(step_node, "planning_time", step_context);
          break;
        case StepType::SET_VACUUM:
          step.bool_value = require_bool(step_node, "enabled", step_context);
          break;
        case StepType::SET_PAYLOAD:
          step.bool_value = require_bool(step_node, "attached", step_context);
          break;
        case StepType::NONE:
          break;
      }
      action_set.steps.push_back(std::move(step));
    }

    action_sets.push_back(std::move(action_set));
  }

  return action_sets;
}
}  // namespace

/**
 * @brief Middleware node that executes YAML-defined action sets against MoveIt and controller topics.
 */
class Arm2MiddlewareNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the middleware node, load configuration, and initialize ROS interfaces.
   *
   * @param None
   * @return None
   */
  Arm2MiddlewareNode()
  : Node("arm2_middleware")
  {
    const auto controller_topics = load_controller_topic_config();
    // 接收命令话题
    command_topic_ = declare_parameter<std::string>("command_topic", "/arm2/middleware/command");
    // 视觉提供的物体位置
    target_point_topic_ = declare_parameter<std::string>(
      "target_point_topic", "/arm2/middleware/target_point");
    // 发布自身状态
    middleware_state_topic_ = declare_parameter<std::string>(
      "middleware_state_topic", "/arm2/middleware/state");
    // 发布目标点到指定action
    task_goal_action_name_ = declare_parameter<std::string>(
      "task_goal_action_name", "/arm2_task_goal");
    // 下层controller的状态
    controller_state_topic_ = declare_parameter<std::string>(
      "controller_state_topic", controller_topics.controller_state_topic);
    // 接收负载状态话题
    payload_active_topic_ = declare_parameter<std::string>(
      "payload_active_topic", controller_topics.payload_active_topic);
    // 发布期望负载状态话题 
    payload_command_topic_ = declare_parameter<std::string>(
      "payload_command_topic", controller_topics.payload_command_topic);
    // 发布吸盘状态话题
    vacuum_activate_topic_ = declare_parameter<std::string>(
      "vacuum_activate_topic", controller_topics.vacuum_activate_topic);
    
    
    action_sets_config_path_ = declare_parameter<std::string>(
      "action_sets_config_path", "config/action_sets.yaml");

    planning_time_ = declare_parameter<double>("planning_time", 5.0);
    action_server_timeout_sec_ = declare_parameter<double>("action_server_timeout_sec", 2.0);
    action_result_timeout_sec_ = declare_parameter<double>("action_result_timeout_sec", 30.0);
    payload_wait_timeout_sec_ = declare_parameter<double>("payload_wait_timeout_sec", 5.0);

    if (planning_time_ <= 0.0) {
      throw std::runtime_error("planning_time must be positive");
    }
    if (action_server_timeout_sec_ <= 0.0) {
      throw std::runtime_error("action_server_timeout_sec must be positive");
    }
    if (action_result_timeout_sec_ <= 0.0) {
      throw std::runtime_error("action_result_timeout_sec must be positive");
    }
    if (payload_wait_timeout_sec_ <= 0.0) {
      throw std::runtime_error("payload_wait_timeout_sec must be positive");
    }

    const auto resolved_action_sets_path = resolve_package_relative_path(
      "rc_arm2_middleware", action_sets_config_path_);
    // 从包下的config/action_sets.yaml中获取动作集
    action_sets_ = load_action_sets_from_yaml(resolved_action_sets_path);
    for (const auto & action_set : action_sets_) {
      action_sets_by_id_[action_set.id] = &action_set;
    }
    // 发布自身状态 "/arm2/middleware/state"
    state_pub_ = create_publisher<Arm2MiddlewareState>(middleware_state_topic_, 10);
    // 发布期望负载状态话题 /arm2/payload_attached
    payload_command_pub_ = create_publisher<std_msgs::msg::Bool>(payload_command_topic_, 10);
    // 发布吸盘状态话题 /arm2/vacuum_activate
    vacuum_activate_pub_ = create_publisher<std_msgs::msg::Bool>(vacuum_activate_topic_, 10);

    // 视觉提供的物体位置 "/arm2/middleware/target_point"
    target_point_sub_ = create_subscription<geometry_msgs::msg::Point>(
      target_point_topic_, 10,
      std::bind(&Arm2MiddlewareNode::target_point_callback, this, std::placeholders::_1));
    // 订阅命令 "/arm2/middleware/command" int32 action_set_id
    command_sub_ = create_subscription<Arm2MiddlewareCommand>(
      command_topic_, 10,
      std::bind(&Arm2MiddlewareNode::command_callback, this, std::placeholders::_1));
    //结构化控制器状态 rc_arm2_msgs/msg/Arm2ControllerState /arm2_controller/state
    controller_state_sub_ = create_subscription<Arm2ControllerState>(
      controller_state_topic_, 10,
      std::bind(&Arm2MiddlewareNode::controller_state_callback, this, std::placeholders::_1));
    // 当前使用的模型是带负载还是空载，/arm2/payload_active
    payload_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      payload_active_topic_, 10,
      std::bind(&Arm2MiddlewareNode::payload_active_callback, this, std::placeholders::_1));
    // 发布目标点动作接口
    task_goal_client_ = rclcpp_action::create_client<PlanToTaskGoal>(this, task_goal_action_name_);
    // 状态机
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Arm2MiddlewareNode::timer_callback, this));

    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "arm2_middleware ready. command_topic=%s target_point_topic=%s action=%s action_sets=%s",
      command_topic_.c_str(), target_point_topic_.c_str(), task_goal_action_name_.c_str(),
      resolved_action_sets_path.string().c_str());
  }

private:
  /**
   * @brief 获取视觉的目标的位置（箱子侧面）
   *
   * @param msg Latest target point message.
   * @return None
   */
  void target_point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    try {
      latest_target_point_ = *msg;
      have_target_point_ = true;
      publish_state();
    } catch (const std::exception & error) {
      abort_execution(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION, error.what());
    }
  }

  /**
   * @brief Accept an action-set command and start the corresponding sequence when idle.
   *
   * @param msg Command message that identifies the requested action set. int32 
   * @return None
   */
  void command_callback(const Arm2MiddlewareCommand::SharedPtr msg)
  {
    try {
      // 忙就拒绝新命令
      if (busy_) {
        last_error_code_ = Arm2MiddlewareState::ERROR_BUSY_REJECTED;
        RCLCPP_WARN(
          get_logger(),
          "Rejecting action_set_id=%d because middleware is busy",
          msg->action_set_id);
        publish_state();
        return;
      }
      // 按序号查找
      const auto * action_set = lookup_action_set(msg->action_set_id);
      // 检查存在
      if (action_set == nullptr) {
        active_action_set_id_ = msg->action_set_id;
        active_action_set_name_.clear();
        active_step_index_ = kIdleStepIndex;
        active_step_type_ = StepType::NONE;
        active_step_label_.clear();
        abort_execution(
          Arm2MiddlewareState::ERROR_UNKNOWN_ACTION_SET,
          "Unknown action_set_id=" + std::to_string(msg->action_set_id));
        return;
      }
      // 设置为待执行
      start_action_set(*action_set);
    } catch (const std::exception & error) {
      abort_execution(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION, error.what());
    }
  }

  /**
   * @brief Mirror controller state updates and abort execution immediately if the controller faults.
   *
   * @param msg Latest controller state message.
   * @return None
   */
  void controller_state_callback(const Arm2ControllerState::SharedPtr msg)
  {
    try {
      latest_controller_state_ = *msg;
      // 只有FAULT会停止当前执行
      if (busy_ && msg->controller_state == Arm2ControllerState::CONTROLLER_STATE_FAULT) {
        abort_execution(
          Arm2MiddlewareState::ERROR_CONTROLLER_FAULT,
          "Controller entered FAULT while executing action set");
        return;
      }
      publish_state();
    } catch (const std::exception & error) {
      abort_execution(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION, error.what());
    }
  }

  /**
   * @brief 检查期望负载状态与订阅负载状态是否一致
   *
   * @param msg Latest payload-active boolean message.
   * @return None
   */
  void payload_active_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    try {
      payload_active_ = msg->data;
      if (wait_state_ == WaitState::WAITING_FOR_PAYLOAD_STATE &&
        payload_active_ == desired_payload_active_)
      {
        wait_state_ = WaitState::IDLE;
        executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_RUNNING;
        ++active_step_index_;
        advance_execution();
        return;
      }
      publish_state();
    } catch (const std::exception & error) {
      abort_execution(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION, error.what());
    }
  }

  /**
   * @brief Poll asynchronous wait conditions for action-server readiness, action results, and payload switches.
   * 状态机
   * @param None
   * @return None
   */
  void timer_callback()
  {
    try {
      // 空闲
      if (wait_state_ == WaitState::IDLE) {
        return;
      }
      // 计算已经过去时间
      const double elapsed = (now() - wait_started_).seconds();
      switch (wait_state_) {
        // 等待action响应 2.0s
        case WaitState::WAITING_FOR_ACTION_SERVER:
          if (task_goal_client_->action_server_is_ready()) {
            if (!pending_task_goal_) {
              abort_execution(
                Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION,
                "Action server became ready without a pending goal");
              return;
            }
            const auto request = pending_task_goal_->request;
            pending_task_goal_.reset();
            dispatch_task_goal(request);
            return;
          }
          if (elapsed >= action_server_timeout_sec_) {
            abort_execution(
              Arm2MiddlewareState::ERROR_ACTION_SERVER_UNAVAILABLE,
              "Timed out waiting for /arm2_task_goal action server");
          }
          return;
        // 等待动作完成 30.0s
        case WaitState::WAITING_FOR_ACTION_RESULT:
          if (elapsed >= action_result_timeout_sec_) {
            abort_execution(
              Arm2MiddlewareState::ERROR_ACTION_TIMEOUT,
              "Timed out waiting for /arm2_task_goal result");
          }
          return;
        // 等待状态切换5.0s
        case WaitState::WAITING_FOR_PAYLOAD_STATE:
          if (elapsed >= payload_wait_timeout_sec_) {
            abort_execution(
              Arm2MiddlewareState::ERROR_PAYLOAD_WAIT_TIMEOUT,
              std::string("Timed out waiting for payload_active=") +
              (desired_payload_active_ ? "true" : "false"));
          }
          return;
        // 动作集执行完成转空闲
        case WaitState::IDLE:
          return;
      }
    } catch (const std::exception & error) {
      abort_execution(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION, error.what());
    }
  }

  /**
   * @brief Find an action set by its configured integer identifier.
   *
   * @param action_set_id Requested action-set identifier.
   * @return const ActionSet * Pointer to the action set when found, otherwise `nullptr`.
   */
  const ActionSet * lookup_action_set(const int32_t action_set_id) const
  {
    const auto match = action_sets_by_id_.find(action_set_id);
    if (match == action_sets_by_id_.end()) {
      return nullptr;
    }
    // 返回动作集指针
    return match->second;
  }

  /**
   * @brief Initialize runtime state for a new action set and begin executing its first step.
   *
   * @param action_set Action-set definition selected by the command interface.
   * @return None
   */
  void start_action_set(const ActionSet & action_set)
  {
    current_action_set_ = &action_set;
    active_action_set_id_ = action_set.id;
    active_action_set_name_ = action_set.name;
    active_step_index_ = 0;
    active_step_type_ = StepType::NONE;
    active_step_label_.clear();

    wait_state_ = WaitState::IDLE;
    // 自身状态更新
    executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_RUNNING;
    busy_ = true;
    desired_payload_active_ = false;
    pending_task_goal_.reset();
    invalidate_active_goal();

    last_error_code_ = Arm2MiddlewareState::ERROR_NONE;
    last_move_success_ = true;
    last_move_error_code_ = 0;
    last_action_set_success_ = true;

    publish_state();
    // 阻塞循环
    advance_execution();
  }

  /**
   * @brief Drive the current action set forward until the next asynchronous wait point or completion.
   * 执行动作集
   * @param None
   * @return None
   */
  void advance_execution()
  {
    while (busy_ && current_action_set_ != nullptr) {
      // 结束
      if (active_step_index_ < 0 ||
        static_cast<std::size_t>(active_step_index_) >= current_action_set_->steps.size())
      {
        finish_action_set();
        return;
      }

      const auto & step = current_action_set_->steps[static_cast<std::size_t>(active_step_index_)];
      active_step_type_ = step.type;
      active_step_label_ = step.label;
      executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_RUNNING;
      publish_state();

      switch (step.type) {
        case StepType::MOVE_TARGET_OFFSET: {
          if (!have_target_point_) {
            abort_execution(
              Arm2MiddlewareState::ERROR_NO_TARGET_POINT,
              "move_target_offset step requires a cached target point");
            return;
          }

          TaskGoalRequest request;
          request.target_xyz = {
            latest_target_point_.x + step.vector3[0],
            latest_target_point_.y + step.vector3[1],
            latest_target_point_.z + step.vector3[2]};
          request.target_spin = step.target_spin;
          request.planning_time = step.planning_time.value_or(planning_time_);
          start_task_goal(request);
          return;
        }
        case StepType::MOVE_FIXED_POSE: {
          TaskGoalRequest request;
          request.target_xyz = step.vector3;
          request.target_spin = step.target_spin;
          request.planning_time = step.planning_time.value_or(planning_time_);
          start_task_goal(request);
          return;
        }
        case StepType::SET_VACUUM:
          publish_vacuum(step.bool_value);
          ++active_step_index_;
          break;
        case StepType::SET_PAYLOAD:
          publish_payload_command(step.bool_value);
          if (payload_active_ == step.bool_value) {
            ++active_step_index_;
            break;
          }
          desired_payload_active_ = step.bool_value;
          wait_state_ = WaitState::WAITING_FOR_PAYLOAD_STATE;
          executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_WAITING_FOR_PAYLOAD_STATE;
          wait_started_ = now();
          publish_state();
          return;
        case StepType::NONE:
          abort_execution(
            Arm2MiddlewareState::ERROR_INVALID_CONFIG,
            "Encountered invalid step type NONE during execution");
          return;
      }
    }
  }

  /**
   * @brief Start a task-goal step, either by sending immediately or waiting for the action server.
   *
   * @param request Fully resolved MoveIt task-goal request.
   * @return None
   */
  void start_task_goal(const TaskGoalRequest & request)
  {
    if (task_goal_client_->action_server_is_ready()) {
      // 向action发送目标
      dispatch_task_goal(request);
      return;
    }
    // 暂存 request
    pending_task_goal_ = PendingTaskGoal{request};
    // 等待action响应
    wait_state_ = WaitState::WAITING_FOR_ACTION_SERVER;
    // 自身状态
    executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_WAITING_FOR_ACTION_SERVER;
    // 开始等待action响应的时间
    wait_started_ = now();
    publish_state();
  }

  /**
   * @brief Send a resolved task goal to `/arm2_task_goal` and register result callbacks.
   * 向action发送目标
   * @param request Fully resolved MoveIt task-goal request.
   * @return None
   */
  void dispatch_task_goal(const TaskGoalRequest & request)
  {
    // 组装 action goal
    PlanToTaskGoal::Goal goal;
    goal.target_xyz = request.target_xyz;
    goal.target_spin = request.target_spin;
    goal.planning_time = request.planning_time;
    goal.execute = true;

    // 递增 goal_sequence_，生成这次 goal 的唯一序号
    const std::uint64_t sequence = ++goal_sequence_;
    wait_state_ = WaitState::WAITING_FOR_ACTION_RESULT;
    executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_WAITING_FOR_ACTION_RESULT;
    wait_started_ = now();
    active_goal_handle_.reset();
    publish_state();

    // 注册两个异步回调
    auto options = rclcpp_action::Client<PlanToTaskGoal>::SendGoalOptions();
    // action 接受请求回调
    options.goal_response_callback =
      [this, sequence](GoalHandlePlanToTaskGoal::SharedPtr goal_handle)
      {
        // 防止旧 goal 的延迟回调”污染当前状态。
        // 比如 middleware 已经取消了旧动作、开始了新动作，那旧动作晚到的回调会因为序号不匹配被直接忽略。
        if (sequence != goal_sequence_) {
          return;
        }
        // 如果 goal_handle 为空，说明 goal 被 action server 拒绝，直接 abort_execution(...)
        if (!goal_handle) {
          abort_execution(
            Arm2MiddlewareState::ERROR_GOAL_REJECTED,
            "Task goal was rejected by /arm2_task_goal");
          return;
        }
        // 否则把 active_goal_handle_ 保存下来，后面如果要 cancel 就靠它 
        active_goal_handle_ = goal_handle;
      };
    // action 完成请求回调
    options.result_callback =
      [this, sequence](const GoalHandlePlanToTaskGoal::WrappedResult & result)
      {
        // 防止旧 goal 的延迟回调”污染当前状态。
        // 比如 middleware 已经取消了旧动作、开始了新动作，那旧动作晚到的回调会因为序号不匹配被直接忽略。
        if (sequence != goal_sequence_) {
          return;
        }
        // 收到最终结果后，先清掉等待状态
        active_goal_handle_.reset();
        // 清空暂存 request
        pending_task_goal_.reset();
        wait_state_ = WaitState::IDLE;
        executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_RUNNING;
        // 记录这次action完成状态
        if (result.result) {
          last_move_success_ = result.result->success;
          last_move_error_code_ = result.result->error_code;
        } else {
          last_move_success_ = false;
          last_move_error_code_ = -1;
        }
        if (!last_move_success_) {
          last_action_set_success_ = false;
        }

        // 最后 ++active_step_index_，继续执行动作集合里的下一步 advance_execution()
        ++active_step_index_;
        advance_execution();
      };
    // 真正异步发送 goal
    // 注意它是异步的，所以函数本身不会阻塞等待，而是立刻返回，后续靠上面两个 callback 继续推进状态机。
    task_goal_client_->async_send_goal(goal, options);
  }

  /**
   * @brief Publish a payload-model switch request.
   *
   * @param attached Requested payload model state.
   * @return None
   */
  void publish_payload_command(const bool attached)
  {
    std_msgs::msg::Bool msg;
    msg.data = attached;
    payload_command_pub_->publish(msg);
  }

  /**
   * @brief Publish a vacuum actuator command.
   *
   * @param enabled Requested vacuum state.
   * @return None
   */
  void publish_vacuum(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    vacuum_activate_pub_->publish(msg);
  }

  /**
   * @brief Abort the current action set, clear wait state, and publish an aborted middleware state.
   *
   * @param error_code Middleware error code that explains why execution stopped.
   * @param reason Human-readable reason used for logging.
   * @return None
   */
  void abort_execution(const int32_t error_code, const std::string & reason)
  {
    if (busy_ || current_action_set_ != nullptr) {
      cancel_active_goal_if_any();
    } else {
      invalidate_active_goal();
    }

    pending_task_goal_.reset();
    wait_state_ = WaitState::IDLE;
    desired_payload_active_ = false;
    busy_ = false;
    current_action_set_ = nullptr;
    executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_ABORTED;
    last_error_code_ = error_code;
    last_action_set_success_ = false;

    RCLCPP_WARN(get_logger(), "Middleware aborted: %s", reason.c_str());
    publish_state();
  }

  /**
   * @brief Cancel the currently active task goal when one exists and invalidate stale callbacks.
   *
   * @param None
   * @return None
   */
  void cancel_active_goal_if_any()
  {
    if (active_goal_handle_) {
      task_goal_client_->async_cancel_goal(active_goal_handle_);
    }
    invalidate_active_goal();
  }

  /**
   * @brief Invalidate the locally tracked task goal so late callbacks are ignored.
   *
   * @param None
   * @return None
   */
  void invalidate_active_goal()
  {
    // 递增序列号，之前的序列号将变成旧的，会在dispatch_task_goal中失效
    ++goal_sequence_;
    active_goal_handle_.reset();
  }

  /**
   * @brief Mark the current action set as completed and return middleware to idle.
   *
   * @param None
   * @return None
   */
  void finish_action_set()
  {
    //转空闲
    busy_ = false;
    wait_state_ = WaitState::IDLE;
    desired_payload_active_ = false;
    pending_task_goal_.reset();
    // 该动作集的序列号被失效
    invalidate_active_goal();
    current_action_set_ = nullptr;
    executor_state_ = Arm2MiddlewareState::EXECUTOR_STATE_IDLE;
    active_action_set_id_ = 0;
    active_action_set_name_.clear();
    active_step_index_ = kIdleStepIndex; // = -1
    active_step_type_ = StepType::NONE;
    active_step_label_.clear();
    publish_state();
  }

  /**
   * @brief Publish the current executor snapshot on `/arm2/middleware/state`.
   *
   * @param None
   * @return None
   */
  void publish_state()
  {
    Arm2MiddlewareState msg;
    msg.executor_state = executor_state_;
    msg.active_step_type = step_type_to_msg(active_step_type_);
    msg.last_error_code = last_error_code_;
    msg.busy = busy_;
    msg.active_action_set_id = active_action_set_id_;
    msg.active_action_set_name = active_action_set_name_;
    msg.active_step_index = active_step_index_;
    msg.active_step_label = active_step_label_;
    msg.have_target_point = have_target_point_;
    msg.latest_target_point = {latest_target_point_.x, latest_target_point_.y, latest_target_point_.z};
    msg.controller_state = latest_controller_state_.controller_state;
    msg.controller_payload_attached = latest_controller_state_.payload_attached;
    msg.controller_pending_payload_switch = latest_controller_state_.pending_payload_switch;
    msg.controller_detail_code = latest_controller_state_.detail_code;
    msg.payload_active = payload_active_;
    msg.last_move_success = last_move_success_;
    msg.last_move_error_code = last_move_error_code_;
    msg.last_action_set_success = last_action_set_success_;
    state_pub_->publish(msg);
  }

  std::string command_topic_;
  std::string target_point_topic_;
  std::string middleware_state_topic_;
  std::string task_goal_action_name_;
  std::string controller_state_topic_;
  std::string payload_active_topic_;
  std::string payload_command_topic_;
  std::string vacuum_activate_topic_;
  std::string action_sets_config_path_;

  double planning_time_{5.0};
  double action_server_timeout_sec_{2.0};
  double action_result_timeout_sec_{30.0};
  double payload_wait_timeout_sec_{5.0};

  std::vector<ActionSet> action_sets_;
  std::unordered_map<int32_t, const ActionSet *> action_sets_by_id_;

  rclcpp::Publisher<Arm2MiddlewareState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vacuum_activate_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_point_sub_;
  rclcpp::Subscription<Arm2MiddlewareCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Arm2ControllerState>::SharedPtr controller_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_active_sub_;

  rclcpp_action::Client<PlanToTaskGoal>::SharedPtr task_goal_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Point latest_target_point_{};
  bool have_target_point_{false};
  Arm2ControllerState latest_controller_state_{};
  bool payload_active_{false};

  const ActionSet * current_action_set_{nullptr};
  WaitState wait_state_{WaitState::IDLE};
  // 自身状态
  int32_t executor_state_{Arm2MiddlewareState::EXECUTOR_STATE_IDLE};
  int32_t active_action_set_id_{0};
  std::string active_action_set_name_;
  int32_t active_step_index_{kIdleStepIndex};
  StepType active_step_type_{StepType::NONE};
  std::string active_step_label_;
  bool busy_{false};
  bool desired_payload_active_{false};
  int32_t last_error_code_{Arm2MiddlewareState::ERROR_NONE};
  bool last_move_success_{true};
  int32_t last_move_error_code_{0};
  bool last_action_set_success_{true};
  rclcpp::Time wait_started_{0, 0, RCL_ROS_TIME};
  std::optional<PendingTaskGoal> pending_task_goal_;
  GoalHandlePlanToTaskGoal::SharedPtr active_goal_handle_;
  std::uint64_t goal_sequence_{0};
};

/**
 * @brief Start the middleware ROS node and spin until shutdown.
 *
 * @param argc Command-line argument count.
 * @param argv Command-line argument vector.
 * @return int Process exit code.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Arm2MiddlewareNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
