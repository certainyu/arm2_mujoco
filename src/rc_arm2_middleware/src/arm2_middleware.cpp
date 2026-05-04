#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_map.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{
constexpr int kIdleStep = -1;

using PlanToTaskGoal = rc_arm2_moveit_client::action::PlanToTaskGoal;
using GoalHandlePlanToTaskGoal = rclcpp_action::ClientGoalHandle<PlanToTaskGoal>;
using Arm2ControllerState = rc_arm2_msgs::msg::Arm2ControllerState;
using Arm2MiddlewareCommand = rc_arm2_msgs::msg::Arm2MiddlewareCommand;
using Arm2MiddlewareState = rc_arm2_msgs::msg::Arm2MiddlewareState;

struct Pose4D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double spin{0.0};
};

/**
 * @brief 暂存一个尚未真正发送出去的 `/arm2_task_goal` 目标。
 *
 * @details
 * 当 middleware 需要等待 action server 变为可用时，会先把目标位姿放进这个结构体，
 * 待 server 可用后再真正发出 goal。
 */
struct PendingTaskGoal
{
  Pose4D pose{};
};

/**
 * @brief 保存 middleware 需要复用的底层 topic 名称。
 *
 * @details
 * 这些 topic 会从 `arm2_control.yaml` 的其他节点参数中读取，
 * 这样 middleware 不需要在自己的参数段里重复声明同一组通信接口。
 */
struct ControllerTopicConfig
{
  std::string controller_state_topic{"/arm2_controller/state"};
  std::string payload_active_topic{"/arm2/payload_active"};
  std::string payload_command_topic{"/arm2/payload_attached"};
  std::string vacuum_activate_topic{"/arm2/vacuum_activate"};
};

/**
 * @brief 将长度为 4 的参数数组解析为一个 4D 位姿。
 *
 * @param values 输入数组，顺序必须为 `[x, y, z, spin]`。
 * @param name 参数名，仅用于报错信息。
 * @return Pose4D 解析后的 4D 位姿结构体。
 *
 * @throw std::runtime_error 当数组长度不是 4 时抛出。
 */
Pose4D parse_pose4(const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 4U) {
    throw std::runtime_error(name + " must contain exactly 4 values");
  }
  return Pose4D{values[0], values[1], values[2], values[3]};
}

/**
 * @brief 将扁平的 pose 数组解析为多个 4D 位姿。
 *
 * @param values 输入数组，格式为连续的 `[x, y, z, spin, ...]`。
 * @param name 参数名，仅用于报错信息。
 * @return std::vector<Pose4D> 解析后的位姿列表。
 *
 * @throw std::runtime_error 当数组长度不是 4 的整数倍时抛出。
 */
std::vector<Pose4D> parse_pose4_list(const std::vector<double> & values, const std::string & name)
{
  if (values.size() % 4U != 0U) {
    throw std::runtime_error(name + " length must be a multiple of 4");
  }
  std::vector<Pose4D> out;
  out.reserve(values.size() / 4U);
  for (std::size_t i = 0; i < values.size(); i += 4U) {
    out.push_back(Pose4D{values[i], values[i + 1U], values[i + 2U], values[i + 3U]});
  }
  return out;
}

/**
 * @brief 在 YAML 参数映射中查找某个节点对应的参数列表。
 *
 * @param parameter_map 由整份 YAML 文件解析得到的参数映射。
 * @param node_name 目标节点名，例如 `arm2_torque_trajectory_controller`。
 * @return const std::vector<rclcpp::Parameter> & 该节点下的参数列表引用。
 *
 * @throw std::runtime_error 当找不到对应节点参数段时抛出。
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
 * @brief 在一个节点参数列表中查找指定字符串参数。
 *
 * @param parameters 某个节点的参数列表。
 * @param name 待查找的参数名。
 * @return std::string 参数值。
 *
 * @throw std::runtime_error 当参数不存在时抛出。
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
 * @brief 从 `arm2_control.yaml` 中读取 middleware 需要复用的底层 topic 配置。
 *
 * @details
 * 其中：
 * - `controller_state_topic`、`payload_active_topic`、`payload_command_topic`
 *   读取自 `arm2_torque_trajectory_controller`
 * - `vacuum_activate_topic` 读取自 `arm2_mujoco_real`
 *
 * @return ControllerTopicConfig 解析后的 topic 配置结构体。
 *
 * @throw std::runtime_error 当 YAML 缺少必需参数段或参数项时抛出。
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
}  // namespace

/**
 * @brief arm2 中间层业务状态机节点。
 *
 * @details
 * 该节点把点位命令、底层控制器状态、带载状态和 `/arm2_task_goal` action
 * 串联起来，实现抓取、放置、回默认以及错误恢复回默认的宏动作流程。
 */
class Arm2MiddlewareNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造 middleware 节点并完成所有接口初始化。
   *
   * @details
   * 构造函数会：
   * - 读取 middleware 自身参数与底层 topic 配置
   * - 解析默认点、暂存点、仓库点
   * - 创建状态发布、命令发布、订阅器、action client 和定时器
   *
   * @return void 无返回值。
   *
   * @throw std::runtime_error 当参数非法或必需配置缺失时抛出。
   */
  Arm2MiddlewareNode()
  : Node("arm2_middleware")
  {
    const auto controller_topics = load_controller_topic_config();
    // 接收外部发布命令话题 command_topic: /arm2/middleware/command
    command_topic_ = declare_parameter<std::string>("command_topic", 
      "/arm2/middleware/command");
    // 订阅视觉提供物体位置 target_point_topic: /arm2/middleware/target_point
    target_point_topic_ = declare_parameter<std::string>("target_point_topic", 
      "/arm2/middleware/target_point");
    // 发布middle状态 middleware_state_topic: /arm2/middleware/state
    middleware_state_topic_ = declare_parameter<std::string>(
      "middleware_state_topic", "/arm2/middleware/state");
    // 发布命令动作接口
    task_goal_action_name_ = declare_parameter<std::string>("task_goal_action_name", 
      "/arm2_task_goal");
    // 订阅机械臂状态 /arm2_controller/state 自定义接口：rc_arm2_msgs/msg/Arm2ControllerState
    controller_state_topic_ = declare_parameter<std::string>(
      "controller_state_topic", controller_topics.controller_state_topic);
    // 订阅/arm2/payload_active 机械臂使用的模型是带负载还是空载
    payload_active_topic_ = declare_parameter<std::string>(
      "payload_active_topic", controller_topics.payload_active_topic);
    // 发布/arm2/payload_attached 希望使用的模型是带负载还是空载
    payload_command_topic_ = declare_parameter<std::string>(
      "payload_command_topic", controller_topics.payload_command_topic);
    // 发布/arm2/vacuum_activate 开关吸盘
    vacuum_activate_topic_ = declare_parameter<std::string>(
      "vacuum_activate_topic", controller_topics.vacuum_activate_topic);

    planning_time_ = declare_parameter<double>("planning_time", 5.0);
    action_server_timeout_sec_ = declare_parameter<double>("action_server_timeout_sec", 2.0);
    action_result_timeout_sec_ = declare_parameter<double>("action_result_timeout_sec", 30.0);
    payload_wait_timeout_sec_ = declare_parameter<double>("payload_wait_timeout_sec", 5.0);

    // 暂存点位
    staging_pose_ = parse_pose4(
      declare_parameter<std::vector<double>>("staging_pose", std::vector<double>{0.18, -0.04, 0.24, 0.0}),
      "staging_pose");
    // 默认（初始）点位
    default_pose_ = parse_pose4(
      declare_parameter<std::vector<double>>("default_pose", std::vector<double>{0.10, -0.04, 0.24, 0.0}),
      "default_pose");
    //仓库点位
    warehouse_poses_ = parse_pose4_list(
      declare_parameter<std::vector<double>>(
        "warehouse_poses",
        std::vector<double>{0.25, -0.12, 0.24, 0.0, 0.25, 0.04, 0.24, 0.0}),
      "warehouse_poses");
    
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
    if (warehouse_poses_.empty()) {
      throw std::runtime_error("warehouse_poses must contain at least one pose");
    }
    // 发布middle状态 middleware_state_topic: /arm2/middleware/state
    state_pub_ = create_publisher<Arm2MiddlewareState>(middleware_state_topic_, 10);
    // 发布/arm2/payload_attached 当前使用的模型是带负载还是空载
    payload_command_pub_ = create_publisher<std_msgs::msg::Bool>(payload_command_topic_, 10);
    // 发布/arm2/vacuum_activate 开关吸盘
    vacuum_activate_pub_ = create_publisher<std_msgs::msg::Bool>(vacuum_activate_topic_, 10);
    // 订阅视觉提供物体位置 target_point_topic: /arm2/middleware/target_point
    target_point_sub_ = create_subscription<geometry_msgs::msg::Point>(
      target_point_topic_, 10,
      std::bind(&Arm2MiddlewareNode::target_point_callback, this, std::placeholders::_1));
    // 接收外部发布命令话题 command_topic: /arm2/middleware/command
    command_sub_ = create_subscription<Arm2MiddlewareCommand>(
      command_topic_, 10,
      std::bind(&Arm2MiddlewareNode::command_callback, this, std::placeholders::_1));
    // 订阅机械臂状态 /arm2_controller/state 自定义接口：rc_arm2_msgs/msg/Arm2ControllerState
    controller_state_sub_ = create_subscription<Arm2ControllerState>(
      controller_state_topic_, 10,
      std::bind(&Arm2MiddlewareNode::controller_state_callback, this, std::placeholders::_1));
    // 订阅/arm2/payload_active 当前使用的模型是带负载还是空载
    payload_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      payload_active_topic_, 10,
      std::bind(&Arm2MiddlewareNode::payload_active_callback, this, std::placeholders::_1));
    
    // 命令动作接口
    task_goal_client_ = rclcpp_action::create_client<PlanToTaskGoal>(this, task_goal_action_name_);
    // 周期性检查等待中的异步事件是否已就绪或超时。
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Arm2MiddlewareNode::timer_callback, this));

    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "arm2_middleware ready. command_topic=%s target_point_topic=%s action=%s",
      command_topic_.c_str(), target_point_topic_.c_str(), task_goal_action_name_.c_str());
  }

private:
  enum class MacroState
  {
    NONE, //无
    PICK, //抓取
    PLACE,//放置
    DEFAULT,//默认位置
    RECOVERING_DEFAULT,//错误
  };

  enum class WaitState
  {
    IDLE,
    WAITING_FOR_ACTION_SERVER,
    WAITING_FOR_ACTION_RESULT,
    WAITING_FOR_PAYLOAD_STATE,
  };

  /**
   * @brief 接收并缓存最新的目标点。
   *
   * @param msg 来自视觉或上层模块的三维目标点消息。
   * @return void 无返回值。
   *
   * @details
   * 该目标点不会立即触发动作，只会在后续 `PICK` 宏动作真正执行抓取步骤时被读取。
   */
  void target_point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    try {
      latest_target_point_ = *msg;
      have_target_point_ = true;
      publish_state();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "target_point_callback failed: %s", error.what());
      enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
    }
  }

  /**
   * @brief 接收上层宏动作命令并启动对应状态机。
   *
   * @param msg middleware 命令消息，描述要执行的宏动作模式以及相关参数。
   * @return void 无返回值。
   *
   * @details
   * - `PICK` 会进入抓取流程
   * - `PLACE` 会进入放置流程
   * - `DEFAULT` 会进入回默认点流程
   * - 如果当前正忙，则不抢占，而是进入错误恢复流程
   */
  void command_callback(const Arm2MiddlewareCommand::SharedPtr msg)
  {
    try {
      // 无
      if (msg->mode == Arm2MiddlewareCommand::MODE_NONE) {
        publish_state();
        return;
      }
      // 忙就拒绝执行
      if (busy_) {
        RCLCPP_WARN(get_logger(), "Received new middleware command while busy; starting recovery");
        enter_error_recovery(Arm2MiddlewareState::ERROR_BUSY_REJECTED);
        return;
      }

      // 命令执行状态
      active_command_mode_ = msg->mode;
      // 放置点位序号
      active_warehouse_index_ = msg->warehouse_index;
      // J4目标角度（不用）
      active_target_spin_ = msg->target_spin;

      switch (msg->mode) {
        // 抓取
        case Arm2MiddlewareCommand::MODE_PICK:
          start_macro(MacroState::PICK, 0);
          return;
        // 放置
        case Arm2MiddlewareCommand::MODE_PLACE:
          if (msg->warehouse_index < 0 ||
            static_cast<std::size_t>(msg->warehouse_index) >= warehouse_poses_.size())
          {
            RCLCPP_WARN(get_logger(), "Received invalid warehouse index %d", msg->warehouse_index);
            enter_error_recovery(Arm2MiddlewareState::ERROR_INVALID_WAREHOUSE_INDEX);
            return;
          }
          start_macro(MacroState::PLACE, 0);
          return;
        // 默认点位
        case Arm2MiddlewareCommand::MODE_DEFAULT:
          start_macro(MacroState::DEFAULT, 10);
          return;
        // 错误恢复，回到默认点位->无
        default:
          RCLCPP_WARN(get_logger(), "Received unknown middleware mode %d", msg->mode);
          enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
          return;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "command_callback failed: %s", error.what());
      enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
    }
  }

  /**
   * @brief 处理底层控制器状态更新。
   *
   * @param msg `Arm2ControllerState` 状态消息。
   * @return void 无返回值。
   *
   * @details
   * 该回调会持续缓存控制器状态；当底层控制器进入 `FAULT` 时，
   * middleware 会立即转入“回默认点”的恢复路径。
   */
  void controller_state_callback(const Arm2ControllerState::SharedPtr msg)
  {
    try {
      latest_controller_state_ = *msg;
      if (msg->controller_state == Arm2ControllerState::CONTROLLER_STATE_FAULT) {
        RCLCPP_WARN(get_logger(), "Controller entered FAULT; starting recovery-to-default");
        enter_error_recovery(Arm2MiddlewareState::ERROR_CONTROLLER_FAULT);
        return;
      }
      publish_state();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "controller_state_callback failed: %s", error.what());
      enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
    }
  }

  /**
   * @brief 处理当前带载状态更新。
   *
   * @param msg `std_msgs::msg::Bool`，`true` 表示当前动力学模型为带载。
   * @return void 无返回值。
   *
   * @details
   * 当状态机正等待 `/arm2/payload_active` 与期望状态一致时，
   * 收到匹配的状态值会推动状态机进入下一步。
   */
  void payload_active_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    try {
      payload_active_ = msg->data;
      if (wait_state_ == WaitState::WAITING_FOR_PAYLOAD_STATE &&
        payload_active_ == desired_payload_active_)
      {
        wait_state_ = WaitState::IDLE;
        step_index_ += 1;
        advance_state_machine();
        return;
      }
      publish_state();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "payload_active_callback failed: %s", error.what());
      enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
    }
  }

  /**
   * @brief 周期性检查等待中的异步事件是否已就绪或超时。
   *
   * @return void 无返回值。
   *
   * @details
   * 该定时器不会主动推进正常流程，只负责三类等待状态：
   * - 等待 action server 可用
   * - 等待 `/arm2_task_goal` 返回 result
   * - 等待 `/arm2/payload_active` 与期望载荷状态一致
   */
  void timer_callback()
  {
    try {
      // 空闲
      if (wait_state_ == WaitState::IDLE) {
        return;
      }

      const double elapsed = (now() - wait_started_).seconds();
      switch (wait_state_) {
        // 
        case WaitState::WAITING_FOR_ACTION_SERVER:
          if (task_goal_client_->action_server_is_ready()) {
            if (!pending_task_goal_) {
              enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
              return;
            }
            const auto pose = *pending_task_goal_;
            pending_task_goal_.reset();
            dispatch_task_goal(pose.pose);
            return;
          }
          if (elapsed >= action_server_timeout_sec_) {
            RCLCPP_WARN(get_logger(), "Timed out waiting for /arm2_task_goal action server");
            enter_error_recovery(Arm2MiddlewareState::ERROR_ACTION_SERVER_UNAVAILABLE);
          }
          return;
        // 等待动作执行完成
        case WaitState::WAITING_FOR_ACTION_RESULT:
          if (elapsed >= action_result_timeout_sec_) {
            RCLCPP_WARN(get_logger(), "Timed out waiting for /arm2_task_goal result");
            enter_error_recovery(Arm2MiddlewareState::ERROR_ACTION_TIMEOUT);
          }
          return;
        // 等待带载状态切换
        case WaitState::WAITING_FOR_PAYLOAD_STATE:
          if (elapsed >= payload_wait_timeout_sec_) {
            RCLCPP_WARN(
              get_logger(),
              "Timed out waiting for payload_active=%s",
              desired_payload_active_ ? "true" : "false");
            enter_error_recovery(Arm2MiddlewareState::ERROR_PAYLOAD_WAIT_TIMEOUT);
          }
          return;
        // 空闲
        case WaitState::IDLE:
          return;
      }
    }
    // 错误恢复 
    catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "timer_callback failed: %s", error.what());
      enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
    }
  }

  /**
   * @brief 启动一个新的宏动作。
   *
   * @param macro_state 要进入的宏状态。
   * @param step_index 该宏状态的起始步骤号。
   * @return void 无返回值。
   *
   * @details
   * 该函数会重置等待状态、清空挂起的 action goal，并立即开始推进状态机。
   */
  void start_macro(MacroState macro_state, int step_index)
  {
    macro_state_ = macro_state;
    step_index_ = step_index;
    busy_ = true;
    in_error_recovery_ = false;
    wait_state_ = WaitState::IDLE;
    pending_task_goal_.reset();
    invalidate_active_goal();
    publish_state();
    advance_state_machine();
  }

  /**
   * @brief 驱动当前宏动作状态机向前执行。
   *
   * @return void 无返回值。
   *
   * @details
   * 该函数会根据 `macro_state_` 分派到不同的流程推进函数；
   * 如果某一步需要等待异步事件，则会暂停在当前步骤，等待后续回调或超时定时器继续处理。
   */
  void advance_state_machine()
  {
    while (busy_) {
      bool progressed = false;
      try {
        switch (macro_state_) {
          case MacroState::NONE:
            return;
          case MacroState::PICK:
            progressed = advance_pick();
            break;
          case MacroState::PLACE:
            progressed = advance_place();
            break;
          case MacroState::DEFAULT:
            progressed = advance_default(false);
            break;
          case MacroState::RECOVERING_DEFAULT:
            progressed = advance_default(true);
            break;
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "advance_state_machine failed: %s", error.what());
        enter_error_recovery(Arm2MiddlewareState::ERROR_INTERNAL_EXCEPTION);
        return;
      }
      publish_state();
      if (!progressed) {
        return;
      }
    }
  }

  /**
   * @brief 推进抓取宏动作的状态机。
   *
   * @return bool `true` 表示本次调用已同步完成一步并可继续推进；
   * `false` 表示需要等待异步事件或流程已结束。
   *
   * @details
   * 抓取流程包含：打开吸盘、抓取目标点、切换带载、移动到暂存点、切回卸载、
   * 关闭吸盘、返回默认点。
   */
  bool advance_pick()
  {
    switch (step_index_) {
      case 0:
        step_index_ = 1;
        return true;
      // 开吸盘
      case 1:
        publish_vacuum(true);
        step_index_ = 2;
        return true;
      // 发布目标点位
      case 2:
        if (!have_target_point_) {
          RCLCPP_WARN(get_logger(), "PICK requested without a cached target point");
          enter_error_recovery(Arm2MiddlewareState::ERROR_NO_TARGET_POINT);
          return false;
        }
        start_task_goal(Pose4D{
          latest_target_point_.x,
          latest_target_point_.y,
          latest_target_point_.z,
          active_target_spin_});
        step_index_ = 3;
        return false;
      // 切换负载
      case 4:
        publish_payload_command(true);
        step_index_ = 5;
        return true;
      // 等待切换
      case 5:
        if (payload_active_ == true) {
          step_index_ = 6;
          return true;
        }
        desired_payload_active_ = true;
        wait_state_ = WaitState::WAITING_FOR_PAYLOAD_STATE;
        wait_started_ = now();
        return false;
      // 暂存点位
      case 6:
        start_task_goal(staging_pose_);
        step_index_ = 7;
        return false;
      // 切换负载
      case 8:
        publish_payload_command(false);
        step_index_ = 9;
        return true;
      // 等待切换
      case 9:
        if (payload_active_ == false) {
          step_index_ = 10;
          return true;
        }
        desired_payload_active_ = false;
        wait_state_ = WaitState::WAITING_FOR_PAYLOAD_STATE;
        wait_started_ = now();
        return false;
      // 关闭吸盘
      case 10:
        publish_vacuum(false);
        step_index_ = 11;
        return true;
      // 回到默认
      case 11:
        start_task_goal(default_pose_);
        step_index_ = 12;
        return false;
      // 无
      case 13:
        finish_to_none();
        return false;
      default:
        return false;
    }
  }

  /**
   * @brief 推进放置宏动作的状态机。
   *
   * @return bool `true` 表示本次调用已同步完成一步并可继续推进；
   * `false` 表示需要等待异步事件或流程已结束。
   *
   * @details
   * 放置流程包含：打开吸盘、移动到暂存点、切换带载、移动到指定仓库点、
   * 关闭带载、关闭吸盘、返回默认点。
   */
  bool advance_place()
  {
    switch (step_index_) {
      case 0:
        step_index_ = 1;
        return true;
      case 1:
        publish_vacuum(true);
        step_index_ = 2;
        return true;
      case 2:
        start_task_goal(staging_pose_);
        step_index_ = 3;
        return false;
      case 4:
        publish_payload_command(true);
        step_index_ = 5;
        return true;
      case 5:
        if (payload_active_ == true) {
          step_index_ = 6;
          return true;
        }
        desired_payload_active_ = true;
        wait_state_ = WaitState::WAITING_FOR_PAYLOAD_STATE;
        wait_started_ = now();
        return false;
      case 6:
        if (active_warehouse_index_ < 0 ||
          static_cast<std::size_t>(active_warehouse_index_) >= warehouse_poses_.size())
        {
          enter_error_recovery(Arm2MiddlewareState::ERROR_INVALID_WAREHOUSE_INDEX);
          return false;
        }
        start_task_goal(warehouse_poses_[static_cast<std::size_t>(active_warehouse_index_)]);
        step_index_ = 7;
        return false;
      case 8:
        publish_payload_command(false);
        step_index_ = 9;
        return true;
      case 9:
        if (payload_active_ == false) {
          step_index_ = 10;
          return true;
        }
        desired_payload_active_ = false;
        wait_state_ = WaitState::WAITING_FOR_PAYLOAD_STATE;
        wait_started_ = now();
        return false;
      case 10:
        publish_vacuum(false);
        step_index_ = 11;
        return true;
      case 11:
        start_task_goal(default_pose_);
        step_index_ = 12;
        return false;
      case 13:
        finish_to_none();
        return false;
      default:
        return false;
    }
  }

  /**
   * @brief 推进“回默认点”或“错误恢复回默认点”流程。
   *
   * @param recovering_default `true` 表示当前处于错误恢复状态；
   * `false` 表示普通的 `DEFAULT` 命令。
   * @return bool `true` 表示同步推进成功且仍可继续；`false` 表示需要等待异步事件或已结束。
   *
   * @details
   * 该流程只发一个默认点 action，不再切换吸盘或载荷状态。
   */
  bool advance_default(bool recovering_default)
  {
    if (step_index_ == 10) {
      start_task_goal(default_pose_);
      step_index_ = 11;
      return false;
    }
    if (step_index_ == 12) {
      if (recovering_default) {
        finish_to_none();
      } else {
        finish_to_none();
      }
      return false;
    }
    return false;
  }

  /**
   * @brief 启动一次 `/arm2_task_goal` 请求。
   *
   * @param pose 要发送给 `PlanToTaskGoal` 的 4D 目标位姿。
   * @return void 无返回值。
   *
   * @details
   * 如果 action server 已可用，则立即发送；
   * 否则先把目标缓存到 `pending_task_goal_`，并进入等待 server 可用状态。
   */
  void start_task_goal(const Pose4D & pose)
  {
    if (task_goal_client_->action_server_is_ready()) {
      dispatch_task_goal(pose);
      return;
    }
    pending_task_goal_ = PendingTaskGoal{pose};
    wait_state_ = WaitState::WAITING_FOR_ACTION_SERVER;
    wait_started_ = now();
  }

  /**
   * @brief 真正向 `/arm2_task_goal` action server 发送 goal。
   *
   * @param pose 目标 4D 位姿，包含位置和 spin。
   * @return void 无返回值。
   *
   * @details
   * 该函数会：
   * - 组装 `PlanToTaskGoal::Goal`
   * - 注册 goal response 和 result 回调
   * - 进入等待 result 状态
   */
  void dispatch_task_goal(const Pose4D & pose)
  {
    PlanToTaskGoal::Goal goal;
    goal.target_xyz = {pose.x, pose.y, pose.z};
    goal.target_spin = pose.spin;
    goal.planning_time = planning_time_;
    goal.execute = true;

    const std::uint64_t sequence = ++goal_sequence_;
    wait_state_ = WaitState::WAITING_FOR_ACTION_RESULT;
    wait_started_ = now();
    active_goal_handle_.reset();

    auto options = rclcpp_action::Client<PlanToTaskGoal>::SendGoalOptions();
    options.goal_response_callback =
      [this, sequence](GoalHandlePlanToTaskGoal::SharedPtr goal_handle)
      {
        if (sequence != goal_sequence_) {
          return;
        }
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "Task goal was rejected by /arm2_task_goal");
          enter_error_recovery(Arm2MiddlewareState::ERROR_GOAL_REJECTED);
          return;
        }
        active_goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this, sequence](const GoalHandlePlanToTaskGoal::WrappedResult & result)
      {
        if (sequence != goal_sequence_) {
          return;
        }
        active_goal_handle_.reset();
        wait_state_ = WaitState::IDLE;
        last_task_goal_success_ = result.result ? result.result->success : false;
        last_task_goal_error_code_ = result.result ? result.result->error_code : -1;

        if (macro_state_ == MacroState::RECOVERING_DEFAULT) {
          if (!last_task_goal_success_) {
            last_error_code_ = Arm2MiddlewareState::ERROR_DEFAULT_RECOVERY_FAILED;
          }
          step_index_ = 12;
          publish_state();
          finish_to_none();
          return;
        }

        if (step_index_ == 3 || step_index_ == 7 || step_index_ == 11 || step_index_ == 12) {
          step_index_ += 1;
          advance_state_machine();
          return;
        }
        publish_state();
      };

    task_goal_client_->async_send_goal(goal, options);
  }

  /**
   * @brief 发布动力学模型切换请求。
   *
   * @param attached `true` 表示请求切到带载模型，`false` 表示请求切回空载模型。
   * @return void 无返回值。
   */
  void publish_payload_command(bool attached)
  {
    std_msgs::msg::Bool msg;
    msg.data = attached;
    payload_command_pub_->publish(msg);
  }

  /**
   * @brief 发布吸盘开关命令。
   *
   * @param enabled `true` 表示打开吸盘，`false` 表示关闭吸盘。
   * @return void 无返回值。
   */
  void publish_vacuum(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    vacuum_activate_pub_->publish(msg);
  }

  /**
   * @brief 进入统一错误恢复流程。
   *
   * @param error_code 触发恢复的错误码。
   * @return void 无返回值。
   *
   * @details
   * middleware 的策略是：发生可处理错误时，不直接空闲，而是先切到
   * `RECOVERING_DEFAULT`，尝试把机械臂送回默认点。
   */
  void enter_error_recovery(int error_code)
  {
    if (macro_state_ == MacroState::RECOVERING_DEFAULT) {
      last_error_code_ = Arm2MiddlewareState::ERROR_DEFAULT_RECOVERY_FAILED;
      finish_to_none();
      return;
    }

    last_error_code_ = error_code;
    in_error_recovery_ = true;
    busy_ = true;
    macro_state_ = MacroState::RECOVERING_DEFAULT;
    step_index_ = 10;
    pending_task_goal_.reset();
    wait_state_ = WaitState::IDLE;
    cancel_active_goal_if_any();
    publish_state();
    advance_state_machine();
  }

  /**
   * @brief 如果当前存在活动的 task goal，则尝试取消它。
   *
   * @return void 无返回值。
   *
   * @details
   * 无论取消请求是否最终成功，函数都会立即使本地保存的 goal 句柄失效，
   * 避免旧回调继续干扰当前状态机。
   */
  void cancel_active_goal_if_any()
  {
    if (active_goal_handle_) {
      task_goal_client_->async_cancel_goal(active_goal_handle_);
    }
    invalidate_active_goal();
  }

  /**
   * @brief 使当前保存的 action goal 句柄失效。
   *
   * @return void 无返回值。
   *
   * @details
   * 通过增加 `goal_sequence_` 的方式，让旧 goal 的异步回调自动失效。
   */
  void invalidate_active_goal()
  {
    ++goal_sequence_;
    active_goal_handle_.reset();
  }

  /**
   * @brief 结束当前流程并回到空闲态。
   *
   * @return void 无返回值。
   *
   * @details
   * 该函数会清理等待状态、挂起 goal、活动命令上下文，并发布最终的空闲状态。
   */
  void finish_to_none()
  {
    wait_state_ = WaitState::IDLE;
    desired_payload_active_ = false;
    pending_task_goal_.reset();
    invalidate_active_goal();
    macro_state_ = MacroState::NONE;
    step_index_ = kIdleStep;
    busy_ = false;
    in_error_recovery_ = false;
    active_command_mode_ = Arm2MiddlewareCommand::MODE_NONE;
    active_warehouse_index_ = -1;
    active_target_spin_ = 0.0;
    publish_state();
  }

  /**
   * @brief 发布 middleware 当前的结构化状态。
   *
   * @return void 无返回值。
   *
   * @details
   * 发布内容包括：
   * - 当前宏状态
   * - 当前步骤号
   * - 是否忙 / 是否处于错误恢复
   * - 最近错误码
   * - 当前命令上下文
   * - 最近一次 task goal 的结果摘要
   */
  void publish_state()
  {
    Arm2MiddlewareState msg;
    msg.macro_state = macro_state_to_msg(macro_state_);
    msg.step_index = step_index_;
    msg.busy = busy_;
    msg.in_error_recovery = in_error_recovery_;
    msg.last_error_code = last_error_code_;
    msg.active_command_mode = active_command_mode_;
    msg.warehouse_index = active_warehouse_index_;
    msg.controller_state = latest_controller_state_.controller_state;
    msg.payload_active = payload_active_;
    msg.last_task_goal_success = last_task_goal_success_;
    msg.last_task_goal_error_code = last_task_goal_error_code_;
    state_pub_->publish(msg);
  }

  /**
   * @brief 将内部宏状态枚举转换为消息中定义的整型常量。
   *
   * @param macro_state 内部宏状态枚举值。
   * @return int32_t `Arm2MiddlewareState.msg` 中的 `MACRO_STATE_*` 常量值。
   */
  int32_t macro_state_to_msg(MacroState macro_state) const
  {
    switch (macro_state) {
      case MacroState::NONE:
        return Arm2MiddlewareState::MACRO_STATE_NONE;
      case MacroState::PICK:
        return Arm2MiddlewareState::MACRO_STATE_PICK;
      case MacroState::PLACE:
        return Arm2MiddlewareState::MACRO_STATE_PLACE;
      case MacroState::DEFAULT:
        return Arm2MiddlewareState::MACRO_STATE_DEFAULT;
      case MacroState::RECOVERING_DEFAULT:
        return Arm2MiddlewareState::MACRO_STATE_RECOVERING_DEFAULT;
    }
    return Arm2MiddlewareState::MACRO_STATE_NONE;
  }

  std::string command_topic_;
  std::string target_point_topic_;
  std::string middleware_state_topic_;
  std::string task_goal_action_name_;
  std::string controller_state_topic_;
  std::string payload_active_topic_;
  std::string payload_command_topic_;
  std::string vacuum_activate_topic_;

  double planning_time_{5.0};
  double action_server_timeout_sec_{2.0};
  double action_result_timeout_sec_{30.0};
  double payload_wait_timeout_sec_{5.0};

  Pose4D staging_pose_{};
  Pose4D default_pose_{};
  std::vector<Pose4D> warehouse_poses_;

  rclcpp::Publisher<Arm2MiddlewareState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vacuum_activate_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_point_sub_;
  rclcpp::Subscription<Arm2MiddlewareCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Arm2ControllerState>::SharedPtr controller_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_active_sub_;

  rclcpp_action::Client<PlanToTaskGoal>::SharedPtr task_goal_client_;
  rclcpp::TimerBase::SharedPtr timer_;


  // 视觉提高最新的抓取点点
  geometry_msgs::msg::Point latest_target_point_{};
  bool have_target_point_{false};
  Arm2ControllerState latest_controller_state_{};
  bool payload_active_{false};

  MacroState macro_state_{MacroState::NONE};
  WaitState wait_state_{WaitState::IDLE};
  int step_index_{kIdleStep};
  bool busy_{false};
  bool in_error_recovery_{false};
  bool desired_payload_active_{false};
  int32_t last_error_code_{Arm2MiddlewareState::ERROR_NONE};
  int32_t active_command_mode_{Arm2MiddlewareCommand::MODE_NONE};
  int32_t active_warehouse_index_{-1};
  double active_target_spin_{0.0};
  bool last_task_goal_success_{false};
  int32_t last_task_goal_error_code_{0};
  rclcpp::Time wait_started_{0, 0, RCL_ROS_TIME};
  std::optional<PendingTaskGoal> pending_task_goal_;
  GoalHandlePlanToTaskGoal::SharedPtr active_goal_handle_;
  std::uint64_t goal_sequence_{0};
};

/**
 * @brief middleware 节点程序入口。
 *
 * @param argc 命令行参数个数。
 * @param argv 命令行参数数组。
 * @return int 程序退出码，正常情况下返回 0。
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Arm2MiddlewareNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
