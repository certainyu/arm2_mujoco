#include "dmbot_serial/usb2canfd_dm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
std::vector<float> to_float_vector(
  const std::vector<double> & values,
  std::size_t expected_size,
  const std::string & name)
{
  if (values.size() != expected_size) {
    throw std::runtime_error(
            name + " must contain exactly " + std::to_string(expected_size) + " values");
  }

  std::vector<float> out;
  out.reserve(values.size());
  for (double value : values) {
    out.push_back(static_cast<float>(value));
  }
  return out;
}
}  // namespace

Usb2canfdDMNode::Usb2canfdDMNode()
: Node("usb2canfd_dm_node")
{
  this->declare_parameter<std::string>("sn", "9940F4E149D904A69924737E3DE6629F");
  this->declare_parameter<int64_t>("nom_baud", 1000000);
  this->declare_parameter<int64_t>("dat_baud", 2000000);
  this->declare_parameter<std::vector<int64_t>>(
    "motor_ids", std::vector<int64_t>{0x01, 0x02, 0x03, 0x04});
  this->declare_parameter<std::vector<int64_t>>(
    "master_ids", std::vector<int64_t>{0x101, 0x102, 0x103, 0x104});
  this->declare_parameter<std::vector<int64_t>>(
    "motor_types", std::vector<int64_t>{1, 1, 1, 1});
  this->declare_parameter<int64_t>("control_mode", 0);
  this->declare_parameter<std::string>("command_topic", "/arm2/command/effort");
  this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
  this->declare_parameter<std::vector<double>>("kp", std::vector<double>{10.0, 15.0, 15.0, 1.0});
  this->declare_parameter<std::vector<double>>("kd", std::vector<double>{0.5, 0.95, 0.95, 0.03});

  const auto sn = this->get_parameter("sn").as_string();
  const auto nom_baud = static_cast<uint32_t>(this->get_parameter("nom_baud").as_int());
  const auto dat_baud = static_cast<uint32_t>(this->get_parameter("dat_baud").as_int());
  const auto command_topic = this->get_parameter("command_topic").as_string();
  const auto joint_state_topic = this->get_parameter("joint_state_topic").as_string();
  const auto motor_ids_param = this->get_parameter("motor_ids").as_integer_array();
  const auto master_ids_param = this->get_parameter("master_ids").as_integer_array();
  const auto motor_types_param = this->get_parameter("motor_types").as_integer_array();
  const auto control_mode_value = static_cast<int>(this->get_parameter("control_mode").as_int());
  const auto kp_param = this->get_parameter("kp").as_double_array();
  const auto kd_param = this->get_parameter("kd").as_double_array();

  switch (control_mode_value) {
    case 0:
      control_mode_ = damiao::MIT_MODE;
      break;
    case 1:
      control_mode_ = damiao::POS_VEL_MODE;
      break;
    case 2:
      control_mode_ = damiao::VEL_MODE;
      break;
    case 3:
      control_mode_ = damiao::POS_FORCE_MODE;
      break;
    default:
      control_mode_ = damiao::VEL_MODE;
      break;
  }

  RCLCPP_INFO(this->get_logger(), "Initializing DM Motor Driver with SN: %s", sn.c_str());

  const std::size_t motor_num = std::min(
    motor_ids_param.size(), std::min(master_ids_param.size(), motor_types_param.size()));
  init_data_.reserve(motor_num);
  motor_ids_.reserve(motor_num);

  for (std::size_t i = 0; i < motor_num; ++i) {
    const auto can_id = static_cast<uint16_t>(motor_ids_param[i]);
    const auto master_id = static_cast<uint16_t>(master_ids_param[i]);
    const auto motor_type_val = static_cast<int>(motor_types_param[i]);

    damiao::DM_Motor_Type motor_type = damiao::DM4310;
    if (motor_type_val == 3) {
      motor_type = damiao::DM4340;
    }

    damiao::DmActData data{};
    data.motorType = motor_type;
    data.mode = control_mode_;
    data.can_id = can_id;
    data.mst_id = master_id;
    init_data_.push_back(data);

    motor_id_to_index_[can_id] = i;
    motor_ids_.push_back(can_id);
  }

  RCLCPP_INFO(this->get_logger(), "Motor IDs count: %zu", motor_ids_.size());

  try {
    motor_control_ = std::make_shared<damiao::Motor_Control>(nom_baud, dat_baud, sn, &init_data_);
    RCLCPP_INFO(this->get_logger(), "MotorControl initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MotorControl: %s", e.what());
    throw;
  }

  motor_count_ = motor_ids_.size();
  kp_arr_ = to_float_vector(kp_param, motor_count_, "kp");
  kd_arr_ = to_float_vector(kd_param, motor_count_, "kd");
  

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  
  command_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    command_topic, qos,
    std::bind(&Usb2canfdDMNode::command_callback, this, std::placeholders::_1));

  joint_state_publisher_ =
    this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&Usb2canfdDMNode::publish_joint_state, this));

  RCLCPP_INFO(
    this->get_logger(),
    "DM Motor Driver Node initialized. command_topic=%s joint_state_topic=%s",
    command_topic.c_str(), joint_state_topic.c_str());
}

Usb2canfdDMNode::~Usb2canfdDMNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down DM Motor Driver Node");
  motor_control_.reset();
}

void Usb2canfdDMNode::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  try {
    if (
      msg->position.size() != motor_count_ ||
      msg->velocity.size() != motor_count_ ||
      msg->effort.size() != motor_count_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Command count mismatch: expected %zu, got position=%zu velocity=%zu effort=%zu",
        motor_count_, msg->position.size(), msg->velocity.size(), msg->effort.size());
      return;
    }

    std::vector<float> q_arr(motor_count_);
    std::vector<float> dq_arr(motor_count_);
    std::vector<float> tau_arr(motor_count_);
    for (std::size_t i = 0; i < motor_count_; ++i) {
      q_arr[i] = static_cast<float>(msg->position[i]);
      dq_arr[i] = static_cast<float>(msg->velocity[i]);
      tau_arr[i] = static_cast<float>(msg->effort[i]);
    }

    // 打印发送的数组
    std::ostringstream ss;
    ss << "Cmd arrays: q=[";
    for (std::size_t i = 0; i < motor_count_; ++i) {
      ss << q_arr[i];
      if (i + 1 < motor_count_) ss << ", ";
    }
    ss << "], dq=[";
    for (std::size_t i = 0; i < motor_count_; ++i) {
      ss << dq_arr[i];
      if (i + 1 < motor_count_) ss << ", ";
    }
    ss << "], tau=[";
    for (std::size_t i = 0; i < motor_count_; ++i) {
      ss << tau_arr[i];
      if (i + 1 < motor_count_) ss << ", ";
    }
    ss << "], kp=[";
    for (std::size_t i = 0; i < motor_count_; ++i) {
      ss << kp_arr_[i];
      if (i + 1 < motor_count_) ss << ", ";
    }
    ss << "], kd=[";
    for (std::size_t i = 0; i < motor_count_; ++i) {
      ss << kd_arr_[i];
      if (i + 1 < motor_count_) ss << ", ";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    motor_control_->CtrlMotors(
      q_arr.data(), dq_arr.data(), kp_arr_.data(), kd_arr_.data(), tau_arr.data());

    RCLCPP_INFO(this->get_logger(), "Motor commands sent successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected error in command_callback: %s", e.what());
  }
}

void Usb2canfdDMNode::publish_joint_state()
{
  try {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name.reserve(motor_count_);
    joint_state_msg.position.reserve(motor_count_);
    joint_state_msg.velocity.reserve(motor_count_);
    joint_state_msg.effort.reserve(motor_count_);

    for (std::size_t i = 0; i < motor_count_; ++i) {
      joint_state_msg.name.push_back("joint_" + std::to_string(i));
      joint_state_msg.position.push_back(motor_control_->current_motor_pos[i]);
      joint_state_msg.velocity.push_back(motor_control_->current_motor_vel[i]);
      joint_state_msg.effort.push_back(motor_control_->current_motor_tor[i]);
    }

    joint_state_publisher_->publish(joint_state_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected error in publish_joint_state: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Usb2canfdDMNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
