#ifndef DMBOT_SERIAL_DEBUGGER_NODE_HPP_
#define DMBOT_SERIAL_DEBUGGER_NODE_HPP_

#include <array>
#include <atomic>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RobotCommandPublisher : public rclcpp::Node
{
public:
  RobotCommandPublisher();
  ~RobotCommandPublisher() override;

private:
  std::vector<sensor_msgs::msg::JointState> create_commands();
  static float clamp(float value, float min_value, float max_value);
  void apply_q_limits_to_commands();
  void timer_callback();
  void keyboard_listener();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread keyboard_thread_;
  std::atomic<bool> running_{false};

  int current_command_idx_{0};
  float swing_phase_{0.0F};

  std::array<std::pair<float, float>, 6> motor_gains_{};
  std::vector<sensor_msgs::msg::JointState> commands_;
  std::array<std::pair<float, float>, 4> joint_q_limits_{};
};

#endif  // DMBOT_SERIAL_DEBUGGER_NODE_HPP_
