#include "dmbot_serial/debugger_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

RobotCommandPublisher::RobotCommandPublisher()
: Node("robot_command_publisher")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("robot_command", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RobotCommandPublisher::timer_callback, this));

  motor_gains_ = {
    std::pair<float, float>{0.001F, 0.005F}, {5.0F, 1.0F}, {5.0F, 1.0F},
    {5.0F, 1.0F}, {0.0F, 0.0F}, {0.0F, 0.0F}};

  commands_ = create_commands();
  joint_q_limits_ = {{{-1.865F, 2.365F}, {-1.85F, 0.0F}, {-2.13861F, 0.0F}, {-0.5313F, 1.2963F}}};
  apply_q_limits_to_commands();

  running_.store(true);

  RCLCPP_INFO(this->get_logger(), "Robot Command Publisher 已启动");
  RCLCPP_INFO(this->get_logger(), "按数字键 1-9/0 切换不同的发布消息 (0对应命令10)");
  RCLCPP_INFO(this->get_logger(), "当前命令: 1");

  keyboard_thread_ = std::thread(&RobotCommandPublisher::keyboard_listener, this);
}

RobotCommandPublisher::~RobotCommandPublisher()
{
  running_.store(false);
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
}

std::vector<sensor_msgs::msg::JointState> RobotCommandPublisher::create_commands()
{
  std::vector<sensor_msgs::msg::JointState> commands;
  commands.reserve(10);

  sensor_msgs::msg::JointState cmd1;
  cmd1.position.resize(6);
  cmd1.velocity.resize(6);
  cmd1.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd1.position[i] = 0.0F;
    cmd1.velocity[i] = 0.0F;
    cmd1.effort[i] = 0.0F;
  }
  commands.push_back(cmd1);

  sensor_msgs::msg::JointState cmd2;
  cmd2.position.resize(6);
  cmd2.velocity.resize(6);
  cmd2.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd2.position[i] = (i == 1) ? -0.1F : 0.0F;
    cmd2.velocity[i] = 0.0F;
    cmd2.effort[i] = 0.0F;
  }
  commands.push_back(cmd2);

  sensor_msgs::msg::JointState cmd3;
  cmd3.position.resize(6);
  cmd3.velocity.resize(6);
  cmd3.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd3.position[i] = (i == 2) ? -0.1F : 0.0F;
    cmd3.velocity[i] = 0.0F;
    cmd3.effort[i] = 0.0F;
  }
  commands.push_back(cmd3);

  sensor_msgs::msg::JointState cmd4;
  cmd4.position.resize(6);
  cmd4.velocity.resize(6);
  cmd4.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd4.position[i] = (i == 3) ? 0.1F : 0.0F;
    cmd4.velocity[i] = 0.0F;
    cmd4.effort[i] = 0.0F;
  }
  commands.push_back(cmd4);

  sensor_msgs::msg::JointState cmd5;
  cmd5.position.resize(6);
  cmd5.velocity.resize(6);
  cmd5.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd5.position[i] = 0.0F;
    cmd5.velocity[i] = 0.0F;
    cmd5.effort[i] = 0.0F;
  }
  commands.push_back(cmd5);

  sensor_msgs::msg::JointState cmd6;
  cmd6.position.resize(6);
  cmd6.velocity.resize(6);
  cmd6.effort.resize(6);
  for (int i = 0; i < 6; ++i) {
    cmd6.position[i] = 0.0F;
    cmd6.velocity[i] = 0.0F;
    cmd6.effort[i] = 0.0F;
  }
  commands.push_back(cmd6);

  sensor_msgs::msg::JointState cmd7;
  cmd7.position.resize(6);
  cmd7.velocity.resize(6);
  cmd7.effort.resize(6);
  const std::array<float, 6> q7{{1.2F, -1.2F, -1.4F, 0.9F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    cmd7.position[i] = q7[i];
    cmd7.velocity[i] = 0.0F;
    cmd7.effort[i] = 0.0F;
  }
  commands.push_back(cmd7);

  sensor_msgs::msg::JointState cmd8;
  cmd8.position.resize(6);
  cmd8.velocity.resize(6);
  cmd8.effort.resize(6);
  const std::array<float, 6> q8{{-1.2F, -0.4F, -0.6F, -0.3F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    cmd8.position[i] = q8[i];
    cmd8.velocity[i] = 0.0F;
    cmd8.effort[i] = 0.0F;
  }
  commands.push_back(cmd8);

  sensor_msgs::msg::JointState cmd9;
  cmd9.position.resize(6);
  cmd9.velocity.resize(6);
  cmd9.effort.resize(6);
  const std::array<float, 6> q9{{0.8F, -1.6F, -1.9F, 1.1F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    cmd9.position[i] = q9[i];
    cmd9.velocity[i] = 0.0F;
    cmd9.effort[i] = 0.0F;
  }
  commands.push_back(cmd9);

  sensor_msgs::msg::JointState cmd10;
  cmd10.position.resize(6);
  cmd10.velocity.resize(6);
  cmd10.effort.resize(6);
  const std::array<float, 6> q10{{0.2F, -0.3F, -0.5F, -0.5F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    cmd10.position[i] = q10[i];
    cmd10.velocity[i] = 0.0F;
    cmd10.effort[i] = 0.0F;
  }
  commands.push_back(cmd10);

  return commands;
}

float RobotCommandPublisher::clamp(float value, float min_value, float max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

void RobotCommandPublisher::apply_q_limits_to_commands()
{
  for (auto & cmd : commands_) {
    const std::size_t count = std::min<std::size_t>(4, cmd.position.size());
    for (std::size_t i = 0; i < count; ++i) {
      cmd.position[i] = clamp(cmd.position[i], joint_q_limits_[i].first, joint_q_limits_[i].second);
    }
  }
}

void RobotCommandPublisher::timer_callback()
{
  if (current_command_idx_ >= static_cast<int>(commands_.size())) {
    return;
  }

  auto msg = commands_[current_command_idx_];
  if (current_command_idx_ == 4) {
    const float swing = 0.1F * std::sin(swing_phase_);
    swing_phase_ += 0.15F;
    const std::array<float, 6> base_q{{0.0F, -0.2F, -0.2F, 0.2F, 0.0F, 0.0F}};
    for (int i = 0; i < 6; ++i) {
      msg.position[i] = base_q[i] + swing;
      msg.velocity[i] = 0.0F;
    }
    const std::size_t count = std::min<std::size_t>(4, msg.position.size());
    for (std::size_t i = 0; i < count; ++i) {
      msg.position[i] = clamp(msg.position[i], joint_q_limits_[i].first, joint_q_limits_[i].second);
    }
  }

  publisher_->publish(msg);
  const char * enable_status = current_command_idx_ == 5 ? "禁用" : "启用";
  RCLCPP_INFO(this->get_logger(), "发布命令 %d: %s", current_command_idx_ + 1, enable_status);
}

void RobotCommandPublisher::keyboard_listener()
{
  termios old_settings{};
  termios new_settings{};
  if (tcgetattr(STDIN_FILENO, &old_settings) != 0) {
    RCLCPP_WARN(this->get_logger(), "无法读取终端属性，键盘监听未启动");
    return;
  }

  new_settings = old_settings;
  new_settings.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 1;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

  while (running_.load() && rclcpp::ok()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
      char key = 0;
      if (read(STDIN_FILENO, &key, 1) <= 0) {
        continue;
      }
      if (key >= '1' && key <= '9') {
        current_command_idx_ = static_cast<int>(key - '1');
        const char * status = current_command_idx_ == 5 ? "禁用" : "启用";
        RCLCPP_INFO(this->get_logger(), "切换到命令 %c: %s", key, status);
      } else if (key == '0') {
        current_command_idx_ = 9;
        const char * status = current_command_idx_ == 5 ? "禁用" : "启用";
        RCLCPP_INFO(this->get_logger(), "切换到命令 10: %s", status);
      } else if (key == 'q') {
        RCLCPP_INFO(this->get_logger(), "退出程序");
        running_.store(false);
        break;
      }
    }
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
