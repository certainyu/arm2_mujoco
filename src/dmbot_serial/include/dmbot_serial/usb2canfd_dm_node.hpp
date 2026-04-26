#ifndef DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_
#define DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "dmbot_serial/protocol/damiao.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class Usb2canfdDMNode : public rclcpp::Node
{
public:
  Usb2canfdDMNode();
  ~Usb2canfdDMNode() override;

private:
  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_joint_state();

  damiao::Control_Mode control_mode_{damiao::MIT_MODE};
  std::vector<damiao::DmActData> init_data_;
  std::shared_ptr<damiao::Motor_Control> motor_control_;
  std::vector<uint16_t> motor_ids_;
  std::unordered_map<uint16_t, std::size_t> motor_id_to_index_;
  std::size_t motor_count_{0};
  std::vector<float> kp_arr_;
  std::vector<float> kd_arr_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_
