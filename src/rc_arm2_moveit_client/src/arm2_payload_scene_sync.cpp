#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{
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
}  // namespace

class Arm2PayloadSceneSync
{
public:
  explicit Arm2PayloadSceneSync(const rclcpp::Node::SharedPtr & node)
  : node_(node), planning_scene_interface_("", true)
  {
    payload_active_topic_ = get_or_declare<std::string>(node_, "payload_active_topic", "/arm2/payload_active");
    object_id_ = get_or_declare<std::string>(node_, "object_id", "payload_cube");
    link_name_ = get_or_declare<std::string>(node_, "link_name", "tool0");
    payload_cube_side_ = get_or_declare<double>(node_, "payload_cube_side", 0.08);

    payload_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      payload_active_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {this->payload_callback(msg);});

    RCLCPP_INFO(
      node_->get_logger(),
      "Payload planning scene sync ready. topic=%s link=%s side=%.3f",
      payload_active_topic_.c_str(), link_name_.c_str(), payload_cube_side_);
  }

private:
  void payload_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      attach_payload();
    } else {
      detach_payload();
    }
  }

  void attach_payload()
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = link_name_;
    attached.touch_links = {"tool0", "l4"};
    attached.object.id = object_id_;
    attached.object.header.frame_id = link_name_;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive cube;
    cube.type = shape_msgs::msg::SolidPrimitive::BOX;
    cube.dimensions = {payload_cube_side_, payload_cube_side_, payload_cube_side_};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.z = payload_cube_side_ * 0.5;

    attached.object.primitives.push_back(cube);
    attached.object.primitive_poses.push_back(pose);

    if (planning_scene_interface_.applyAttachedCollisionObject(attached)) {
      RCLCPP_INFO(node_->get_logger(), "Attached payload cube to %s in MoveIt scene", link_name_.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to attach payload cube in MoveIt scene");
    }
  }

  void detach_payload()
  {
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = link_name_;
    attached.object.id = object_id_;
    attached.object.header.frame_id = link_name_;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(attached);
    planning_scene_interface_.removeCollisionObjects({object_id_});
    RCLCPP_INFO(node_->get_logger(), "Detached payload cube from MoveIt scene");
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_sub_;
  std::string payload_active_topic_;
  std::string object_id_;
  std::string link_name_;
  double payload_cube_side_{0.08};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "arm2_payload_scene_sync",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto sync = std::make_shared<Arm2PayloadSceneSync>(node);
  (void)sync;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
