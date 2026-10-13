// ROS 2 port of XTDrone's hybrid_astar_test (chapter 31): the ROS 1 code created a ros::NodeHandle in many
// classes; here they all use the one node created in main().
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace xtd {
inline rclcpp::Node::SharedPtr & node() {
  static rclcpp::Node::SharedPtr n;
  return n;
}
inline rclcpp::Time now() { return node()->now(); }
inline geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x(); m.y = q.y(); m.z = q.z(); m.w = q.w();
  return m;
}
}  // namespace xtd
