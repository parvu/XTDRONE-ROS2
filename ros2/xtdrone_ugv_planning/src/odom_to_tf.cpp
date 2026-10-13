// Odometry -> TF (odom frame -> base frame) for the UGV (chapter 31): the CAT Vehicle's wheel odometry TF of
// ROS 1, here from the Gazebo ground-truth odometry (/ugv_0/ground_truth/odom), for slam_toolbox.
//   ros2 run xtdrone_ugv_planning odom_to_tf --ros-args -p odom_topic:=/ugv_0/ground_truth/odom
#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("odom_to_tf");
  const auto topic = node->declare_parameter<std::string>("odom_topic", "/ugv_0/ground_truth/odom");
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
      topic, rclcpp::SensorDataQoS(), [broadcaster](const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;
        t.header = msg->header;
        t.child_frame_id = msg->child_frame_id;
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;
        broadcaster->sendTransform(t);
      });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
