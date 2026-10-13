#include "hybrid_astar_test/publisher/path_publisher.hpp"

PathPublisher::PathPublisher(const std::string& topic) {
    // latched, so that a path follower started later still gets the last path
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    path_publisher_  = xtd::node()->create_publisher<nav_msgs::msg::Path>(topic + "_path", qos);
    poses_publisher_ = xtd::node()->create_publisher<geometry_msgs::msg::PoseArray>(topic + "_poses", qos);
}

void PathPublisher::Publish(const std::vector<Eigen::Vector3f>& path) {
    PublishPathData(path);
    PublishPoseData(path);
}

void PathPublisher::PublishPathData(const std::vector<Eigen::Vector3f>& path) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = xtd::now();
    for (size_t i = 0; i < path.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = path[i].x();
        pose.pose.position.y = path[i].y();
        pose.pose.orientation = xtd::yawQuaternion(path[i].z());
        path_msg.poses.push_back(pose);
    }
    path_publisher_->publish(path_msg);
}

void PathPublisher::PublishPoseData(const std::vector<Eigen::Vector3f>& path) {
    geometry_msgs::msg::PoseArray poses_msg;
    poses_msg.header.frame_id = "map";
    poses_msg.header.stamp = xtd::now();
    for (size_t i = 0; i < path.size(); i++) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = path[i].x();
        pose.position.y = path[i].y();
        pose.orientation = xtd::yawQuaternion(path[i].z());
        poses_msg.poses.push_back(pose);
    }
    poses_publisher_->publish(poses_msg);
}
