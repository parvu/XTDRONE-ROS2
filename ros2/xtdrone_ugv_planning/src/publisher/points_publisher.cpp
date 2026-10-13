#include "hybrid_astar_test/publisher/points_publisher.hpp"

PointsPublisher::PointsPublisher(const std::string& topic) {
    poses_publisher_  = xtd::node()->create_publisher<geometry_msgs::msg::PointStamped>(topic + "_point", 1);
    vector_publisher_ = xtd::node()->create_publisher<geometry_msgs::msg::PoseStamped>(topic + "_vector", 1);
}

void PointsPublisher::Publish(const Eigen::Vector3f& point) {
    geometry_msgs::msg::PoseStamped vector_msg;
    vector_msg.header.frame_id = "map";
    vector_msg.header.stamp = xtd::now();
    vector_msg.pose.position.x = point.x();
    vector_msg.pose.position.y = point.y();
    vector_msg.pose.orientation = xtd::yawQuaternion(point.z());
    vector_publisher_->publish(vector_msg);
    geometry_msgs::msg::PointStamped points_msg;
    points_msg.header = vector_msg.header;
    points_msg.point.x = point.x();
    points_msg.point.y = point.y();
    poses_publisher_->publish(points_msg);
}
