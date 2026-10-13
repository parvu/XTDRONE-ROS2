#ifndef HYBRID_ASTAR_TEST_PUBLISHER_POINTS_PUBLISHER_HPP
#define HYBRID_ASTAR_TEST_PUBLISHER_POINTS_PUBLISHER_HPP
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "hybrid_astar_test/ros2_compat.hpp"

class PointsPublisher {
public:
    explicit PointsPublisher(const std::string& topic);
    void Publish(const Eigen::Vector3f& point);
private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr poses_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vector_publisher_;
};
#endif
