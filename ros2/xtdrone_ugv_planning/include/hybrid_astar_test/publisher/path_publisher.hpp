#ifndef HYBRID_ASTAR_TEST_PUBLISHER_PATH_PUBLISHER_H_PP
#define HYBRID_ASTAR_TEST_PUBLISHER_PATH_PUBLISHER_H_PP
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "hybrid_astar_test/ros2_compat.hpp"

class PathPublisher {
public:
    explicit PathPublisher(const std::string& topic);
    void Publish(const std::vector<Eigen::Vector3f>& path);
private:
    void PublishPathData(const std::vector<Eigen::Vector3f>& path);
    void PublishPoseData(const std::vector<Eigen::Vector3f>& path);
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_publisher_;
};
#endif
