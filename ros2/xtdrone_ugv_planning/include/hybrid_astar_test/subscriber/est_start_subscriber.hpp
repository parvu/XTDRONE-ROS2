#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_EST_START_SUBSCRIBER_HPP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_EST_START_SUBSCRIBER_HPP
#include <mutex>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "hybrid_astar_test/ros2_compat.hpp"

class EstStartSubscriber {
public:
    explicit EstStartSubscriber(const std::string& topic);
    bool GetStart(Eigen::Vector3f& start);
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;
    Eigen::Vector3f start_point;
    bool            get_start = false;
    std::mutex      global_mtx;
};
#endif
