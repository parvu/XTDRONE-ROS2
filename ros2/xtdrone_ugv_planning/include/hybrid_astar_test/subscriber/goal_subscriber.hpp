#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_GOAL_SUBSCRIBER_HPP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_GOAL_SUBSCRIBER_HPP
#include <mutex>
#include <string>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "hybrid_astar_test/ros2_compat.hpp"

class GoalSubscriber {
public:
    explicit GoalSubscriber(const std::string& topic);
    bool GetGoal(Eigen::Vector3f& goal);
    bool HasGoal();  // a new goal is waiting (GetGoal takes it)
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
    Eigen::Vector3f goal_point;
    bool            get_goal = false;
    std::mutex      global_mtx;
};
#endif
