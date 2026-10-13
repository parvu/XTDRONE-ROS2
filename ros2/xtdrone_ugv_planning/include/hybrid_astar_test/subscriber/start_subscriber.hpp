#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_START_SUBSCRIBER_HPP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_START_SUBSCRIBER_HPP
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "hybrid_astar_test/ros2_compat.hpp"

// The vehicle pose from TF (world_frame -> base_frame), like the ROS 1 thread that polled the listener.
class StartSubscriber {
public:
    StartSubscriber(const std::string& base_frame, const std::string& world_frame);
    bool GetStart(Eigen::Vector3f& start);
private:
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::string base_frame_, world_frame_;
};
#endif
