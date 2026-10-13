#ifndef HYBRID_ASTAR_TEST_PUBLISHER_TF_BROADCASTER_HPP
#define HYBRID_ASTAR_TEST_PUBLISHER_TF_BROADCASTER_HPP
#include <memory>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include "hybrid_astar_test/ros2_compat.hpp"

// Identity transform frame -> child_frame ("map" -> "path"); the ROS 1 node re-sent it at 10 Hz,
// a static transform does the same in ROS 2.
class TfBroadcaster {
public:
    TfBroadcaster(const std::string& child_frame, const std::string& frame);
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};
#endif
