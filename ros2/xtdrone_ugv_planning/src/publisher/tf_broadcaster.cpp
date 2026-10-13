#include "hybrid_astar_test/publisher/tf_broadcaster.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

TfBroadcaster::TfBroadcaster(const std::string& child_frame, const std::string& frame) {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(xtd::node());
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = xtd::now();
    t.header.frame_id = frame;
    t.child_frame_id = child_frame;
    t.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(t);
}
