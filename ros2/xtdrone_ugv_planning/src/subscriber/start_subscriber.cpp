#include "hybrid_astar_test/subscriber/start_subscriber.hpp"

StartSubscriber::StartSubscriber(const std::string& base_frame, const std::string& world_frame)
    : base_frame_(base_frame), world_frame_(world_frame) {
    buffer_ = std::make_unique<tf2_ros::Buffer>(xtd::node()->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, xtd::node(), false);
}

bool StartSubscriber::GetStart(Eigen::Vector3f& start) {
    try {
        auto t = buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
        start(0) = t.transform.translation.x;
        start(1) = t.transform.translation.y;
        start(2) = static_cast<float>(tf2::getYaw(t.transform.rotation));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(xtd::node()->get_logger(), *xtd::node()->get_clock(), 5000, "%s", ex.what());
        return false;
    }
}
