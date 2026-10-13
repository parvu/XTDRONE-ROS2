#include "hybrid_astar_test/subscriber/est_start_subscriber.hpp"

EstStartSubscriber::EstStartSubscriber(const std::string& topic) {
    subscriber_ = xtd::node()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, 1, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(global_mtx);
            start_point(0) = msg->pose.pose.position.x;
            start_point(1) = msg->pose.pose.position.y;
            start_point(2) = static_cast<float>(tf2::getYaw(msg->pose.pose.orientation));
            get_start = true;
        });
}

bool EstStartSubscriber::GetStart(Eigen::Vector3f& start) {
    std::lock_guard<std::mutex> lock(global_mtx);
    start = start_point;
    auto flag = get_start;
    get_start = false;
    return flag;
}
