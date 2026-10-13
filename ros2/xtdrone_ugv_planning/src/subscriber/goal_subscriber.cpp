#include "hybrid_astar_test/subscriber/goal_subscriber.hpp"

GoalSubscriber::GoalSubscriber(const std::string& topic) {
    subscriber_ = xtd::node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(global_mtx);
            goal_point(0) = msg->pose.position.x;
            goal_point(1) = msg->pose.position.y;
            goal_point(2) = static_cast<float>(tf2::getYaw(msg->pose.orientation));
            get_goal = true;
        });
}

bool GoalSubscriber::GetGoal(Eigen::Vector3f& goal) {
    std::lock_guard<std::mutex> lock(global_mtx);
    if (get_goal)
        goal = goal_point;
    auto flag = get_goal;
    get_goal = false;
    return flag;
}

bool GoalSubscriber::HasGoal() {
    std::lock_guard<std::mutex> lock(global_mtx);
    return get_goal;
}
