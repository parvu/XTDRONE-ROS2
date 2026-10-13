// Hybrid A* path search for the CAT Vehicle UGV (chapter 31), ROS 2 port of XTDrone's hybrid_astar_test.
//   ros2 run xtdrone_ugv_planning hybrid_astar_node [--ros-args -p config:=<base_config.yaml>]
// Map /map (any resolution, reduced to map_resolution of the config), start from TF map -> base_frame,
// goal from /goal_pose (RViz 2D Goal Pose); smoothed path on /front_end_path.
#include <chrono>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "hybrid_astar_test/global_definition/constants.hpp"
#include "hybrid_astar_test/navigation/path_search.hpp"
#include "hybrid_astar_test/ros2_compat.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    xtd::node() = std::make_shared<rclcpp::Node>("hybrid_a_star_node");
    const std::string config = xtd::node()->declare_parameter<std::string>(
        "config", ament_index_cpp::get_package_share_directory("xtdrone_ugv_planning") + "/config/base_config.yaml");

    if (!HybridAStar::Constants::LoadParamFromYaml(config)) {
        rclcpp::shutdown();
        return -1;
    }

    auto searcher = std::make_shared<HybridAStar::HybridAStarGraphSearcher>();
    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
        rclcpp::spin_some(xtd::node());
        searcher->Run();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
