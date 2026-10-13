#ifndef HYBRID_ASTAR_TEST_SUBSCRIBER_COSTMAP_SUBSCRIBER_HPP
#define HYBRID_ASTAR_TEST_SUBSCRIBER_COSTMAP_SUBSCRIBER_HPP
#include <cmath>
#include <mutex>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "hybrid_astar_test/ros2_compat.hpp"

class CostmapSubscriber {
public:
    CostmapSubscriber(const std::string& topic, float cell_size, float margin, bool unknown_occupied,
                      float free_radius, float shadow_depth);
public:
    // vehicle_x/y (map frame, NAN if unknown): unknown cells within free_radius of it stay free
    bool GetCostMap(nav_msgs::msg::OccupancyGrid& map, float& map_origin_x, float& map_origin_y,
                    float vehicle_x = NAN, float vehicle_y = NAN);
private:
    void CostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
    std::mutex      global_mtx_;
    bool            get_map = false;
    float           cell_size_;
    float           margin_;
    bool            unknown_occupied_;
    float           free_radius_;
    float           shadow_depth_;
    nav_msgs::msg::OccupancyGrid costmap_;
};
#endif
