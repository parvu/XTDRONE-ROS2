#include "hybrid_astar_test/subscriber/costmap_subscriber.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

CostmapSubscriber::CostmapSubscriber(const std::string& topic, float cell_size, float margin, bool unknown_occupied,
                                     float free_radius, float shadow_depth)
    : cell_size_(cell_size), margin_(margin), unknown_occupied_(unknown_occupied), free_radius_(free_radius),
      shadow_depth_(shadow_depth) {
    // slam_toolbox and map_server latch the map (transient local)
    subscriber_ = xtd::node()->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { CostMapCallback(msg); });
}

void CostmapSubscriber::CostMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(global_mtx_);
    costmap_ = *msg;
    get_map = true;
}

// The planner works on cells of cell_size (base_config.yaml map_resolution, 1 m for the CAT Vehicle). The ROS 1
// node needed a map of exactly that resolution; a finer map (slam_toolbox: 0.1 m) is reduced here, a cell being
// occupied when any of its source cells is (> 50). Gmapping's map covered a fixed 200 m x 200 m; slam_toolbox's
// only what the lidar has seen, which may not include the vehicle itself (the SICK lidar looks forward), so the
// map gets a margin of free cells on every side.
// unknown_occupied: a cell the lidar has not seen (more than half of its source cells unknown) up to shadow_depth
// behind a seen obstacle, as seen from the vehicle, is an obstacle too. The lidar sees only the near side of a parked car; with the unseen rest
// free, the planner threaded the UGV between two cars where it does not fit. Further from obstacles unknown cells
// stay free: a map made from one place has thin unknown gaps between the lidar beams all over the road. Unknown cells within free_radius of the vehicle stay free (the SICK lidar, at the
// front, does not see around the vehicle itself), and so does the margin.
bool CostmapSubscriber::GetCostMap(nav_msgs::msg::OccupancyGrid& map, float& map_origin_x, float& map_origin_y,
                                   float vehicle_x, float vehicle_y) {
    std::lock_guard<std::mutex> lock(global_mtx_);
    if (!get_map) {
        return false;
    }
    const auto& in = costmap_;
    const int factor = std::max(1, static_cast<int>(std::lround(cell_size_ / in.info.resolution)));
    const int pad = static_cast<int>(std::ceil(margin_ / cell_size_));
    const int width = (static_cast<int>(in.info.width) + factor - 1) / factor + 2 * pad;
    const int height = (static_cast<int>(in.info.height) + factor - 1) / factor + 2 * pad;
    map.header = in.header;
    map.info = in.info;
    map.info.resolution = in.info.resolution * factor;
    if (std::fabs(map.info.resolution - cell_size_) < 1e-3f) {
        map.info.resolution = cell_size_;  // the planner compares it with cell_size exactly
    }
    map.info.width = width;
    map.info.height = height;
    map_origin_x = in.info.origin.position.x - pad * cell_size_;
    map_origin_y = in.info.origin.position.y - pad * cell_size_;
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.data.assign(static_cast<size_t>(width) * height, 0);
    std::vector<int> unknown(static_cast<size_t>(width) * height, 0);
    for (int row = 0; row < static_cast<int>(in.info.height); ++row) {
        for (int col = 0; col < static_cast<int>(in.info.width); ++col) {
            const int8_t v = in.data[col + row * in.info.width];
            const size_t i = (col / factor + pad) + (row / factor + pad) * width;
            if (v > 50) {
                map.data[i] = 100;
            } else if (v < 0) {
                ++unknown[i];
            }
        }
    }
    if (unknown_occupied_) {
        const bool have_vehicle = std::isfinite(vehicle_x) && std::isfinite(vehicle_y);
        const std::vector<int8_t> seen = map.data;  // occupied cells the lidar saw
        // in the shadow of a seen obstacle: walking from the cell towards the vehicle (where the map was scanned
        // from), an occupied cell comes within shadow_depth
        auto near_obstacle = [&](int c, int r) {
            if (!have_vehicle) {
                return false;
            }
            const float cx = map_origin_x + (c + 0.5f) * cell_size_, cy = map_origin_y + (r + 0.5f) * cell_size_;
            const float dx = vehicle_x - cx, dy = vehicle_y - cy, d = std::hypot(dx, dy);
            const float step = 0.5f * cell_size_;
            for (float t = step; t <= std::min(shadow_depth_, d); t += step) {
                const int cc = static_cast<int>(std::floor((cx + dx / d * t - map_origin_x) / cell_size_));
                const int rr = static_cast<int>(std::floor((cy + dy / d * t - map_origin_y) / cell_size_));
                if (cc >= 0 && rr >= 0 && cc < width && rr < height && seen[cc + rr * width] != 0) {
                    return true;
                }
            }
            return false;
        };
        for (int r = pad; r < height - pad; ++r) {
            for (int c = pad; c < width - pad; ++c) {
                const size_t i = c + r * width;
                if (map.data[i] != 0 || 2 * unknown[i] <= factor * factor || !near_obstacle(c, r)) {
                    continue;
                }
                if (have_vehicle && std::hypot(map_origin_x + (c + 0.5f) * cell_size_ - vehicle_x,
                                               map_origin_y + (r + 0.5f) * cell_size_ - vehicle_y) < free_radius_) {
                    continue;
                }
                map.data[i] = 100;
            }
        }
    }
    return true;
}
