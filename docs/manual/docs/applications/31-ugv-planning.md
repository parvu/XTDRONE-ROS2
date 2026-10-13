# 31. Motion planning of UGVs

!!! success "Ported and tested"
    Package `xtdrone_ugv_planning` (rclcpp ports of XTDrone's `hybrid_astar_test` and `pure_pursuit`), scenario
    `outdoor1_ugv`. Test: `scripts/tests/run_chapter_tests.sh ch31`: slam_toolbox maps `outdoor1` from the CAT
    Vehicle's front lidar, hybrid A* plans a lane change to (20, −1.5) and pure pursuit drives the UGV there
    (within 2 m; about 0.9 m in the test runs).

The CAT Vehicle UGV maps its surroundings with its SICK lidar, plans a path with hybrid A* (kinematic car model,
smoothed on a Voronoi diagram) and follows it with pure pursuit. ROS 1 used Gmapping; the port uses slam_toolbox.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_ugv                 # ugv_0 at (1, 1)
ros2 launch xtdrone_ugv_planning ugv_planning.launch.py rviz:=true                   # SLAM, planner, follower
```

Set the goal with RViz's *2D Goal Pose* tool, or

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 20.0, y: -1.5}, orientation: {w: 1.0}}}"
```

The path is published on `/front_end_path`; pure pursuit sends speed and steering angle on `/ugv_0/cmd_vel`
(`linear.x` m/s, `angular.z` front-wheel angle in rad, as in chapter 11).

| Launch argument | Default | |
|---|---|---|
| `speed` | `3.0` | pure-pursuit cruise speed, m/s |
| `scan_matching` | `false` | slam_toolbox scan matching. Off: the map is built on the ground-truth odometry, which keeps the map frame fixed while the vehicle turns |
| `rviz` | `false` | RViz with the map, the path and the search |

| Planner parameter (`hybrid_astar_node`) | Default | |
|---|---|---|
| `base_frame` | `ugv_0/base_footprint` | start of the plan (TF from `map`) |
| `goal_topic` | `/goal_pose` | |
| `replan` | `false` | one plan per goal. ROS 1 replanned every 10 s from the moving vehicle; on a growing slam_toolbox map those plans often ran backwards |
| `map_margin` | `20` | m of free cells around the map (the forward lidar does not map the area around the vehicle) |
| `unknown_occupied` | `true` | unseen cells up to `unknown_shadow_depth` (3 m) behind a seen obstacle, as seen from the vehicle, are obstacles |
| `unknown_free_radius` | `5` | m around the vehicle where unseen cells stay free |

The vehicle model (size, wheelbase, safety distance, map resolution) is in `config/base_config.yaml`: the CAT
Vehicle, 0.3 m safety distance, 0.5 m cells.

## Notes

- **Units.** The ROS 1 planner mixed metres and grid cells (start, goal, motion primitives, Dubins radius, smoother
  limits), which is only right on 1 m cells; with the CAT Vehicle (2.2 m × 4.6 m) on 1 m cells and a 0.5 m margin
  the 7.4 m road of `outdoor1` was nearly blocked. The port converts consistently, so finer cells work.
- **Unseen obstacles.** The lidar sees only the near side of the two parked cars ahead of the start; with the rest
  free, the planner sent the UGV into a gap between them where it does not fit. Unseen cells behind seen obstacles
  are now obstacles.
- **Curbs.** The sidewalks of `outdoor1` (0.15 m) are below the lidar (0.93 m) and not in the map, so the planner
  does not avoid them, and the UGV cannot climb them. The road between them is y = −3.7…3.7 m; keep goals on it.
  Behind the parked cars (x > 22 m) there is no way through for the CAT Vehicle.
- Fixed memory errors of the original (found with AddressSanitizer): the footprint table overflowed for vehicles
  larger than 64 cells, the heading index overflowed at a heading just below 2π, the smoother copied the Voronoi
  diagram, and a freed grid was used after the next map.
