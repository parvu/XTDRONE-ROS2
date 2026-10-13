# 18. 2D laser SLAM (Cartographer)

!!! success "Ported and tested"
    `ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=cartographer` (`config/cartographer_2d.lua`, port of
    `sensing/slam/laser_slam/cartographer`). Test: `scripts/tests/run_chapter_tests.sh ch18`: the same 1 m square as
    chapter 17, flown on the Cartographer pose only. The trajectory stays within 0.1 m of the ground truth.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic
ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=cartographer rviz:=true
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
ros2 run nav2_map_server map_saver_cli -f ~/indoor3          # map from cartographer_occupancy_grid_node
```

Cartographer gets the scans of `/<vehicle>/scan` and the Gazebo IMU `/<vehicle>/imu_gazebo` (ROS 1: the MAVROS IMU).
It publishes `map → base_link` and `/tracked_pose`, and `laser_transfer` forwards `/tracked_pose` to PX4 as in
chapter 17.

## Configuration (`cartographer_2d.lua`)

| Setting | Value | Why |
|---|---|---|
| `use_imu_data` | true | the 2D default, as in ROS 1 |
| `provide_odom_frame` | false | one transform `map → base_link` |
| `publish_tracked_pose` | true | `laser_transfer` forwards this topic, not a TF lookup |
| `num_accumulated_range_data` | 1 (ROS 1: 10) | a multirotor tilts while accelerating; accumulating scans smeared the motion and Cartographer lost track during braking |
| `min_z`, `max_z` | ±0.5 m | after gravity alignment only points near the scan plane are used; floor hits of a tilted scan are dropped |
| `use_online_correlative_scan_matching` | true | robust initial guess without odometry |

With these settings Cartographer follows the vehicle within a few centimetres, also through 2 m position steps.
