# 17. 2D laser SLAM (HectorSLAM)

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** HectorSLAM has no ROS 2 release. Use **slam_toolbox** (`ros-jazzy-slam-toolbox`) with `/iris_0/scan`, and an rclpy port of `laser_transfer.py` that sends the 2D pose to MAVROS. Save maps with `ros2 run nav2_map_server map_saver_cli`.

## What the original chapter does

An `iris_2d_lidar` flies in `indoor3`. HectorSLAM builds the map from the laser scans, `laser_transfer.py` gives PX4 the SLAM pose, and `map_saver` stores the map in `motion_planning/2d/map` for chapter 20.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `iris_2d_lidar` publishes `/iris_0/scan` (`sensor_msgs/LaserScan`, frame `laser_2d`, 512 samples, 40 Hz).
- The `indoor3` world is converted: `world:=indoor3 vehicles:=iris_2d_lidar` or `scenario:=indoor3`.

The [migration plan](../reference/status.md) lists the phases.
