# 18. 2D laser SLAM (Cartographer)

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** `ros-jazzy-cartographer-ros` (binary) with an `xtdrone_2d.lua` configuration and an rclpy `laser_transfer.py`.

## What the original chapter does

Same as chapter 17, with Cartographer instead of HectorSLAM.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `/iris_0/scan` and `/iris_0/imu_gazebo` are available. Cartographer can use both.
- The `indoor3` world is converted.

The [migration plan](../reference/status.md) lists the phases.
