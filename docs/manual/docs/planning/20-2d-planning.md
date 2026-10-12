# 20. 2D motion planning

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** **Nav2** (`ros-jazzy-navigation2`) replaces `move_base`, with the map from slam_toolbox or Cartographer (chapters 17 and 18), and an rclpy bridge from Nav2's `cmd_vel` to `/xtdrone/iris_0/cmd_vel_flu`.

## What the original chapter does

The 2D-lidar iris plans paths in `indoor3` with `move_base`, either on a saved map or while mapping, and follows them in Offboard mode.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- The XTDrone velocity interface `/xtdrone/iris_0/cmd_vel_flu` (`geometry_msgs/Twist`) is what a Nav2 controller needs.
- `/iris_0/scan` and the `indoor3` world are available.

The [migration plan](../reference/status.md) lists the phases.
