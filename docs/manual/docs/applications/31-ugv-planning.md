# 31. Motion planning of UGVs

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** slam_toolbox instead of Gmapping, and rclcpp ports of the hybrid A* and pure-pursuit packages, on the UGV of chapter 11.

## What the original chapter does

The catvehicle UGV maps `outdoor1` with Gmapping, plans with hybrid A* and follows the path with pure pursuit.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- The `outdoor1` world is converted. The UGV model is not (chapter 11).

The [migration plan](../reference/status.md) lists the phases.
