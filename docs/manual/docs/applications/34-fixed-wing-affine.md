# 34. Fixed-wing drone swarm affine formation

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** rclcpp port of `fixed_wing_formation_control`, with its custom messages as a ROS 2 interface package, on `plane` vehicles.

## What the original chapter does

Six VTOLs in fixed-wing mode fly an affine formation maneuver. The control law is a C++ catkin package with custom messages.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- Multiple planes and standard VTOLs with distinct ports: `vehicles:="standard_vtol*6"`.

The [migration plan](../reference/status.md) lists the phases.
