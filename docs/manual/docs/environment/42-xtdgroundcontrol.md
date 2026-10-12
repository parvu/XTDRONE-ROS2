# 42. XTDGroundControl

!!! warning "Planned: phase 3"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** Port the PyQt5 ground station (`control/XTDGroundControl/python`) from rospy to rclpy. The Qt part stays the same.

## What the original chapter does

XTDGroundControl is a PyQt5 ground station for many vehicles of different types. It replaces the keyboard for complex tasks: it selects vehicles, sends commands and shows their states.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- It sends the same XTDrone commands (`/xtdrone/<type>_<id>/cmd`, `cmd_vel_flu`, ...) that the ported communication nodes accept.

The [migration plan](../reference/status.md) lists the phases.
