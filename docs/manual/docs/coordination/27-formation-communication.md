# 27. Formation control with a communication model

!!! warning "Planned: phase 3"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** rclpy port of `coordination/formation_communication_demo` (the channel-capacity model) and of the XTDGroundControl Qt interface.

## What the original chapter does

Twenty iris keep a formation while a communication model limits the channel capacity between them. The demo runs from tmux scripts, controlled by keyboard or XTDGroundControl.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- 20 iris on distinct MAVLink ports: `vehicles:="iris*20"`. The XTDrone port overlay removes PX4's limit of 10 offboard ports (chapter 48).

The [migration plan](../reference/status.md) lists the phases.
