# 30. Preliminary self-driving car

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** rclpy ports of `lane_detection_rover.py` and `rover_self_driving.py`, with a stereo camera kit on the rover model.

## What the original chapter does

A rover with a front stereo camera drives in `outdoor3`. A colour filter extracts the yellow lane markings and a controller follows the lane.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `world:=outdoor3` is converted. The lane markings and roads are there (roads are converted to flat boxes).
- The PX4 rover (`vehicles:=rover`) and `rover_communication` work (chapter 8). The camera kit for the rover is still missing.

The [migration plan](../reference/status.md) lists the phases.
