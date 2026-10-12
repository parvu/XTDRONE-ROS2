# 32. UAV and USV cooperation

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** VRX for Gazebo Harmonic (`vrx` `main` branch with ROS 2 Jazzy) replaces the Classic `sandisland` world and WAM-V model. The UAVs fly from `xtdrone_bringup` in the same world.

## What the original chapter does

Six UAVs and WAM-V boats (USVs) cooperate in the VRX ocean world `sandisland`.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- VRX itself runs on Gazebo Harmonic. The UAV part of the scenario works once the VRX world is on the resource path.

The [migration plan](../reference/status.md) lists the phases.
