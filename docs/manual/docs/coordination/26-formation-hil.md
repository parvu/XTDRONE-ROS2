# 26. UAV formation hardware-in-the-loop simulation

!!! warning "Planned: phase 3"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** ROS 2 multi-machine networking (same `ROS_DOMAIN_ID`, DDS discovery) instead of a ROS 1 master. The leader and follower nodes run on the companion boards.

## What the original chapter does

Six development boards (NX, Raspberry Pi, ODROID) run the leader and follower nodes. The laptop runs XTDrone. The boards and the laptop share one ROS 1 master.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- ROS 2 needs no master: set the same `ROS_DOMAIN_ID` on all machines and make sure multicast works on the network.
- The formation demo of chapter 25 runs as separate leader and follower nodes.

The [migration plan](../reference/status.md) lists the phases.
