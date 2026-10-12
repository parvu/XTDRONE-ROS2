# 22. Swarm motion planning

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** EGO-Planner-Swarm ROS 2 port on the `ego_swarm` scenarios.

## What the original chapter does

Four (or eight) iris with RealSense cameras fly in `ego_swarm.world` with EGO-Planner-Swarm and VINS-Fusion. `ego_swarm_goal.py` sends the goals.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `scenario:=ego_swarm` (4 iris, world `ego_swarm_4`) and `scenario:=ego_swarm_8` are converted and start all vehicles with their sensors.

The [migration plan](../reference/status.md) lists the phases.
