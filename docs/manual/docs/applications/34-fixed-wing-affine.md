# 34. Fixed-wing drone swarm affine formation

!!! info "Not ported: the source is not in the repository"
    The original chapter describes a fixed-wing affine formation controller that XTDrone never published in this
    repository. There is nothing to port. The parts it builds on work on ROS 2 and Gazebo Harmonic and are tested:
    standard VTOLs (chapter 8) and the fixed-wing formation framework of [chapter 35](35-fixed-wing-ugv.md).

## What the original chapter does

Six VTOLs take off, transition to fixed-wing flight and fly an *affine formation maneuver*: the formation as a whole
is translated, rotated, scaled and sheared by moving a few leaders. The followers compute their positions from the
leaders through a stress matrix (affine formation control). The control law ran as a C++ catkin package with its own
messages, in the same style as chapter 35.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## Building it on the ROS 2 port

If you have the affine controller's source, the chapter 35 package `xtdrone_fixed_wing_formation` is the
template for porting it:

- `pack_fw_states` gathers each VTOL's MAVROS topics into `FwStates` and sends attitude and throttle commands
  (`setpoint_raw/attitude`) to PX4 in OFFBOARD mode;
- `task_main` runs a per-vehicle state machine and a formation control law (`ABS_FORMATION_CONTROLLER`); replace the
  control law with the affine one;
- the messages are a ROS 2 interface package (`rosidl`), with field names in lower case;
- `scripts/tests/fw_formation_test.py` shows the take-off, transition (`MAV_CMD_DO_VTOL_TRANSITION`) and OFFBOARD
  switch from a script.

Six VTOLs start with

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:="standard_vtol*6"
```

Keep the vehicles within about 1000 m of the world origin: Gazebo Harmonic's physics (DART) represents the ground
plane as a box about 2100 m wide, so a vehicle spawned beyond it falls.
