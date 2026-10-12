# XTDrone on ROS 2 Jazzy and Gazebo Harmonic

XTDrone is a UAV simulation platform built on PX4, ROS and Gazebo. It covers multirotors, fixed-wing
aircraft, VTOLs, ground vehicles, multi-vehicle swarms and many perception and planning examples.

This manual is the ROS 2 edition of the [XTDrone manual](https://www.yuque.com/xtdrone/manual_en).
It keeps the 48 chapters of the original, and rewrites each one for this stack:

| Layer | Original manual | This manual |
|---|---|---|
| OS | Ubuntu 16.04 / 18.04 / 20.04 | **Ubuntu 24.04** |
| ROS | ROS 1 Kinetic / Melodic / Noetic | **ROS 2 Jazzy** |
| Simulator | Gazebo Classic 9 / 11 | **Gazebo Harmonic** (gz-sim 8) |
| Autopilot | PX4 1.11 / 1.13 (`sitl_gazebo`) | **PX4 v1.16.2** (`gz_bridge`), pinned as a submodule |
| Flight controller link | MAVROS | **MAVROS 2** |
| Build | catkin | colcon |

## What stays the same

The interfaces of the original are kept, so that algorithms and scripts written for XTDrone carry over:

- vehicles are named `<type>_<id>` (`iris_0`, `typhoon_h480_2`, `plane_0`, ...);
- MAVROS topics live under `/<type>_<id>/mavros/...`;
- XTDrone commands go to `/xtdrone/<type>_<id>/cmd`, `cmd_vel_flu`, `cmd_pose_enu`, ...;
- sensors publish on the same topics (`/iris_0/stereo_camera/left/image_raw`, `/iris_0/scan`, ...);
- the keyboard controls use the same keys;
- the MAVLink port scheme of the multi-vehicle generator is kept (offboard ports 24540+N / 34580+N).

## What changes

- One launch file, `xtdrone_bringup/xtdrone.launch.py`, replaces the ROS 1 launch files and the
  launch-file generator. A `scenario:=` argument reproduces each original launch file, e.g.
  `scenario:=indoor1`. A `vehicles:=` argument describes a fleet in one line, e.g. `vehicles:="iris*6"`.
- Python scripts run as ROS 2 executables, for example
  `ros2 run xtdrone_py multirotor_communication iris 0`, instead of `python multirotor_communication.py iris 0`.
- Worlds and models are converted from Gazebo Classic automatically at build time.
- The PX4 iris airframe becomes PX4's Gazebo `x500` airframe. XTDrone keeps the name `iris` and its
  sensor variants.

See [Porting notes](reference/porting-notes.md) for the full list.

## Quick start

```bash
git clone --recursive <this repository> ~/XTDRONE-ROS2
cd ~/XTDRONE-ROS2
./scripts/setup_ubuntu_24_04.sh        # once; installs ROS 2 Jazzy, Gazebo Harmonic, MAVROS 2, PX4 deps
./scripts/build_xtdrone_ros2.sh        # builds PX4, converts worlds/models, builds the colcon workspace
source scripts/xtdrone_env.sh          # in every terminal
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 communication:=basic
```

In a second terminal:

```bash
source ~/XTDRONE-ROS2/scripts/xtdrone_env.sh
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

Press `i` until the upward speed is above 0.3 m/s, then `b` for offboard and `t` to arm. The iris takes
off. Chapter 2 has the full walk-through.

## Chapter status

Every chapter starts with a status box:

!!! success "Ported and tested"
    The chapter runs on ROS 2 Jazzy and Gazebo Harmonic, and its test in `scripts/tests` passes.

!!! info "Ported"
    The code and the instructions are converted, but there is no automated test, or the test has not passed yet.

!!! warning "Planned"
    The original chapter depends on components that are not converted yet. The box names the migration
    phase and the ROS 2 approach. The rest of the chapter explains the topic and what is already available.

The [chapter status](reference/status.md) page lists all chapters.
