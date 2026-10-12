# 1. XTDrone video introduction

!!! success "No porting needed"
    Background chapter.

The original chapter embeds the XTDrone introduction video. You can find it on the
[original manual page](https://www.yuque.com/xtdrone/manual_en/introduction).

XTDrone is described in two papers. Cite one of them if you use XTDrone in your research:

- K. Xiao, S. Tan, G. Wang, X. An, X. Wang and X. Wang, "XTDrone: A Customizable Multi-rotor UAVs Simulation
  Platform," *ICRAS 2020*, pp. 55-61, doi: 10.1109/ICRAS49812.2020.9134922. Preprint: arXiv:2003.09700.
- K. Xiao, L. Ma, S. Tan, Y. Cong, X. Wang, "Implementation of UAV Coordination Based on a Hierarchical Multi-UAV
  Simulation Platform," *Advances in Guidance, Navigation and Control*, LNEE vol. 644, 2022. Preprint: arXiv:2005.01125.

## Architecture on ROS 2 and Gazebo Harmonic

The single-vehicle architecture of the papers maps to this stack as follows:

```text
            ┌───────────────────────── Gazebo Harmonic (gz sim) ─────────────────────────┐
            │  world (converted from Classic)     vehicle model <type>_<id>              │
            │                                     ├ motors / control surfaces (gz systems)│
            │                                     ├ IMU, baro, NavSat (for PX4)          │
            │                                     └ XTDrone sensors (cameras, lidars)    │
            └──────────┬──────────────────────────────────────┬──────────────────────────┘
          gz-transport │ (IMU, GPS, baro, motor commands)     │ gz-transport (images, scans, odometry)
                       ▼                                      ▼
              PX4 SITL v1.16.2 (gz_bridge)             ros_gz_bridge
                       │ MAVLink UDP 24540+N / 34580+N        │ ROS 2 topics /<type>_<id>/...
                       ▼                                      ▼
                 MAVROS 2 /<type>_<id>/mavros  ◄──────►  XTDrone nodes (xtdrone_py):
                                                         communication, keyboard, perception,
                                                         planning, coordination
```

Compared to Gazebo Classic, PX4 no longer talks to the simulator through a MAVLink plugin
(`libgazebo_mavlink_interface.so`). PX4's built-in `gz_bridge` module reads the sensors and writes the actuator
commands over gz-transport directly. Sensor data reaches ROS 2 through `ros_gz_bridge` instead of the
`gazebo_ros` plugins.
