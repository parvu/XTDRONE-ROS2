# 32. UAV and USV cooperation

!!! success "Ported and tested"
    VRX v3.1.2 (Gazebo Harmonic / ROS 2 Jazzy) as the submodule `ros2/vrx`; world `sandisland`, scenario
    `sandisland`, `ros2 launch xtdrone_bringup usv.launch.py`. Test: `scripts/tests/run_chapter_tests.sh ch32`: the
    WAM-V floats and drives 5 m on its thrusters, and an iris takes off from the island to 5 m and lands.

[VRX](https://github.com/osrf/vrx) simulates unmanned surface vehicles in a maritime environment (waves, wind,
buoyancy). XTDrone's Classic `sandisland` launch put six iris on the sand island next to the WAM-V. VRX v3 calls
that world `sydney_regatta`.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=sandisland communication:=basic   # six iris on the island
ros2 launch xtdrone_bringup usv.launch.py                                                 # WAM-V at (158, 108)
ros2 topic pub -r 10 /wamv/thrusters/left/thrust  std_msgs/msg/Float64 "data: 200.0"
ros2 topic pub -r 10 /wamv/thrusters/right/thrust std_msgs/msg/Float64 "data: 200.0"
ros2 run xtdrone_py multirotor_keyboard_control iris 6 vel                                # the UAVs as usual
```

| Topic (VRX v3) | ROS 1 (VRX Classic) |
|---|---|
| `/wamv/thrusters/left/thrust`, `/wamv/thrusters/right/thrust` (Float64, N) | `/wamv/thrusters/left_thrust_cmd` (Float32, −1…1) |
| `/wamv/thrusters/left/pos`, `/wamv/thrusters/right/pos` (Float64, rad) | `/wamv/thrusters/left_thrust_angle` |

The WAM-V's world pose is on `/world/sydney_regatta/pose/info` (gz) and as TF.

## Notes

- `sandisland` is generated from VRX's `sydney_regatta.sdf` by `ros2/xtdrone_gz/scripts/make_sandisland.py`. It adds
  the AirPressure system, without which PX4 has no barometer. The world keeps its VRX name, so VRX's spawn launch and
  wave settings work unchanged.
- The iris stand on the island; its ground is about 2.3 m above the water. Water has no collision surface, so a
  vehicle spawned over water sinks.
- `usv.launch.py` uses a WAM-V with thrusters only (`sensors:=true` for VRX's full set). VRX turns on three cameras,
  a 3D lidar, GPS and IMU, which run at about a tenth of real time without a GPU.
- The VRX build is the longest part of `build_xtdrone_ros2.sh`. `XTDRONE_NO_VRX=1` leaves it out.
