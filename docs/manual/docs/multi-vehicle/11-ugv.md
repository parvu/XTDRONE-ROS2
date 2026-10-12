# 11. Configuration and control of UGVs

!!! warning "Planned: phase 4"
    The XTDrone UGV (`sitl_config/ugv`, the catvehicle Ackermann model with ROS 1 `position_controllers` /
    `velocity_controllers` and a Velodyne plugin) is not converted yet.
    **ROS 2 approach:** convert the URDF to Gazebo Harmonic with `gz_ros2_control` (Ackermann steering
    controller) or the gz `AckermannSteering` system, a `gpu_lidar` instead of the Velodyne plugin, and an rclpy
    port of `ugv_keyboard_control.py` publishing `/ugv_<id>/cmd_vel`.

## What the original chapter does

The PX4 rover (chapter 8) runs a full PX4 instance per vehicle, which is expensive for many ground vehicles.
XTDrone therefore provides a lightweight UGV without an autopilot:

```bash
# ROS 1 original
cp -r ~/XTDrone/sitl_config/ugv/* ~/catkin_ws/src/
roslaunch px4 outdoor3_ugv.launch
python ugv_keyboard_control.py 2
```

## What works today

- The PX4 R1 rover (`vehicles:=rover`, chapter 8) for single ground-vehicle tests.
- Every converted world, including `outdoor1` and `outdoor3`, which the UGV scenarios use.
