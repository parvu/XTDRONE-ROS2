# 14. Visual-inertial odometry (VIO)

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** A VINS-Fusion ROS 2 port (stereo + IMU) with the XTDrone config `px4_sitl_stereo_imu_config.yaml` updated to the new topics, and an rclpy port of `vins_transfer.py`.

## What the original chapter does

VINS-Fusion runs in stereo + IMU mode on `iris_stereo_camera`. `vins_transfer.py` sends its odometry to `/iris_0/mavros/vision_pose/pose`.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- Stereo images on `/iris_0/stereo_camera/...` and the 250 Hz IMU on `/iris_0/imu_gazebo` (`sensor_msgs/Imu`).
- The camera-to-IMU extrinsics changed with the x500 frame: the stereo link is at (0.12, 0, 0.03) m relative to `base_link`, and the IMU is at the `base_link` origin.
- Images are stamped with simulation time. VINS must run with `use_sim_time:=true`; the `/clock` bridge is always started.

The [migration plan](../reference/status.md) lists the phases.
