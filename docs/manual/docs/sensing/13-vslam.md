# 13. Visual SLAM

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** ORB-SLAM3 with its community ROS 2 wrapper, fed by `/iris_0/stereo_camera/{left,right}/image_raw` (available now), and an rclpy port of `vslam_transfer.py` that publishes the pose to `/iris_0/mavros/vision_pose/pose`.

## What the original chapter does

The original chapter builds the XTDrone-modified ORB-SLAM2 (stereo mode), flies an `iris_stereo_camera` in `indoor1` with the keyboard, and sends the ORB-SLAM2 camera pose to PX4 as vision pose. ORB-SLAM3 works the same way.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `iris_stereo_camera` publishes `/iris_0/stereo_camera/left|right/image_raw` and `camera_info` (752×480, baseline 0.12 m), the same topics as in Classic.
- `scenario:=indoor1` reproduces the original world and vehicle.
- `ekf:=vision` (chapter 6) configures PX4 to fuse the SLAM pose.

The [migration plan](../reference/status.md) lists the phases.
