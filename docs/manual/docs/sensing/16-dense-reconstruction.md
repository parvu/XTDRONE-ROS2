# 16. 3D dense reconstruction (VINS-Fusion + RTAB-Map)

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** `ros-jazzy-rtabmap-ros` (binary) with the RGB-D topics of `iris_realsense_camera`, and odometry from VIO (chapter 14) or, until VIO is ported, from the ground truth (chapter 15).

## What the original chapter does

The sparse SLAM/VIO maps are not enough for obstacle avoidance. The chapter fuses VINS-Fusion odometry with the RealSense depth image in RTAB-Map to build a dense point cloud and an OctoMap (`/rtabmap/octomap_binary`).

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `iris_realsense_camera` publishes `/iris_0/realsense/depth_camera/color/image_raw`, `.../depth/image_raw`, `.../depth/points` and `camera_info`.
- Ground-truth odometry on `/iris_0/ground_truth/odom` can stand in for VIO.

The [migration plan](../reference/status.md) lists the phases.
