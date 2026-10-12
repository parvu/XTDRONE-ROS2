# 21. 3D motion planning

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** A ROS 2 port of EGO-Planner fed by the RealSense depth image and a camera pose (VIO or ground truth), and an rclpy `ego_transfer.py`.

## What the original chapter does

EGO-Planner takes the depth map of `iris_realsense_camera` and the camera pose from VINS-Fusion, plans 3D trajectories in `indoor1`, and `ego_transfer.py` sends them to the XTDrone communication node.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- Depth images: `/iris_0/realsense/depth_camera/depth/image_raw`; camera pose: `/iris_0/ground_truth/odom` as a stand-in for VIO.
- Position setpoints go through `/xtdrone/iris_0/cmd_pose_enu`.

The [migration plan](../reference/status.md) lists the phases.
