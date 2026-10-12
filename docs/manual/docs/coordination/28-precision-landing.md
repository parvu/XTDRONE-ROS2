# 28. Multi-UAV precision landing

!!! warning "Planned: phase 4"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** `ros-jazzy-apriltag-ros` on `/iris_N/camera/image_raw`, with the XTDrone tag bundle configuration, and an rclpy port of `precision_landing.py`.

## What the original chapter does

Three iris with downward cameras find the AprilTags with IDs 0 to 2 in `outdoor2` and land on them.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `scenario:=outdoor2_precision_landing` starts the three `iris_downward_camera` at the original positions.
- The AprilTag models (`apriltag0-2`, `apriltag3-5`, `landing1`, `landing2`) are converted with their textures.
- Downward images: `/iris_N/camera/image_raw` and `camera_info`.

The [migration plan](../reference/status.md) lists the phases.
