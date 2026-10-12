# 19. 3D laser SLAM

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** A-LOAM has no official ROS 2 release. Use **KISS-ICP** (`ros-jazzy-kiss-icp`, binary) or a community A-LOAM ROS 2 port on `/iris_0/velodyne_points`.

## What the original chapter does

A-LOAM runs on the 3D lidar of `iris_3d_gpu_lidar` in `outdoor1`, and `laser_transfer.py` sends its pose to PX4.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `iris_3d_lidar` / `iris_3d_gpu_lidar` publish `/iris_0/velodyne_points` (`sensor_msgs/PointCloud2`, 32 × 512 points, ±15°, 15 Hz). Gazebo Harmonic always renders lidars on the GPU, so both models are the same.
- The point cloud has the fields x, y, z, intensity, ring. A-LOAM needs the `ring` field.

The [migration plan](../reference/status.md) lists the phases.
