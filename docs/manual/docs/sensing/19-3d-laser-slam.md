# 19. 3D laser odometry (A-LOAM → KISS-ICP)

!!! success "Ported and tested"
    Package `xtdrone_slam`: `laser_slam_3d.launch.py` (KISS-ICP v1.2.3, submodule `ros2/kiss-icp`) and
    `laser_transfer` in `kiss_icp` mode. Test: `scripts/tests/run_chapter_tests.sh ch19`: an `iris_3d_lidar` in
    `outdoor1` takes off and flies a 2 m square on the lidar odometry only, with no GPS and no ground truth. The
    odometry stays within 0.45 m of the Gazebo ground truth (0.34–0.42 m in the test runs).

A-LOAM has no maintained ROS 2 port. [KISS-ICP](https://github.com/PRBonn/kiss-icp) replaces it: a point-to-point
ICP odometry that needs no tuning for the sensor. Like A-LOAM in this chapter, it gives position, height and attitude.

## Run

```bash
# 1. outdoor1 with an iris_3d_lidar; PX4 takes its position, height and yaw from vision
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_3d_lidar ekf:=vision communication:=basic
# 2. KISS-ICP + laser_transfer (odometry -> /iris_0/mavros/vision_pose/pose); rviz:=true shows the map
ros2 launch xtdrone_slam laser_slam_3d.launch.py
# 3. keyboard: take off and fly slowly
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

`laser_slam_3d.launch.py` starts these:
- **TF:** the static transform `base_link → laser_3d` (0.12 m up).
- **KISS-ICP** (its own `odometry.launch.py`) on `/iris_0/velodyne_points` with `base_link` as base frame. It
  publishes `/kiss/odometry` and the TF `odom_lidar → base_link`.
- **`laser_transfer iris 0 kiss_icp`:** sends that TF at 30 Hz to `/iris_0/mavros/vision_pose/pose`.

`iris_3d_lidar` and `iris_3d_gpu_lidar` publish the same cloud: 32 × 512 points, ±15°, 15 Hz, with the fields x, y,
z, intensity and ring. Gazebo Harmonic renders every lidar on the GPU (or on the CPU when there is no GPU).

!!! note "Changes from the ROS 1 version"
    - A-LOAM → KISS-ICP. `laser_transfer.py iris 0 aloam` becomes `laser_transfer iris 0 kiss_icp`; the `aloam` mode
      (TF `camera_init → aft_mapped`, as A-LOAM publishes it) is kept for a community A-LOAM port.
    - The cloud has no per-point time, so KISS-ICP runs without deskewing (it says so at startup). At the speeds
      of this chapter that makes no visible difference.
