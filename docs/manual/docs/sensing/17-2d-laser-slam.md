# 17. 2D laser SLAM (HectorSLAM → slam_toolbox)

!!! success "Ported and tested"
    Package `xtdrone_slam`: `laser_slam_2d.launch.py slam:=slam_toolbox` and `laser_transfer` (the port of
    `sensing/slam/laser_slam/script/laser_transfer.py`). Test: `scripts/tests/run_chapter_tests.sh ch17`: an
    `iris_2d_lidar` in `indoor3` takes off and flies a 1 m square on the SLAM pose only, with no GPS and no ground
    truth. The SLAM trajectory stays within 0.4 m of the Gazebo ground truth.

HectorSLAM has no ROS 2 release. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) replaces it. Like
Hector in this chapter, it runs as a pure scan matcher without odometry, and its pose is the drone's position source.

## Run

```bash
# 1. indoor3 with an iris_2d_lidar (2D lidar + downward range finder); PX4 takes its position from the SLAM
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic
# 2. slam_toolbox + laser_transfer (pose -> /iris_0/mavros/vision_pose/pose), rviz:=true shows map and scans
ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox rviz:=true
# 3. keyboard: fly slowly; stay below the invisible ceiling of indoor3 (2.5 m) so the lidar sees the walls
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
# 4. save the map (for chapter 20)
ros2 run nav2_map_server map_saver_cli -f ~/indoor3
```

`laser_slam_2d.launch.py` starts these:
- **TF:** the static transforms `odom → base_link` (identity: no odometry) and `base_link → laser_2d`.
- **slam_toolbox** (its lifecycle launch, `config/slam_toolbox_2d.yaml`, scans from `/<vehicle>/scan`).
- **`laser_transfer`:** reads `map → base_link` and publishes it at 30 Hz on `/<vehicle>/mavros/vision_pose/pose`.
  The z coordinate is the downward range finder (`/<vehicle>/distance`), as in ROS 1.

`ekf:=laser2d` sets EKF2 to use that pose: horizontal position, height and yaw from vision, plus the barometer, with
GPS and the magnetometer off. The SLAM pose is stamped with its sensor time, converted to ROS time, so that EKF2 can
compensate the SLAM delay.

!!! tip "Fly slowly"
    The vehicle tilts while it accelerates, and the 2D scan then hits the floor. The test caps PX4's horizontal
    speed and acceleration (`px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1`). The ROS 1 chapter likewise asked to fly
    slowly with the keyboard.

!!! note "Changes from the ROS 1 version"
    - HectorSLAM → slam_toolbox. HectorSLAM's `/iris_0/pose` (`laser_transfer.py iris 0 hector`) becomes the TF
      `map → base_link` (`laser_transfer iris 0 slam_toolbox`).
    - The `iris_2d_lidar` model has the downward range finder of the Classic model again (`laser_1d`, topic
      `distance`). It is a narrow fan of 5 rays pitched 86°: a GPU lidar looking exactly straight down measures
      wrong ranges.
    - The model's ground-truth odometry is kept away from PX4. Gazebo's OdometryPublisher also published
      `/model/<name>/odometry_with_covariance`, which PX4's `gz_bridge` fuses as external vision. This had silently
      given every vision-mode EKF a perfect position.
    - `indoor3`: the invisible ceiling is now static at the top of the walls (2.75 m). In Classic it was a dynamic
      box that fell onto the walls, and in gz-sim that slowed the world to 5 % of real time.

## Visualise

`rviz:=true` shows the map being built (`/map`), the current scan (`/iris_0/scan`) and the TF tree.

```bash
ros2 topic echo /iris_0/mavros/vision_pose/pose --once     # the SLAM pose that PX4 receives
ros2 topic hz /iris_0/mavros/vision_pose/pose              # 30 Hz; none = the SLAM lost the scan matching
ros2 topic echo /iris_0/ground_truth/odom --once           # truth, to judge the SLAM error
```

The scan should lie on the walls of the map. If it slides off them, the vehicle tilts too much: fly slower. The
saved map (`~/indoor3.pgm`) opens in any image viewer.
