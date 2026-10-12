# 13. Visual SLAM (ORB-SLAM3)

!!! success "Ported and tested"
    Package `xtdrone_orbslam3`: `stereo_node` (the port of XTDrone's `ros_stereo.cc`) and `stereo.launch.py`, with
    `laser_transfer` in `orbslam3` mode (the port of `vslam_transfer.py`). Test:
    `scripts/tests/run_chapter_tests.sh ch13`: an `iris_stereo_camera` in `outdoor1` takes off and flies a 2 m square
    on the ORB-SLAM3 pose only, with no GPS and no ground truth. The SLAM trajectory stays within 0.2 m of the Gazebo
    ground truth.

ORB-SLAM3 v1.0 runs in stereo mode. It is XTDrone's own copy in `sensing/slam/vslam/ORB_SLAM3`, built outside the
colcon workspace because it is a plain CMake project with Pangolin as its viewer.

## Build

```bash
# Pangolin v0.9.1 and ORB-SLAM3 into ~/.cache/xtdrone/orbslam3 (XTDRONE_CACHE moves it), about 15 min on 4 cores
./scripts/build_orbslam3.sh
# then the workspace again: xtdrone_orbslam3 builds stereo_node only when ORB-SLAM3 is there
./scripts/build_xtdrone_ros2.sh
```

`build_orbslam3.sh` builds a patched copy and leaves XTDrone's sources untouched:
- C++17 (Pangolin 0.9 needs it) and OpenCV 4. Under C++17, `LoopClosing::mnFullBAIdx` becomes an `int`: it is
  incremented with `++`, which C++17 forbids on a `bool`.
- Four time utilities (`DBoW2/DUtils/Timestamp.*`, `g2o/stuff/timeutil.*`) are missing from the XTDrone copy, because
  ORB-SLAM3's `.gitignore` matched them. The script downloads them from ORB-SLAM3 v1.0.
- Only the library is built; the examples need ROS 1 or RealSense.

## Run

```bash
# 1. outdoor1 with an iris_stereo_camera; PX4 takes its position, height and yaw from vision
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_stereo ekf:=vision communication:=basic
# 2. ORB-SLAM3 + laser_transfer (pose -> /iris_0/mavros/vision_pose/pose); viewer:=true opens Pangolin
ros2 launch xtdrone_orbslam3 stereo.launch.py
# 3. keyboard: take off and fly slowly
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

`stereo.launch.py` starts these:
- **`stereo_node`** with `ORBvoc.txt` and `config/xtdrone_stereo.yaml` (752 × 480, fx 376, baseline 0.12 m). It
  pairs the left and right images by exact time stamp, and publishes the camera pose as
  `/orbslam3/vision_pose/pose`, converted from the optical frames of ORB-SLAM3 to x forward, y left, z up.
- **`laser_transfer iris 0 orbslam3`:** sends that pose at 30 Hz to `/iris_0/mavros/vision_pose/pose`, stamped with
  its image time.

The SLAM frame starts at the vehicle with its x axis along the vehicle's heading. With `ekf:=vision`, PX4's local
frame is this frame. In `outdoor1_stereo` the vehicle faces the world's y axis, so the local x axis is the world's y
axis.

!!! note "Changes from the ROS 1 version"
    - ORB-SLAM2 → ORB-SLAM3 v1.0 (XTDrone ships both; the ROS 1 chapter used ORB-SLAM2, ORB-SLAM3 works the same way).
    - The world is `outdoor1` instead of `indoor1`: after the conversion the walls of `indoor1` are plain, too few
      features to track.
    - The vehicle starts at (3, 0) facing +y (scenario `outdoor1_stereo`). The world origin is inside a parked
      utility vehicle, and facing +x the camera sees mostly a large plain red arrow (`arrow_red`), on which ORB-SLAM3
      loses tracking as soon as the vehicle climbs.
    - Left and right images are paired only with equal time stamps. Under load Gazebo drops left and right frames
      independently, and a pair from two different render steps breaks the stereo matching while the vehicle moves.
