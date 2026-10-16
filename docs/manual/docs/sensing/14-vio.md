# 14. Visual-inertial odometry (VINS-Fusion)

!!! success "Ported and tested"
    Package `xtdrone_slam`: `vio.launch.py` (VINS-Fusion, submodule `ros2/vins-fusion`) and `laser_transfer`
    in `vins` mode (the port of `vins_transfer.py`). Test: `scripts/tests/run_chapter_tests.sh ch14`: an
    `iris_stereo_camera` in `outdoor1` takes off and flies a 2 m square on the VINS pose only, with no GPS and no
    ground truth. The VINS trajectory stays within 0.6 m of the Gazebo ground truth (0.23–0.56 m in the test runs).

VINS-Fusion runs in stereo + IMU mode on the stereo camera and the IMU of `iris_stereo_camera`. The ROS 2 port is
[mzahana/VINS-Fusion-ROS2](https://github.com/mzahana/VINS-Fusion-ROS2) (branch `ros2_jazzy`), built by
`scripts/build_xtdrone_ros2.sh` with the packages `vins`, `camera_models`, `loop_fusion` and `global_fusion`.

## Run

```bash
# 1. outdoor1 with an iris_stereo_camera; PX4 takes its position, height and yaw from vision
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_stereo ekf:=vision communication:=basic
# 2. VINS-Fusion + laser_transfer (VINS pose -> /iris_0/mavros/vision_pose/pose); loop:=true adds loop_fusion
ros2 launch xtdrone_slam vio.launch.py
# 3. keyboard: wait for "Initialization finish!" in the VINS output, then take off and fly slowly
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

To watch the estimate, add `rviz:=true` to step 2 (the layout is `config/vio.rviz`; edit it in RViz and save over
`ros2/xtdrone_slam/config/vio.rviz` to keep your changes):

```bash
ros2 launch xtdrone_slam vio.launch.py rviz:=true          # loop:=true rviz:=true for loop closure too
```

| RViz display | Topic | Shows |
|---|---|---|
| VINS path | `/path` | the estimated trajectory |
| Feature points | `/point_cloud` | the 3D points of the sliding window |
| Marginalised points | `/margin_cloud` | points leaving the window (off by default) |
| Key poses | `/key_poses` | the keyframe positions |
| Camera | `/camera_pose_visual` | the camera frustum |
| Feature tracks | `/image_track` | the left image with the tracked features |

The fixed frame is `world`. The path stays empty until "Initialization finish!": move the vehicle a little first.
A thin or fading track image means VINS is losing features and will drift. The same topics work in your own RViz.

```bash
ros2 topic hz /odometry                       # VINS rate; none = not initialised or lost
ros2 topic echo /odometry --once              # the current pose
ros2 run rqt_image_view rqt_image_view /image_track   # the tracked features without RViz
```

`vio.launch.py` starts these:
- **`vins_node`** with `config/vins/xtdrone_stereo_imu.yaml`: images from `/iris_0/stereo_camera/left|right/image_raw`
  (752 × 480, 90° field of view, pinhole without distortion: `cam0_pinhole.yaml`, `cam1_pinhole.yaml`), IMU from
  `/iris_0/imu_gazebo`, extrinsics fixed.
- **`laser_transfer iris 0 vins`:** reads the TF `world → body` published by VINS and sends it at 30 Hz to
  `/iris_0/mavros/vision_pose/pose`.

!!! note "Changes from the ROS 1 version"
    - The world is `outdoor1` instead of `indoor1`. After the conversion the walls of `indoor1` are plain, and VINS
      loses its features and diverges after a few metres. The city of `outdoor1` has enough texture.

    - `outdoor1` is a final, committed world (`ros2/xtdrone_gz/worlds/outdoor1.sdf`, its models in
      `world_models/`), not converted by `classic_to_gz.py` any more. Compared with the Classic world it has half
      of the parked cars, houses, hydrants, signs and lamp posts, no `arrow_red`, `cessna_c172`, shelves, boxes,
      pallet, sandy path or user lights, and its terrain is raised from z = -5 to -0.1 m, above the ocean (-2 m)
      and below the roads and sidewalks. Edit the SDF directly to change it.
    - The camera-to-IMU extrinsics changed with the x500 frame: the stereo link is at (0.12, 0, 0.03) m in
      `base_link`, the cameras at ±0.06 m, and the IMU at the `base_link` origin.
    - `vins_node` and `loop_fusion_node` accept exactly one argument, the config file, so the launch file starts them
      as plain processes, without ROS arguments. They use the message stamps (simulation time), not the node clock.
    - `vins_transfer.py` becomes `laser_transfer` in `vins` mode, the same node as for the laser SLAM chapters.
