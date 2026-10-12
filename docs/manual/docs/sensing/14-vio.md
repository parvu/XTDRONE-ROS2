# 14. Visual-inertial odometry (VINS-Fusion)

!!! success "Ported and tested"
    Package `xtdrone_slam`: `vio.launch.py` (VINS-Fusion, submodule `ros2/vins-fusion`) and `laser_transfer`
    in `vins` mode (the port of `vins_transfer.py`). Test: `scripts/tests/run_chapter_tests.sh ch14`: an
    `iris_stereo_camera` in `outdoor1` takes off and flies a 2 m square on the VINS pose only, with no GPS and no
    ground truth. The VINS trajectory stays within 0.3 m of the Gazebo ground truth.

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

`vio.launch.py` starts these:
- **`vins_node`** with `config/vins/xtdrone_stereo_imu.yaml`: images from `/iris_0/stereo_camera/left|right/image_raw`
  (752 × 480, 90° field of view, pinhole without distortion: `cam0_pinhole.yaml`, `cam1_pinhole.yaml`), IMU from
  `/iris_0/imu_gazebo`, extrinsics fixed.
- **`laser_transfer iris 0 vins`:** reads the TF `world → body` published by VINS and sends it at 30 Hz to
  `/iris_0/mavros/vision_pose/pose`.

!!! note "Changes from the ROS 1 version"
    - The world is `outdoor1` instead of `indoor1`. After the conversion the walls of `indoor1` are plain, and VINS
      loses its features and diverges after a few metres. The city of `outdoor1` has enough texture.
    - In `outdoor1` the vehicle starts at (3, 0) facing +y (scenario `outdoor1_stereo`, shared with chapter 13): the
      world origin is inside a parked utility vehicle, from which a multirotor cannot take off, and facing +x the
      cameras see mostly a plain red arrow. The VINS frame starts with its x axis along the vehicle's heading, so
      the local x axis is the world's y axis.
    - The camera-to-IMU extrinsics changed with the x500 frame: the stereo link is at (0.12, 0, 0.03) m in
      `base_link`, the cameras at ±0.06 m, and the IMU at the `base_link` origin.
    - `vins_node` and `loop_fusion_node` accept exactly one argument, the config file, so the launch file starts them
      as plain processes, without ROS arguments. They use the message stamps (simulation time), not the node clock.
    - `vins_transfer.py` becomes `laser_transfer` in `vins` mode, the same node as for the laser SLAM chapters.
