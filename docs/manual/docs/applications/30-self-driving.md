# 30. Preliminary self-driving car

!!! success "Ported; tested with the UGV"
    `ros2 run xtdrone_py lane_detection` and `self_driving` (ports of `sensing/lane_detection/lane_detection_*.py`
    and `control/*_self_driving.py`); model `rover_with_lidar_stereo`. Test: `scripts/tests/run_chapter_tests.sh ch30`:
    the CAT Vehicle UGV drives 30 m along the `outdoor3` track by itself, with the lane found in about 90 % of the
    frames. The PX4 rover variant is ported but not covered by a test.

Classic computer vision keeps a vehicle in its lane on the `outdoor3` track:
1. `lane_detection` keeps the yellow pixels (HSV threshold), finds line segments (Canny + Hough) and fits a left
   and a right lane line.
2. It publishes the offset of the lane centre from the image centre on `/<v>/lane_mid_error`. The value is 1000 if
   fewer than two lines are found.
3. `self_driving` drives at a constant speed, steers with −Kp · error and stops when the lane is lost.

## UGV (CAT Vehicle, chapter 11)

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor3_ugv
ros2 run xtdrone_py lane_detection ugv 0        # /ugv_0/triclops/triclops/left/image -> /ugv_0/lane_mid_error
ros2 run xtdrone_py self_driving ugv 0 --speed 3
```

The ROS 1 UGV script read a RealSense image that the catvehicle model does not have. The port uses the triclops
front camera, which looks ahead the same way.

## PX4 rover

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor3 vehicles:=rover_with_lidar_stereo communication:=basic
ros2 run xtdrone_py rover_keyboard_control 1       # b (OFFBOARD), t (ARM), then Ctrl-C
ros2 run xtdrone_py lane_detection rover 0      # /rover_0/front_cam/left/image_raw
ros2 run xtdrone_py self_driving rover 0        # -> /xtdrone/rover_0/cmd_vel_flu
```

`rover_with_lidar_stereo` is PX4's `r1_rover` with a front stereo camera (`front_cam/{left,right}/image_raw`, tilted
0.3 rad down) and a 3D lidar (`velodyne_points`). The r1 rover is much smaller than the Classic Polaris rover, so
its camera sits low; you may need to retune the HSV range in `PROFILES` (`xtdrone_py/self_driving.py`).

`/<v>/image_lane` shows the yellow mask with the fitted lines (`ros2 run rqt_image_view rqt_image_view`).
