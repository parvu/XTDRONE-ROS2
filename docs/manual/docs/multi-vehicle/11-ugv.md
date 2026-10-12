# 11. Configuration and control of UGVs

!!! success "Ported"
    Models `ugv` and `ugv_tags3_5` (`ros2/xtdrone_gz/scripts/generate_ugv_models.py`), scenario `outdoor3_ugv`,
    `ros2 run xtdrone_py ugv_keyboard_control` and `ugv_communication`. Test: `scripts/tests/run_chapter_tests.sh ch11`
    (both UGVs drive straight and turn left; the lidar and camera topics publish).

The PX4 rover (chapter 8) runs a full PX4 instance per vehicle, which is expensive for many ground vehicles.
XTDrone therefore has a lighter UGV without an autopilot: the [CAT Vehicle](https://github.com/jmscslgroup/catvehicle),
a Ford Escape with Ackermann steering, a SICK 2D lidar, a 3D lidar, three cameras and an AprilTag plate on the roof
(tags 0–2 on `ugv`, 3–5 on `ugv_tags3_5`) for the landing chapters.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor3_ugv       # ugv_0 and ugv_1 on outdoor3
ros2 run xtdrone_py ugv_keyboard_control 2
```

Keys: `w`/`x` speed up/down (0.1 m/s steps, up to 20 m/s), `a`/`d` steer left/right (0.03 rad steps, the
steering is limited to 0.6 rad), `s` stop, `g` switch between all UGVs and the leader (`/ugv/leader/...`), and the
digit keys send `mission0`…`mission9` on `/ugv_<i>/cmd` and mask the keyboard.

UGVs mix with PX4 vehicles in one scenario or fleet spec, for example `vehicles:="ugv,iris*2"`. They take no PX4
instance, so the MAVLink ports of the PX4 vehicles do not change.

## Topics (per UGV, `/ugv_<i>/...`, the ROS 1 names)

| Topic | Type | |
|---|---|---|
| `cmd_vel` | Twist | command: `linear.x` speed (m/s), `angular.z` front steering angle (rad) |
| `ackermann_cmd_vel` | Twist | speed and yaw rate, from `ugv_communication` to the gz AckermannSteering system |
| `front_laser_points` | LaserScan | SICK LMS291, 180°, 80 m |
| `lidar_points` | PointCloud2 | 3D lidar (100 × 20 rays, 50 m) |
| `camera_left/image_raw_left`, `camera_right/image_raw_right` | Image | side cameras |
| `triclops/triclops/left/image` | Image | front camera |
| `ground_truth/odom` | Odometry | Gazebo ground truth |

!!! note "Changes from the ROS 1 version"
    - The model is generated from the original xacro (`catvehicle1-3.xacro`, `catvehicle4-6.xacro`): it is expanded
      with ROS 2 xacro, converted by `gz sdf -p`, and the Classic plugins are replaced. The gz
      `AckermannSteering` system replaces `gazebo_ros_control` and the Simulink `cmdvel2gazebo` controller, and
      `gpu_lidar` sensors replace the ray and block-laser plugins. The body collides as a box instead of the
      visual mesh, and the roof tags use the AprilTag plates of chapter 28.
    - AckermannSteering takes a yaw rate. `ugv_communication`, started automatically for every UGV, converts the
      XTDrone steering angle δ with the bicycle model ω = v·tan δ / L (wheel base L = 2.6 m). Like a real car, the
      UGV does not steer while it stands still.
    - The 3D lidar publishes PointCloud2 (ROS 1: PointCloud from the block laser plugin).
