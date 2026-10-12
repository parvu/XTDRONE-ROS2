# Packages, launch arguments and topics

## Repository layout

| Path | Content |
|---|---|
| `PX4-Autopilot/` | PX4 v1.16.2 submodule (unchanged) |
| `ros2/xtdrone_gz/` | vehicle models (`models/`), hand-made worlds (`worlds/`), PX4 overlays (`px4/`), converters (`scripts/`); converted Classic assets in `generated/` (build output) |
| `ros2/xtdrone_bringup/` | `xtdrone.launch.py`, `spawn_vehicle.launch.py`, scenarios (`config/scenarios/*.yaml`), fleet logic |
| `ros2/xtdrone_py/` | rclpy nodes: communication, keyboard, ground truth, demos, Gazebo helpers |
| `ros2/fomation_demo/` | package `xtdrone_formation_demo` (chapter 25) |
| `ros2/px4_gz_simulation/` | package `xtdrone_px4_gz`: stock `make px4_sitl gz_x500` + Micro XRCE-DDS agent |
| `sitl_config/`, `communication/`, `control/`, … | the original ROS 1 / Classic XTDrone sources (input of the converters, reference) |
| `scripts/` | setup, build, environment, verification, tests, manual build |
| `docs/manual/` | this manual (MkDocs) |

## `xtdrone_bringup/xtdrone.launch.py`

| Argument | Default | Description |
|---|---|---|
| `scenario` | — | scenario name in `config/scenarios` or a YAML path; overrides `world` and `vehicles` |
| `world` | `indoor1` | world name or `.sdf` path |
| `vehicles` | `iris_stereo_camera` | fleet spec `MODEL[*N],...` |
| `gui` | `true` | Gazebo GUI |
| `mavros` | `true` | MAVROS 2 per vehicle |
| `bridge` | `true` | ros_gz_bridge per vehicle (sensors, ground truth, gimbal) |
| `communication` | `none` | `basic` / `enhanced` XTDrone communication nodes |
| `px4` | `true` | `false`: spawn models only (chapter 43) |
| `ekf` | `gps` | EKF2 preset `gps` / `vision` (after the XTDrone defaults `NAV_DLL_ACT=0 COM_RCL_EXCEPT=4`) |
| `mavros_plugins` | `xtdrone` | `xtdrone`: only the MAVROS plugins XTDrone uses (sys_status, sys_time, command, param, local/global position, imu, altitude, home_position, setpoint_*, vision_pose, vision_speed, odometry); `all`: MAVROS' PX4 default set |
| `px4_params` | — | extra PX4 parameters `NAME=value ...` |
| `speed_factor` | `1.0` | target real-time factor |
| `gz_verbose` | `1` | Gazebo verbosity |
| `px4_dir` | auto | PX4 checkout (must be v1.16.2 unless `XTDRONE_ALLOW_ANY_PX4=1`) |
| `rootfs` | `~/.ros/xtdrone_px4` | PX4 instance working directories |

`spawn_vehicle.launch.py`: `model`, `id`, `instance`, `x y z R P Y`, `world`, `communication`, `ekf`, `px4_params`, `mavros_plugins`.

Vehicles requested at z ≤ 0 spawn at z = 0.3 m (`SPAWN_CLEARANCE` in `xtdrone_bringup/sim.py`).

## `xtdrone_py` executables

| Executable | Arguments | Original script |
|---|---|---|
| `multirotor_communication` | `<type> <id>` | communication/multirotor_communication.py |
| `multirotor_communication_enhanced` | `<type> <id>` | communication/multirotor_communication_enhanced.py |
| `plane_communication` | `<id>` | communication/plane_communication.py |
| `vtol_communication` | `<type> <id>` | communication/vtol_communication.py |
| `rover_communication` | `<id>` | communication/rover_communication.py |
| `multirotor_keyboard_control` | `<type> <num> vel|accel` | control/keyboard/multirotor_keyboard_control.py |
| `plane_keyboard_control` | `<num>` | control/keyboard/plane_keyboard_control.py |
| `vtol_keyboard_control` | `<type> <num> vel|accel` | control/keyboard/vtol_keyboard_control.py |
| `rover_keyboard_control` | `<num>` | control/keyboard/rover_keyboard_control.py |
| `get_local_pose` | `<type> <num>` | sensing/pose_ground_truth/get_local_pose.py |
| `attitude_control_demo` | `<type> angular_speed|angle [hover]` | control/attitude_contorl_demo.py |
| `control_gazebo_models` | `[--world W]` | control/control_gazebo_models.py |
| `control_gazebo_vehicles` | `<type> <rows> <cols> [--world W]` | control/control_gazebo_vehicles.py |
| `contact_detect` | `<type> <num>` | sensing/contact_detect/contact_detect.py |
| `wind_control` | `[--world W]` | (Classic ROS wind plugin) |

Launch: `xtdrone_py multi_vehicle_communication.launch.py` (`<type>_num`, `enhanced`).

## Topics per vehicle `<n>` = `<type>_<id>`

| Topic | Type | Source |
|---|---|---|
| `/<n>/mavros/...` | MAVROS 2 | MAVROS |
| `/xtdrone/<n>/cmd`, `cmd_*` | String, Pose, Twist | XTDrone nodes → communication node |
| `/<n>/ground_truth/odom` | nav_msgs/Odometry | gz OdometryPublisher |
| `/<n>/imu_gazebo` | sensor_msgs/Imu | gz IMU |
| `/<n>/benchmarker/collision` | ros_gz_interfaces/Contacts | gz contact (bumper) |
| sensor topics | see chapter 12 | gz sensors |
| `/<n>/gimbal/{yaw,roll,pitch}_cmd` | std_msgs/Float64 (in) | typhoon_h480 only |

Global: `/clock` (rosgraph_msgs/Clock), `/wind_xtdrone` (in, with `wind_control`).
