# Packages, launch arguments and topics

## Repository layout

| Path | Content |
|---|---|
| `PX4-Autopilot/` | PX4 v1.16.2 submodule (unchanged) |
| `ros2/xtdrone_gz/` | vehicle models (`models/`), hand-made worlds (`worlds/`), PX4 overlays (`px4/`), converters (`scripts/`); converted Classic assets in `generated/` (build output) |
| `ros2/xtdrone_bringup/` | `xtdrone.launch.py`, `spawn_vehicle.launch.py`, scenarios (`config/scenarios/*.yaml`), fleet logic |
| `ros2/xtdrone_py/` | rclpy nodes: communication, keyboard, ground truth, demos, Gazebo helpers |
| `ros2/fomation_demo/` | package `xtdrone_formation_demo`: leader/followers, consensus, avoidance, communication model (chapters 24–27) |
| `ros2/xtdrone_msgs/` | messages of the communication-model demo (`UAVDataMsg`, `AllUAVData`, `CommVerify`, …) |
| `ros2/xtdrone_groundcontrol/` | XTDGroundControl, the PyQt5 ground station (chapter 42) |
| `ros2/xtdrone_slam/` | laser SLAM launch (slam_toolbox / Cartographer), RTAB-Map launch, `laser_transfer`, `odom_relay` (chapters 16–18) |
| `ros2/xtdrone_planning/` | Nav2 launch and parameters for 2D planning (chapter 20) |
| `ros2/xtdrone_gz_plugins/` | Gazebo Harmonic systems: `xtdrone::ActorMotion` (chapter 38) |
| `ros2/vrx/` | VRX v3.1.2 submodule (ocean, WAM-V; chapter 32) |
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
| `bridge` | `true` | one lazy ros_gz_bridge for the fleet (sensors, ground truth, gimbal; a topic is converted only while it has subscribers) |
| `communication` | `none` | `basic` / `enhanced` XTDrone communication nodes |
| `px4` | `true` | `false`: spawn models only (chapter 43) |
| `ekf` | `gps` | EKF2 preset `gps`, `vision` (ground truth via `get_local_pose`) or `laser2d` (2D laser SLAM pose + range finder height + baro), after the XTDrone defaults `NAV_DLL_ACT=0 COM_RCL_EXCEPT=4` |
| `mavros_plugins` | `auto` | `xtdrone`: the MAVROS plugins XTDrone uses (sys_status, sys_time, command, local/global position, imu, altitude, home_position, setpoint_*, vision_pose, vision_speed, odometry); `swarm`: only sys_status, sys_time, command, local_position, home_position, setpoint_raw, vision_pose, vision_speed; `all`: MAVROS' PX4 default set; `auto`: `swarm` with the swarm MAVLink profile, else `xtdrone`. No `param` plugin except with `all` (XTDrone sets parameters at startup) |
| `mavlink_profile` | `auto` | PX4 stream rates on the MAVROS link: `full`, `swarm` (lower rates of the streams XTDrone does not need at full rate), `auto` (`swarm` above 6 vehicles) |
| `uxrce_dds` | `false` | start PX4's uXRCE-DDS client (for px4_msgs code; needs `MicroXRCEAgent udp4 -p 8888`) |
| `px4_params` | — | extra PX4 parameters `NAME=value NAME=value` (or comma-separated) |
| `speed_factor` | `1.0` | target real-time factor |
| `gz_verbose` | `1` | Gazebo verbosity |
| `px4_dir` | auto | PX4 checkout (must be v1.16.2 unless `XTDRONE_ALLOW_ANY_PX4=1`) |
| `rootfs` | `~/.ros/xtdrone_px4` | PX4 instance working directories |

`spawn_vehicle.launch.py`: `model`, `id`, `instance`, `x y z R P Y`, `world`, `communication`, `ekf`, `px4_params`, `mavros_plugins`, `mavlink_profile`, `uxrce_dds`.

Scenario files (`config/scenarios/*.yaml`) contain `world`, `vehicles` (`model`, `type`, `id`, `pose`) and optional static `objects` (`model`, `name`, `pose`).

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
| `precision_landing` | `<type> <id> [--detector opencv|apriltag_ros]` | control/precision_landing.py |
| `ugv_keyboard_control` | `<num>` | control/keyboard/ugv_keyboard_control.py |
| `ugv_communication` | `<id>` | (catvehicle cmdvel2gazebo controller; started by the launch for every UGV) |
| `lane_detection` | `ugv|rover <id>` | sensing/lane_detection/lane_detection_{ugv,rover}.py |
| `self_driving` | `ugv|rover <id> [--speed]` | control/{ugv,rover}_self_driving.py |
| `control_actor` | `<actor id> <x> <y> <speed>` | control/actor/control_actor.py (launch: `control_actors.launch.py`) |

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

## `xtdrone_formation_demo` executables

| Executable | Arguments | Original script |
|---|---|---|
| `leader` | `<type> <6|9|18>` | coordination/formation_demo/leader.py |
| `follower` | `<type> <id> <num>` | coordination/formation_demo/follower.py |
| `follower_consensus` | `<type> <id> <num> [vel|accel]` | coordination/formation_demo/follower_consensus.py |
| `avoid` | `<type> <num> [vel|accel]` | coordination/formation_demo/avoid.py |
| `formation_comm_controller` | `<type> <id> <num> [--pos-file]` | formation_communication_demo/control/script/sim_addleader_1.py |
| `communication_verify` | `[num] [--leaders]` | formation_communication_demo/control/script/communication_verify_1.py |
| `uav_data_aggregator` | `<type> <num>` | (UAV-state collection of the modified keyboard / xtdrone_qt) |
| `formation_comm_all` | `<type> <num> [--leaders] [--pos-file]` | all of the three above in one process |

Launch files: `formation.launch.py` (chapters 24, 25), `formation_communication.launch.py` (chapter 27).
`xtdrone_groundcontrol`: `ros2 run xtdrone_groundcontrol xtdgroundcontrol` (chapter 42).

Other launch files: `xtdrone_bringup usv.launch.py` (VRX WAM-V into `sandisland`), `xtdrone_slam laser_slam_2d.launch.py`
(`slam:=slam_toolbox|cartographer`), `xtdrone_slam rtabmap_rgbd.launch.py` (`odom:=groundtruth|visual|external`),
`xtdrone_planning nav2_2d.launch.py`.

Vehicle models added in phase 4: `ugv`, `ugv_tags3_5` (CAT Vehicle, generated by `generate_ugv_models.py`),
`rover_with_lidar_stereo` (r1 rover + front stereo camera + 3D lidar); `iris_2d_lidar` has the downward range finder
`distance` again.
