# Testing

The chapters are tested headless with `scripts/tests/run_chapter_tests.sh`. Each case starts the simulation of
a chapter (`ros2 launch xtdrone_bringup xtdrone.launch.py gui:=false ...`), runs a check, and stops everything.

```bash
source scripts/xtdrone_env.sh
scripts/tests/run_chapter_tests.sh                 # all cases
scripts/tests/run_chapter_tests.sh ch03 ch09       # selected cases
XTDRONE_TEST_GUI=true scripts/tests/run_chapter_tests.sh ch08plane   # watch it
```

Logs are written to `~/.ros/xtdrone_tests/<case>.log`. The script ends with a PASS/FAIL summary and exits
non-zero if a case failed.

## Cases

| Case | What it checks |
|---|---|
| `worlds` | every installed world loads in `gz sim -s` for 1000 iterations without SDF errors |
| `ch03` | `scenario:=indoor1`: stereo images and IMU publish; keyboard-style flight of `iris_0` |
| `ch05` | flight with `multirotor_communication_enhanced` (position hold) |
| `ch06` | flight with `ekf:=vision` and `get_local_pose` as the vision source |
| `ch07` | the attitude demo, driven through a pseudo terminal in `angular_speed` and `angle` mode, reaches and holds (1, 0, 2) |
| `ch08plane` | plane: AUTO.TAKEOFF + ARM, climbs above 15 m |
| `ch08vtol` | standard_vtol: multirotor-mode flight |
| `ch08rover` | rover: OFFBOARD forward drive of 3 m |
| `ch09` | six iris take off, hover and land together |
| `ch10` | a second iris is added with `spawn_vehicle.launch.py` and flies with the first |
| `ch12` | all sensor topics of the iris sensor models publish |
| `ch24` | 9 iris at 8 m with `controller:=consensus avoid:=true`: waiting grid, `cube`, `pyramid` (≤ 0.8 m per follower), pyramid kept while the leader moves |
| `ch27` | 20 iris (`speed_factor:=0.5`): channel model up, AUTO.TAKEOFF, FORM_1 moves the leader; UAVs 1-12 follow it, UAV 13 (no lower-id neighbour on the channel) holds |
| `ch28` | three `iris_downward_camera` 0.5 m off their tags at 4-6 m land on tags 0-2 (≤ 0.3 m; 0.02 m reached) |
| `ch42` | XTDGroundControl, offscreen: two iris climb above 2 m from the GUI, the state panel shows them, they land |
| `ch11` | two CAT Vehicle UGVs on `outdoor3` drive straight and steer left; their lidar and camera topics publish |
| `ch16` | `iris_realsense_camera` turns and flies in `indoor1`; RTAB-Map's cloud map has > 10 000 points |
| `ch13` / `ch14` | `iris_stereo_camera` in `outdoor1` (scenario `outdoor1_stereo`) flies a 2 m square on the ORB-SLAM3 / VINS-Fusion pose only (no GPS, `ekf:=vision`); SLAM vs ground truth ≤ 0.5 m (VINS 0.6 m), compared in the SLAM frame (it starts along the vehicle's heading). ch13 needs `scripts/build_orbslam3.sh` |
| `ch17` / `ch18` | `iris_2d_lidar` in `indoor3` flies a 1 m square (the walls of `indoor3` leave no room for 2 m around its start) on the slam_toolbox / Cartographer pose only (no GPS, `ekf:=laser2d`); SLAM vs ground truth ≤ 0.5 m |
| `ch19` | `iris_3d_lidar` in `outdoor1` (scenario `outdoor1_3d_lidar`) flies a 2 m square on the KISS-ICP odometry only (no GPS, `ekf:=vision`); odometry vs ground truth ≤ 0.5 m |
| `ch21` | `iris_realsense_camera` in `indoor1` (scenario `indoor1_ego`, four columns) flies 11.5 m with EGO-Planner on the depth image and the ground truth as pose; goal within 1 m, never down to the floor, at least 0.45 m beside the straight line (it went around the columns) |
| `ch22` | the 4 iris of `ego_swarm` fly with EGO-Planner-Swarm through the windows of `ego_swarm_4` to XTDrone's goals; every vehicle within 1 m of its goal, no two closer than 0.5 m, none down to the floor (positions in `ego_swarm_trace.csv`). |
| `ch23` | `typhoon_h480` in `outdoor1` hovers at 5 m, gimbal 60° down; YOLO (OpenCV DNN, YOLOv4-tiny) detects the walking pedestrian (waits until simulation time 330 s, the pedestrian's second pass), then `yolo_human_tracking` follows it: the typhoon moves and the person stays detected |
| `ch20` | Nav2 flies to three goals while slam_toolbox maps `indoor3` (≤ 0.5 m each) |
| `ch30` | the UGV keeps its lane on `outdoor3` for 30 m (`lane_detection` + `self_driving`) |
| `ch31` | the UGV in `outdoor1` (scenario `outdoor1_ugv`): slam_toolbox maps from the front lidar, hybrid A* plans to (20, −1.5) (a lane change), pure pursuit drives there; within 2 m of the goal, planner and follower still running |
| `ch33` | `iris_arm` in `grasping` (scenario `grasping_arm`): vision sees the cube, MoveIt moves the arm to it, the gripper takes it and lifts it (≥ 1 cm); the iris takes off and the cube stays within 0.5 m of it above 1.5 m |
| `ch32` | `sandisland` + VRX WAM-V: the boat floats and drives 5 m on its thrusters; an iris takes off from the island |
| `ch38` | two `robocup` actors walk to commanded points (`/actor_i/cmd_motion`, `xtdrone::ActorMotion`) |
| `ch25` | 6 iris at 8 m: the formation demo forms the waiting grid, `T` and `triangle` (≤ 0.8 m per follower) and keeps the triangle while the leader moves |
| `ch15` | `vision_pose` / `vision_speed` publish |
| `ch44` | the bumper reports the ground contact |
| `ch36` | `make_gazebo_model.py` generates the `landing1` panel; it spawns into a running world |
| `ch37` | `control_gazebo_models` moves the parked cars of `outdoor1` |
| `ch41` | a 5 m/s wind set through `wind_control` makes a hovering iris lean into it |
| `ch43` | 4 iris without PX4 in `empty_zero_g`; `speed_factor:=2` reaches a real time factor of 2 |

## The flight check

`scripts/tests/xtdrone_flight_test.py` flies like a keyboard user, through the XTDrone command topics:

1. wait until MAVROS reports `connected`, a local position and a home position;
2. wait until the vehicles rest (spawn drops, estimator settling) and take the altitude reference;
3. send an upward `cmd_vel_flu`, then `OFFBOARD` and `ARM` (keys `i`, `b`, `t`). PX4 can refuse to arm for a
   few seconds after the home position is set; like a user, the test presses `b` and `t` again until it arms;
4. check that every vehicle climbs more than 2 m;
5. send `HOVER` (key `s`) and check that the altitude drifts less than 0.5 m in 5 s;
6. send `AUTO.LAND` (key `n`) and wait until all vehicles disarm.

It can also check topics by itself: `--topics /a /b` (each topic delivers a message) or `--exist-topics`.
Topic subscriptions are released after the check, so camera streams do not keep slowing the flight part down.

The `worlds` case allows 15 minutes per world, because `indoor4` needs about 8 minutes to load on a CPU-only machine.

## Unit tests

```bash
cd ros2/xtdrone_bringup && python3 -m pytest -q test     # fleet spec, ports, scenarios, bridge config
```
