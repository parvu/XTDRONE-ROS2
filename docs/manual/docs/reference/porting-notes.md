# Porting notes: ROS 1 / Gazebo Classic → ROS 2 / Gazebo Harmonic

For readers who know the original XTDrone, and for porting your own XTDrone code.

## Commands

| ROS 1 / Classic | ROS 2 / Harmonic |
|---|---|
| `roslaunch px4 indoor1.launch` | `ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1` |
| `python3 generator.py` + `roslaunch px4 multi_vehicle.launch` | `ros2 launch xtdrone_bringup xtdrone.launch.py world:=... vehicles:="iris*6,plane*2"` |
| `python multirotor_communication.py iris 0` | `ros2 run xtdrone_py multirotor_communication iris 0` |
| `bash multi_vehicle_communication.sh` | `ros2 launch xtdrone_py multi_vehicle_communication.launch.py iris_num:=6`, or `communication:=basic` |
| `python multirotor_keyboard_control.py iris 1 vel` | `ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel` |
| `python get_local_pose.py iris 1` | `ros2 run xtdrone_py get_local_pose iris 1` |
| `rostopic echo /iris_0/mavros/state` | `ros2 topic echo /iris_0/mavros/state` |
| `killall gzserver gzclient` | `pkill -f "gz sim"` |
| editing `rcS` for EKF parameters | `ekf:=gps|vision`, `px4_params:="NAME=value ..."` |
| `PX4_SIM_SPEED_FACTOR=6 roslaunch ...` | `speed_factor:=6` |

## Unchanged interfaces

- Vehicle names `<type>_<id>`; MAVROS under `/<type>_<id>/mavros/...`.
- XTDrone command topics `/xtdrone/<type>_<id>/{cmd, cmd_pose_flu, cmd_pose_enu, cmd_vel_flu, cmd_vel_enu, cmd_accel_flu, cmd_accel_enu}`,
  `/xtdrone/leader/...`, and the string commands (`ARM`, `OFFBOARD`, `HOVER`, `AUTO.LAND`, …).
- Sensor topics (chapter 12) and frame IDs.
- MAVLink offboard ports 24540+N / 34580+N (chapter 48).

## Changed behaviour

| Area | Change |
|---|---|
| iris / solo | fly with PX4's x500 frame (heavier, hover throttle ≈ 0.6) |
| tailsitter | PX4's *quad* tailsitter instead of XTDrone's duo tailsitter |
| typhoon_h480 | converted model with an XTDrone airframe (4901); landing gear fixed |
| 2D lidar | 40 Hz instead of 500 Hz |
| 3D lidar topic | `/<type>_<id>/velodyne_points` instead of `/velodyne_points` |
| ground truth | `/<type>_<id>/ground_truth/odom` instead of `/gazebo/model_states` |
| moving models | gz `set_pose(_vector)` service instead of `/gazebo/set_model_states` |
| contact topic | `ros_gz_interfaces/Contacts` instead of `gazebo_msgs/ContactsState` |
| world physics | DART, 4 ms step; all worlds converted automatically |
| world fixes by the converter | SDF 1.9 / DART reject things Classic accepted, so `classic_to_gz.py` fixes them and prints a warning: zero-size shapes become 1 mm; duplicate sibling names get a `_N` suffix; collision meshes with COLLADA line primitives (SketchUp edges) use a `*_collision.dae` copy without lines; Classic-only `frame=""` attributes and stray `<friction>`/`<contact>` under `<collision>` are dropped; the unpublished `actor_walk` meshes are replaced by Classic's `walk.dae` |
| materials | OGRE scripts → PBR materials (textures look different) |
| actors | lidars see actors without the collision plugin; ROS-controlled actors not ported yet |
| sim time | Gazebo publishes `/clock`; sensor messages carry simulation time. Use `use_sim_time:=true` for nodes that combine sensor data with other clocks |

## Porting a rospy script

The `xtdrone_py` package shows the pattern:

```python
# ROS 1                                         # ROS 2
rospy.init_node('x')                            rclpy.init(); node = rclpy.create_node('x')
rospy.Subscriber(t, T, cb, queue_size=1)        node.create_subscription(T, t, cb, 1)
rospy.Publisher(t, T, queue_size=1)             node.create_publisher(T, t, 1)
rospy.ServiceProxy(s, S)(args)                  cli = node.create_client(S, s); cli.call_async(S.Request(...))
rospy.Rate(30) loop                             node.create_timer(1/30, callback); rclpy.spin(node)
```

Two pitfalls:

1. **QoS**: MAVROS 2 publishes state and pose topics with the *sensor data* QoS (best effort). Subscribe with
   `rclpy.qos.qos_profile_sensor_data`, or you receive nothing.
2. **Service calls inside callbacks**: a synchronous call from a callback deadlocks a single-threaded executor. Use
   `call_async` and handle the result in a done-callback (see `xtdrone_py/common.py`).

Use full topic names (`/iris_0/mavros/...`). Relative names resolve against the node namespace.

## Findings from phases 3 and 4

- **Ground truth must not reach PX4.** Gazebo's OdometryPublisher (the `/<vehicle>/ground_truth/odom` source) also
  publishes `/model/<name>/odometry_with_covariance` by default. PX4's `gz_bridge` subscribes to exactly that topic
  and fuses it as external vision. The models now send that output to an unused topic; vision-mode EKF setups get
  the ground truth only through `get_local_pose`, as intended.
- **Classic invisible ceilings.** The indoor and zhihang worlds have a dynamic 300 m × 300 m box released at 5 m. In
  Classic (ODE) it falls onto the walls and rests there. In gz-sim (DART) the same contact situation slowed `indoor3`
  to 5 % of real time. The converter makes these ceilings static at the height where they would rest: on the walls.
- **AckermannSteering odometry** shares `/model/<name>/odometry` with the ground truth unless `odom_topic` is set.
- **GPU lidars looking straight down** (pitch exactly 90°) measure wrong ranges, because the render camera is
  degenerate. The range finders are pitched 86°.
- **Many vehicles on one machine:** see chapter 43 (swarm MAVLink profile, MAVROS plugin set, no MAVROS `param`
  plugin, one lazy bridge, network tuning).
- **Orphaned processes.** If a launch is killed with SIGKILL, its children keep running: PX4's `rcS` shell, static
  TF publishers, SLAM nodes. They then disturb later runs (stale poses, CPU load). The test runner's cleanup kills
  them by executable path.

## Findings from phase 5

- **Wall clock in ROS 2 ports.** Code that reads `rclcpp::Clock()` (or an unattached `RCL_ROS_TIME` clock) runs on
  the wall clock even with `use_sim_time`. With Gazebo slower than real time this breaks anything that times
  trajectories: the EGO-Planner port did (chapter 21), and `xtdrone_planning/scripts/patch_ego_planner.py` moves
  it to the node clock. Messages paired by time stamp (a depth image and a camera pose) must use the same clock.
- **Heightmaps.** gz-sim draws a heightmap at its `<pos>` only (link and visual poses are ignored) and scales the
  image to its brightest pixel, where Classic added the link pose and scaled pixel / 255. The converter rewrites
  `<pos>` and `<size>` so the terrain has the Classic heights; before that `city_terrain` covered the roads of
  `outdoor1`.
- **Actors with `walk.dae`.** A bare skin name was found by Classic in its media; the mesh origin is at the hips, so
  gz-sim needs the waypoints 1 m higher. Classic-saved worlds also carry the skin as `<link>` visuals, which the
  converter drops. A converted world can load without errors and still show no actor: check it in a camera image.
- **Spawn points inside models.** The origin of `outdoor1` is inside a parked utility vehicle: a multirotor
  spawned there cannot climb past 0.4 m. Its scenarios start the vehicles beside it.
- **Visual odometry needs texture.** After the conversion the walls of `indoor1` are plain, and ORB-SLAM3 and
  VINS lose their features there; chapters 13 and 14 use the city of `outdoor1`. Plain, close surfaces in front of
  the camera (the large red arrow of `outdoor1`) also break ORB-SLAM3 tracking.
- **Stereo pairs.** Under load Gazebo drops left and right camera frames independently; pair them by exact time
  stamp, not approximately.
