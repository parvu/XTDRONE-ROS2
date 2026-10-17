# 21. 3D motion planning (EGO-Planner)

!!! success "Ported and tested"
    Package `xtdrone_planning`: `ego_planner.launch.py` with the EGO-Planner ROS 2 port (submodule
    `ros2/ego-planner-swarm`, branch `ros2_version`), `ego_transfer` and `ego_pos_cmd` (the ports of
    `ego_transfer.py` and XTDrone's modified trajectory server). Test: `scripts/tests/run_chapter_tests.sh ch21`: an
    `iris_realsense_camera` in `indoor1` flies 11.5 m down the cafe through four staggered columns, two of them
    on the straight line, planning on its depth camera with the ground truth as pose. It reaches the goal in
    about 65 s, up to about 1 m beside the straight line.

## Run

```bash
# 1. indoor1 with an iris_realsense_camera and four obstacle columns; PX4 takes its pose from vision
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1_ego ekf:=vision communication:=basic
# 2. pose source: the ground truth (or VIO, chapter 14)
ros2 run xtdrone_py get_local_pose iris 1
# 3. take off to about 1.2 m with the keyboard (offboard, arm, climb), then hover and close it
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
# 4. EGO-Planner to a goal in the local frame (here the world frame: the pose is the ground truth)
ros2 launch xtdrone_planning ego_planner.launch.py target_x:=0 target_y:=-4 target_z:=1.2
```

`goal:=rviz` instead plans to each 2D goal set in RViz (`/goal_pose`). `max_vel`, `max_acc`, `inflation`,
`floor` (the bottom of the planner's map), `ceiling` (its virtual ceiling) and `map_size_x|y|z` are launch arguments;
the defaults of the first five are those of XTDrone's ROS 1 `advanced_param_xtdrone.xml` and `run_in_xtdrone.launch`
(1 m/s, 2 m/s², −1 m, 3 m), except `inflation`, which is 0.35 m here instead of 0.2 m: with 0.2 m the planned path
passed 0.2 m from the columns and the iris, whose arms are 0.25 m from its centre, touched them.

`min_z` (default 1.2 m, ENU) is the lowest position the vehicle is commanded to. The planner sees the floor only
when the depth camera looks at it, and its trajectories dipped through it (z < 0) or ran at 0.3 to 0.7 m, where the
drone hit the tables and chairs of `indoor1`. Set `min_z:=0.0` to turn the limit off (the swarm of chapter 22 does).

`start_delay` (default 6 s, waypoint goals only) is how long the planner builds its map before it plans the first
trajectory: the planner waits for `/traj_start_trigger` and the launch sends it after that time. Without the delay
the map was still empty, the first path ran straight through the columns, and in about one run of ten the vehicle
ended against one. `start_delay:=0` starts at once.

`ego_planner.launch.py` starts these:
- **`ego_planner_node`** with the parameters of the port's `advanced_param.launch.py` and of XTDrone's
  `run_in_xtdrone.launch`. The grid map projects the depth image of the RealSense camera
  (`/iris_0/realsense/depth_camera/depth/image_raw`, 640 × 480, 32-bit float metres, fx = fy = 554.25) with the
  camera pose.
- **`traj_server`**: it samples EGO's B-spline into `quadrotor_msgs/PositionCommand` on
  `/xtdrone/iris_0/planning/pos_cmd`.
- **`ego_transfer`**: it publishes the camera pose (`/iris_0/camera_pose`, optical frame, stamped with
  simulation time) and EGO's odometry (`/iris_0/ego/odom`: MAVROS odometry with the velocity in the world frame
  and reliable QoS).
- **`ego_pos_cmd`**: it turns each position command into `/xtdrone/iris_0/cmd_pose_enu` (position and yaw), as
  XTDrone's modified ROS 1 trajectory server did.

!!! note "Changes from the ROS 1 version"
    - **Simulation time.** The port timed its trajectories with the wall clock. With Gazebo slower than real
      time (about 0.45× here with two rendered cameras), the trajectory server ran ahead of the vehicle, and the
      planner replanned from where the trajectory said the vehicle was, past the obstacles. `build_xtdrone_ros2.sh`
      copies the port's `plan_manage` package into the workspace and patches it
      (`xtdrone_planning/scripts/patch_ego_planner.py`) to use the node clock, which follows `/clock` with
      `use_sim_time`, as ROS 1 did. The submodule itself is not modified.
    - The camera pose is stamped with simulation time like the depth image: the grid map pairs them by time
      stamp, and with mismatched clocks it never integrated a depth image.
    - Jazzy has only `cv_bridge/cv_bridge.hpp`; the build gives the EGO packages a `cv_bridge.h` that includes it.
    - The vehicle starts at (0, 7.5) facing −y, down the cafe. The scenario `indoor1_ego` adds four columns
      (`obstacle_column`, 0.5 m × 0.5 m × 3 m), two of them on the straight line, so the test shows that the
      planner goes around obstacles. The depth camera also sees the visual-only counter and furniture of the
      cafe, which the planner avoids as well.

!!! tip "Narrow obstacles"
    EGO-Planner is made for cluttered spaces such as forests. In front of a wide flat wall the depth camera sees
    no way around it, unknown space counts as free, and the gradient-based optimisation can stall against the
    wall. A 2 m panel across the path stopped the vehicle in about half of the runs; the columns did not.

## Visualise

```bash
ros2 topic echo /xtdrone/iris_0/planning/pos_cmd --once     # the next setpoint of the planned trajectory
ros2 topic echo /iris_0/ego/odom --once                     # the odometry EGO plans from
ros2 run rqt_image_view rqt_image_view /iris_0/realsense/depth_camera/depth/image_raw   # what the planner sees
ros2 topic list | grep -i -E "grid_map|planning"            # EGO's map and trajectory topics, for RViz
rviz2                                                       # Fixed Frame: world
```

In RViz add a `MarkerArray` or `Path` display on the EGO trajectory topics found by the last command, and a
`PointCloud2` on its occupancy map. With `goal:=rviz`, the "2D Goal Pose" tool sets the goal.
