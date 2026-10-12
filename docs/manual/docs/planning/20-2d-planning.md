# 20. 2D motion planning (move_base → Nav2)

!!! success "Ported and tested"
    Package `xtdrone_planning`: `nav2_2d.launch.py` and `config/nav2_2d.yaml` (generated from Nav2's defaults by
    `scripts/make_nav2_params.py`). Test: `scripts/tests/run_chapter_tests.sh ch20`: while slam_toolbox maps
    `indoor3`, Nav2 flies the `iris_2d_lidar` to three goals (2.5 m apart); each is reached within 0.4 m.

Nav2 replaces `move_base`. As in the ROS 1 chapter, the drone flies at a constant height and Nav2 plans in 2D.
The planner is NavFn in Dijkstra mode; the ROS 1 chapter chose Dijkstra over A*. The controller is DWB, Nav2's
version of the DWA planner, set up holonomic because a multirotor can move sideways.

## Plan while mapping

```bash
# simulation, SLAM pose -> PX4 (chapter 17); "enhanced" holds the height while Nav2 sends zero vertical speed
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=enhanced \
    px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1
ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox
# take off to about 1 m with the keyboard (i, b, t, then s to hover), then close the keyboard
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
ros2 launch xtdrone_planning nav2_2d.launch.py rviz:=true        # set a "2D Goal Pose" in RViz
```

Close the keyboard before you send goals: its commands would fight with Nav2's, as the ROS 1 chapter warns.
Goals can also come from code, through the `navigate_to_pose` action (see `scripts/tests/nav2_test.py`).

## What the launch starts

`controller_server`, `planner_server`, `behavior_server`, `bt_navigator` and their lifecycle manager. The remaps
are `cmd_vel → /xtdrone/<vehicle>/cmd_vel_flu` and `odom → /<vehicle>/mavros/local_position/odom` (velocity
feedback). The map comes from the SLAM (`/map`) and the robot pose from TF `map → base_link`.

| Parameter (`nav2_2d.yaml`) | Value |
|---|---|
| frames | global and local costmap and behaviors in `map` (the SLAM chain has a static `odom → base_link`) |
| robot radius | 0.4 m, inflation 0.7 m |
| obstacles | `/iris_0/scan` |
| DWB | holonomic, up to 0.4 m/s (ROS 1 DWA: 0.2–0.3 m/s) |
| goal tolerance | 0.2 m, any yaw |

## Plan on a saved map

Save the slam_toolbox map with its serialization service (`/slam_toolbox/serialize_map`), then start slam_toolbox in
localization mode (`mode: localization`, `map_file_name`) instead of mapping, and run Nav2 the same way. Plain
`map_server` + AMCL does not fit here: AMCL needs a moving odometry frame, and this setup has none.
