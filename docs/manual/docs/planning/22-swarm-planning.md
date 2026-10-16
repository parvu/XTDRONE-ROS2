# 22. Swarm motion planning (EGO-Planner-Swarm)

!!! success "Ported and tested"
    Package `xtdrone_planning`: `ego_swarm.launch.py`, one EGO-Planner (chapter 21) per vehicle, sharing their
    trajectories. Test: `scripts/tests/run_chapter_tests.sh ch22`: the four `iris_realsense_camera` of
    `scenario:=ego_swarm` fly from x = −7 to XTDrone's goals at x = 9, through the windows of the wall at x = −5,
    in about 60–80 s; no two vehicles come closer than 1.7 m, and none comes below 0.3 m.

## Run

```bash
# 1. four iris with RealSense cameras in ego_swarm_4; PX4 takes its pose from vision
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=ego_swarm ekf:=vision communication:=basic
# 2. pose source: the ground truth, in one world frame for all vehicles
ros2 run xtdrone_py get_local_pose iris 4
# 3. take off all four to about 0.8 m (keyboard: control all, offboard, arm, climb), then hover
ros2 run xtdrone_py multirotor_keyboard_control iris 4 vel
# 4. the planners, with XTDrone's goals (ego_swarm_goal.sh)
ros2 launch xtdrone_planning ego_swarm.launch.py count:=4
```

`count:=8` with `scenario:=ego_swarm_8` uses the goals of `ego_swarm_goal_8.sh`. `goals:="x,y,z;x,y,z;..."` sets
others, and `goal_mode:=rviz` takes each vehicle's goals from `/iris_<i>/goal_pose`.

The course: a wall across the field at x = −5 (model `22`) with one window per lane, about 1.2 m wide, at
heights 0.2–0.9 m (`iris_0`, y = 5), 0.7–1.4 m (`iris_1`, y = 1.2), 0.7–2.2 m (`iris_2`, y = −1.2) and 0.5–1.2 m
(`iris_3`, y = −4), a fence around the 20 m × 20 m field, and 1 m cylinders.

!!! note "Changes from the ROS 1 version"
    - The planners avoid each other through the trajectories they broadcast on every replan (`/broadcast_bspline`).
      EGO-Planner-Swarm's sequential start, in which each planner waits for the trajectories of the one before it
      (sent once), is off (`sequential_start:=false`, a parameter the build adds to the port). On this machine
      the hand-over was lost in about half of the runs and left vehicles waiting, even with transient-local QoS.
      The planners start 4 s apart, `iris_0` last.
    - The planner parameters are those of XTDrone's `advanced_param_xtdrone.xml`: inflation 0.2 m, ground height
      −1 m, virtual ceiling 3 m (launch arguments `inflation`, `floor`, `ceiling`).
    - Before the converter applied the `<state>` of the saved world, the wall stood along the field and the
      cylinders were half underground, an easier course on which all four vehicles arrived.
    - MAVROS runs on simulation time, like PX4. With MAVROS on wall time, the four PX4 instances reset their time
      sync about 40 s after the planners started (the simulation slows down abruptly), dropped the vision pose
      and blind-landed; see the porting notes.

## Visualise

```bash
ros2 topic echo /xtdrone/iris_0/planning/pos_cmd --once      # the trajectory setpoint of vehicle 0
ros2 topic list | grep -i -E "planning|grid_map"             # trajectory and map topics of each vehicle
rviz2                                                        # Fixed Frame: world
```

In RViz add `Path`/`MarkerArray` displays on the trajectory topics of every vehicle. Each vehicle plans its own
trajectory, and the others' trajectories come in as obstacles: the paths should bend away from each other.
`goal_mode:=rviz` sends each vehicle's goal from `/iris_<i>/goal_pose` (the "2D Goal Pose" tool publishes one).
