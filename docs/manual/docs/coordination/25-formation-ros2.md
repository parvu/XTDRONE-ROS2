# 25. Multi-UAV formation (ROS 2)

!!! success "Ported and tested"
    Package `xtdrone_formation_demo` (`ros2/fomation_demo`). It was already ROS 2 code. It now runs against the
    MAVROS 2 topics of this setup (sensor-data QoS for the MAVROS poses) and builds with `--symlink-install`.
    Test: `scripts/tests/run_chapter_tests.sh ch25`: six iris take off to 8 m, form the waiting grid, `T` and
    `triangle`, and keep the triangle while the leader moves.

The original chapter (it is not public on Yuque any more) ran the leader/follower formation of chapter 24 as
ROS 2 nodes, connected to ROS 1 through a bridge. With ROS 2 Jazzy and MAVROS 2, no bridge is needed.

## Run

```bash
# 1. six iris on the waiting grid, ground truth as the common position reference (chapter 15)
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=formation_6 ekf:=vision communication:=basic
# 2. ground truth -> PX4
ros2 run xtdrone_py get_local_pose iris 6
# 3. keyboard: take off all vehicles (i … b … t), climb to about 8 m, hover (s)
ros2 run xtdrone_py multirotor_keyboard_control iris 6 vel
# 4. leader + 5 followers
ros2 launch xtdrone_formation_demo formation.launch.py uav_type:=iris uav_count:=6
```

Supported fleet sizes are 6, 9 and 18. Start the demo only after all vehicles hover: the followers immediately
steer towards their formation positions. In the keyboard window, press `g` to control the leader: `w`/`x`/`a`/`d`
move it (`/xtdrone/leader/cmd_vel_flu`), and the digit keys switch formations through `/xtdrone/leader/cmd`.
With 6 vehicles, `1` is T, `2` diamond and `3` triangle; `0` returns to the waiting grid (`origin`).

Fly the leader high enough: formations place followers up to 6 m below it (T), so about 8 m is needed.

The vehicles must start on the formation's waiting grid. `scenario:=formation_6` places six iris on it; for 9 and 18
vehicles `vehicles:="iris*9"` and `vehicles:="iris*18"` already produce the matching grid.

The followers need all vehicles in one coordinate frame, so use the ground truth (`ekf:=vision` with
`get_local_pose`). With GPS, each vehicle's local frame starts at its own takeoff point.

## Topics

| Topic | Type | From → to |
|---|---|---|
| `/xtdrone/leader/cmd_vel_flu`, `/xtdrone/leader/cmd` | Twist, String | keyboard → leader |
| `/xtdrone/formation_pattern` | Float32MultiArray | leader → followers |
| `/xtdrone/<type>_<i>/cmd_vel_enu`, `/xtdrone/<type>_<i>/cmd` | Twist, String | leader/followers → communication nodes |
| `/<type>_<i>/mavros/local_position/pose` | PoseStamped | MAVROS → leader/followers |
