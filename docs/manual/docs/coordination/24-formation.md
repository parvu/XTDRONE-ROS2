# 24. Multi-UAV formation

!!! success "Ported and tested"
    Package `xtdrone_formation_demo` (`ros2/fomation_demo`): `leader`, `follower`, `follower_consensus` and `avoid`,
    rclpy ports of `coordination/formation_demo`. Test: `scripts/tests/run_chapter_tests.sh ch24`: nine iris take
    off to 8 m, hold the waiting grid, switch to `cube` and `pyramid` with the consensus followers and the collision
    avoidance, keep the pyramid while the leader moves, and land.

Six, nine or eighteen iris fly in formation. The keyboard steers the leader and the digit keys switch formations.
When the formation changes, the leader assigns the new positions with the Kuhn-Munkres (KM) algorithm, so the
total distance flown is small, and builds a communication topology for the consensus controller
([paper](https://arxiv.org/abs/2005.01125)).

## Run

```bash
# 1. nine iris on the waiting grid, ground truth as the common position reference (chapter 15)
ros2 launch xtdrone_bringup xtdrone.launch.py world:=empty vehicles:="iris*9" ekf:=vision communication:=basic
# 2. ground truth -> PX4 (all vehicles share the Gazebo frame)
ros2 run xtdrone_py get_local_pose iris 9
# 3. keyboard: i (climb) ... b (OFFBOARD) ... t (ARM), climb to about 8 m, s (hover)
ros2 run xtdrone_py multirotor_keyboard_control iris 9 vel
# 4. leader, consensus followers and collision avoidance
ros2 launch xtdrone_formation_demo formation.launch.py uav_count:=9 controller:=consensus avoid:=true
```

In the keyboard window press `g` to control the leader. `w`/`x`/`a`/`d`/`i`/`,` move it, and the digit keys select
a formation:

| Vehicles | `0` | `1` | `2` | `3` |
|---|---|---|---|---|
| 6 | waiting grid | T | diamond | triangle |
| 9 | waiting grid | cube | pyramid | triangle |
| 18 | waiting grid | cuboid | sphere | diamond |

Formations reach up to 4 m (9 vehicles) or 6 m (6 vehicles) below the leader, so fly the leader at about 8 m.

### Launch arguments of `formation.launch.py`

| Argument | Default | Meaning |
|---|---|---|
| `uav_type` | `iris` | vehicle type |
| `uav_count` | `6` | 6, 9 or 18 |
| `controller` | `simple` | `simple`: every follower tracks the leader (`follower.py`, chapter 25); `consensus`: every follower tracks its local leaders from `/xtdrone/communication_topology` (`follower_consensus.py`) |
| `control_type` | `vel` | `vel` or `accel` (consensus controller and avoidance; `accel` commands `cmd_accel_enu`) |
| `avoid` | `false` | start `avoid.py`: a pairwise push perpendicular to the line between two vehicles closer than 1.5 m |

The ROS 1 scripts did the same: `run_formation.sh iris 9` corresponds to `controller:=simple avoid:=true` and
`run_formation_consensus.sh iris 9 vel` to `controller:=consensus avoid:=true`.

!!! note "Changes from the ROS 1 version"
    - The leader publishes the formation and topology at 30 Hz instead of 200 Hz. The followers run at 30 Hz,
      and 200 Hz Python publishers to 8 to 17 subscribers load the machine noticeably.
    - In ROS 1, `0` on the keyboard sent `waiting`, which the leader ignored. Here it returns to the waiting grid
      (`origin`).

## Many vehicles on one machine

The 9-vehicle test runs at about 0.6 to 0.75 of real time on a 6-core machine without a GPU. Above six vehicles,
`xtdrone.launch.py` automatically switches to `mavlink_profile:=swarm`:
- **Lower PX4 stream rates:** only the streams the XTDrone nodes use are kept at full rate.
- **Fewer MAVROS plugins:** `mavros_plugins:=swarm`.

See chapter 43 for large swarms.

## Topics

| Topic | Type | From → to |
|---|---|---|
| `/xtdrone/leader/cmd_vel_flu`, `/xtdrone/leader/cmd` | Twist, String | keyboard → leader |
| `/xtdrone/formation_pattern` | Float32MultiArray (3 × (N−1)) | leader → followers |
| `/xtdrone/communication_topology` | Int32MultiArray (N × N) | leader → consensus followers |
| `/xtdrone/<type>_<i>/avoid_vel` (`avoid_accel`) | Vector3 | avoid → followers |
| `/xtdrone/<type>_<i>/cmd_vel_enu` (`cmd_accel_enu`) | Twist | followers → communication nodes |
| `/<type>_<i>/mavros/local_position/pose` | PoseStamped | MAVROS → leader, followers, avoid |
