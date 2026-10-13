# 35. Fixed-wing drone formation tracking a UGV

!!! success "Ported and tested"
    Package `xtdrone_fixed_wing_formation` (messages and the four nodes of XTDrone's
    `coordination/fixed_wing_formation_control`), scenario `outdoor2_fw_formation`. Test:
    `scripts/tests/run_chapter_tests.sh ch35`: three standard VTOLs take off, transition to fixed-wing flight and
    fly the triangle formation; the leader circles over the UGV, and over the last minute the followers keep their
    slots (mean error < 40 m) and nobody comes down.

Three standard VTOLs fly in formation over a ground vehicle. The leader circles over the UGV; the two followers
fly with fixed north/east offsets from the leader, controlled by XTDrone's formation law (a lateral guidance law and
TECS for height and speed) that sends attitude and throttle to PX4 in OFFBOARD mode.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor2_fw_formation mavlink_profile:=light \
  px4_params:=MIS_TAKEOFF_ALT=40,VT_TRANS_TIMEOUT=30,VT_F_TRANS_THR=0.75,ASPD_DO_CHECKS=0,SDLOG_MODE=-1
ros2 launch xtdrone_fixed_wing_formation fixed_wing_formation.launch.py
```

Take off (AUTO.TAKEOFF and arm, e.g. QGroundControl or `multirotor_keyboard_control`), transition each VTOL to
fixed-wing flight and start the formation, the leader first:

```bash
ros2 service call /standard_vtol_0/mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 3000, param1: 4.0}"
ros2 run xtdrone_fixed_wing_formation switch_fw_mode --ros-args -p plane_id:=0 -p mode:=1 -p formation_type:=2
# then the same for plane_id 1 and 2
```

`formation_type`: 1 north-south line, 2 triangle, 3 east-west line (offsets of 30 m). `scripts/tests/
fw_formation_test.py` does the whole sequence. Drive the UGV with chapter 11's keyboard control; the leader follows.

| Node (per plane, `plane_id`) | ROS 1 executables | |
|---|---|---|
| `pack_fw_states` | `pack_leader_states`, `pack_fw1_states`, ... | MAVROS → `FwStates`; `FwCmd` → `setpoint_raw/attitude` (followers); the orbit over the UGV (leader) |
| `task_main` | `leader_main`, `follower1_main`, ... | mode state machine and the formation controller |
| `vir_sim_leader` | `vir_sim_leader` | the leader's `FwStates` as `LeaderStates` |
| `switch_fw_mode` | `switch_fw1_mode`, ... (keyboard) | parameters `mode`, `formation_type` instead of keyboard input |

Topics are in `/standard_vtol_N/fixed_wing_formation_control/` (`fw_states`, `fw_cmd`, `fw_cmd_mode`,
`leader_states`, `formation_control_states`).

## Notes

- **Messages.** ROS 2 field names are lower case: `planeID` → `plane_id`, `err_P_N` → `err_p_n`, … .
- **Start positions.** ROS 1 started the VTOLs at x = −1500 m. Gazebo Harmonic's ground plane is a box about 2100 m
  wide, so they start at x = −600 m.
- **The leader's orbit.** ROS 1 sent the UGV position with PX4's old "loiter" type mask, which PX4 v1.16 ignores. The
  leader now flies a 150 m circle around the UGV (`pack_fw_states` parameters `orbit_radius`, `orbit_lead_angle`):
  a plain position setpoint makes it cross the UGV in turns too tight for the followers.
- **Controller tuning.** In OFFBOARD attitude mode PX4's own limits do not apply, and with ROS 1's settings the
  followers stalled, spun or dove into the ground. The follower limits are now those of PX4 v1.16's
  `standard_vtol`: pitch −15…20° (ROS 1: ±45°), roll 25° (45°), airspeed 12…20 m/s (12…22), minimum throttle 0.2
  (0.1); the roll and pitch commands are slew-limited (30°/s, 15°/s), TECS is softer (time constants 8 and 10 s,
  climb and sink 3 m/s) and runs on simulation time (ROS 1: wall time), and a stall guard levels a follower below
  10 m/s.
- **Airspeed.** The VTOL model has a Gazebo airspeed sensor (PX4's airspeed simulation stalled with three VTOLs).
  PX4's airspeed selector looks for sensors only while disarmed: arm after the airspeed is healthy (the test waits
  for it in `sys_status`), or the transition times out and PX4 returns the VTOL to multicopter flight for good. Under
  heavy load a PX4 instance has also lost its airspeed right when it was armed, and stayed so after disarming; the
  test arms each VTOL on the ground and checks the airspeed for a few seconds before the take-off (if it is lost,
  restart the simulation; the test runner does so up to twice).
- **Transitions one at a time**, each VTOL straight into the formation: VTOLs left loitering after their transition
  flew into the ones still taking off.
- **CPU load.** Three VTOLs, their MAVROS nodes and Gazebo fill a 6-core machine. When the PX4 instances fall
  behind Gazebo's clock, their timer-driven modules (airspeed selector, battery simulation) stop and the estimator
  can diverge; the transitions then fail. The scenario's UGV is `ugv_nosensor` (no cameras and lidars: about twice
  the real-time factor), `mavlink_profile:=light` turns off the MAVLink streams nobody reads (each MAVROS used a whole
  core with the full set), and `SDLOG_MODE=-1` turns off PX4's flight log. The swarm profile is too slow for the
  formation controller (position at 5 Hz).
