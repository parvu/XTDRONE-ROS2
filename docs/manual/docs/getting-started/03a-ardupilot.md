# 3b. ArduPilot branch

The `Ardupilot` branch of this repository flies the vehicles with **ArduPilot 4.7.2** (Copter, Plane and Rover SITL,
the latest stable release) instead of PX4. The ROS 2 interface (topics, keyboard control, communication nodes) is the
same; this page lists what is different.

## Installation and launch

```bash
scripts/build_ardupilot.sh          # ArduPilot SITL (waf), the ArduPilot Gazebo plugin, SITL_Models (fetched, not tracked)
scripts/build_xtdrone_ros2.sh       # calls it
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
```

| Launch argument | Meaning |
|---|---|
| `sitl:=false` | no autopilot (the models are spawned without the ArduPilot plugin) |
| `params:=NAME=value,...` | ArduPilot parameters, on top of the defaults of the vehicle family and the `ekf:=` preset |
| `ardupilot_dir:=` | the ArduPilot sources (default: the submodule) |
| `ekf:=gps|vision|laser2d` | EKF3 source preset (chapter 6) |

One SITL process (`arducopter`, `arduplane` or `ardurover`, `--model JSON`) runs per vehicle, talking to the
`ArduPilotPlugin` of the model over UDP port 9002 + 10 × instance and to MAVROS over UDP 14550 + instance. The
parameter files are in `ros2/xtdrone_gz/ardupilot/` (`xtdrone.parm`, `xtdrone-plane.parm`, `xtdrone-rover.parm`,
`xtdrone-vtol.parm`); the world step is 4 ms, so `SCHED_LOOP_RATE` is 100.

## Differences you will notice

* **Mode names.** XTDrone keeps the PX4 names on `/xtdrone/<vehicle>/cmd`; they are mapped in `xtdrone_py/common.py`:
  `OFFBOARD` → `GUIDED`, `AUTO.LAND` → `LAND` (`QLAND` for VTOLs), `AUTO.TAKEOFF` → `GUIDED` plus a takeoff command
  (copter) or `TAKEOFF` (plane), `AUTO.MISSION` → `AUTO`.
* **Arming and takeoff (multirotors).** ARM waits until the local position has streamed for 15 s (an earlier arming gave
  wandering headings). `GUIDED` needs a takeoff command after arming: the communication node sends it (1 m); a setpoint
  cancels a running takeoff, so the node streams setpoints only after the takeoff.
* **Thrust of `cmd_attitude`.** In `GUIDED` the thrust of an attitude target is a climb rate (0.5 holds the altitude).
* **Speeds.** `WP_SPD` and `WP_ACC` (m/s, m/s²) replace `WPNAV_SPEED`/`WPNAV_ACCEL`; the SLAM test cases use
  `params:=WP_SPD=1,WP_ACC=1`.
* **Missions** (chapter 29): item 0 of a mission is the home position, and `AUTO` mode arms only with
  `AUTO_OPTIONS=3`.
* **Planes** ignore position and velocity setpoints from a companion computer. `plane_communication` sends a position
  command as `MAV_CMD_DO_REPOSITION`: the plane loiters around the point (`WP_LOITER_RAD`).
* **VTOLs** (`standard_vtol`, ArduPlane QuadPlane `skywalker_x8_quad`): the communication node hovers in `QLOITER`
  and flies with RC overrides (a velocity command is a stick position; MAVROS runs with system id 255 so that the
  autopilot accepts them). `plane` switches to `GUIDED`: the forward transition starts and the plane loiters around the
  position commands. The formation of chapter 35 uses `GUIDED` attitude targets for the followers and a
  `DO_REPOSITION` of the leader over the UGV.
* **Local frames.** With GPS the local ENU frame of a vehicle starts where it was spawned; with a vision EKF it is
  the world frame.
* **Typhoon H480** is replaced by `iris_gimbal`, an iris with a 3-axis gimbal camera (same joint and topic names:
  `/<v>/gimbal/{yaw,roll,pitch}_cmd`, `cgo3_camera/image_raw`; `typhoon_h480` is an alias).
* **Rover** is SITL_Models' R1 (skid steering, ArduRover); the **plane** is ardupilot_gazebo's Zephyr flying wing
  (collision shapes simplified: a belly box with low friction).

## Not ported

| Vehicle | State |
|---|---|
| `tailsitter` | SITL_Models' `skycat_tvbs` (tilt-vector tailsitter) spawns and connects, but its physics blows up at the 4 ms world step when the vehicle arms (SITL aborts with SIGFPE in `Location::offset`: the model's position becomes NaN). |
| `tiltrotor` | SITL_Models has no tilt-rotor model. |

## Known limitations

* The vehicle-level gains of ArduPilot differ from PX4: the angular-speed mode of chapter 7 drifts by metres.
* Chapters 13, 14 and 20 (SLAM and Nav2 accuracy) have run-to-run variation; their test tolerances are 2.0 m,
  1.0 m and 0.7 m on this branch (0.45 m, 0.6 m and 0.5 m on the PX4 branch). A 2D-SLAM map frame has the heading of the
  estimator at the start, and a Nav2 run flew away once.
* Chapter 22 (EGO-Planner-Swarm): two of the four vehicles stall at the start of the obstacle field (the planner
  agents report time differences above its limit when the simulation runs at a real-time factor of about 0.5).
* Chapter 27 (20 vehicles) is too heavy for the 4 ms world step.
* `outdoor` worlds with SITL run at a real-time factor of about 0.5-1.2 depending on the number of vehicles.

## Debugging a SITL crash

`XTDRONE_SITL_GDB=1 ros2 launch xtdrone_bringup xtdrone.launch.py ...` runs each SITL process under `gdb` and prints
a backtrace into the launch log when it crashes.
