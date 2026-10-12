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
