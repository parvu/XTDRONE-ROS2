# 7. Multirotor attitude control example

!!! success "Ported and tested"
    `ros2 run xtdrone_py attitude_control_demo`. Test: `scripts/tests/run_chapter_tests.sh ch07` drives the
    interactive demo through a pseudo terminal (`t`, then `b`) in both modes and checks that the vehicle reaches
    and holds the target (1, 0, 2).

## About the example

1. All other XTDrone multirotor examples use position, velocity or acceleration setpoints. This one uses attitude setpoints.
2. Attitude commands go to `/<type>_0/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`). The demo
   supports two modes: (1) attitude + thrust and (2) body rate + thrust. The `type_mask` selects the mode.
3. The controller is a PID: position → attitude → body rate / thrust. Its gains are not tuned for every vehicle.
4. The yaw reference starts at 0.
5. The tilt is limited to 0.3 rad (`angle_max` in the code).

The original demo assumed a hover throttle of 0.55, which suited the Classic iris. The Gazebo Harmonic iris uses
PX4's x500 airframe, which needs a normalised thrust of **0.73** to hover; that is the default here. The altitude
loop has no integral term, so a wrong hover throttle leaves a height error (0.6 → about 0.25 m too low). Pass
another value as the optional third argument for other vehicles. `XTDRONE_DEBUG=1` prints the controller state
once per second.

## Run

Because the manoeuvres are aggressive, use GPS-based estimation (the default `ekf:=gps`).

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
```

The example combines communication and control in one node:

```bash
ros2 run xtdrone_py attitude_control_demo iris angular_speed      # or: angle
ros2 run xtdrone_py attitude_control_demo iris angle 0.73          # with hover throttle
```

## Keys

| Key | Action |
|---|---|
| `t` | arm and take off to (0, 0, 2) in OFFBOARD position mode |
| `b` | start attitude control towards the target point |
| `4`/`1`, `5`/`2`, `6`/`3` | increase/decrease target x, y, z by 0.2 m |
| `7`/`8` | increase/decrease target yaw by 0.05 rad |
| `r` | return to the takeoff point |
| `h` | hover (AUTO.LOITER) |
| `l` | land (AUTO.LAND) |
| `a`/`d` | arm/disarm |
| `p` | print the help |
| `q` | quit |

Changes from the ROS 1 script:

- the OFFBOARD mode is requested only when the vehicle is not already in OFFBOARD (the original called the
  service in every loop);
- `h` uses AUTO.LOITER, because PX4 has no mode called HOVER.
