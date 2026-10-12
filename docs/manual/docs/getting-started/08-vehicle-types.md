# 8. Configuration and control of different UAVs

!!! success "Ported and tested"
    Multirotors (iris, solo, typhoon_h480), plane and the VTOLs (standard_vtol, tailsitter, tiltrotor) run on
    PX4 v1.16 Gazebo Harmonic airframes. PX4's tailsitter is a *quad* tailsitter, unlike XTDrone's Classic duo
    tailsitter.
    Tests: `scripts/tests/run_chapter_tests.sh ch08plane ch08vtol ch08rover`.

## Vehicles

| XTDrone type | XTDrone models (`vehicles:=`) | Gazebo Harmonic base | PX4 airframe (`SYS_AUTOSTART`) |
|---|---|---|---|
| `iris` | `iris`, `iris_stereo_camera`, `iris_realsense_camera`, `iris_downward_camera`, `iris_2d_lidar`, `iris_3d_lidar`, `iris_3d_gpu_lidar` | PX4 `x500` | 4001 `gz_x500` |
| `solo` | `solo`, `solo_stereo_camera` | PX4 `x500` | 4001 `gz_x500` |
| `typhoon_h480` | `typhoon_h480` (hexacopter, CGO3 gimbal camera) | converted from the XTDrone Classic model | 4901 `gz_typhoon_h480` (XTDrone) |
| `plane` | `plane` | PX4 `rc_cessna` | 4003 `gz_rc_cessna` |
| `standard_vtol` | `standard_vtol` | PX4 `standard_vtol` | 4004 `gz_standard_vtol` |
| `rover` | `rover` (`rover_with_lidar_stereo` is an alias) | PX4 `r1_rover` | 4009 `gz_r1_rover` |
| `tailsitter` | `tailsitter` | PX4 `quadtailsitter` | 4018 `gz_quadtailsitter` |
| `tiltrotor` | `tiltrotor` | PX4 `tiltrotor` | 4020 `gz_tiltrotor` |

`iris` and `solo` keep their XTDrone names, but they fly with PX4's x500 frame and parameters: a larger and
heavier quadrotor than the Classic iris. The XTDrone sensors are mounted on it (chapter 12).

!!! note "Ground vehicles"
    Like the original, the manual recommends the UGV of chapter 11 over the PX4 rover for multi-vehicle work.
    PX4 v1.16 ships the R1 rover (Ackermann/differential, `r1_rover`) for Gazebo Harmonic.

## Choosing vehicles

In the original, you edited the vehicle names and SDF files in `outdoor3.launch`. Here you pass them as an argument:

```bash
# one solo in outdoor3; the vehicle is called solo_0
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor3 vehicles:=solo
```

The spec format is `MODEL[*COUNT],MODEL[*COUNT],...`. The vehicle type is the model name up to the first
sensor suffix (`iris_stereo_camera` → `iris`), and vehicles are numbered per type from 0. To set exact poses,
write a scenario file (chapter 9).

The spawn height comes from the model, so you no longer need the Classic `z=1` correction for solo.

## Flight modes, arming

XTDrone mostly uses the PX4 [Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard.html) mode: the companion
program sends setpoints and PX4 tracks them. It also uses Takeoff, Land, Return, Hold (Loiter) and Mission.
Arm before takeoff and disarm after landing.

## Multirotor

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor3 vehicles:=solo
ros2 run xtdrone_py multirotor_communication solo 0
ros2 run xtdrone_py multirotor_keyboard_control solo 1 vel
```

`vel` selects velocity control and `accel` acceleration control. Velocity control is easier by hand, while
acceleration control suits motion planners. As in chapter 2: press `i` until vz > 0.3 m/s, then `b`
(OFFBOARD) and `t` (arm). When the vehicle is high enough, press `s` to hover.

Fly the hexacopter the same way:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor3 vehicles:=typhoon_h480
ros2 run xtdrone_py multirotor_communication typhoon_h480 0
ros2 run xtdrone_py multirotor_keyboard_control typhoon_h480 1 vel
```

## Fixed-wing

A fixed-wing needs space, so use `outdoor2`, and use GPS (`ekf:=gps`, the default).

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:=plane
ros2 run xtdrone_py plane_communication 0
ros2 run xtdrone_py plane_keyboard_control 1
```

- There is no subtype and no velocity/acceleration choice: the keyboard sets a position setpoint (north, east, up).
- Take off in **Takeoff** mode.
- In Offboard, the plane flies to the setpoint and then circles it.

Recommended sequence: `v` (AUTO.TAKEOFF) → `t` (arm) → set a setpoint with `w`/`x`/`a`/`d`/`i`/`,` and send it with `o` →
`s` (loiter) → `b` (OFFBOARD). Keep the setpoint height above 20 m. Press `s` before switching to Offboard;
with `k` (idle) the plane glides without power.

To land, use Land, Mission or Return mode (see [PX4 fixed-wing landing](https://docs.px4.io/v1.16/en/flight_modes_fw/land.html)).

## VTOL (quadplane)

A VTOL has two flight modes, multirotor and fixed-wing, which makes it the hardest vehicle to operate.

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:=standard_vtol
ros2 run xtdrone_py vtol_communication standard_vtol 0
```

The communication node starts in multirotor mode:

```text
standard_vtol_0: multirotor
standard_vtol_0: transition to multirotor
standard_vtol_0: communication initialized
```

```bash
ros2 run xtdrone_py vtol_keyboard_control standard_vtol 1 vel
```

In multirotor mode, fly it like a multirotor (`vel` = velocity control). Before you press `o` to switch to
fixed-wing mode, send a suitable setpoint. Right after the transition, send `s` (loiter): the VTOL flies to
the setpoint in fixed-wing mode and circles it.

## Rover

The rover takes a forward speed and a steering command:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor3 vehicles:=rover
ros2 run xtdrone_py rover_communication 0
ros2 run xtdrone_py rover_keyboard_control 1
```

`w`/`x` change the speed, `a`/`d` the steering, then `b` (OFFBOARD) and `t` (arm).

PX4 v1.16 drives the R1 rover with its differential-drive controller. An Offboard velocity setpoint becomes a
speed and a bearing in the local frame. In `outdoor3` the ground is the Sonoma raceway mesh, so the rover is
spawned 0.3 m above it (chapter 2).
