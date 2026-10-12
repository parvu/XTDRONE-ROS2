# 6. PX4 EKF configuration

!!! success "Ported and tested"
    Selected with the launch argument `ekf:=gps|vision`. Test: `scripts/tests/run_chapter_tests.sh ch06` (vision).

By default, PX4 estimates horizontal position from GPS and height from GPS and the barometer. Visual
positioning (SLAM, VIO, motion capture, or the ground truth of chapter 15) needs a different EKF2
configuration. Whatever the source, check that `/<type>_<id>/mavros/local_position/pose` is correct before you
fly autonomously.

## Parameter names in PX4 v1.16

PX4 1.14 replaced the bit masks of the original chapter:

| PX4 1.11 / 1.13 | PX4 v1.16 | Meaning |
|---|---|---|
| `EKF2_AID_MASK` bit 0 (GPS) | `EKF2_GPS_CTRL` (7 = lon/lat, altitude, 3D velocity; 0 = off) | GPS fusion |
| `EKF2_AID_MASK` bits 3, 4 (vision position, yaw) | `EKF2_EV_CTRL` (bits: 1 horizontal position, 2 vertical position, 4 velocity, 8 yaw) | External vision fusion |
| `EKF2_HGT_MODE` (0 baro, 3 vision) | `EKF2_HGT_REF` (0 baro, 1 GPS, 2 range, 3 vision) | Height reference |
| — | `EKF2_BARO_CTRL` (0/1) | Barometer fusion |
| — | `EKF2_MAG_TYPE` (5 = none) | Magnetometer; off when vision provides yaw |

## Select a preset at launch

`xtdrone.launch.py` sets the parameters of every vehicle in the launch from a preset. It always sets
`NAV_DLL_ACT=0 COM_RCL_EXCEPT=4` first, so vehicles arm and fly Offboard without a ground station or RC
(chapter 29):

| `ekf:=` | Parameters | Use |
|---|---|---|
| `gps` (default) | `EKF2_GPS_CTRL=7 EKF2_EV_CTRL=0 EKF2_HGT_REF=1 EKF2_BARO_CTRL=1 EKF2_MAG_TYPE=0` | GPS + barometer, outdoor and default |
| `vision` | `EKF2_GPS_CTRL=0 EKF2_EV_CTRL=15 EKF2_HGT_REF=3 EKF2_BARO_CTRL=0 EKF2_MAG_TYPE=5` | GPS denied; vision position, height and yaw (the original `EKF2_AID_MASK 24` + `EKF2_HGT_MODE 3`) |

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 ekf:=vision
```

With `ekf:=vision`, PX4 needs a pose on `/iris_0/mavros/vision_pose/pose`. It comes from your SLAM/VIO system
or from the ground truth (`ros2 run xtdrone_py get_local_pose iris 1`, chapter 15). Otherwise
`local_position` stays invalid and the vehicle cannot arm in OFFBOARD.

To set other parameters at startup, use `px4_params`:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 ekf:=vision px4_params:="EKF2_EV_DELAY=20 EKF2_EVP_NOISE=0.05"
```

### How it works

PX4's startup script `rcS` sources `px4-rc.simulator` after the airframe defaults and before the estimator
starts. The launch file puts `xtdrone_gz/px4/init.d-posix` first on the `PATH` of each PX4 process. The
XTDrone version of `px4-rc.simulator` found there applies the `NAME=value` pairs in the environment variable
`XTDRONE_PX4_PARAMS` and then runs PX4's own `px4-rc.simulator`. You no longer edit `rcS` in the build folder or delete `~/.ros/eeprom` as in the original:
the launch sets the parameters on every start, and they overwrite stored values.

## Change parameters at runtime

MAVROS 2 exposes all PX4 parameters as ROS parameters of the node `/<type>_<id>/mavros/param`:

```bash
ros2 param get /iris_0/mavros/param EKF2_EV_CTRL
ros2 param set /iris_0/mavros/param EKF2_EV_DELAY 20
```

EKF2 fusion settings take effect after a PX4 restart. Use the launch argument for those. QGroundControl
also works: *Vehicle Setup → Parameters*, with the vehicle selected at the top.
