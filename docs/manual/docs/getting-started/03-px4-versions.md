# 3. Basic configuration (PX4 1.13 notes)

!!! success "Merged into chapter 2"
    The original manual has one installation chapter for PX4 1.11 and one for PX4 1.13. This manual uses
    PX4 v1.16.2 only, installed as described in [chapter 2](02-basic-configuration.md). This page lists what
    changed for readers who know the 1.11 or 1.13 setup.

| Topic | PX4 1.11 / 1.13 with Gazebo Classic | PX4 v1.16.2 with Gazebo Harmonic |
|---|---|---|
| PX4 location | `~/PX4_Firmware` (separate clone) | `PX4-Autopilot` submodule of this repository |
| Build target | `make px4_sitl_default gazebo` | `make px4_sitl_default` (run by `scripts/build_xtdrone_ros2.sh`) |
| Simulator interface | `sitl_gazebo` plugins (`libgazebo_mavlink_interface.so`) | PX4 module `gz_bridge` (gz-transport) |
| XTDrone launch, world and model files | copied into the PX4 tree | stay in `ros2/xtdrone_*` packages, found through `GZ_SIM_RESOURCE_PATH` |
| Startup script edits (`rcS`) | edit the PX4 `rcS` | PATH overlays `px4-rc.mavlink` and `px4-rc.simulator` in `xtdrone_gz/px4`, PX4 tree untouched |
| `~/.bashrc` setup | `source Tools/setup_gazebo.bash`, `ROS_PACKAGE_PATH` | `source scripts/xtdrone_env.sh` |
| Launch | `roslaunch px4 indoor1.launch` | `ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1` |
| Parameter storage | `~/.ros/eeprom/parameters*`, `~/.ros/sitl*` | `~/.ros/xtdrone_px4/<instance>/parameters.bson` |
| Mixer files | `ROMFS/.../mixers` | Control allocation parameters (`CA_*`) in the airframe file |
| EKF parameters | `EKF2_AID_MASK`, `EKF2_HGT_MODE` | `EKF2_GPS_CTRL`, `EKF2_EV_CTRL`, `EKF2_HGT_REF`, `EKF2_BARO_CTRL` (chapter 6) |

The original 1.13 chapter fixed two problems: the simulation slowed down over time, and mixed fleets
lagged. PX4 v1.16 with Gazebo Harmonic runs in lockstep with the simulator clock, so neither applies.

To reset a vehicle's stored PX4 parameters, delete its instance directory:

```bash
rm -rf ~/.ros/xtdrone_px4/0     # instance 0, i.e. the first vehicle
```
