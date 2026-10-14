# 3. PX4 v1.16.2

!!! success "Merged into chapter 2"
    This manual uses PX4 v1.16.2 only, installed as described in [chapter 2](02-basic-configuration.md). This page
    summarises where its pieces are in this repository.

| Topic | PX4 v1.16.2 with Gazebo Harmonic |
|---|---|
| PX4 location | `PX4-Autopilot` submodule of this repository |
| Build target | `make px4_sitl_default` (run by `scripts/build_xtdrone_ros2.sh`) |
| Simulator interface | PX4 module `gz_bridge` (gz-transport) |
| XTDrone world and model files | `ros2/xtdrone_*` packages, found through `GZ_SIM_RESOURCE_PATH` |
| Startup script changes | PATH overlays `px4-rc.mavlink` and `px4-rc.simulator` in `xtdrone_gz/px4`; the PX4 tree is untouched |
| Environment | `source scripts/xtdrone_env.sh` |
| Launch | `ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1` |
| Parameters | `px4_params:=NAME=value,...` at launch; working directory `~/.ros/xtdrone_px4/<instance>/` |
| Actuator outputs | control allocation parameters (`CA_*`) in the airframe file |
| EKF configuration | `EKF2_GPS_CTRL`, `EKF2_EV_CTRL`, `EKF2_HGT_REF`, `EKF2_BARO_CTRL` (chapter 6) |

PX4 runs in lockstep with the simulator clock, so a slow simulation slows PX4 down with it instead of making it
lag behind.

PX4 saves the parameters it is given in its working directory and would load them again on the next start.
`xtdrone.launch.py` deletes the saved parameters before each start, so every launch begins with the airframe
defaults plus the `px4_params` of that launch.
