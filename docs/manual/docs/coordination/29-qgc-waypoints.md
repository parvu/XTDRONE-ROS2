# 29. Set waypoints for multiple UAVs with QGroundControl

!!! success "Ported and tested"
    No XTDrone code is involved. PX4 v1.16 already gives every SITL instance its own mission storage, so the
    `rcS` change of the original chapter is no longer needed. Test: `scripts/tests/run_chapter_tests.sh ch29`: three
    iris get different missions over MAVLink (MAVROS `mission/push`, as QGroundControl uploads them), read them back
    unchanged, and fly them in AUTO.MISSION, each to its own waypoint (within 2 m).

## The original problem

With PX4 1.11–1.13, all SITL instances shared one `dataman` file. Missions uploaded to different vehicles
overwrote each other, and every vehicle flew the last mission. XTDrone patched `rcS` to use
`dataman start -f dataman$((px4_instance+1))`.

In this setup, every PX4 instance runs in its own working directory, `~/.ros/xtdrone_px4/<instance>/`,
which holds its own dataman storage. Nothing needs to change.

## Use it

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:="iris*3"
./QGroundControl-x86_64.AppImage
```

QGroundControl listens on UDP 14550 and finds every vehicle: each PX4 instance N sends its GCS link from
port 18570+N. Select a vehicle at the top of the *Plan* view, draw its mission and upload it. Then select the
next vehicle. Start each mission from the *Fly* view.

## Data-link loss

PX4 v1.16 airframes set `NAV_DLL_ACT=2`: on a lost ground-station link the vehicle returns, and without a
ground station it refuses to arm. XTDrone vehicles are usually flown by companion programs through MAVROS,
so `xtdrone.launch.py` sets `NAV_DLL_ACT=0` (and `COM_RCL_EXCEPT=4`, Offboard without RC) for every vehicle.
To get PX4's behaviour back when you fly with QGroundControl:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:="iris*3" px4_params:="NAV_DLL_ACT=2"
```
