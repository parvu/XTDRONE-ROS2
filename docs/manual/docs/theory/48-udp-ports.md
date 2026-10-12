# 48. UDP port matching

!!! success "Rewritten for PX4 v1.16"
    The XTDrone port scheme of the original generator is kept through the `px4-rc.mavlink` overlay.

## Links of one PX4 SITL instance N

PX4 v1.16 starts several MAVLink instances (`px4-rc.mavlink`) per SITL instance:

| Link | PX4 local port | Remote port | Used by |
|---|---|---|---|
| GCS | 18570+N | 14550 | QGroundControl (broadcast) |
| Offboard (onboard computer) | **34580+N** (XTDrone) | **24540+N** (XTDrone) | MAVROS 2 of vehicle N |
| Onboard payload | 14280+N | 14030+N | payload/camera protocols |
| Gimbal | 13030+N | 13280+N | gimbal protocol |

PX4's stock script uses 14580+N / 14540+N for the offboard link, and gives every instance N > 9 the same remote
port 14549. MAVROS instances cannot share a port, so stock PX4 limits MAVROS to 10 vehicles. XTDrone's generator
already used 34580+N / 24540+N. The overlay `ros2/xtdrone_gz/px4/init.d-posix/px4-rc.mavlink` (PX4's v1.16.2 file
with only these two lines changed) restores that scheme for any number of vehicles. The launch file puts its
directory first on PX4's `PATH`, and `rcS` sources `px4-rc.mavlink` by name.

There is no TCP port to the simulator any more (Classic `4560+N`): `gz_bridge` talks to Gazebo over gz-transport.

## MAVROS side

For vehicle N, the launch file sets:

```yaml
fcu_url: udp://:24540+N@127.0.0.1:34580+N     # listen on 24540+N, send to PX4 at 34580+N
tgt_system: N+1                               # MAV_SYS_ID of instance N
tgt_component: 1
```

and runs MAVROS in the namespace `/<type>_<id>` (topics under `/<type>_<id>/mavros/...`).

## Check the ports

```bash
ss -ulpn | grep -E 'px4|mavros'                   # bound UDP ports per process
ros2 topic echo --once /iris_3/mavros/state       # connected: true = ports match for vehicle 3
```

When you add vehicles by hand (chapter 10), the `instance` argument fixes all ports. Give every running vehicle
its own instance number.
