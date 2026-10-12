# 43. Accelerated simulation, teleported vehicles and large swarms

!!! success "Ported and tested"
    `speed_factor:=` and `px4:=false` arguments of `xtdrone.launch.py`, the `empty_zero_g` world and
    `ros2 run xtdrone_py control_gazebo_vehicles`. Test: `scripts/tests/run_chapter_tests.sh ch43`
    (`speed_factor:=2` reaches a real time factor of 2.0 with one iris on the 6-core test machine).

This chapter collects ideas for training learning-based methods, where faster simulation matters.

## Run faster than real time

PX4 SITL runs in lockstep with the simulator, so the simulation can run faster than real time if the CPU keeps up:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 speed_factor:=4
```

This replaces `PX4_SIM_SPEED_FACTOR=6 roslaunch px4 indoor1.launch`. The launch does two things:

1. it sets the target real-time factor of the world (gz service `/world/<world>/set_physics`). PX4 v1.16 does this
   itself only when PX4 spawns the model; XTDrone spawns the models, so the launch does it;
2. it passes `PX4_SIM_SPEED_FACTOR` to every PX4 instance, so that PX4 stretches its failsafe timeouts (RC, offboard,
   data link) by the same factor.

The real time factor shown in the Gazebo GUI tells you what the machine reaches. Sensors with rendering
(cameras, lidars) are usually the limit.

## Move vehicles by pose ("teleporting")

Gazebo can place any model at any pose (chapter 37). With PX4 in the loop, a sudden jump breaks the state
estimate, so teleport only vehicles that run **without PX4**. With your own kinematic model, Gazebo also
skips the vehicle dynamics, which allows much larger swarms.

```bash
# 100 iris models, no PX4/MAVROS, in a world without gravity
ros2 launch xtdrone_bringup xtdrone.launch.py world:=empty_zero_g vehicles:="iris*100" px4:=false gui:=true
# move them on a 10 x 10 grid along +x at 0.5 m/s
ros2 run xtdrone_py control_gazebo_vehicles iris 10 10
```

- `px4:=false` spawns the models and their sensor bridges, without PX4 and MAVROS.
- `empty_zero_g` is the `empty` world with `<gravity>0 0 0</gravity>`, so the vehicles do not fall between
  pose updates. The original chapter edited the gravity and stopped ODE with `stop_gazebo_ode.sh`.
- `control_gazebo_vehicles` updates all poses at 20 Hz with one `set_pose_vector` call. Write your own kinematic
  or dynamic model in the same way (`ros2/xtdrone_py/xtdrone_py/gazebo_models.py`).

Spawning hundreds of models takes a while, because the launch staggers them by 0.5 s each. Rendering many
camera sensors is expensive, so use `vehicles:="iris*N"` (no cameras).

## Many PX4 vehicles on one machine

Every PX4 vehicle runs PX4, MAVROS and its Gazebo physics. What a machine manages depends mostly on the number of
cores. On the 6-core test machine without a GPU, 9 iris run at about 0.6–0.75 of real time and 20 iris at about
0.3. The launch keeps the per-vehicle cost low:

- **`mavlink_profile:=swarm`** (automatic above 6 vehicles): PX4 sends only what the XTDrone nodes use at full rate.
  For example, ATTITUDE (100 Hz) is off, IMU drops to 10 Hz and global position to 5 Hz; local position stays at
  30 Hz. MAVROS CPU use for 9 vehicles fell from 168 % to 88 %.
- **`mavros_plugins:=swarm`** (with the swarm profile): only the MAVROS plugins the XTDrone nodes need, so fewer DDS
  endpoints have to be discovered.
- **No MAVROS `param` plugin** (unless `mavros_plugins:=all`): it downloads all ~900 PX4 parameters per vehicle at
  connection. With 9 vehicles at once, that kept MAVROS above 400 % CPU.
- **One lazy bridge** for the whole fleet instead of one process per vehicle, converting only subscribed topics.
- **PX4's uXRCE-DDS client is off** (`uxrce_dds:=false`); XTDrone talks to PX4 through MAVROS.

Run `scripts/tune_network_for_swarms.sh` once (sudo). It reserves PX4's MAVLink ports 34580–34779, which lie inside
the kernel's ephemeral port range, so that no random socket takes one first. It also enlarges the socket buffers;
with the 208 KB default, DDS discovery packets get dropped when 10 or more vehicles start, and some topics then
never connect. `scripts/verify_ubuntu_24_04.sh` reports whether it is set.

When the machine cannot keep real time, the nodes that run on wall-clock time see the simulation slowed down.
Examples are MAVROS heartbeats, the 30 Hz XTDrone controllers and the vision pose of `get_local_pose`. To keep a
margin, run below real time on purpose, for example `speed_factor:=0.5` for 20 vehicles.
