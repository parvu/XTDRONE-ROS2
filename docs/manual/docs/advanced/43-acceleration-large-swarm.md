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
