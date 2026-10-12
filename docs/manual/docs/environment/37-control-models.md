# 37. Control the movement of environment models

!!! success "Ported and tested"
    `ros2 run xtdrone_py control_gazebo_models`. Test: `scripts/tests/run_chapter_tests.sh ch37`.

Some tasks need moving obstacles or targets. Gazebo Classic offered the ROS topic `/gazebo/set_model_states`.
Gazebo Harmonic offers the service `/world/<world>/set_pose_vector` of the UserCommands system, which every
converted XTDrone world loads. `control_gazebo_models` calls it through the gz-transport Python bindings.

## Example

The two cars `rover_static` and `rover_static_clone` in `outdoor1` move along +x at 0.5 m/s:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor1 vehicles:=iris
ros2 run xtdrone_py control_gazebo_models              # world detected automatically
ros2 run xtdrone_py control_gazebo_models --world outdoor1
```

The original chapter first made the cars non-static and disabled their gravity, because Classic would not move
static models. In Gazebo Harmonic, `set_pose` moves static models too, so the world stays as it is.

## Your own motions

Copy `ros2/xtdrone_py/xtdrone_py/gazebo_models.py`. `PoseSetter.set([(name, x, y, z), ...])` places models.
`loop()` shows a simple kinematic update at 20 Hz. The same call from the command line:

```bash
gz service -s /world/outdoor1/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 1000 --req 'name: "rover_static", position: {x: 75, y: 1, z: 0}'
```

!!! warning "Do not teleport PX4 vehicles"
    Moving a vehicle that PX4 controls corrupts its state estimate. Chapter 43 explains how to teleport
    vehicles that run without PX4.
