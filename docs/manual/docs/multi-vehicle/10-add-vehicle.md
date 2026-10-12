# 10. Add a UAV to a running simulation

!!! success "Ported and tested"
    `ros2 launch xtdrone_bringup spawn_vehicle.launch.py`. Test: `scripts/tests/run_chapter_tests.sh ch10`.

Some scenarios need new vehicles while the simulation runs. A vehicle consists of the Gazebo model, its PX4
instance, its MAVROS node and its sensor bridge. `spawn_vehicle.launch.py` starts all four for one vehicle in
the running world.

The ROS 1 example added `iris_1` (an `iris_realsense_camera`) at (3, 0, 0.5). The ROS 2 equivalent:

```bash
ros2 launch xtdrone_bringup spawn_vehicle.launch.py model:=iris_realsense_camera id:=1 instance:=1 x:=3 y:=0 z:=0.5
```

| Argument | Meaning |
|---|---|
| `model` | XTDrone model (chapter 8) |
| `id` | the N in `<type>_N`; must be unused |
| `instance` | global PX4 instance; must not be used by a running vehicle. It sets the MAVLink ports 24540+instance / 34580+instance and the system ID instance+1 |
| `x y z R P Y` | spawn pose |
| `world` | world name; detected from the running Gazebo if empty |
| `communication` | `none`, `basic` or `enhanced` |
| `ekf`, `px4_params` | as in chapter 6 |

In the ROS 1 version you had to choose consistent ports by hand (`fcu_url`, `mavlink_udp_port`,
`mavlink_tcp_port`). Here they follow from `instance`. Stopping the launch (`Ctrl+C`) stops the PX4, MAVROS
and bridge processes of that vehicle. The Gazebo model stays in the world: remove it in the GUI's entity
tree, or with `gz service -s /world/<world>/remove ...`.
