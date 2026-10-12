# 9. Multi-vehicle hybrid simulation

!!! success "Ported and tested"
    The launch-file generator is replaced by the `vehicles:=` argument and scenario YAML files. Test (6 × iris):
    `scripts/tests/run_chapter_tests.sh ch09`.

Multi-vehicle simulation is CPU heavy. A good laptop runs about 10 vehicles without cameras. Each camera
or lidar adds rendering load.

## Describe the fleet

The original `generator.py` asked for the vehicle types, counts and rows, and wrote `multi_vehicle.launch`.
Now you pass the fleet directly:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=empty vehicles:="iris*6,typhoon_h480*3,plane*2"
```

- `MODEL*COUNT` adds `COUNT` vehicles of an XTDrone model (chapter 8 lists the models). Without `*COUNT`, one vehicle is added.
- Vehicles are named `<type>_<id>` and numbered per type: `iris_0` … `iris_5`, `typhoon_h480_0` … `typhoon_h480_2`, `plane_0`, `plane_1`.
- Like `generator.py`, each type gets its own block of columns, at most 3 vehicles per row, 3 m apart.
- The global PX4 instance N follows the order in the spec. It sets the MAVLink ports (chapter 48) and the system ID (N+1).

For exact poses, write a **scenario** file. The ROS 1 launch files are already converted into scenarios in
`ros2/xtdrone_bringup/config/scenarios/`:

```yaml
# my_fleet.yaml
world: outdoor2
vehicles:
  - {model: iris_downward_camera, type: iris, id: 0, pose: [-3, 0, 0, 0, 0, 0]}
  - {model: iris_downward_camera, type: iris, id: 1, pose: [-4.5, 0, 0, 0, 0, 0]}
  - {model: typhoon_h480, type: typhoon_h480, id: 0, pose: [3, 3, 0, 0, 0, 1.57]}
```

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=$PWD/my_fleet.yaml
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor2_precision_landing   # built-in scenario
```

`pose` is x, y, z, roll, pitch, yaw in the world frame.

Each vehicle gets its own PX4 instance, MAVROS node and sensor bridge. The launch starts the vehicles 0.5 s apart.
Wait until all of them show `connected: true`:

```bash
ros2 topic echo --once /iris_5/mavros/state
```

## Communication for many vehicles

`multi_vehicle_communication.sh` becomes a launch file with one argument per type:

```bash
ros2 launch xtdrone_py multi_vehicle_communication.launch.py iris_num:=6 typhoon_h480_num:=3 plane_num:=2
# with position hold for the multirotors:
ros2 launch xtdrone_py multi_vehicle_communication.launch.py iris_num:=6 enhanced:=true
```

Arguments: `iris_num`, `typhoon_h480_num`, `solo_num`, `standard_vtol_num`, `plane_num`, `rover_num`, `enhanced`.
`Ctrl+C` stops all of them, which replaces `stop_communication.sh`.

You can also start the communication nodes with the simulation: `communication:=basic` or `communication:=enhanced` in `xtdrone.launch.py`.

## Keyboard control of many vehicles

```bash
ros2 run xtdrone_py multirotor_keyboard_control iris 6 vel
```

The keyboard controls all six iris at once. With 6, 9 or 18 vehicles, the digit keys send the formation
commands of chapter 24. Real multi-vehicle missions need your own control scripts. The communication
topics are the interface: `/xtdrone/<type>_<id>/cmd`, `cmd_vel_flu`, `cmd_pose_enu`, and so on.
