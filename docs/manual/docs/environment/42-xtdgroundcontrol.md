# 42. XTDGroundControl

!!! success "Ported and tested"
    Package `xtdrone_groundcontrol`: the PyQt5 ground station of `control/XTDGroundControl/python`, ported to
    Python 3 and rclpy. Test: `scripts/tests/run_chapter_tests.sh ch42` drives the window offscreen. It sets two
    iris, then Run, Control, offboard, up 1 m/s and arm; both climb above 2 m, the state panel shows their poses,
    and they land.

XTDGroundControl is a ground station for many vehicles of different types. It replaces the keyboard for complex
tasks: it selects vehicles, sends commands, and shows their states and tracks on a map of the world.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=indoor1 vehicles:="iris*3" communication:=basic
ros2 run xtdrone_groundcontrol xtdgroundcontrol
```

1. Enter the number of vehicles of each type, choose the map and `vel` or `accel`, and press **Run**.
2. Press **Control**, then tick the vehicle types to command.
3. Choose a command: `offboard`, `arm`, `take off`, `hover`, `land`, `return home` or `disarm`. Set the
   forward/left/up/yaw speeds in the boxes; they are sent like the keyboard's `cmd_vel_flu`.
4. `control the leader` sends to `/xtdrone/leader/...` for the formation demos (chapters 24 and 25), and
   `control all` sends to every ticked vehicle.
5. **Stop** sends `AUTO.RTL` to all vehicles.

The right panel shows the ground-truth position and the body velocity of each vehicle when there are fewer than
10, and the map shows their tracks.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/xtdrone/<type>_<i>/cmd`, `cmd_vel_flu` (`cmd_accel_flu`) | String, Twist | GUI → communication nodes |
| `/xtdrone/leader/cmd`, `/xtdrone/leader/cmd_vel_flu` | String, Twist | GUI → formation leader |
| `/ugv_<i>/cmd`, `/ugv_<i>/cmd_vel` | String, Twist | GUI → UGVs (chapter 11) |
| `/<type>_<i>/ground_truth/odom` | Odometry | Gazebo → GUI (positions) |
| `/<type>_<i>/mavros/local_position/velocity_body` | TwistStamped | MAVROS → GUI |

!!! note "Changes from the ROS 1 version"
    - Python 3: the `print` statements are fixed. The sender loop that ran in a `multiprocessing.Process` is a
      thread with its own rclpy node, fed by the same queues (`queue.Queue`). The receiver `QThread` spins its own
      executor.
    - Positions come from the ground-truth odometry bridged by `xtdrone.launch.py` (`/<type>_<i>/ground_truth/odom`,
      formerly `/xtdrone/<type>_<i>/ground_truth/odom`).
    - The `quadplane` row commands the vehicle type `standard_vtol` (the PX4 v1.16 name).
    - matplotlib is forced to the Qt5Agg backend. On Ubuntu 24.04 the default TkAgg backend cannot run next to the
      Qt window.
    - The C++ variant (`control/XTDGroundControl/cplusplus`, `coordination/formation_communication_demo/xtdrone_qt`)
      is not ported; the Python version covers the same functions.
