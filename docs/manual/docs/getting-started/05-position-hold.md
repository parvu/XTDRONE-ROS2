# 5. Single-direction position hold

!!! success "Ported and tested"
    `ros2 run xtdrone_py multirotor_communication_enhanced`. Test: `scripts/tests/run_chapter_tests.sh ch05`.

The basic communication node forwards the commanded velocity to MAVROS and adds a hover function. The
**enhanced** node also holds position along single axes. If the commanded velocity along an axis is 0, it
holds the position along that axis:

- vx, vy ≠ 0 and vz = 0: the vehicle holds its altitude;
- vx ≠ 0 and vy = vz = 0: the vehicle flies along a straight line;
- vx = vy = vz = 0: the vehicle hovers.

The hold works in both the FLU (`cmd_vel_flu`) and the ENU (`cmd_vel_enu`) frame. It is a proportional controller
on the position error, so tune the gains for your vehicle in
`ros2/xtdrone_py/xtdrone_py/multirotor_communication.py` (class `EnhancedMultirotorCommunication`):

```python
self.hold_kp_x = self.hold_kp_y = self.hold_kp_z = self.hold_kp_yaw = 1.0
```

## Use it

```bash
# terminal 1
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
# terminal 2: instead of multirotor_communication
ros2 run xtdrone_py multirotor_communication_enhanced iris 0
# terminal 3
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

Or start the enhanced node with the simulation:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 communication:=enhanced
```

Fly a pattern with the keyboard, with and without the enhanced node. With position hold, the vehicle follows
the commanded path more closely: it does not drift along the axes you are not commanding.
