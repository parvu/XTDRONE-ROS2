# 24. Multi-UAV formation

!!! warning "Planned: phase 3"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** rclpy ports of `coordination/formation_demo` (`leader.py`, `follower.py`, `follower_consensus.py`, `avoid.py`). The ROS 2 formation demo of chapter 25 already covers the leader/follower part.

## What the original chapter does

Six, nine or eighteen iris fly in formation. The keyboard steers the leader, and the digit keys switch formations (cube, pyramid, triangle, ...). The followers use the ground-truth positions so that all vehicles share one coordinate frame.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- 9 iris: `ros2 launch xtdrone_bringup xtdrone.launch.py world:=empty vehicles:="iris*9" ekf:=vision communication:=basic`.
- Ground truth for all of them: `ros2 run xtdrone_py get_local_pose iris 9`.
- Keyboard with formation keys: `ros2 run xtdrone_py multirotor_keyboard_control iris 9 vel`.

The [migration plan](../reference/status.md) lists the phases.
