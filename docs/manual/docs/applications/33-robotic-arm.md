# 33. Aerial robotic arm operation and gripping

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** MoveIt 2 and `gz_ros2_control` for the `le_arm` mounted on the iris, with rclcpp ports of the grasping and vision nodes.

## What the original chapter does

An iris carries a small robotic arm (`le_arm`). MoveIt plans the arm motion and a vision node detects the object to grab.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- The `grasping` world is converted. The arm model and its controllers are not.

The [migration plan](../reference/status.md) lists the phases.
