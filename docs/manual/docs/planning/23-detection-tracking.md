# 23. Target detection and tracking

!!! warning "Planned: phase 5"
    This chapter depends on components that are not converted yet.
    **ROS 2 / Gazebo Harmonic approach:** A YOLO ROS 2 node (e.g. `yolo_ros` with Ultralytics) instead of `darknet_ros`, and rclpy ports of `gimbal_control.py` and `yolo_human_tracking.py`.

## What the original chapter does

A `typhoon_h480` uses its gimbal camera to detect people with YOLO (darknet_ros) and tracks them. The chapter also covers training on a custom data set.

See the [original chapter](https://www.yuque.com/xtdrone/manual_en) for the ROS 1 instructions.

## What already works on ROS 2 and Gazebo Harmonic

- `typhoon_h480` publishes `/typhoon_h480_0/cgo3_camera/image_raw` and `camera_info`.
- The gimbal is commanded with `std_msgs/Float64` angles (rad) on `/typhoon_h480_0/gimbal/yaw_cmd`, `roll_cmd` and `pitch_cmd`.
- Actors (people) walk in the converted `outdoor1`, `outdoor4` and `robocup` worlds (chapter 38).

The [migration plan](../reference/status.md) lists the phases.
