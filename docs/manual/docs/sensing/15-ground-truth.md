# 15. Use the ground-truth pose

!!! success "Ported and tested"
    `ros2 run xtdrone_py get_local_pose`. Tests: `scripts/tests/run_chapter_tests.sh ch15 ch06`.

Gazebo knows the exact pose of every vehicle, much like a motion-capture system in a real lab. The ground truth
has two uses:

- reliable global positioning for tasks such as swarm formation;
- a reference to evaluate the accuracy of SLAM algorithms.

## Where it comes from

Gazebo Classic published all model states on `/gazebo/model_states`. Each XTDrone vehicle model for Gazebo Harmonic
contains an `OdometryPublisher` system. The launch file bridges it to

```text
/<type>_<id>/ground_truth/odom      nav_msgs/msg/Odometry, 50 Hz, world frame (ENU)
```

## Feed it to PX4

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 communication:=basic ekf:=vision
ros2 run xtdrone_py get_local_pose iris 1        # "1" = one iris
```

`get_local_pose` publishes, at 30 Hz:

- `/iris_0/mavros/vision_pose/pose` (`geometry_msgs/PoseStamped`);
- `/iris_0/mavros/vision_speed/speed` (`geometry_msgs/Vector3Stamped`, world-frame velocity).

To make PX4 fuse them, select `ekf:=vision` (chapter 6).

!!! warning "Only one source of vision pose"
    `get_local_pose` publishes `/mavros/vision_pose/pose`. If you use the ground truth to evaluate a SLAM system that
    also publishes `vision_pose`, do not run both. Subscribe to `/<type>_<id>/ground_truth/odom` directly instead.

## Visualise

```bash
ros2 topic echo /iris_0/ground_truth/odom --once         # exact pose in the world frame
ros2 topic hz /iris_0/ground_truth/odom                  # 50 Hz
ros2 topic echo /iris_0/mavros/local_position/pose --once   # what PX4 estimates; with ekf:=vision it follows the ground truth
```

In PlotJuggler (`ros2 run plotjuggler plotjuggler`), plot the x, y and z of both topics against time. In RViz, a
`Pose` display on `/iris_0/mavros/vision_pose/pose` shows the pose that PX4 receives.
