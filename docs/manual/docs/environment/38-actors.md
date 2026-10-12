# 38. Actors (pedestrians)

!!! success "Ported and tested"
    Scripted actors work in the converted worlds (`outdoor1`, `outdoor4`, `robocup`). The ROS-controlled actors of
    `robocup` use the new Gazebo system `xtdrone::ActorMotion` (package `xtdrone_gz_plugins`, the port of the Classic
    `ros_actor_cmd_pose_plugin`) and `ros2 run xtdrone_py control_actor`. Test: `scripts/tests/run_chapter_tests.sh
    ch38`: two actors walk to commanded points (one of them 57 m) and arrive within 0.3 m.

## Actors in Gazebo Harmonic

An `<actor>` with a `<skin>` (animated mesh) and a `<script>` (waypoint trajectory) works the same way in
Gazebo Harmonic as in Classic. The world converter keeps the trajectories and makes the Classic media meshes
(`walk.dae`, ...) available as `model://xtdrone_classic_media/models/...`.

```xml
<actor name="walker">
  <skin><filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename></skin>
  <animation name="walking">
    <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>true</loop>
    <trajectory id="0" type="walking">
      <waypoint><time>0</time><pose>0 0 1.0 0 0 0</pose></waypoint>
      <waypoint><time>10</time><pose>10 0 1.0 0 0 0</pose></waypoint>
    </trajectory>
  </script>
</actor>
```

## Actor collisions

Classic actors have no collisions, so XTDrone used `libActorCollisionsPlugin.so` to make lidars see them.
Gazebo Harmonic renders lidars on the GPU (`gpu_lidar`), and rendered sensors see actor meshes without any
collision plugin. The converter therefore drops the plugin. Physics contacts with actors are still not modelled.

## ROS-controlled actors

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=robocup        # six actors at their start positions
ros2 run xtdrone_py control_actor 0 -34 34 5                           # actor_0 walks to (-34, 34) at 5 m/s
ros2 launch xtdrone_py control_actors.launch.py                        # all six, as control_actors.sh
```

Each actor with the `xtdrone::ActorMotion` system walks, with its walking animation, to the last commanded point at
the commanded speed and stops there. The world converter adds the system to the actors that had the Classic ROS
actor plugin (their `init_pose` becomes the start pose).

| Topic | Type | |
|---|---|---|
| `/actor_<i>/cmd_motion` | `geometry_msgs/Vector3` | x, y, z = speed (m/s); ROS 1: `ros_actor_cmd_pose_plugin_msgs/ActorMotion` (x, y, v) |
| `/actor_<i>/pose` | `geometry_msgs/Pose` | the actor's world pose, 10 Hz (gz-sim's pose topics leave actors out) |

`xtdrone.launch.py` bridges both topics for every actor with the system. To use it in your own world:

```xml
<actor name="walker_0">
  <skin>...</skin><animation name="walking">...</animation>
  <pose>3 4 1 0 0 0</pose>
  <plugin filename="xtdrone_actor_motion" name="xtdrone::ActorMotion">
    <animation_x_vel>2.0</animation_x_vel>   <!-- optional: animation speed per m walked -->
  </plugin>
</actor>
```
