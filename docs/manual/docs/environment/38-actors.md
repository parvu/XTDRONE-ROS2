# 38. Actors (pedestrians)

!!! warning "Partly ported (phase 4)"
    Actors with scripted trajectories work in the converted worlds (`outdoor1`, `outdoor4`, `robocup`), and the
    ROS-controlled `robocup` actors stand at their original start positions (the converter takes them from the
    plugin's `init_pose`).
    The XTDrone ROS actor control plugin (`ros_actor_cmd_pose_plugin`, `control_actor.py`) is not ported.
    **ROS 2 approach:** move actors with the gz `set_pose` service from an rclpy port of `control_actor.py`, or with the gz
    `FollowActor` system.

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

## ROS-controlled actors (original)

The original chapter starts `robocup.launch` with six actors driven by a ROS plugin and moves them with
`control_actors.sh` (actor number, target x, target y, speed). In the converted `robocup` world, these actors
are present at their start positions but stand still until the control part is ported. Gazebo Harmonic reports
actor poses in neither `pose/info` nor `gz model`, so a ported controller also needs another way to check them.
