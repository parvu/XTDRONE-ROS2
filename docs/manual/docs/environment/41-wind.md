# 41. Wind

!!! success "Ported and tested"
    Every converted world loads the gz `WindEffects` system and has a `<wind>` element.
    `ros2 run xtdrone_py wind_control` changes the wind at runtime from ROS 2, replacing XTDrone's
    `libgazebo_ros_wind_plugin_xtdrone.so`. Test: `scripts/tests/run_chapter_tests.sh ch41`: with a 5 m/s wind
    along +x, a hovering iris leans 3.5° into the wind to hold its position (0° without wind).

## Wind in Gazebo Harmonic

Gazebo Classic used PX4's `libgazebo_wind_plugin.so`, with a mean, a variance and gusts. Gazebo Harmonic has:

- a world-level `<wind><linear_velocity>x y z</linear_velocity></wind>`, in m/s in the world (ENU) frame;
- the rotor model (`MulticopterMotorModel`) uses the world wind for the rotor drag, so multirotors feel it without
  further settings;
- the `WindEffects` system, which also applies a force to links with `<enable_wind>true</enable_wind>`,
  proportional to the link mass and the relative velocity. Its parameters can add noise and height dependence:

```xml
<plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
  <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
  <horizontal>
    <magnitude><time_for_rise>10</time_for_rise>
      <sin><amplitude_percent>0.05</amplitude_percent><period>60</period></sin>
      <noise type="gaussian"><mean>0</mean><stddev>0.0002</stddev></noise>
    </magnitude>
    <direction><time_for_rise>30</time_for_rise>
      <sin><amplitude>5</amplitude><period>20</period></sin>
      <noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise>
    </direction>
  </horizontal>
</plugin>
<wind><linear_velocity>0 1 0</linear_velocity></wind>
```

The converted XTDrone worlds load `WindEffects` with default settings. Copy the block above into a world for gusts.

## Change the wind from ROS 2

```bash
ros2 run xtdrone_py wind_control                 # world detected automatically
ros2 topic pub --once /wind_xtdrone geometry_msgs/msg/Twist "{linear: {x: 3.0, y: 0.0, z: 0.0}}"
```

As in the original, the linear part of the `Twist` on `/wind_xtdrone` is the wind velocity, and the angular part is
unused. The node publishes it as `gz.msgs.Wind` on `/world/<world>/wind`. Check it in Gazebo with
`gz topic -e -t /world/<world>/wind`.
