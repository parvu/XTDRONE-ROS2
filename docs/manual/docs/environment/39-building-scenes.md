# 39. Building experiment scenes

!!! info "Ported (new workflow)"
    Gazebo Harmonic has no Building Editor. Build scenes in another tool, or keep using the Gazebo Classic
    Building Editor and convert the result with `classic_to_gz.py`.

## Option 1: Classic Building Editor + converter

If Gazebo Classic is installed somewhere (another machine, a container), draw the building as described in the
original chapter (*Edit → Building Editor*), save the model and the world, then convert:

```bash
python3 ~/XTDRONE-ROS2/ros2/xtdrone_gz/scripts/classic_to_gz.py \
  --worlds my_scene.world --model-path ~/.gazebo/models ~/.cache/xtdrone/gazebo_models \
  --classic-media ~/.cache/xtdrone/gazebo-classic/media --out ~/xtdrone_scenes
export GZ_SIM_RESOURCE_PATH=~/xtdrone_scenes/models:$GZ_SIM_RESOURCE_PATH
ros2 launch xtdrone_bringup xtdrone.launch.py world:=$HOME/xtdrone_scenes/worlds/my_scene.sdf vehicles:=iris
```

The converter sets the physics PX4 needs (`max_step_size` 0.004, `real_time_update_rate` 250), so the manual
edit of the original is not needed.

## Option 2: meshes from Blender, SketchUp or Sweet Home 3D

Export the building as COLLADA (`.dae`), glTF (`.glb`) or OBJ. Wrap it in a model whose visual and collision
use the mesh:

```xml
<model name="my_building">
  <static>true</static>
  <link name="link">
    <visual name="visual"><geometry><mesh><uri>model://my_building/meshes/building.glb</uri></mesh></geometry></visual>
    <collision name="collision"><geometry><mesh><uri>model://my_building/meshes/building.glb</uri></mesh></geometry></collision>
  </link>
</model>
```

## Option 3: Gazebo Fuel

The [Fuel](https://app.gazebosim.org/fuel/models) model library has many buildings and props. Add them to a
running world with the GUI's *Resource Spawner*, then save the world (*World → Save world as*).

## Add a ceiling

To keep UAVs indoors, add an invisible ceiling: a box with a collision and no visual. This works unchanged in
Gazebo Harmonic, and all converted `indoor*` worlds have one:

```xml
<model name="ceiling">
  <static>true</static>
  <pose>0 0 5 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>300 300 0.5</size></box></geometry></collision>
  </link>
</model>
```
