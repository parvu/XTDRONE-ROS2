# 40. Construction of large scenes

!!! info "Ported (workflow)"
    Heightmaps, sky, actors and image models all exist in Gazebo Harmonic. Some SDF details differ, as listed below.

## Terrain from a heightmap

1. Get a heightmap image, e.g. from [terrain.party](https://terrain.party/) or
   [heightmapper](https://tangrams.github.io/heightmapper/). Increase the contrast in GIMP: white is high, black is low.
2. Use the image in a heightmap model. Gazebo Harmonic needs a square image of (2^n)+1 pixels per side (e.g. 513 × 513):

```xml
<model name="terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><heightmap>
        <uri>model://terrain/materials/textures/heightmap.png</uri>
        <size>5000 5000 300</size><pos>0 0 0</pos>
      </heightmap></geometry>
    </collision>
    <visual name="visual">
      <geometry><heightmap>
        <use_terrain_paging>false</use_terrain_paging>
        <texture><diffuse>model://terrain/materials/textures/grass.png</diffuse>
                 <normal>model://terrain/materials/textures/flat_normal.png</normal><size>20</size></texture>
        <texture><diffuse>model://terrain/materials/textures/rock.png</diffuse>
                 <normal>model://terrain/materials/textures/flat_normal.png</normal><size>20</size></texture>
        <blend><min_height>100</min_height><fade_dist>20</fade_dist></blend>
        <uri>model://terrain/materials/textures/heightmap.png</uri>
        <size>5000 5000 300</size><pos>0 0 0</pos>
      </heightmap></geometry>
    </visual>
  </link>
</model>
```

The `terrain_generator` tool of the original chapter writes a Gazebo Classic model. Convert it with
`classic_to_gz.py` (chapter 39). Gazebo Harmonic has no terrain paging, so you do not need to clear `~/.gazebo/paging`.

## Sky

```xml
<scene>
  <sky></sky>
</scene>
```

Gazebo Harmonic (OGRE 2) draws a sky dome. The Classic `<clouds>` settings are ignored.

## Walking people and moving vehicles

Use actors with trajectories, as in chapter 38. For a car, use a mesh as the actor's visual instead of a skin:

```xml
<actor name="car">
  <link name="link"><visual name="visual">
    <geometry><mesh><uri>model://car_golf/meshes/golf.dae</uri></mesh></geometry>
  </visual></link>
  <script><loop>true</loop>
    <trajectory id="0" type="drive">
      <waypoint><time>0</time><pose>-1 -1 0 0 0 -1.57</pose></waypoint>
      <waypoint><time>8</time><pose>-1 3 0 0 0 -1.57</pose></waypoint>
    </trajectory>
  </script>
</actor>
```

## Image models (landing pad "H")

Gazebo Harmonic ignores OGRE material scripts. Put the PNG directly on the visual as a PBR albedo map
(chapter 36):

```xml
<model name="H">
  <static>true</static>
  <link name="link">
    <collision name="collision"><geometry><box><size>1 1 0.01</size></box></geometry></collision>
    <visual name="visual">
      <geometry><box><size>1 1 0.01</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse>
        <pbr><metal><albedo_map>model://H/materials/textures/h.png</albedo_map></metal></pbr>
      </material>
    </visual>
  </link>
</model>
```

The XTDrone landing models (`H`, `landing1`, `landing2`, `apriltag*`) are converted this way at build time.
