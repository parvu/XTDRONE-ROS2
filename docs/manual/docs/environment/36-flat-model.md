# 36. Make a flat (image) Gazebo model

!!! success "Ported and tested"
    `ros2/xtdrone_gz/tools/make_gazebo_model/script/make_gazebo_model.py` textures a COLLADA panel. It now also copies the
    image next to the mesh, where Gazebo Harmonic looks for COLLADA textures. The output directory changes from
    `~/.gazebo/models` to a directory on `GZ_SIM_RESOURCE_PATH`. The white border (`-w`) needs ImageMagick
    (`convert`, installed by the setup script). Test: `scripts/tests/run_chapter_tests.sh ch36`.

Flat panels with QR codes, AprilTags or landing marks are common targets in UAV research. This chapter makes
such a panel model from a PNG image, and generates AprilTag arrays.

## Choose a model directory

Gazebo Harmonic finds models through `GZ_SIM_RESOURCE_PATH`. Use one directory for your own models and add it
in every terminal, or in `~/.bashrc` after `xtdrone_env.sh`:

```bash
mkdir -p ~/xtdrone_models
export GZ_SIM_RESOURCE_PATH=~/xtdrone_models:$GZ_SIM_RESOURCE_PATH
```

`xtdrone.launch.py` keeps the directories already in `GZ_SIM_RESOURCE_PATH`.

## make_gazebo_model

```bash
cd ~/XTDRONE-ROS2/ros2/xtdrone_gz/tools/make_gazebo_model/script
python3 make_gazebo_model.py -i IMAGE_DIRECTORY -g ~/xtdrone_models -s SIZE_IN_MILLIMETER -w CONTOUR_SIZE_IN_MM
```

- `-i`: directory with the PNG images;
- `-g`: model directory (the original default was `~/.gazebo/models`);
- `-s`, `-w`: panel size and white border width.

The script creates one model per image, named like the image file. Use it in a world with
`<include><uri>model://<name></uri></include>`, or spawn it into a running simulation:

```bash
ros2 run ros_gz_sim create -world indoor1 -file ~/xtdrone_models/landing1/model.sdf -name pad -x 2
```

## AprilTag arrays

The AprilTag-based precision landing (chapter 28) needs the exact size and spacing of the tags. Generate a
3 × 1 array:

```bash
cd ~/XTDRONE-ROS2/sensing/object_detection_and_tracking/apriltag/script
python3 make_tag.py --nx 3 --ny 1 --marginx 0.00 --marginy 0.00 --tsize 0.36 --tspace 0.5 --startid 0 --tfam t36h11 --borderbits 1
```

The script writes `target.pdf`. Convert it to PNG, e.g. `pdftoppm -png -r 300 target.pdf apriltag0-2`. Then make
the model with `make_gazebo_model.py -w 0`, because the border is already part of the image. If you change
the tag size or spacing, update the `apriltag_ros` configuration of chapter 28 too.

## Textured panels without COLLADA

Gazebo Classic models often used OGRE material scripts (`materials/scripts/*.material`), which Gazebo Harmonic
ignores. Use a PBR albedo map instead:

```xml
<visual name="visual">
  <geometry><box><size>1 1 0.01</size></box></geometry>
  <material>
    <diffuse>1 1 1 1</diffuse>
    <pbr><metal><albedo_map>model://H/materials/textures/h.png</albedo_map></metal></pbr>
  </material>
</visual>
```

`ros2/xtdrone_gz/scripts/classic_to_gz.py` converts existing Classic models this way automatically.
