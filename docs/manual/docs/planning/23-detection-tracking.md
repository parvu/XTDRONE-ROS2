# 23. Target detection and tracking

!!! success "Ported and tested"
    Package `xtdrone_py` (`yolo.py`): `gimbal_control`, `yolo_detection` and `yolo_human_tracking`, the ports of
    `gimbal_control.py`, `darknet_ros` and `yolo_human_tracking.py`. Test: `scripts/tests/run_chapter_tests.sh
    ch23`: a `typhoon_h480` in `outdoor1` hovers at 8 m with its gimbal camera 60° down, YOLO finds the walking
    pedestrian, and the visual servo follows it (about 36 m along the street).

## Run

```bash
# 1. outdoor1 with a typhoon_h480 (it starts at (2, 2), beside the pedestrian's path)
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1 communication:=basic
# 2. gimbal 60 deg down, and YOLO on the gimbal camera
ros2 run xtdrone_py gimbal_control typhoon_h480 0 --pitch -60
ros2 run xtdrone_py yolo_detection typhoon_h480 0
# 3. take off to about 8 m with the keyboard and hover, then start the tracking
ros2 run xtdrone_py multirotor_keyboard_control typhoon_h480 1 vel
ros2 run xtdrone_py yolo_human_tracking typhoon_h480 0
```

- **`gimbal_control`** commands the gimbal joints (`/typhoon_h480_0/gimbal/pitch_cmd`, `roll_cmd`, `yaw_cmd`)
  and publishes the camera orientation (body attitude × gimbal pitch) on `/xtdrone/typhoon_h480_0/cam_pose`.
- **`yolo_detection`** runs YOLOv4-tiny (COCO classes, the Darknet network family of `darknet_ros`) with OpenCV's
  DNN module on the CPU: no darknet build and no GPU. The weights are downloaded to `~/.cache/xtdrone/yolo` on
  first use (`--cfg`, `--weights`, `--names` select others). Detections go to `/typhoon_h480_0/detections`
  (`vision_msgs/Detection2DArray`), and an annotated image to `/typhoon_h480_0/detections/image`.
- **`yolo_human_tracking`** steers towards the first `person` detection with the image-based visual servo of
  the ROS 1 script, through `/xtdrone/typhoon_h480_0/cmd_vel_flu`.

The pedestrian of `outdoor1` walks a 240 s loop (simulation time): along y = 3 from x = 0 to 41 in the first 40 s,
then far from the start. From the start point it is in view at 0–40 s and again from 240 s.

!!! note "Changes from the ROS 1 version"
    - `darknet_ros` (YOLOv3 on darknet) → YOLOv4-tiny with OpenCV DNN, so no CUDA build is needed. The network
      input is 960 × 544 (`--size`): the gimbal camera is 640 × 360 with a 115° field of view, a pedestrian
      10 m away is about 45 px tall, and at the usual 416 × 416 YOLOv4-tiny finds nothing (score 0.95 at
      960 × 544, about 70 ms per image).
    - The gimbal joints are commanded directly instead of MAVROS mount control. The cgo3 camera joint turns about
      −y, so "60° down" is the joint angle −1.05 rad (its lower limit).
    - The typhoon starts at (2, 2) instead of the origin, which is inside a parked utility vehicle in the converted
      world.
    - In the converted `outdoor1` the pedestrian and the terrain needed fixes in the world converter: the
      `walk.dae` skin is found in the Classic media, its waypoints go up 1 m (the mesh origin is at the hips), and
      the `city_terrain` heightmap is drawn at the heights of Classic. Before that the terrain lay over the
      roads and the pedestrian.
    - Training on a custom data set (the second part of the ROS 1 chapter) is the Darknet procedure of the
      original manual; the `.cfg`/`.weights` it produces load with `--cfg`/`--weights`.
