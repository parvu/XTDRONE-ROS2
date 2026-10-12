# 28. Multi-UAV precision landing

!!! success "Ported and tested"
    `ros2 run xtdrone_py precision_landing`, the port of `control/precision_landing.py`; models `apriltag0-2` and
    `apriltag3-5` rebuilt for Gazebo Harmonic. Test: `scripts/tests/run_chapter_tests.sh ch28`: three
    `iris_downward_camera` start 0.5 m off their tags at 4, 5 and 6 m and land within 2 cm of the tag centres
    (OpenCV detector).

Three iris with downward cameras find the AprilTags with the IDs 0, 1 and 2 and land on them: vehicle `iris_i` lands
on tag `i`. In the original chapter, the tags were on a UGV (catvehicle). That UGV belongs to chapter 11, which is
not ported yet, so here the tag plate lies on the `outdoor2` runway at the origin. The tags are 0.5 m tag36h11
markers, 0.75 m apart along x: tag 0 at x = −0.75, tag 1 at 0 and tag 2 at 0.75.

## Run

```bash
# 1. outdoor2 with three iris_downward_camera at x = -3, -4.5, -6 and the tag plate at the origin
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor2_precision_landing communication:=basic
# 2. keyboard: take off and fly the vehicles over the tags (each sees its tag from about 4-6 m)
ros2 run xtdrone_py multirotor_keyboard_control iris 3 vel
# 3. one landing node per vehicle (multi_precision_landing.sh in ROS 1)
ros2 run xtdrone_py precision_landing iris 0
ros2 run xtdrone_py precision_landing iris 1
ros2 run xtdrone_py precision_landing iris 2
```

As soon as a node sees its tag, it commands `cmd_vel_enu = Kp (tag − position)` horizontally and −0.5 m/s
vertically (Kp = 1, as in ROS 1), and PX4 disarms after touchdown. The camera sees the tags from up to about 6 m.
Its field of view is 120°, so the tag does not have to be directly below. When the vehicle flies at different
heights, fly the higher ones over the lower ones; the vehicles start in a row on the x axis.

The camera sees the x500 arms on the left and right edges of the image. The tags are found in the free centre.

## Detectors

| `--detector` | What it uses |
|---|---|
| `opencv` (default) | `cv2.aruco` with the `DICT_APRILTAG_36h11` dictionary plus `solvePnP` on `/<v>/camera/image_raw` and `camera_info`. Needs no extra package (Ubuntu's `python3-opencv` 4.6 and newer OpenCV both work) |
| `apriltag_ros` | the TF `<camera frame> → tag36h11:<id>` published by `ros-jazzy-apriltag-ros` (install with `scripts/setup_ubuntu_24_04_extras.sh`); start one `apriltag_node` per vehicle on `/<v>/camera/image_raw` with `size: 0.5`. Other frame names: `--tag-frame` |

Other options: `--tag-id` (default: the vehicle id) and `--tag-size` (black square edge, default 0.5 m).

!!! note "Changes from the ROS 1 version"
    ROS 1 looked up `map → tag_<id>` in one TF tree. That needed MAVROS configs with per-vehicle frame names
    (`px4_config_iris_N.yaml`, copied into the MAVROS installation) and static camera transforms. The ROS 2 node
    turns the tag position from the camera frame into the vehicle's local ENU frame itself. It uses its own pose
    and the camera mounting of `iris_downward_camera` (image top = vehicle front), so no TF setup is needed. The
    last tag estimate is kept when the tag leaves the image close to the ground, like the latest transform in
    ROS 1.

## Models

`apriltag0-2` (IDs 0, 1, 2) and `apriltag3-5` (IDs 5, 4, 3 from −x to +x) are static 2.5 m × 1.0 m plates
with collision, built from the textures of `sitl_config/models`. Add them to any scenario with `objects:`:

```yaml
objects:
  - {model: apriltag0-2, name: apriltag0_2, pose: [0, 0, 0.06, 0, 0, 0]}  # z 0.06: above the outdoor2 runway visual
```
