# 16. 3D dense reconstruction (VINS-Fusion + RTAB-Map)

!!! success "Ported and tested (odometry: ground truth until VINS-Fusion is ported)"
    `ros2 launch xtdrone_slam rtabmap_rgbd.launch.py` (RTAB-Map from `ros-jazzy-rtabmap-ros`). Test:
    `scripts/tests/run_chapter_tests.sh ch16`: an `iris_realsense_camera` turns a full circle and flies 1.5 m in
    `indoor1`; RTAB-Map builds a cloud map of more than 10 000 points.

The sparse maps of visual SLAM and VIO cannot be used for obstacle avoidance. RTAB-Map fuses an odometry with the
depth images of the RealSense camera into a dense point cloud, an OctoMap and a 2D grid.

## Run

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=indoor1 vehicles:=iris_realsense_camera communication:=basic
ros2 launch xtdrone_slam rtabmap_rgbd.launch.py rviz:=true
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel        # fly slowly through the rooms
```

Outputs: `/rtabmap/cloud_map`, `/rtabmap/octomap_full`, `/rtabmap/octomap_binary`, `/rtabmap/map`, and the database
`~/.ros/rtabmap.db` (open it with `rtabmap-databaseViewer`).

| `odom:=` | Odometry |
|---|---|
| `groundtruth` (default) | Gazebo ground truth through `odom_relay` (`odom → base_link`) |
| `visual` | RTAB-Map's RGB-D visual odometry. It loses track on the plain walls of `indoor1`; use it in textured scenes |
| `external` | any `nav_msgs/Odometry` given by `odom_topic:=` (for VINS-Fusion once chapter 14 is ported) |

The images carry the frame `depth_camera_base`. The launch publishes it as the optical frame of the camera
(z forward), 0.12 m in front of `base_link`. RTAB-Map uses exact RGB/depth synchronisation: the gz `rgbd_camera`
stamps both images identically, while approximate sync paired frames 0.05–0.1 s apart and broke the odometry in
turns.

!!! note "Slow without a GPU"
    The RGB-D camera is rendered on the CPU. With RTAB-Map the simulation runs at about 0.2–0.3 of real time on the
    6-core test machine.
