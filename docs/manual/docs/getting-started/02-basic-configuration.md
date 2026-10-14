# 2. Basic configuration of the simulation platform

!!! success "Ported and tested"
    Replaces the original chapters 2 and 3. Test: `scripts/tests/run_chapter_tests.sh ch03`.

## Notes before you start

1. Chapters 2 to 8 are the basic tutorials. Read them before the others.
2. Many steps are involved. Do them in order and check the output of each one.
3. You need basic ROS 2 knowledge: nodes, topics, services, `ros2 launch`, `ros2 run`, colcon workspaces.
   The [ROS 2 Jazzy tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) are a good start.
4. Use a native Ubuntu 24.04 installation if you can. Chapter 4 explains Docker.

## What gets installed

| Component | Version | Installed by |
|---|---|---|
| ROS 2 | Jazzy (desktop) | `scripts/setup_ubuntu_24_04.sh` |
| Gazebo | Harmonic (`gz-harmonic`) | same |
| MAVROS | MAVROS 2 + extras + GeographicLib datasets | same |
| ros_gz | bridge, sim, image, interfaces | same |
| PX4 build dependencies | compilers, Python modules, GStreamer | same |
| PX4-Autopilot | v1.16.2, `PX4-Autopilot` submodule | `git submodule update` |
| Gazebo Classic model library | `osrf/gazebo_models` (converted) | `scripts/build_xtdrone_ros2.sh` |

## 1. Get the source code

```bash
git clone https://github.com/<your-fork>/XTDRONE-ROS2.git ~/XTDRONE-ROS2
cd ~/XTDRONE-ROS2
git submodule update --init --recursive PX4-Autopilot
```

The submodule is PX4 **v1.16.2** at commit `54f0455ffcd755534539a7cf33a09a20bf71d29d`. The launch files check
this commit. PX4 1.16 uses Gazebo Harmonic as its simulator and supports Ubuntu 24.04. Unlike the original manual, you do **not** copy launch files, worlds or models into
the PX4 tree: XTDrone keeps its files in its own ROS 2 packages and points Gazebo and PX4 at them.

## 2. Install the dependencies

```bash
cd ~/XTDRONE-ROS2
./scripts/setup_ubuntu_24_04.sh
```

The script asks for your sudo password and:

- installs PX4's Python requirements from Ubuntu packages where available (so numpy and friends stay the
  versions ROS 2 Jazzy is built against);
- runs PX4's own `Tools/setup/ubuntu.sh --no-nuttx`, which adds the Gazebo apt source
  (packages.osrfoundation.org), installs `gz-harmonic` and the SITL build dependencies, and pip-installs the
  remaining PX4 Python modules (`nunavut`, `pymavlink`, `pyros-genmsg`, `pyulog`) for your user;
- adds the ROS 2 apt source (`ros2-apt-source` package);
- installs `ros-jazzy-desktop`, `ros-jazzy-ros-gz`, `ros-jazzy-mavros`, `ros-jazzy-mavros-extras`
  and `ros-dev-tools`;
- downloads the GeographicLib datasets MAVROS needs;
- runs `scripts/tune_network_for_swarms.sh`, which reserves the PX4 MAVLink ports and enlarges the socket buffers
  for DDS (`/etc/sysctl.d/60-xtdrone.conf`; matters for 10+ vehicles, chapter 43);
- initialises rosdep.

The chapters on SLAM, navigation, precision landing with `apriltag_ros`, UGVs and the heavier ports need more
ROS 2 packages. Install them when you get there:

```bash
./scripts/setup_ubuntu_24_04_extras.sh          # slam_toolbox, Cartographer, Nav2, RTAB-Map, apriltag_ros, ros2_controllers, ...
./scripts/setup_ubuntu_24_04_extras.sh --all    # plus MoveIt 2 and the build dependencies of ORB-SLAM3, VINS-Fusion, ...
```

!!! warning "Python virtual environments and other ROS distributions"
    PX4 and colcon must use the system Python (`/usr/bin/python3`). If your login shell activates a virtualenv
    (for example an ArduPilot venv from `~/.profile`), or sources another ROS distribution in `~/.bashrc`, the
    XTDrone scripts strip those from their environment. Use
    `source scripts/xtdrone_env.sh` in your own terminals (see step 4).

## 3. Build

```bash
./scripts/build_xtdrone_ros2.sh
```

The first run takes 10 to 30 minutes. It:

1. builds PX4 SITL: `make px4_sitl_default` in `PX4-Autopilot`. The build includes `gz_bridge`, PX4's Gazebo
   Harmonic interface, and fails if Gazebo Harmonic is missing;
2. copies the XTDrone PX4 airframes (for example the hexacopter `4901_gz_typhoon_h480`) into the PX4 build;
3. downloads the model library the original worlds use (`osrf/gazebo_models`) and its media (textures, actor
   meshes) to `~/.cache/xtdrone`;
4. converts all XTDrone worlds (`ros2/xtdrone_gz/classic_assets/worlds`) and the models they use to Gazebo Harmonic
   (`ros2/xtdrone_gz/generated`);
5. regenerates the XTDrone vehicle models (`iris`, `iris_stereo_camera`, ..., `typhoon_h480`);
6. creates the colcon workspace `~/xtdrone_ros2_ws` with links to `ros2/xtdrone_gz`, `ros2/xtdrone_bringup`,
   `ros2/xtdrone_py` and the other XTDrone ROS 2 packages, and runs `colcon build --symlink-install`.

Set `XTDRONE_WS` to use another workspace directory.

Then check the installation:

```bash
./scripts/verify_ubuntu_24_04.sh
```

Every line must report `ok`.

## 4. Set up each terminal

```bash
source ~/XTDRONE-ROS2/scripts/xtdrone_env.sh
```

This leaves any Python venv, removes a ROS 1 environment, sources `/opt/ros/jazzy/setup.bash` and the
XTDrone workspace, and sets `XTDRONE_ROOT`. You can add the line to `~/.bashrc` if you do not use ROS 1 or
other virtualenvs in the same shells.

## 5. Start the simulation

The original `roslaunch px4 indoor1.launch` becomes:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
```

`scenario:=indoor1` reproduces the original `indoor1.launch`: world `indoor1` and one `iris_stereo_camera`
named `iris_0` at (0, 7.5, 1). The launch file starts:

- the Gazebo Harmonic server with the converted world, and the GUI a few seconds later, once the vehicles
  are spawned (a GUI that connects while vehicles are being created can miss them);
- PX4 SITL instance 0, attached to the Gazebo model `iris_0`;
- MAVROS 2 in the namespace `/iris_0/mavros`, connected to `udp://:24540@127.0.0.1:34580`;
- a `ros_gz_bridge` that publishes the stereo images, the IMU and the ground truth under `/iris_0/...`;
- a `/clock` bridge.

Every vehicle also gets `NAV_DLL_ACT=0 COM_RCL_EXCEPT=4`, so it arms and flies Offboard without a ground
station or RC (chapter 29). Vehicles requested at z ≤ 0 spawn 0.3 m above the ground and drop onto it.

Check that MAVROS is connected:

```bash
ros2 topic echo --once /iris_0/mavros/state
```

```text
header: ...
connected: true
armed: false
guided: false
manual_input: true
mode: AUTO.LOITER
system_status: 3
```

If `connected` stays `false`, see [Troubleshooting](../reference/troubleshooting.md).

Useful launch arguments, all optional (list them with `ros2 launch xtdrone_bringup xtdrone.launch.py --show-args`):

| Argument | Default | Meaning |
|---|---|---|
| `scenario` | — | Scenario YAML from `xtdrone_bringup/config/scenarios` (one per original launch file) |
| `world` | `indoor1` | World, if no scenario is given |
| `vehicles` | `iris_stereo_camera` | Fleet, if no scenario is given, e.g. `"iris*3,plane"` |
| `gui` | `true` | Start the Gazebo GUI |
| `communication` | `none` | Also start the XTDrone communication node per vehicle: `basic` or `enhanced` |
| `ekf` | `gps` | PX4 EKF2 preset: `gps` or `vision` (chapter 6) |
| `mavros_plugins` | `xtdrone` | Load only the MAVROS plugins XTDrone uses; `all` loads MAVROS' full PX4 set |

## 6. Fly with the keyboard

Start the communication node, which turns XTDrone commands into MAVROS setpoints. Wait until Gazebo shows
the vehicle.

```bash
ros2 run xtdrone_py multirotor_communication iris 0
```

```text
iris_0: communication initialized
```

You can instead start it with the simulation: `ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 communication:=basic`.

In another terminal, start the keyboard control (`1` is the number of vehicles, `vel` is velocity control):

```bash
ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
```

Take off the way the original manual recommends:

1. press `i` until the upward velocity is above 0.3 m/s;
2. press `b` to switch to OFFBOARD;
3. press `t` to arm. The vehicle climbs.

Press `s` or `k` to hover, `w`/`x`/`a`/`d` to move, `j`/`l` to yaw, `n` to land and `y` to disarm. The help
text lists all keys. Press `Ctrl+C` to quit.

!!! note "Topics carry the vehicle name"
    MAVROS topics and services are prefixed with the vehicle name, e.g. `/iris_0/mavros/state` and
    `/iris_0/mavros/cmd/arming`. XTDrone commands go to `/xtdrone/iris_0/cmd`.

## 7. QGroundControl

QGroundControl connects automatically on UDP 14550. Each PX4 instance sends its GCS stream from port 18570+N.
Download the AppImage from the [QGroundControl site](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

## Simulator basics

- **Pause and step**: use the play/pause and step buttons in the Gazebo GUI, bottom left.
- **Real time factor (RTF)**: the bottom-right panel shows the sim time, real time and RTF. An RTF below 1 means
  the simulation runs slower than real time. Camera and lidar sensors are the most expensive part, and
  Gazebo Harmonic renders them with OGRE 2. On a machine without a GPU, use vehicles without cameras
  (`vehicles:=iris`) where you can.
- **Headless**: `gui:=false` runs only the server, which is useful for tests and on remote machines.
- **Camera streams cost the most**: images are rendered and converted only while some node subscribes to
  them. Without a GPU, two subscribed 752×480 streams can slow the whole simulation down noticeably.
  Close image viewers you do not need.
- **Large worlds load slowly**: `indoor4` takes several minutes to build its collision shapes on a CPU-only
  machine (about 8 minutes on the 6-core test host), `indoor5` about 2.5 minutes. The others load in seconds.
- **Restart cleanly**: `Ctrl+C` in the launch terminal stops everything. If a Gazebo or PX4 process is left over,
  `pkill -f "gz sim"; pkill -x px4` cleans up. This replaces the original `killall gzserver gzclient`.
- **Do not drag vehicles with the mouse**: moving a vehicle in the GUI corrupts PX4's state estimate. Chapter 43
  covers teleporting vehicles.
- **Indoor worlds** have an invisible ceiling with collision, so vehicles cannot fly out.
- **Lighting**: worlds converted from Classic keep their lights. Gazebo Harmonic renders them with physically
  based materials, so some scenes look brighter or darker than in Classic.
