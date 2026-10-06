# Ubuntu 22.04 / ROS 2 Humble support

Ubuntu 22.04 is supported by ROS 2 Humble. XTDrone is being migrated incrementally. The formation demo and a ROS 2 launch entrypoint for PX4 SITL with the repository-pinned PX4 and Gazebo Garden are available. The existing XTDrone Gazebo Classic models, ROS 1 Gazebo plugins, and multi-vehicle launch scenarios have not yet been ported.

## Prerequisites

- Ubuntu 22.04 LTS
- A user with `sudo` access
- Internet access
- The repository's pinned PX4-Autopilot v1.15.4 submodule
- Gazebo Garden and PX4's Gazebo simulation dependencies
- The Micro XRCE-DDS Agent executable (`MicroXRCEAgent`)

## Install

From the repository root, initialize the pinned PX4 checkout and its dependencies:

```bash
git submodule update --init --recursive PX4-Autopilot
```

This checks out PX4 v1.15.4 at commit `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`. Then install ROS 2 dependencies:

```bash
./scripts/setup_ubuntu_22_04.sh
```

The script installs ROS 2 Humble, colcon, rosdep, and the message packages used by the migrated formation demo. Gazebo Garden and the Micro XRCE-DDS Agent are separate prerequisites. PX4 v1.15.4's `Tools/setup/ubuntu.sh` installs Gazebo Garden on Ubuntu 22.04.

## Build

Create a separate ROS 2 workspace so the existing ROS 1 packages are not picked up by colcon:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/xtdrone_ros2_ws/src
ln -s /path/to/XTDrone/ros2/fomation_demo \
  ~/xtdrone_ros2_ws/src/xtdrone_formation_demo
ln -s /path/to/XTDrone/ros2/px4_gz_simulation \
  ~/xtdrone_ros2_ws/src/xtdrone_px4_gz
cd ~/xtdrone_ros2_ws
colcon build
source install/setup.bash
```

Replace `/path/to/XTDrone` with the absolute path to this checkout. The ROS 2 package lives under the historical directory name `ros2/fomation_demo`; its ROS package name is `xtdrone_formation_demo`.

## Start PX4 SITL with Gazebo Garden

Use the pinned PX4 v1.15.4 checkout and Gazebo Garden. The launch file verifies the PX4 commit and rejects other versions. It locates `PX4-Autopilot` in this repository automatically when launched from the repository or one of its subdirectories:

```bash
source /opt/ros/humble/setup.bash
source ~/xtdrone_ros2_ws/install/setup.bash
ros2 launch xtdrone_px4_gz px4_gz.launch.py vehicle:=x500
```

The launch file starts the Micro XRCE-DDS Agent on UDP port 8888 and runs PX4's `make px4_sitl gz_x500` target from this repository's `PX4-Autopilot` submodule. Set `XTDRONE_ROOT` to this repository root or `PX4_DIR` to its `PX4-Autopilot` directory if launching from elsewhere. Set `start_agent:=false` if the agent is already running; set `agent_port:=<port>` to use another port. PX4 and Gazebo output appears in the launch terminal. Stop all launched processes with `Ctrl+C`.

PX4 v1.15.4 can also detect Gazebo Harmonic, but its Ubuntu installer selects Garden and this setup uses Garden. The PX4 Gazebo models/worlds are in the nested `PX4-Autopilot/Tools/simulation/gz` submodule; initialize it along with PX4 using the recursive submodule command above.

This is currently a single-vehicle PX4/Gazebo Garden bring-up path, not a port of XTDrone's custom models, ROS 1 sensor plugins, or multi-vehicle scenarios. The existing formation demo still consumes MAVROS-compatible topics and therefore does not yet control this PX4 DDS setup directly.

## Run the formation demo

The launch file starts one leader and the requested number of followers. Supported total vehicle counts are 6, 9, and 18:

```bash
ros2 launch xtdrone_formation_demo formation.launch.py uav_type:=iris uav_count:=6
```

Or use the repository helper:

```bash
ros2/fomation_demo/run_follower.sh iris 6
```

The nodes consume the existing XTDrone formation topics and MAVROS local-pose topics using ROS 2 message types. A ROS 2 simulator/bridge that publishes those topics must be running separately; the ROS 1 PX4/Gazebo launch files do not satisfy that requirement.

Stop the demo with `Ctrl+C` in the terminal running `ros2 launch`.

## Verify

After building the workspace, run:

```bash
./scripts/verify_ubuntu_22_04.sh
```

Set `XTDRONE_WS` if the workspace is somewhere other than `~/xtdrone_ros2_ws`. To check the simulator launch arguments, run `ros2 launch xtdrone_px4_gz px4_gz.launch.py --show-args`.
