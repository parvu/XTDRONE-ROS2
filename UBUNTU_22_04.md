# Ubuntu 22.04 / ROS 2 Humble support

Ubuntu 22.04 is supported by ROS 2 Humble. XTDrone is being migrated incrementally: the ROS 2 multi-UAV formation demo is packaged and buildable with Humble, while the main PX4/Gazebo launch stack and the remaining ROS 1 packages have not yet been ported. This guide sets up and verifies that ROS 2 package; it does not claim the legacy simulator is ROS 2 compatible.

## Prerequisites

- Ubuntu 22.04 LTS
- A user with `sudo` access
- Internet access

## Install

From the repository root, run:

```bash
./scripts/setup_ubuntu_22_04.sh
```

The script installs ROS 2 Humble, colcon, rosdep, and the message packages used by the migrated formation demo. It does not clone PX4 or modify the legacy ROS 1 dependencies.

## Build

Create a separate ROS 2 workspace so the existing ROS 1 packages are not picked up by colcon:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/xtdrone_ros2_ws/src
ln -s /path/to/XTDrone/ros2/fomation_demo \
  ~/xtdrone_ros2_ws/src/xtdrone_formation_demo
cd ~/xtdrone_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Replace `/path/to/XTDrone` with the absolute path to this checkout. The ROS 2 package lives under the historical directory name `ros2/fomation_demo`; its ROS package name is `xtdrone_formation_demo`.

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

Set `XTDRONE_WS` if the workspace is somewhere other than `~/xtdrone_ros2_ws`.
