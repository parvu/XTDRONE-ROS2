#!/usr/bin/env bash
# Optional packages for the XTDrone phase 4 and 5 chapters (SLAM, navigation, perception,
# planning, manipulation). Run after setup_ubuntu_24_04.sh, as your normal user; sudo is
# used for apt.
#   ./scripts/setup_ubuntu_24_04_extras.sh            # phase 4 (ROS 2 binary packages)
#   ./scripts/setup_ubuntu_24_04_extras.sh --all      # phase 4 + build deps of the phase 5 ports
set -euo pipefail

ROS_DISTRO=jazzy
ALL=false
case "${1:-}" in
  --all) ALL=true ;;
  "") ;;
  *) echo "usage: $0 [--all]" >&2; exit 2 ;;
esac

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this script as your normal user; it calls sudo where needed." >&2
  exit 1
fi
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} is not installed. Run scripts/setup_ubuntu_24_04.sh first." >&2
  exit 1
fi

# Phase 4: chapters 11, 16-18, 20, 28, 30, 32, 38.
PHASE4=(
  ros-${ROS_DISTRO}-slam-toolbox          # ch 17 (replaces hector_slam)
  ros-${ROS_DISTRO}-cartographer-ros      # ch 18
  ros-${ROS_DISTRO}-navigation2           # ch 20, 31 (replaces move_base)
  ros-${ROS_DISTRO}-nav2-bringup
  ros-${ROS_DISTRO}-rtabmap-ros           # ch 16 dense reconstruction
  ros-${ROS_DISTRO}-apriltag-ros          # ch 28 precision landing
  ros-${ROS_DISTRO}-apriltag-msgs
  ros-${ROS_DISTRO}-robot-localization
  ros-${ROS_DISTRO}-pcl-ros
  ros-${ROS_DISTRO}-pointcloud-to-laserscan
  ros-${ROS_DISTRO}-octomap-server
  ros-${ROS_DISTRO}-image-pipeline
  ros-${ROS_DISTRO}-ros2-controllers      # ch 11 UGV (diff drive / Ackermann)
  ros-${ROS_DISTRO}-gz-ros2-control
  ros-${ROS_DISTRO}-ackermann-msgs
  ros-${ROS_DISTRO}-tf-transformations
  python3-transforms3d
)

# Phase 5: build dependencies of the source ports (ORB-SLAM3, VINS-Fusion, A-LOAM/KISS-ICP,
# EGO-Planner, MoveIt 2 arm, fixed-wing formation).
PHASE5=(
  ros-${ROS_DISTRO}-moveit                # ch 33 aerial manipulator
  ros-${ROS_DISTRO}-vision-opencv
  libceres-dev libgoogle-glog-dev libgflags-dev libeigen3-dev libsuitesparse-dev
  libpcl-dev libboost-all-dev
  libglew-dev libepoxy-dev libwayland-dev libxkbcommon-dev wayland-protocols  # Pangolin (ORB-SLAM3)
  libarmadillo-dev
  python3-pip python3-venv
)

packages=("${PHASE4[@]}")
if [[ "${ALL}" == true ]]; then
  packages+=("${PHASE5[@]}")
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends "${packages[@]}"

printf '\nExtras installed. Rebuild the workspace: ./scripts/build_xtdrone_ros2.sh\n'
