#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO=humble
WORKSPACE="${XTDRONE_WS:-${HOME}/xtdrone_ros2_ws}"

if [[ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  echo "ROS ${ROS_DISTRO} is not installed. Run scripts/setup_ubuntu_22_04.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if ! command -v ros2 >/dev/null; then
  echo "ROS 2 command-line tools are missing." >&2
  exit 1
fi

if ! command -v colcon >/dev/null; then
  echo "colcon is missing. Install python3-colcon-common-extensions." >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "XTDrone ROS 2 workspace is not built at ${WORKSPACE}." >&2
  echo "Build the xtdrone_formation_demo package as described in UBUNTU_22_04.md." >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
ros2 pkg prefix xtdrone_formation_demo
ros2 launch xtdrone_formation_demo formation.launch.py --show-args

printf '\nROS 2 Humble and the XTDrone formation package are available.\n'
