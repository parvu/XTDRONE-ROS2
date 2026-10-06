#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO=humble

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

require_command apt-get

if [[ ! -f /etc/os-release ]]; then
  echo "This setup script is intended for Ubuntu." >&2
  exit 1
fi

# shellcheck disable=SC1091
. /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  echo "This setup script requires Ubuntu 22.04." >&2
  exit 1
fi
export UBUNTU_CODENAME

if [[ -n "${SUDO}" ]]; then
  echo "This script will ask sudo to install system packages."
fi

export DEBIAN_FRONTEND=noninteractive
${SUDO} apt-get update
${SUDO} apt-get install -y --no-install-recommends \
  curl \
  gnupg \
  lsb-release \
  python3-numpy \
  python3-scipy \
  python3-rosdep \
  python3-colcon-common-extensions

${SUDO} install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | ${SUDO} gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg
${SUDO} chmod a+r /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null
${SUDO} apt-get update
${SUDO} apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-rclpy \
  ros-${ROS_DISTRO}-std-msgs

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  ${SUDO} rosdep init
fi
rosdep update

printf '\nSetup complete. Open a new terminal and run:\n'
printf '  source /opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
printf '  git submodule update --init --recursive PX4-Autopilot\n'
printf '  mkdir -p ~/xtdrone_ros2_ws/src\n'
printf '  ln -s "%s/ros2/fomation_demo" ~/xtdrone_ros2_ws/src/xtdrone_formation_demo\n' "${ROOT_DIR}"
printf '  ln -s "%s/ros2/px4_gz_simulation" ~/xtdrone_ros2_ws/src/xtdrone_px4_gz\n' "${ROOT_DIR}"
printf '  cd ~/xtdrone_ros2_ws && colcon build\n'
