#!/usr/bin/env bash
# Install ROS 2 Jazzy, Gazebo Harmonic, MAVROS 2 and the PX4 v1.16.2 SITL
# build dependencies on Ubuntu 24.04. Run as a normal user; sudo is used for apt.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO=jazzy

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this script as your normal user; it calls sudo where needed." >&2
  exit 1
fi

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

require_command apt-get
require_command sudo

# shellcheck disable=SC1091
. /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "24.04" ]]; then
  echo "This setup script requires Ubuntu 24.04." >&2
  exit 1
fi

# Packages from noble-updates (e.g. libunwind8) only fit their -dev packages when the
# updates pocket is configured; without it PX4's dependencies fail with "held broken packages".
if ! grep -rqsE '(^Suites:.*|^deb .*)[[:space:]]noble-updates([[:space:]]|$)' \
     /etc/apt/sources.list /etc/apt/sources.list.d/*.sources /etc/apt/sources.list.d/*.list; then
  cat >&2 <<'MSG'
The Ubuntu "noble-updates" apt pocket is not configured. Packages already installed from it
(e.g. libunwind8) then conflict with their -dev packages. Add it and update, for example:

  sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
  sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' /etc/apt/sources.list.d/ubuntu.sources
  sudo apt update && sudo apt upgrade

then run this script again.
MSG
  exit 1
fi

echo "This script will ask sudo to install system packages."
export DEBIAN_FRONTEND=noninteractive

# The ros2-apt-source package (installed below) replaces the older manual ROS 2 apt
# setup (ros2.list + ros-archive-keyring.gpg); keeping both makes apt fail with
# "Conflicting values set for option Signed-By".
for legacy in /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list; do
  if [[ -f "${legacy}" ]] && grep -q 'packages.ros.org/ros2' "${legacy}"; then
    echo "Removing legacy ROS 2 apt source ${legacy} (replaced by ros2-apt-source)"
    sudo rm -f "${legacy}"
  fi
done
# PX4's setup script and pip must use the system Python, not a virtualenv
# activated by the login shell (e.g. an ArduPilot venv from ~/.profile).
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -vxF "${VIRTUAL_ENV}/bin" | paste -sd: -)"
  export PATH
  unset VIRTUAL_ENV
fi

PX4_DIR="${ROOT_DIR}/PX4-Autopilot"
if [[ ! -f "${PX4_DIR}/Tools/setup/ubuntu.sh" ]]; then
  git -C "${ROOT_DIR}" submodule update --init --recursive PX4-Autopilot
fi

# Python modules from Ubuntu first: PX4's setup below pip-installs only what is
# still missing, so numpy & co. stay the apt versions ROS 2 Jazzy is built against.
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  git \
  gnupg \
  lsb-release \
  python3-argcomplete \
  python3-cerberus \
  python3-coverage \
  python3-dev \
  python3-empy \
  python3-future \
  python3-jinja2 \
  python3-jsonschema \
  python3-kconfiglib \
  python3-lxml \
  python3-matplotlib \
  python3-numpy \
  python3-packaging \
  python3-pandas \
  python3-pip \
  python3-pkgconfig \
  python3-psutil \
  python3-pycryptodome \
  python3-pytest \
  python3-pygments \
  python3-requests \
  python3-serial \
  python3-setuptools \
  python3-six \
  python3-sympy \
  python3-toml \
  python3-venv \
  python3-wheel \
  python3-yaml \
  tmux \
  wget \
  imagemagick \
  xmlstarlet

# PX4 v1.16.2 SITL build dependencies and Gazebo Harmonic (packages.osrfoundation.org),
# using PX4's own setup script; the NuttX (flight controller hardware) toolchain is skipped.
bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx

# ROS 2 apt source (official ros2-apt-source package).
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  tmp_deb="$(mktemp --suffix=.deb)"
  curl -fsSL -o "${tmp_deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i "${tmp_deb}"
  rm -f "${tmp_deb}"
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-dev-tools \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-mavros-extras \
  ros-${ROS_DISTRO}-mavros-msgs \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ros-gz-interfaces

# MAVROS needs the GeographicLib geoid/magnetic datasets.
sudo "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"

# Network settings for large fleets (reserved PX4 MAVLink ports, DDS socket buffers).
"${ROOT_DIR}/scripts/tune_network_for_swarms.sh"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

printf '\nSetup complete. Next steps:\n'
printf '  cd "%s"\n' "${ROOT_DIR}"
printf '  ./scripts/build_xtdrone_ros2.sh\n'
printf '  ./scripts/verify_ubuntu_24_04.sh\n'
