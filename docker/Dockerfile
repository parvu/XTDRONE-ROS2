# XTDrone on ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.16.2 (Ubuntu 24.04).
# Build from the repository root (with the PX4-Autopilot submodule checked out):
#   docker build -f docker/Dockerfile -t xtdrone:jazzy-harmonic .
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
# apt-utils first: without it debconf prints "delaying package configuration, since apt-utils is not installed"
# for every later install (a harmless warning, not an error)
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils \
    && apt-get install -y --no-install-recommends sudo ca-certificates curl git lsb-release \
    && rm -rf /var/lib/apt/lists/*

# The setup script runs as a normal user with sudo, like on a workstation.
ARG USERNAME=xtdrone
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && useradd -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
USER ${USERNAME}
WORKDIR /home/${USERNAME}/XTDRONE-ROS2

# The setup script uses PX4's own Tools/setup/ubuntu.sh, so the repository
# (with the PX4-Autopilot submodule checked out) is copied first.
COPY --chown=${USERNAME}:${USERNAME} . .
RUN ./scripts/setup_ubuntu_24_04.sh && sudo rm -rf /var/lib/apt/lists/*
# (a failed build must fail the image: no "|| true" over the build itself)
RUN ./scripts/build_xtdrone_ros2.sh && rm -rf PX4-Autopilot/build/px4_sitl_default/build_gz*

RUN echo "source /home/${USERNAME}/XTDRONE-ROS2/scripts/xtdrone_env.sh" >> /home/${USERNAME}/.bashrc
CMD ["bash"]
