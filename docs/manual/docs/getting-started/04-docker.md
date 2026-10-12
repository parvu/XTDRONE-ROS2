# 4. Use Docker to deploy the simulation environment

!!! info "Ported"
    `docker/Dockerfile` and `docker/run.sh` build and run an Ubuntu 24.04 image with ROS 2 Jazzy, Gazebo
    Harmonic, MAVROS 2, PX4 v1.16.2 and the XTDrone workspace. The image was not built on the test host,
    which has no Docker access.

The original chapter offered a prebuilt XTDrone image for ROS 1. This edition builds the image from the
repository with the same scripts as a native install, so the container and a native install match.

## Requirements

- Docker Engine 24 or newer, with your user in the `docker` group: `sudo usermod -aG docker $USER`, then log in again;
- for an NVIDIA GPU: the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html);
- about 15 GB of disk space.

## Build the image

```bash
cd ~/XTDRONE-ROS2
git submodule update --init --recursive PX4-Autopilot
docker build -f docker/Dockerfile -t xtdrone:jazzy-harmonic .
```

Inside the image, the build runs `scripts/setup_ubuntu_24_04.sh` and then `scripts/build_xtdrone_ros2.sh` as a user
named `xtdrone`. `.dockerignore` keeps host build output (the PX4 build, generated worlds) out of the image.

## Run

```bash
docker/run.sh
```

`run.sh` allows local X11 connections (`xhost +local:docker`) and starts the container with:

- host networking, so ROS 2 discovery and QGroundControl work with the host;
- the X11 socket, for the Gazebo GUI;
- `--gpus all` if `nvidia-smi` exists, otherwise `/dev/dri` for Intel or AMD GPUs.

Each shell in the container sources `scripts/xtdrone_env.sh`. Then follow chapter 2 from step 5:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 communication:=basic
```

You can also pass a command directly: `docker/run.sh ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1`.
Open more terminals in the same container with `docker exec -it <container> bash`.

## Notes

- Without a GPU, Gazebo renders sensors in software (llvmpipe), which is slow. Use vehicles without cameras or
  `gui:=false`.
- To keep your changes, mount the repository instead of copying it:
  `docker run ... -v ~/XTDRONE-ROS2:/home/xtdrone/XTDRONE-ROS2 ...`. Then run `scripts/build_xtdrone_ros2.sh`
  once inside the container.
