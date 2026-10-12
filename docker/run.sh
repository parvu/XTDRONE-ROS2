#!/usr/bin/env bash
# Start the XTDrone container with the X11 display (and the GPU, if the NVIDIA toolkit is installed).
#   docker/run.sh                       # interactive shell
#   docker/run.sh ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
set -euo pipefail
IMAGE="${XTDRONE_IMAGE:-xtdrone:jazzy-harmonic}"
xhost +local:docker >/dev/null 2>&1 || true
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
else
  GPU_ARGS=(--device /dev/dri)
fi
exec docker run -it --rm --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${GPU_ARGS[@]}" "${IMAGE}" bash -ic "${*:-bash}"
