#!/usr/bin/env bash
# Check the XTDrone ROS 2 Jazzy / Gazebo Harmonic installation (no simulation is started).
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/xtdrone_env.sh" || exit 1
fail=0
check() {
  local what="$1"; shift
  if "$@" >/dev/null 2>&1; then echo "ok    ${what}"; else echo "FAIL  ${what}"; fail=1; fi
}

check "ROS 2 Jazzy (ros2 CLI)" command -v ros2
check "Gazebo Harmonic (gz sim 8)" bash -c "gz sim --versions | grep -q '^8\\.'"
check "gz-transport Python bindings" /usr/bin/python3 -c "import gz.transport13, gz.msgs10"
check "MAVROS 2" ros2 pkg prefix mavros
check "MAVROS GeographicLib datasets" test -d /usr/share/GeographicLib/geoids
check "ros_gz_bridge" ros2 pkg prefix ros_gz_bridge
check "ros_gz_sim" ros2 pkg prefix ros_gz_sim
check "PX4 v1.16.2 submodule" bash -c "[[ \$(git -C '${ROOT_DIR}/PX4-Autopilot' rev-parse HEAD) == 54f0455ffcd755534539a7cf33a09a20bf71d29d ]]"
check "PX4 SITL binary" test -x "${ROOT_DIR}/PX4-Autopilot/build/px4_sitl_default/bin/px4"
check "PX4 gz_bridge built" test -d "${ROOT_DIR}/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge"
check "XTDrone typhoon airframe in PX4 build" bash -c "ls '${ROOT_DIR}'/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4901_gz_typhoon_h480"
for pkg in xtdrone_gz xtdrone_bringup xtdrone_py xtdrone_formation_demo xtdrone_px4_gz; do
  check "package ${pkg}" ros2 pkg prefix "${pkg}"
done
check "converted worlds installed" bash -c "ls \$(ros2 pkg prefix xtdrone_gz)/share/xtdrone_gz/worlds/indoor1.sdf"
check "launch xtdrone.launch.py --show-args" ros2 launch xtdrone_bringup xtdrone.launch.py --show-args
check "xtdrone_py entry points" bash -c "ros2 pkg executables xtdrone_py | grep -q multirotor_communication"

if [[ ${fail} -eq 0 ]]; then
  echo; echo "XTDrone ROS 2 Jazzy / Gazebo Harmonic installation looks complete."
else
  echo; echo "Some checks failed; see the Basic Configuration chapter of the manual." >&2
fi
exit ${fail}
