#!/usr/bin/env bash
# Build ArduPilot SITL (Copter, Plane and Rover 4.7.2, the latest stable release) and the ArduPilot Gazebo plugin
# (Gazebo Harmonic) that connects SITL to the simulation. Run it once; scripts/build_xtdrone_ros2.sh calls it.
#
#   scripts/build_ardupilot.sh [-j N]
#
# ArduPilot's build tool needs a few Python packages (empy 3.3.4, pexpect, ...): they go into a virtualenv in
# ${XTDRONE_CACHE:-~/.cache/xtdrone}/ardupilot-venv, nothing is installed system-wide. SITL needs no MAVProxy.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${XTDRONE_CACHE:-${HOME}/.cache/xtdrone}"
VENV="${CACHE_DIR}/ardupilot-venv"
JOBS="$(nproc)"
[[ "${1:-}" == "-j" ]] && JOBS="$2"

echo "== ArduPilot sources"
git -C "${ROOT_DIR}" submodule update --init --depth 1 --recursive ArduPilot ardupilot_gazebo

echo "== Python environment for waf (${VENV})"
if [[ ! -x "${VENV}/bin/python" ]]; then
  mkdir -p "${CACHE_DIR}"
  python3 -m venv "${VENV}"
fi
"${VENV}/bin/pip" install -q empy==3.3.4 pexpect future pyserial setuptools wheel pymavlink

echo "== ArduPilot SITL (copter, plane, rover)"
(
  cd "${ROOT_DIR}/ArduPilot"
  export PATH="${VENV}/bin:${PATH}"
  [[ -f build/sitl/c4che/_cache.py ]] || ./waf configure --board sitl
  ./waf copter plane rover -j"${JOBS}"
)
for v in arducopter arduplane ardurover; do
  [[ -x "${ROOT_DIR}/ArduPilot/build/sitl/bin/${v}" ]] || { echo "error: ${v} was not built" >&2; exit 1; }
done

echo "== ArduPilot Gazebo plugin"
(
  cd "${ROOT_DIR}/ardupilot_gazebo"
  export GZ_VERSION=harmonic
  cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
  cmake --build build -j"${JOBS}"
)
[[ -f "${ROOT_DIR}/ardupilot_gazebo/build/libArduPilotPlugin.so" ]] || { echo "error: plugin was not built" >&2; exit 1; }
echo "ArduPilot SITL and the Gazebo plugin are built."
