#!/usr/bin/env bash
# Build XTDrone for ROS 2 Jazzy + Gazebo Harmonic:
#   1. PX4 v1.16.2 SITL (gz_bridge against Gazebo Harmonic)
#   2. XTDrone PX4 airframes into the PX4 build
#   3. Gazebo Classic worlds/models -> Harmonic (ros2/xtdrone_gz/generated)
#   4. colcon workspace with the XTDrone ROS 2 packages
# Re-running is safe. Set XTDRONE_WS to change the workspace (default ~/xtdrone_ros2_ws),
# XTDRONE_NO_VRX=1 to leave out VRX (chapter 32, the longest part of the build).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_REQUIRED=jazzy
WORKSPACE="${XTDRONE_WS:-${HOME}/xtdrone_ros2_ws}"
CACHE_DIR="${XTDRONE_CACHE:-${HOME}/.cache/xtdrone}"
PX4_DIR="${ROOT_DIR}/PX4-Autopilot"
GZ_PKG="${ROOT_DIR}/ros2/xtdrone_gz"
JOBS="${JOBS:-$(nproc)}"

# Leave a Python venv and any other ROS distribution, if the login shell set them up.
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -vxF "${VIRTUAL_ENV}/bin" | paste -sd: -)"
  unset VIRTUAL_ENV
fi
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${ROS_DISTRO_REQUIRED:-jazzy}" ]]; then
  PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -v "^/opt/ros/${ROS_DISTRO}" | paste -sd: -)"
  unset PYTHONPATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROS_DISTRO ROS_VERSION LD_LIBRARY_PATH PKG_CONFIG_PATH
fi
export PATH

ROS_DISTRO=${ROS_DISTRO_REQUIRED}
if [[ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
  echo "ROS 2 ${ROS_DISTRO} is not installed. Run scripts/setup_ubuntu_24_04.sh first." >&2
  exit 1
fi
command -v gz >/dev/null || { echo "Gazebo (gz) is missing. Run scripts/setup_ubuntu_24_04.sh first." >&2; exit 1; }

if [[ ! -f "${PX4_DIR}/Makefile" ]]; then
  git -C "${ROOT_DIR}" submodule update --init --recursive PX4-Autopilot
fi

echo "== Building PX4 SITL (px4_sitl_default)"
# PX4 must not see the ROS environment while configuring (it would pick up ROS' Python).
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  make -C "${PX4_DIR}" px4_sitl_default -j"${JOBS}"
if [[ ! -d "${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_bridge" ]]; then
  echo "error: PX4 was built without gz_bridge; is gz-harmonic (gz-transport13) installed?" >&2
  exit 1
fi

echo "== Installing XTDrone airframes into the PX4 build"
cp "${GZ_PKG}"/px4/init.d-posix/airframes/* "${PX4_DIR}/build/px4_sitl_default/etc/init.d-posix/airframes/" 2>/dev/null || true

echo "== Fetching Gazebo Classic model sources"
mkdir -p "${CACHE_DIR}"
if [[ ! -d "${CACHE_DIR}/gazebo_models/.git" ]]; then
  git clone --depth 1 https://github.com/osrf/gazebo_models.git "${CACHE_DIR}/gazebo_models"
fi
if [[ ! -d "${CACHE_DIR}/gazebo-classic/.git" ]]; then
  git clone --depth 1 --filter=blob:none --sparse -b gazebo11 \
    https://github.com/gazebosim/gazebo-classic.git "${CACHE_DIR}/gazebo-classic"
  git -C "${CACHE_DIR}/gazebo-classic" sparse-checkout set media/materials media/models
fi

# Some XTDrone worlds use models from PX4's Gazebo Classic models (big_box*, europallet, rover).
git -C "${PX4_DIR}" submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic

echo "== Converting XTDrone worlds and models to Gazebo Harmonic"
python3 "${GZ_PKG}/scripts/generate_vehicle_models.py" "${GZ_PKG}/models" >/dev/null
# CAT Vehicle UGVs from the original xacro (needs xacro and gz sdf, chapter 11)
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
python3 "${GZ_PKG}/scripts/generate_ugv_models.py" "${GZ_PKG}/models" >/dev/null
# outdoor1 to outdoor4 are not converted: their final worlds and models are committed
# (ros2/xtdrone_gz/worlds/ and world_models/)
WORLDS=()
for w in "${GZ_PKG}"/classic_assets/worlds/*.world; do
  case "$(basename "${w}" .world)" in outdoor1|outdoor2|outdoor3|outdoor4) ;; *) WORLDS+=("${w}") ;; esac
done
rm -rf "${GZ_PKG}/generated"
# shellcheck disable=SC2046
python3 "${GZ_PKG}/scripts/classic_to_gz.py" \
  --worlds "${WORLDS[@]}" \
  --model-path "${GZ_PKG}/classic_assets/models" "${CACHE_DIR}/gazebo_models" \
               "${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" \
  --classic-media "${CACHE_DIR}/gazebo-classic/media" \
  --skip-models $(ls "${GZ_PKG}/models") \
  --out "${GZ_PKG}/generated"

if [[ "${XTDRONE_NO_VRX:-0}" != "1" ]]; then
  git -C "${ROOT_DIR}" submodule update --init ros2/vrx
  python3 "${GZ_PKG}/scripts/make_sandisland.py" "${ROOT_DIR}/ros2/vrx/vrx_gz/worlds/sydney_regatta.sdf" \
    "${GZ_PKG}/generated/worlds" >/dev/null
  # WAM-V with thrusters only (VRX turns all sensors on; cameras and lidar rendered on a CPU cost ~90 %
  # of the real time factor). usv.launch.py passes it to VRX's spawn launch as urdf:=.
  mkdir -p "${GZ_PKG}/generated/usv"
  sed 's|<xacro:if value="$(arg vrx_sensors_enabled)">|<xacro:if value="false"> <!-- XTDrone: no VRX sensors -->|' \
    "${ROOT_DIR}/ros2/vrx/vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro" > "${GZ_PKG}/generated/usv/xtdrone_wamv.urdf.xacro"
fi

echo "== Building the colcon workspace ${WORKSPACE}"
mkdir -p "${WORKSPACE}/src"
for pkg in xtdrone_msgs xtdrone_gz xtdrone_bringup xtdrone_py xtdrone_groundcontrol xtdrone_slam xtdrone_planning xtdrone_gz_plugins xtdrone_orbslam3 xtdrone_ugv_planning xtdrone_fixed_wing_formation xtdrone_robotic_arm; do
  ln -sfn "${ROOT_DIR}/ros2/${pkg}" "${WORKSPACE}/src/${pkg}"
done
ln -sfn "${ROOT_DIR}/ros2/fomation_demo" "${WORKSPACE}/src/xtdrone_formation_demo"
# KISS-ICP v1.2.3 (3D lidar odometry, chapter 19); its CMake fetches Sophus and tsl-robin-map.
git -C "${ROOT_DIR}" submodule update --init ros2/kiss-icp
ln -sfn "${ROOT_DIR}/ros2/kiss-icp/ros" "${WORKSPACE}/src/kiss_icp"
# VINS-Fusion, ROS 2 Jazzy fork (stereo VIO, chapter 14; packages vins, camera_models, loop_fusion, global_fusion)
git -C "${ROOT_DIR}" submodule update --init ros2/vins-fusion
ln -sfn "${ROOT_DIR}/ros2/vins-fusion" "${WORKSPACE}/src/vins-fusion"
# EGO-Planner-Swarm, ROS 2 port (branch ros2_version; chapters 21-22): only the planner packages and
# quadrotor_msgs, not its own simulator (XTDrone flies the PX4 vehicles in Gazebo instead)
git -C "${ROOT_DIR}" submodule update --init ros2/ego-planner-swarm
mkdir -p "${WORKSPACE}/src/ego_planner"
for pkg in planner/plan_env planner/path_searching planner/bspline_opt planner/traj_utils \
           uav_simulator/Utils/quadrotor_msgs; do
  ln -sfn "${ROOT_DIR}/ros2/ego-planner-swarm/src/${pkg}" "${WORKSPACE}/src/ego_planner/$(basename "${pkg}")"
done
# plan_manage (package ego_planner) is copied and patched to follow simulation time (see the patch script)
[[ -L "${WORKSPACE}/src/ego_planner/plan_manage" ]] && rm -f "${WORKSPACE}/src/ego_planner/plan_manage"  # an older build linked it
rsync -a --delete "${ROOT_DIR}/ros2/ego-planner-swarm/src/planner/plan_manage/" "${WORKSPACE}/src/ego_planner/plan_manage/"
python3 "${ROOT_DIR}/ros2/xtdrone_planning/scripts/patch_ego_planner.py" "${WORKSPACE}/src/ego_planner/plan_manage"
# Jazzy has only cv_bridge/cv_bridge.hpp: give the EGO packages a cv_bridge.h that includes it
EGO_META="${WORKSPACE}/xtdrone_ego.meta"
python3 - "${EGO_META}" "${ROOT_DIR}/ros2/xtdrone_planning/compat" <<'PY'
import json, sys
flags = ["-DCMAKE_CXX_FLAGS=-isystem " + sys.argv[2]]
names = ["plan_env", "path_searching", "bspline_opt", "traj_utils", "ego_planner"]
json.dump({"names": {n: {"cmake-args": flags} for n in names}}, open(sys.argv[1], "w"), indent=1)
PY
# VRX v3.1.2 (Gazebo Harmonic ocean, WAM-V USV) for the UAV + USV chapter (32). XTDRONE_NO_VRX=1 skips it.
if [[ "${XTDRONE_NO_VRX:-0}" != "1" ]]; then
  ln -sfn "${ROOT_DIR}/ros2/vrx" "${WORKSPACE}/src/vrx"
else
  rm -f "${WORKSPACE}/src/vrx"
fi

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${WORKSPACE}"
colcon build --symlink-install --parallel-workers "${JOBS}" --metas "${EGO_META}"

printf '\nBuild complete. In every new terminal run:\n'
printf '  source %s/install/setup.bash\n' "${WORKSPACE}"
printf 'Then for example:\n'
printf '  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1\n'
