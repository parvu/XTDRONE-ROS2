#!/usr/bin/env bash
# Headless chapter tests for the XTDrone ROS 2 / Gazebo Harmonic port.
#   scripts/tests/run_chapter_tests.sh              # all cases
#   scripts/tests/run_chapter_tests.sh ch03 ch09    # selected cases
# Logs: ${XTDRONE_TEST_LOGS:-~/.ros/xtdrone_tests}/<case>.log
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/xtdrone_env.sh"
LOG_DIR="${XTDRONE_TEST_LOGS:-${HOME}/.ros/xtdrone_tests}"
mkdir -p "${LOG_DIR}"
FLIGHT="/usr/bin/python3 ${ROOT_DIR}/scripts/tests/xtdrone_flight_test.py"
GUI="${XTDRONE_TEST_GUI:-false}"

cleanup_sim() {
  pkill -INT -f "ros2 launch xtdrone_bringup" 2>/dev/null
  sleep 3
  pkill -KILL -f "ros2 launch xtdrone_bringup" 2>/dev/null
  pkill -KILL -x px4 2>/dev/null
  pkill -KILL -f "gz sim" 2>/dev/null
  pkill -KILL -f "mavros_node" 2>/dev/null
  pkill -KILL -f "parameter_bridge" 2>/dev/null
  pkill -KILL -f "xtdrone_py/lib/xtdrone_py" 2>/dev/null
  sleep 1
}

# run_case NAME "launch args" "test args" [extra command started after launch]
run_case() {
  local name="$1" launch_args="$2" test_args="$3" extra="${4:-}"
  local log="${LOG_DIR}/${name}.log"
  echo "=== ${name}: ros2 launch xtdrone_bringup xtdrone.launch.py ${launch_args}"
  cleanup_sim
  # shellcheck disable=SC2086
  ros2 launch xtdrone_bringup xtdrone.launch.py gui:="${GUI}" ${launch_args} >"${log}" 2>&1 &
  local extra_pid=""
  if [[ -n "${extra}" ]]; then
    sleep 5
    bash -c "${extra}" >>"${log}" 2>&1 &
    extra_pid=$!
  fi
  # shellcheck disable=SC2086
  if ${FLIGHT} ${test_args} 2>&1 | tee -a "${log}"; then
    RESULTS+=("PASS ${name}")
  else
    RESULTS+=("FAIL ${name}  (log: ${log})")
  fi
  [[ -n "${extra_pid}" ]] && kill "${extra_pid}" 2>/dev/null
  cleanup_sim
}

# run_script NAME "launch args" "test script + args" [extra command]: like run_case with another test script
run_script() {
  local name="$1" launch_args="$2" script="$3" extra="${4:-}"
  local saved="${FLIGHT}"
  FLIGHT="/usr/bin/python3 ${script%% *}"
  run_case "${name}" "${launch_args}" "${script#* }" "${extra}"
  FLIGHT="${saved}"
}

# check_world NAME: load a converted world headless and check the clock runs.
check_world() {
  local world="$1" log="${LOG_DIR}/world_${1}.log"
  cleanup_sim
  local file
  file="$(ros2 pkg prefix xtdrone_gz)/share/xtdrone_gz/worlds/${world}.sdf"
  # large worlds (indoor4) take minutes to build their collision shapes on a CPU-only machine
  timeout -k 5 900 gz sim -s -r -v 2 --iterations 1000 "${file}" >"${log}" 2>&1
  local rc=$?
  if [[ ${rc} -eq 0 ]] && ! grep -qE '\[Err\]|Error Code' "${log}"; then
    RESULTS+=("PASS world ${world}")
  elif [[ ${rc} -eq 0 ]]; then
    RESULTS+=("WARN world ${world} (ran, with errors: ${log})")
  else
    RESULTS+=("FAIL world ${world} (rc=${rc}, log: ${log})")
  fi
}

# gz_case NAME "world" "commands": start a world (no vehicles), run commands that print PASS/FAIL lines
gz_case() {
  local name="$1" world="$2" cmds="$3" log="${LOG_DIR}/${1}.log"
  cleanup_sim
  local share; share="$(ros2 pkg prefix xtdrone_gz)/share/xtdrone_gz"
  echo "=== ${name}: gz sim ${world}"
  GZ_SIM_RESOURCE_PATH="${share}/models:${share}/classic_models:${share}/worlds:${ROOT_DIR}/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}" \
    gz sim -s -r -v 1 "${share}/worlds/${world}.sdf" >"${log}" 2>&1 &
  sleep 15
  if GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:${share}/worlds" bash -c "${cmds}" 2>&1 | tee -a "${log}" | grep -q '^PASS'; then
    RESULTS+=("PASS ${name}")
  else
    RESULTS+=("FAIL ${name}  (log: ${log})")
  fi
  cleanup_sim
}

gz_x() { gz model -m "$1" -p 2>/dev/null | grep -A1 XYZ | tail -1 | tr -d '[]' | awk '{print $1}'; }
export -f gz_x

RESULTS=()
CASES=("$@")
[[ ${#CASES[@]} -eq 0 ]] && CASES=(worlds ch03 ch05 ch06 ch07 ch25 ch36 ch37 ch41 ch43 ch08plane ch08vtol ch08rover ch09 ch10 ch12 ch15 ch44)
trap cleanup_sim EXIT

for c in "${CASES[@]}"; do
  case "${c}" in
    worlds)
      for w in $(ls "$(ros2 pkg prefix xtdrone_gz)/share/xtdrone_gz/worlds" | sed 's/\.sdf$//'); do
        check_world "${w}"
      done ;;
    ch03)  # basic configuration: indoor1, iris_stereo_camera, keyboard-style flight
      run_case ch03_indoor1 "scenario:=indoor1 communication:=basic" \
        "--topics /iris_0/stereo_camera/left/image_raw /iris_0/stereo_camera/right/image_raw /iris_0/imu_gazebo --vehicles iris_0" ;;
    ch05)  # single-direction position hold
      run_case ch05_hold "scenario:=indoor1 communication:=enhanced" "--vehicles iris_0" ;;
    ch06)  # EKF with vision (ground truth as vision source), GPS off
      run_case ch06_ekf_vision "scenario:=indoor1 communication:=basic ekf:=vision" "--vehicles iris_0" \
        "ros2 run xtdrone_py get_local_pose iris 1" ;;
    ch07)  # attitude control demo, driven through a pseudo terminal, both modes
      run_case ch07_attitude "scenario:=indoor1" \
        "--topics /iris_0/mavros/local_position/odom /iris_0/mavros/local_position/velocity_body" ;
      run_script ch07_angular_speed "scenario:=indoor1" "${ROOT_DIR}/scripts/tests/attitude_demo_test.py --mode angular_speed" ;
      run_script ch07_angle "world:=empty vehicles:=iris" "${ROOT_DIR}/scripts/tests/attitude_demo_test.py --mode angle" ;;
    ch25)  # ROS 2 leader/follower formation, 6 iris
      run_script ch25_formation "scenario:=formation_6 ekf:=vision communication:=basic" \
        "${ROOT_DIR}/scripts/tests/formation_test.py" "ros2 run xtdrone_py get_local_pose iris 6" ;;
    ch08plane)
      run_case ch08_plane "world:=outdoor2 vehicles:=plane communication:=basic" "--vehicles plane_0 --mode plane" ;;
    ch08vtol)
      run_case ch08_vtol "world:=outdoor2 vehicles:=standard_vtol communication:=basic" "--vehicles standard_vtol_0" ;;
    ch08rover)
      run_case ch08_rover "world:=outdoor3 vehicles:=rover communication:=basic" "--vehicles rover_0 --mode rover" ;;
    ch09)  # multi-vehicle: 6 iris
      run_case ch09_multi "world:=empty vehicles:=iris*6 communication:=basic" \
        "--vehicles iris_0 iris_1 iris_2 iris_3 iris_4 iris_5" ;;
    ch10)  # add a vehicle dynamically to a running world
      run_case ch10_add_node "world:=empty vehicles:=iris communication:=basic" "--vehicles iris_0 iris_1" \
        "sleep 20; ros2 launch xtdrone_bringup spawn_vehicle.launch.py model:=iris id:=1 instance:=1 x:=3 communication:=basic" ;;
    ch12)  # sensor configuration
      run_case ch12_sensors "world:=empty vehicles:=iris_2d_lidar,iris_3d_lidar,iris_realsense_camera,iris_downward_camera" \
        "--topic-timeout 90 --topics /iris_0/scan /iris_1/velodyne_points /iris_1/stereo_camera/left/image_raw /iris_2/realsense/depth_camera/depth/image_raw /iris_2/realsense/depth_camera/color/image_raw /iris_2/realsense/depth_camera/depth/points /iris_3/camera/image_raw /iris_0/ground_truth/odom" ;;
    ch15)  # ground-truth pose into MAVROS vision topics
      run_case ch15_groundtruth "scenario:=indoor1 communication:=basic" \
        "--topics /iris_0/mavros/vision_pose/pose /iris_0/mavros/vision_speed/speed --vehicles iris_0" \
        "ros2 run xtdrone_py get_local_pose iris 1" ;;
    ch44)  # contact sensor topic
      run_case ch44_contact "world:=empty vehicles:=iris" "--topics /iris_0/benchmarker/collision /iris_0/ground_truth/odom" ;;
    ch36)  # flat image model: generate with make_gazebo_model.py, spawn into a running world
      out="$(mktemp -d)"
      gz_case ch36_flat_model empty "
        cd '${ROOT_DIR}/sitl_config/make_gazebo_model/script' &&
        /usr/bin/python3 make_gazebo_model.py -i '${ROOT_DIR}/sitl_config/make_gazebo_model/image/' -g '${out}' -s 1000 -w 0 >/dev/null 2>&1
        test -f '${out}/landing1/meshes/landing1.png' || { echo 'FAIL texture not next to the mesh'; exit; }
        gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
          --req 'sdf_filename: \"${out}/landing1/model.sdf\", name: \"pad\"' >/dev/null
        sleep 2; gz model --list 2>/dev/null | grep -q ' pad$' && echo PASS flat model spawned || echo FAIL" ;;
    ch37)  # move environment models with set_pose_vector
      gz_case ch37_control_models outdoor1 "
        timeout 10 ros2 run xtdrone_py control_gazebo_models >/dev/null 2>&1
        x=\$(gz_x rover_static); echo rover_static x=\$x
        awk -v x=\$x 'BEGIN{exit !(x > 70.5)}' && echo PASS || echo FAIL" ;;
    ch41)  # wind from ROS 2 acts on a hovering iris
      run_script ch41_wind "world:=empty vehicles:=iris communication:=basic" \
        "${ROOT_DIR}/scripts/tests/wind_test.py --wind 5" "sleep 10; ros2 run xtdrone_py wind_control" ;;
    ch43)  # vehicles without PX4 in zero gravity, teleported; speed factor 2
      run_case ch43_no_px4 "world:=empty_zero_g vehicles:=iris*4 px4:=false" "--topics /iris_3/ground_truth/odom" ;
      run_script ch43_speed_factor "world:=empty vehicles:=iris speed_factor:=2.0" \
        "${ROOT_DIR}/scripts/tests/rtf_check.py --expect 2.0" ;;
    *) echo "unknown case ${c}" >&2 ;;
  esac
done

echo
echo "===== XTDrone chapter test summary ====="
printf '%s\n' "${RESULTS[@]}"
! printf '%s\n' "${RESULTS[@]}" | grep -q '^FAIL'
