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
export PYTHONUNBUFFERED=1  # launch output reaches the logs even when a case is killed
mkdir -p "${LOG_DIR}"
FLIGHT="/usr/bin/python3 ${ROOT_DIR}/scripts/tests/xtdrone_flight_test.py"
GUI="${XTDRONE_TEST_GUI:-false}"

cleanup_sim() {
  pkill -INT -f "ros2 launch xtdrone_" 2>/dev/null
  sleep 3
  pkill -KILL -f "ros2 launch xtdrone_" 2>/dev/null
  pkill -KILL -x px4 2>/dev/null
  # PX4's startup script runs in its own shell; if px4 dies while rcS still waits for Gazebo, the
  # orphaned rcS would later send its commands to the next px4 with the same instance number.
  pkill -KILL -f "etc/init.d-posix/rc[S]" 2>/dev/null
  pkill -KILL -f "gz sim" 2>/dev/null
  pkill -KILL -f "mavros_node" 2>/dev/null
  pkill -KILL -f "parameter_bridge" 2>/dev/null
  pkill -KILL -f "xtdrone_py/lib/xtdrone_py" 2>/dev/null
  # match executable paths only: names like slam_toolbox also appear in test and shell arguments
  pkill -KILL -f "^[^ ]*/lib/(vrx_ros|vrx_gz)/" 2>/dev/null
  # children of launches started by test cases (orphaned if their launch was killed)
  pkill -KILL -f "^[^ ]*/lib/(tf2_ros|robot_state_publisher|ros_gz_sim)/" 2>/dev/null
  pkill -KILL -f "^[^ ]*/lib/(xtdrone_slam|xtdrone_formation_demo|slam_toolbox|cartographer_ros|rtabmap_[a-z]+|nav2_[a-z_]+|dwb_[a-z_]+|xtdrone_ugv_planning|xtdrone_fixed_wing_formation|xtdrone_robotic_arm|moveit_ros_move_group|kiss_icp|vins|loop_fusion|ego_planner|xtdrone_planning|xtdrone_orbslam3)/" 2>/dev/null
  pkill -KILL -f "^/usr/bin/python3 [^ ]*/lib/(xtdrone_slam|xtdrone_formation_demo|xtdrone_planning)/" 2>/dev/null
  sleep 1
  # anything still alive slipped past the patterns above: report it (it would disturb the next case), kill it
  local strays
  if ! strays=$("${ROOT_DIR}/scripts/tests/check_strays.sh"); then
    echo "WARNING: cleanup left processes behind; killing them"
    echo "${strays}" | tail -n +2 | sed 's/^/  /'
    STRAYS+=("$(echo "${strays}" | tail -n +2 | wc -l) stray processes at a cleanup of ${CURRENT_CASE:-the first case}")
    "${ROOT_DIR}/scripts/tests/check_strays.sh" --kill >/dev/null || echo "WARNING: some processes survived kill -9"
  fi
  # Fast DDS shared-memory files of the killed processes: hundreds of them made new ROS nodes hang at start
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}

# run_case NAME "launch args" "test args" [extra command started after launch]
run_case() {
  local name="$1" launch_args="$2" test_args="$3" extra="${4:-}"
  local log="${LOG_DIR}/${name}.log"
  CURRENT_CASE="${name}"
  echo "=== ${name}: ros2 launch xtdrone_bringup xtdrone.launch.py ${launch_args}"
  cleanup_sim
  # shellcheck disable=SC2086
  : >"${log}"
  # append: the extra command and the test write to the same file
  ros2 launch xtdrone_bringup xtdrone.launch.py gui:="${GUI}" ${launch_args} >>"${log}" 2>&1 &
  local extra_pid=""
  if [[ -n "${extra}" ]]; then
    sleep 5
    bash -c "${extra}" >>"${log}" 2>&1 &
    extra_pid=$!
  fi
  # shellcheck disable=SC2086
  ${FLIGHT} ${test_args} 2>&1 | tee -a "${log}"
  local rc=${PIPESTATUS[0]}
  [[ -n "${extra_pid}" ]] && kill "${extra_pid}" 2>/dev/null
  cleanup_sim
  # exit code 3: the test found the simulation itself broken (chapter 35: a PX4 instance that stopped its timers
  # under load), not the code under test; run the case again on a fresh simulation (STALL_RETRIES times, default 2)
  local tries=${XTD_RETRIED:-0}
  if [[ ${rc} -eq 3 && ${tries} -lt ${STALL_RETRIES:-2} ]]; then
    mv "${log}" "${log%.log}.stalled$((tries + 1)).log"
    echo "=== ${name}: the simulation stalled (exit 3, log ${log%.log}.stalled$((tries + 1)).log): again"
    XTD_RETRIED=$((tries + 1)) run_case "$@"
    return
  fi
  if [[ ${rc} -eq 0 ]]; then
    RESULTS+=("PASS ${name}${XTD_RETRIED:+ (after ${XTD_RETRIED} stalled simulation(s))}")
  else
    RESULTS+=("FAIL ${name}  (log: ${log})")
  fi
}

# run_script NAME "launch args" "test script + args" [extra command]: like run_case with another test script
run_script() {
  local name="$1" launch_args="$2" script="$3" extra="${4:-}"
  local saved="${FLIGHT}"
  local args=""
  [[ "${script}" == *" "* ]] && args="${script#* }"
  FLIGHT="/usr/bin/python3 ${script%% *}"
  run_case "${name}" "${launch_args}" "${args}" "${extra}"
  FLIGHT="${saved}"
}

# check_world NAME: load a converted world headless and check the clock runs.
check_world() {
  local world="$1" log="${LOG_DIR}/world_${1}.log"
  CURRENT_CASE="world_${world}"
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
  CURRENT_CASE="${name}"
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
STRAYS=()
CASES=("$@")
[[ ${#CASES[@]} -eq 0 ]] && CASES=(worlds ch03 ch05 ch06 ch07 ch24 ch25 ch27 ch28 ch36 ch42 ch37 ch38 ch41 ch43 ch08plane ch08vtol ch08rover ch09 ch11 ch16 ch17 ch18 ch20 ch29 ch30 ch32 ch10 ch12 ch15 ch44 ch13 ch14 ch19 ch21 ch22 ch23 ch31 ch33 ch35)
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
    ch11)  # CAT Vehicle UGVs: Ackermann driving, sensors
      run_script ch11_ugv "scenario:=outdoor3_ugv" "${ROOT_DIR}/scripts/tests/ugv_test.py" ;;
    ch16)  # RTAB-Map dense reconstruction (RGB-D odometry) from the realsense camera
      run_script ch16_rtabmap "world:=indoor1 vehicles:=iris_realsense_camera communication:=basic" \
        "${ROOT_DIR}/scripts/tests/rtabmap_test.py" "sleep 25; ros2 launch xtdrone_slam rtabmap_rgbd.launch.py" ;;
    ch17)  # 2D laser SLAM with slam_toolbox (HectorSLAM in ROS 1): fly on the SLAM pose only
      run_script ch17_slam_toolbox "scenario:=indoor3 ekf:=laser2d communication:=basic px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/laser_slam_test.py --slam slam_toolbox --size 1" \
        "sleep 20; ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox" ;;
    ch18)  # 2D laser SLAM with Cartographer
      run_script ch18_cartographer "scenario:=indoor3 ekf:=laser2d communication:=basic px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/laser_slam_test.py --slam cartographer --size 1" \
        "sleep 20; ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=cartographer" ;;
    ch13)  # stereo visual SLAM (ORB-SLAM3; scripts/build_orbslam3.sh first): fly on the SLAM pose only
      run_script ch13_orbslam3 "scenario:=outdoor1_stereo ekf:=vision communication:=basic px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/laser_slam_test.py --slam orbslam3" \
        "sleep 20; ros2 launch xtdrone_orbslam3 stereo.launch.py" ;;
    ch14)  # stereo visual-inertial odometry (VINS-Fusion): fly on the VIO pose only
      run_script ch14_vins "scenario:=outdoor1_stereo ekf:=vision communication:=basic px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/laser_slam_test.py --slam vins --tolerance 0.6" \
        "sleep 20; ros2 launch xtdrone_slam vio.launch.py" ;;
    ch19)  # 3D laser odometry (KISS-ICP; A-LOAM in ROS 1): fly on the lidar odometry only
      run_script ch19_kiss_icp "scenario:=outdoor1_3d_lidar ekf:=vision communication:=basic px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/laser_slam_test.py --slam kiss_icp" \
        "sleep 20; ros2 launch xtdrone_slam laser_slam_3d.launch.py" ;;
    ch20)  # Nav2 2D planning on slam_toolbox
      run_script ch20_nav2 "scenario:=indoor3 ekf:=laser2d communication:=enhanced px4_params:=MPC_XY_VEL_MAX=1,MPC_ACC_HOR=1" \
        "${ROOT_DIR}/scripts/tests/nav2_test.py" \
        "sleep 20; ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox & sleep 15; ros2 launch xtdrone_planning nav2_2d.launch.py" ;;
    ch30)  # lane keeping of the CAT Vehicle on the outdoor3 track
      run_script ch30_self_driving "scenario:=outdoor3_ugv" "${ROOT_DIR}/scripts/tests/self_driving_test.py" \
        "sleep 25; ros2 run xtdrone_py lane_detection ugv 0 & ros2 run xtdrone_py self_driving ugv 0 --speed 3" ;;
    ch29)  # a different mission per iris (own mission storage per PX4 instance), flown in AUTO.MISSION
      run_script ch29_missions "world:=outdoor2 vehicles:=iris*3 mavros_plugins:=all" \
        "${ROOT_DIR}/scripts/tests/mission_test.py" ;;
    ch31)  # hybrid A* + pure pursuit for the CAT Vehicle on slam_toolbox's map of outdoor1
      run_script ch31_ugv_planning "scenario:=outdoor1_ugv" "${ROOT_DIR}/scripts/tests/ugv_planning_test.py --goal 20,-1.5" ;;
    ch33)  # le_arm under the iris: MoveIt 2 + gz_ros2_control grasp, then take off with the cube
      run_script ch33_robotic_arm "scenario:=grasping_arm" "${ROOT_DIR}/scripts/tests/arm_grasp_test.py" ;;
    ch35)  # three VTOLs in formation, the leader loitering over the UGV (outdoor2)
      run_script ch35_fw_formation "scenario:=outdoor2_fw_formation mavlink_profile:=light px4_params:=MIS_TAKEOFF_ALT=40,VT_TRANS_TIMEOUT=30,VT_F_TRANS_THR=0.75,ASPD_DO_CHECKS=0,SDLOG_MODE=-1,CBRK_BUZZER=782097" \
        "${ROOT_DIR}/scripts/tests/fw_formation_test.py" ;;
    ch32)  # UAVs + VRX WAM-V in sandisland
      run_script ch32_usv "scenario:=sandisland communication:=basic" "${ROOT_DIR}/scripts/tests/usv_test.py" \
        "sleep 40; ros2 launch xtdrone_bringup usv.launch.py" ;;
    ch21)  # EGO-Planner on the RealSense depth image, ground truth as pose: around two panels in indoor1
      run_script ch21_ego_planner "scenario:=indoor1_ego ekf:=vision communication:=basic" \
        "${ROOT_DIR}/scripts/tests/ego_planner_test.py --detour 0.45" \
        "sleep 15; ros2 run xtdrone_py get_local_pose iris 1" ;;
    ch22)  # EGO-Planner-Swarm: 4 iris across the ego_swarm_4 obstacle field, ground truth as pose
      run_script ch22_ego_swarm "scenario:=ego_swarm ekf:=vision communication:=basic" \
        "${ROOT_DIR}/scripts/tests/ego_swarm_test.py --count 4" \
        "sleep 20; ros2 run xtdrone_py get_local_pose iris 4" ;;
    ch23)  # YOLO (OpenCV DNN, YOLOv4-tiny) finds the walking pedestrian of outdoor1 in the typhoon's gimbal camera
      run_script ch23_yolo_tracking "scenario:=outdoor1 communication:=basic" \
        "${ROOT_DIR}/scripts/tests/yolo_tracking_test.py" \
        "sleep 20; ros2 run xtdrone_py gimbal_control typhoon_h480 0 & ros2 run xtdrone_py yolo_detection typhoon_h480 0" ;;
    ch24)  # consensus formation with collision avoidance, 9 iris
      run_script ch24_consensus_formation "world:=empty vehicles:=iris*9 ekf:=vision communication:=basic" \
        "${ROOT_DIR}/scripts/tests/formation_test.py --count 9 --controller consensus --avoid" \
        "ros2 run xtdrone_py get_local_pose iris 9" ;;
    ch25)  # ROS 2 leader/follower formation, 6 iris
      run_script ch25_formation "scenario:=formation_6 ekf:=vision communication:=basic" \
        "${ROOT_DIR}/scripts/tests/formation_test.py" "ros2 run xtdrone_py get_local_pose iris 6" ;;
    ch27)  # formation with the SINR communication model, 20 iris (one process per UAV controller)
      # 20 PX4 + MAVROS on a 6-core machine: half real time leaves headroom for the wall-clock nodes
      run_script ch27_formation_comm "scenario:=formation_comm_20 speed_factor:=0.5" \
        "${ROOT_DIR}/scripts/tests/formation_comm_test.py" ;;
    ch28)  # precision landing on AprilTags 0-2 with the OpenCV detector
      run_script ch28_precision_landing "scenario:=outdoor2_precision_landing communication:=basic" \
        "${ROOT_DIR}/scripts/tests/precision_landing_test.py" ;;
    ch42)  # XTDGroundControl (offscreen Qt) flies two iris
      run_script ch42_groundcontrol "world:=empty vehicles:=iris*2 communication:=basic" \
        "${ROOT_DIR}/scripts/tests/groundcontrol_test.py" ;;
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
        cd '${ROOT_DIR}/ros2/xtdrone_gz/tools/make_gazebo_model/script' &&
        /usr/bin/python3 make_gazebo_model.py -i '${ROOT_DIR}/ros2/xtdrone_gz/tools/make_gazebo_model/image/' -g '${out}' -s 1000 -w 0 >/dev/null 2>&1
        test -f '${out}/landing1/meshes/landing1.png' || { echo 'FAIL texture not next to the mesh'; exit; }
        gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
          --req 'sdf_filename: \"${out}/landing1/model.sdf\", name: \"pad\"' >/dev/null
        sleep 2; gz model --list 2>/dev/null | grep -q ' pad$' && echo PASS flat model spawned || echo FAIL" ;;
    ch37)  # move environment models with set_pose_vector
      gz_case ch37_control_models outdoor1 "
        timeout 10 ros2 run xtdrone_py control_gazebo_models >/dev/null 2>&1
        x=\$(gz_x rover_static); echo rover_static x=\$x
        awk -v x=\$x 'BEGIN{exit !(x > 70.5)}' && echo PASS || echo FAIL" ;;
    ch38)  # ROS-controlled actors (xtdrone_gz_plugins ActorMotion) in robocup
      run_script ch38_actors "world:=robocup vehicles:=iris px4:=false" "${ROOT_DIR}/scripts/tests/actor_test.py" ;;
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
if [[ ${#STRAYS[@]} -gt 0 ]]; then
  echo "cleanup warnings:"
  printf '  %s\n' "${STRAYS[@]}"
fi
! printf '%s\n' "${RESULTS[@]}" | grep -q '^FAIL'
