#!/usr/bin/env bash
# List processes left over from earlier simulation runs (PX4, its rcS shells, Gazebo, MAVROS, bridges,
# ROS nodes of the XTDrone workspace and of the SLAM / planning packages the tests start).
#   ./scripts/tests/check_strays.sh          prints them; exit 1 if there are any
#   ./scripts/tests/check_strays.sh --kill   kills them first, then checks again
# Matches executable paths (ps -o args starts with the program), so it never matches the shell
# that runs it or the command line of a test that only mentions a package name.
PATTERN='^([^ ]*/)?(px4|gz|ruby|mavros_node|parameter_bridge|static_transform_publisher|robot_state_publisher)( |$)'
PATTERN+='|^/bin/sh etc/init\.d-posix/rcS'
PATTERN+='|^[^ ]*/lib/(xtdrone_[a-z_]+|ros_gz_[a-z_]+|vrx_[a-z_]+|tf2_ros|slam_toolbox|cartographer_ros|rtabmap_[a-z]+|nav2_[a-z_]+|dwb_[a-z_]+|kiss_icp|vins|loop_fusion|ego_planner|xtdrone_orbslam3|moveit_ros_move_group|controller_manager)/'
PATTERN+='|^/usr/bin/python3 [^ ]*/(ros2 launch xtdrone_|lib/xtdrone_[a-z_]+/)'
PATTERN+='|^/usr/bin/python3 /opt/ros/[a-z]+/bin/ros2 launch xtdrone_'

strays() {
  ps -eo pid=,args= | while read -r pid args; do
    [[ "${pid}" == "$$" || "${pid}" == "${BASHPID}" ]] && continue
    [[ "${args}" =~ ${PATTERN} ]] && printf '%s %s\n' "${pid}" "${args:0:150}"
  done
}

if [[ "${1:-}" == "--kill" ]]; then
  pids=$(strays | awk '{print $1}')
  [[ -n "${pids}" ]] && kill -KILL ${pids} 2>/dev/null
  sleep 1
fi
left=$(strays)
if [[ -n "${left}" ]]; then
  echo "stray simulation processes:"
  echo "${left}"
  exit 1
fi
echo "no stray simulation processes"
