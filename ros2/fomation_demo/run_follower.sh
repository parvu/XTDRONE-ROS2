#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo "Usage: $0 <uav_type> <uav_count (6, 9, or 18)>" >&2
  exit 2
fi

exec ros2 launch xtdrone_formation_demo formation.launch.py \
  "uav_type:=$1" "uav_count:=$2"
   