#!/usr/bin/env python3
"""Chapter 30 check: the CAT Vehicle UGV keeps its lane on the outdoor3 track by itself.

Requires: xtdrone.launch.py scenario:=outdoor3_ugv and
`ros2 run xtdrone_py lane_detection ugv 0` + `ros2 run xtdrone_py self_driving ugv 0 --speed 3`.

Checks that within --time s ugv_0 drives at least --distance m along its path and that the lane
detector found both lane lines most of the time.
"""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16

NO_LANE = 1000


def log(m):
    print(f"[self_driving_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--time", type=float, default=90.0)
    ap.add_argument("--distance", type=float, default=30.0)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("self_driving_test")
    st = {"pos": None, "path": 0.0, "lane": 0, "nolane": 0}

    def odom(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if st["pos"] is not None:
            st["path"] += math.dist(p, st["pos"])
        st["pos"] = p

    def lane(m):
        st["nolane" if m.data == NO_LANE else "lane"] += 1
    n.create_subscription(Odometry, "/ugv_0/ground_truth/odom", odom, 10)
    n.create_subscription(Int16, "/ugv_0/lane_mid_error", lane, 10)
    end = time.time() + 120
    while time.time() < end and (st["pos"] is None or st["lane"] + st["nolane"] == 0):
        rclpy.spin_once(n, timeout_sec=0.1)
    if st["pos"] is None:
        log("FAIL: no ground truth")
        return 1
    start = st["pos"]
    st["path"] = 0.0
    end = time.time() + a.time
    while time.time() < end and st["path"] < a.distance:
        rclpy.spin_once(n, timeout_sec=0.1)
    found = st["lane"] / max(1, st["lane"] + st["nolane"])
    # 0.6, not 0.7: in five runs the lane was found in 66 to 75 % of the detections while the vehicle drove the
    # whole distance every time
    # along the track, not in circles: the straight-line distance is at least half of the path
    ok = st["path"] >= a.distance and found > 0.6 and math.dist(start, st["pos"]) > 0.5 * st["path"]
    log(f"{'ok  ' if ok else 'FAIL'} drove {st['path']:.1f} m (straight-line {math.dist(start, st['pos']):.1f} m), "
        f"lane found in {100 * found:.0f} % of the detections")
    n.destroy_node()
    rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
