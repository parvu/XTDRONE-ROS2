#!/usr/bin/env python3
"""Chapter 31 check: the CAT Vehicle UGV plans with hybrid A* on the slam_toolbox map of outdoor1 and drives
there with pure pursuit.

Requires: xtdrone.launch.py scenario:=outdoor1_ugv (ugv_0 at (1, 1)); the test starts
`ros2 launch xtdrone_ugv_planning ugv_planning.launch.py`.

1. wait for the map (slam_toolbox) and send a goal on /goal_pose (default (24, 5), beside the road ahead);
2. check that a path to the goal is published, that the UGV drives to within 2 m of the goal and stops there,
   and that the planner is still running.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def log(m):
    print(f"[ugv_planning_test] {m}", flush=True)


def save_map(m, path):
    """The map the plan is made on, as a PGM (occupied black, free white, unknown grey; row 0 at the top is the
    largest y), with its origin in a comment."""
    w, h = m.info.width, m.info.height
    shade = {-1: 205, 0: 254}
    rows = [bytes(shade.get(v, 0 if v > 50 else 254) for v in m.data[r * w:(r + 1) * w]) for r in range(h - 1, -1, -1)]
    with open(path, "wb") as f:
        f.write(f"P5\n# origin {m.info.origin.position.x:.2f} {m.info.origin.position.y:.2f} "
                f"resolution {m.info.resolution:.3f}\n{w} {h}\n255\n".encode())
        f.write(b"".join(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", default="24,5")
    ap.add_argument("--timeout", type=float, default=300.0)
    a = ap.parse_args()
    goal = tuple(float(v) for v in a.goal.split(","))
    rclpy.init()
    n = rclpy.create_node("ugv_planning_test")
    d = {"cmds": 0}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(Odometry, "/ugv_0/ground_truth/odom", lambda m: d.__setitem__("gt", m.pose.pose.position),
                          qos_profile_sensor_data)
    n.create_subscription(OccupancyGrid, "/map", lambda m: d.__setitem__("map", m), latched)
    n.create_subscription(Path, "/front_end_path", lambda m: d.__setitem__("path", m), latched)
    n.create_subscription(Twist, "/ugv_0/cmd_vel", lambda m: d.__setitem__("cmds", d["cmds"] + 1), 10)
    goal_pub = n.create_publisher(PoseStamped, "/goal_pose", 1)

    def wait(s, cond):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
            if cond():
                return True
        return cond()

    logs = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
    os.makedirs(logs, exist_ok=True)
    plan_log = open(os.path.join(logs, "ugv_planning_launch.log"), "w")
    plan = subprocess.Popen(["ros2", "launch", "xtdrone_ugv_planning", "ugv_planning.launch.py"], stdout=plan_log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    ok = True
    try:
        if not wait(240, lambda: "gt" in d and "map" in d):
            log("FAIL: no ground truth or no /map")
            return 1
        start = (d["gt"].x, d["gt"].y)
        log(f"map {d['map'].info.width} x {d['map'].info.height} at {d['map'].info.resolution:.2f} m; "
            f"UGV at ({start[0]:.1f}, {start[1]:.1f}), goal {goal}")
        wait(5, lambda: False)
        save_map(d["map"], os.path.join(logs, "ugv_planning_map.pgm"))
        g = PoseStamped()
        g.header.frame_id = "map"
        g.pose.position.x, g.pose.position.y = goal
        g.pose.orientation.w = 1.0
        goal_pub.publish(g)
        got_path = wait(60, lambda: "path" in d)
        if got_path:
            p = d["path"].poses
            end = (p[-1].pose.position.x, p[-1].pose.position.y)
            log(f"ok   path of {len(p)} poses to ({end[0]:.1f}, {end[1]:.1f})")
            log("path: " + " ".join(f"({q.pose.position.x:.1f},{q.pose.position.y:.1f})" for q in p[::max(1, len(p) // 20)]))
        else:
            log("FAIL: no path on /front_end_path")
        ok &= got_path
        t0 = time.time()
        arrived = wait(a.timeout, lambda: math.dist((d["gt"].x, d["gt"].y), goal) < 2.0)
        wait(10, lambda: False)  # it should stop there
        final = (d["gt"].x, d["gt"].y)
        err = math.dist(final, goal)
        moved = math.dist(final, start)
        good = arrived and err < 2.0
        log(f"{'ok  ' if good else 'FAIL'} UGV at ({final[0]:.1f}, {final[1]:.1f}), {err:.2f} m from the goal, "
            f"drove {moved:.1f} m in {time.time() - t0:.0f} s")
        ok &= good
        alive = plan.poll() is None and "process has died" not in open(plan_log.name).read()
        log(f"{'ok  ' if alive else 'FAIL'} planner and follower still running")
        ok &= alive
    finally:
        os.killpg(plan.pid, signal.SIGINT)
        try:
            plan.wait(15)
        except subprocess.TimeoutExpired:
            os.killpg(plan.pid, signal.SIGKILL)
        n.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
