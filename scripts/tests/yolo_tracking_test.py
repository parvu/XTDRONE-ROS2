#!/usr/bin/env python3
"""Chapter 23 check: YOLO finds the walking pedestrian of outdoor1 in the typhoon's gimbal camera, and
the visual servo keeps following it.

Requires: xtdrone.launch.py scenario:=outdoor1 communication:=basic (its actor_walker node holds the
pedestrian until the drone is above 2 m and both yolo nodes run), and
`ros2 run xtdrone_py gimbal_control typhoon_h480 0` + `ros2 run xtdrone_py yolo_detection typhoon_h480 0`.

1. take off to --altitude (5 m) and hover; the camera looks 60 deg down;
2. start yolo_human_tracking right away (it publishes zero velocity until it sees a person). The
   walker gate opens and the actor walks along y = 3 into view; wait up to 150 s for >=3 person
   detections and for the actor to advance > 5 m (its start point is already in view, so a walker
   whose gate never opens must fail here, not pass on the stationary person);
3. check that the typhoon moves (follows) while detections continue.
"""
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, Twist, Vector3
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

V = "typhoon_h480_0"


def log(m):
    print(f"[yolo_tracking_test] {m}", flush=True)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--altitude", type=float, default=5.0, help="hover height above the ground (m)")
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("yolo_tracking_test")
    d = {"persons": 0, "frames": 0}

    def det(m):
        d["frames"] += 1
        if any(x.results and x.results[0].hypothesis.class_id == "person" for x in m.detections):
            d["persons"] += 1
            d["last"] = time.time()
    n.create_subscription(State, f"/{V}/mavros/state", lambda m: d.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(HomePosition, f"/{V}/mavros/home_position/home", lambda m: d.__setitem__("home", m),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, f"/{V}/ground_truth/odom", lambda m: d.__setitem__("gt", m.pose.pose.position), 1)
    n.create_subscription(Detection2DArray, f"/{V}/detections", det, 10)
    n.create_subscription(Pose, "/actor/pose", lambda m: d.__setitem__("actor", m.position.x), 10)
    cmd = n.create_publisher(String, f"/xtdrone/{V}/cmd", 3)
    vel = n.create_publisher(Twist, f"/xtdrone/{V}/cmd_vel_flu", 1)

    def wait(s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return bool(cond and cond())

    ok = True
    tracker = None
    try:
        if not wait(240, lambda: d.get("st") and d["st"].connected and "home" in d and "gt" in d):
            log("FAIL: typhoon not ready")
            return 1
        up = lambda: vel.publish(Twist(linear=Vector3(z=1.0)))  # noqa: E731
        deadline = time.time() + 90
        while time.time() < deadline and not d["st"].armed:
            cmd.publish(String(data="OFFBOARD"))
            wait(1, every=up)
            cmd.publish(String(data="ARM"))
            wait(2, lambda: d["st"].armed, every=up)
        if not wait(90, lambda: d["gt"].z > a.altitude, every=up):
            log("FAIL: takeoff")
            return 1
        cmd.publish(String(data="HOVER"))
        wait(2, every=lambda: vel.publish(Twist()))
        tracker = subprocess.Popen(["ros2", "run", "xtdrone_py", "yolo_human_tracking", "typhoon_h480", "0"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        # the actor_walker gate (drone > 2 m, both yolo nodes) opens now and the actor walks along
        # y = 3 into view; requiring it to advance catches a walker that never starts, since its
        # start point (0, 3) is already in the camera view
        x0 = d.get("actor", 0.0)
        seen = wait(150, lambda: d["persons"] >= 3 and d.get("actor", 0.0) - x0 > 5.0)
        log(f"{'ok  ' if seen else 'FAIL'} YOLO: person in {d['persons']} of {d['frames']} processed frames, "
            f"actor advanced {d.get('actor', 0.0) - x0:.1f} m")
        ok &= seen
        if seen:
            start = (d["gt"].x, d["gt"].y)
            p0 = d["persons"]
            wait(60)
            moved = math.dist(start, (d["gt"].x, d["gt"].y))
            follow = moved > 1.0 and d["persons"] > p0
            log(f"{'ok  ' if follow else 'FAIL'} tracking: typhoon moved {moved:.1f} m, person seen in "
                f"{d['persons'] - p0} more frames")
            ok &= follow
    finally:
        if tracker:
            os.killpg(tracker.pid, signal.SIGINT)
            try:
                tracker.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(tracker.pid, signal.SIGKILL)
        cmd.publish(String(data="AUTO.LAND"))
        wait(90, lambda: d.get("st") and not d["st"].armed)
        n.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
