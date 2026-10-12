#!/usr/bin/env python3
"""Chapter 21 check: EGO-Planner flies an iris_realsense_camera through the obstacles of a world to a goal.

Requires: xtdrone.launch.py scenario:=indoor1_ego ekf:=vision communication:=basic and
`ros2 run xtdrone_py get_local_pose iris 1` (ground truth as the pose source).

1. take off to ~1.5 m and hold position;
2. start `ros2 launch xtdrone_planning ego_planner.launch.py target_x:=.. target_y:=.. target_z:=..`
   (--target is the displacement from the start point; the test adds the local start position);
3. check that the vehicle reaches the goal (ground truth within 1 m) without coming down to the floor, and
   that the planner got it there (it published B-splines and, with --detour, left the straight line);
4. land.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

V = "iris_0"


def log(m):
    print(f"[ego_planner_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="0,-11.5,1.2",
                    help="goal: x, y displacement from the start and height above it (m, ENU)")
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--detour", type=float, default=0.0,
                    help="required largest sideways distance from the straight start-goal line (m): proof that "
                         "the planner went around the obstacles of the scenario")
    a = ap.parse_args()
    goal = tuple(float(x) for x in a.target.split(","))
    rclpy.init()
    n = rclpy.create_node("ego_planner_test")
    d = {"bsplines": 0, "min_z": 1e9, "side": 0.0}
    n.create_subscription(State, f"/{V}/mavros/state", lambda m: d.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(PoseStamped, f"/{V}/mavros/local_position/pose", lambda m: d.__setitem__("pose", m.pose),
                          qos_profile_sensor_data)

    def gt(m):
        d.setdefault("gt0", m.pose.pose.position)
        d["gt"] = m.pose.pose.position
    n.create_subscription(Odometry, f"/{V}/ground_truth/odom", gt, 1)
    cmd = n.create_publisher(String, f"/xtdrone/{V}/cmd", 3)
    vel = n.create_publisher(Twist, f"/xtdrone/{V}/cmd_vel_flu", 1)
    hold = n.create_publisher(Pose, f"/xtdrone/{V}/cmd_pose_enu", 1)
    try:
        from traj_utils.msg import Bspline
        n.create_subscription(Bspline, f"/xtdrone/{V}/planning/bspline",
                              lambda m: d.__setitem__("bsplines", d["bsplines"] + 1), 10)
    except ImportError:
        log("note: traj_utils not importable, B-spline count not checked")
        d["bsplines"] = None

    def wait(s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return bool(cond and cond())

    def rel():  # ground truth relative to the spawn point
        p, p0 = d["gt"], d["gt0"]
        return (p.x - p0.x, p.y - p0.y, p.z - p0.z)

    ok = True
    ego = None
    try:
        if not wait(240, lambda: d.get("st") and d["st"].connected and "pose" in d and "gt" in d):
            log("FAIL: vehicle not ready")
            return 1
        wait(3)
        p0 = d["pose"].position  # local start position: EGO plans in the local frame
        target = (p0.x + goal[0], p0.y + goal[1], p0.z + goal[2])
        up = lambda: vel.publish(Twist(linear=Vector3(z=0.8)))  # noqa: E731
        deadline = time.time() + 90
        while time.time() < deadline and not d["st"].armed:
            cmd.publish(String(data="OFFBOARD"))
            wait(1, every=up)
            cmd.publish(String(data="ARM"))
            wait(2, lambda: d["st"].armed, every=up)
        if not wait(60, lambda: d["pose"].position.z > target[2] - 0.1, every=up):
            log("FAIL: takeoff")
            return 1
        p = d["pose"].position
        here = Pose()
        here.position.x, here.position.y, here.position.z = p.x, p.y, target[2]
        here.orientation = d["pose"].orientation
        wait(3, every=lambda: hold.publish(here))
        log(f"hovering at {rel()[0]:.1f},{rel()[1]:.1f},{rel()[2]:.1f}; goal {goal}")
        logs = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
        os.makedirs(logs, exist_ok=True)
        ego_log = open(os.path.join(logs, "ego_planner_launch.log"), "w")
        ego = subprocess.Popen(["ros2", "launch", "xtdrone_planning", "ego_planner.launch.py",
                                f"target_x:={target[0]:.2f}", f"target_y:={target[1]:.2f}", f"target_z:={target[2]:.2f}"],
                               stdout=ego_log, stderr=subprocess.STDOUT, start_new_session=True,
                               env={**os.environ, "PYTHONUNBUFFERED": "1", "RCUTILS_LOGGING_BUFFERED_STREAM": "0"})
        log(f"planner output: {ego_log.name}")

        def track():
            r = rel()
            d["min_z"] = min(d["min_z"], r[2])
            # distance from the straight line start -> goal in the horizontal plane
            gx, gy = goal[0], goal[1]
            d["side"] = max(d["side"], abs(gx * r[1] - gy * r[0]) / max(math.hypot(gx, gy), 1e-6))
        t0 = time.time()
        reached = wait(a.timeout, lambda: math.dist(rel(), goal) < 1.0, every=track)
        r = rel()
        log(f"{'ok  ' if reached else 'FAIL'} goal: final {r[0]:.1f},{r[1]:.1f},{r[2]:.1f}, "
            f"{math.dist(r, goal):.2f} m from it after {time.time() - t0:.0f} s")
        ok &= reached
        low = d["min_z"] > 0.1  # standing on the floor is 0; EGO may fly low over the mapped floor
        log(f"{'ok  ' if low else 'FAIL'} altitude: lowest {d['min_z']:.2f} m above the spawn point")
        ok &= low
        if a.detour > 0:
            around = d["side"] >= a.detour
            log(f"{'ok  ' if around else 'FAIL'} detour: up to {d['side']:.2f} m beside the straight line "
                f"(needs {a.detour:.2f} m to pass the obstacles)")
            ok &= around
        if d["bsplines"] is not None:
            planned = d["bsplines"] > 0
            log(f"{'ok  ' if planned else 'FAIL'} planner: {d['bsplines']} B-spline trajectories")
            ok &= planned
    finally:
        if ego:
            os.killpg(ego.pid, signal.SIGINT)
            try:
                ego.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(ego.pid, signal.SIGKILL)
        cmd.publish(String(data="AUTO.LAND"))
        wait(90, lambda: d.get("st") and not d["st"].armed)
        n.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
