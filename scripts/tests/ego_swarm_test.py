#!/usr/bin/env python3
"""Chapter 22 check: EGO-Planner-Swarm flies the iris of the ego_swarm scenario across the obstacle field.

Requires: xtdrone.launch.py scenario:=ego_swarm ekf:=vision communication:=basic and
`ros2 run xtdrone_py get_local_pose iris <count>` (ground truth as the pose source: one world frame).

1. take off all vehicles to ~0.8 m and hold position;
2. start `ros2 launch xtdrone_planning ego_swarm.launch.py count:=<count>` (XTDrone's goals: x = 9, 0.7 m);
3. check that every vehicle reaches its goal (ground truth within 1 m), that no two vehicles come closer than
   0.5 m, and that none comes down to the floor;
4. land.
"""
import argparse
import itertools
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

GOALS = {4: [(9, 5, 0.7), (9, 1.2, 0.7), (9, -1.2, 0.7), (9, -4, 0.7)],
         8: [(9, 7, 0.7), (9, 5, 0.7), (9, 3, 0.7), (9, 1, 0.7), (9, -1, 0.7), (9, -3, 0.7), (9, -5, 0.7),
             (9, -7, 0.7)]}


def log(m):
    print(f"[ego_swarm_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=4)
    ap.add_argument("--timeout", type=float, default=600.0, help="wall seconds for the flight")
    a = ap.parse_args()
    names = [f"iris_{i}" for i in range(a.count)]
    goals = GOALS[a.count]
    rclpy.init()
    n = rclpy.create_node("ego_swarm_test")
    st, pose, gt = {}, {}, {}
    for v in names:
        n.create_subscription(State, f"/{v}/mavros/state", lambda m, v=v: st.__setitem__(v, m), qos_profile_sensor_data)
        n.create_subscription(PoseStamped, f"/{v}/mavros/local_position/pose", lambda m, v=v: pose.__setitem__(v, m.pose),
                              qos_profile_sensor_data)
        n.create_subscription(Odometry, f"/{v}/ground_truth/odom", lambda m, v=v: gt.__setitem__(v, m.pose.pose.position), 1)
    cmd = {v: n.create_publisher(String, f"/xtdrone/{v}/cmd", 3) for v in names}
    vel = {v: n.create_publisher(Twist, f"/xtdrone/{v}/cmd_vel_flu", 1) for v in names}
    hold = {v: n.create_publisher(Pose, f"/xtdrone/{v}/cmd_pose_enu", 1) for v in names}
    sent = {}  # last position command of each vehicle (from ego_pos_cmd), for the trace
    for v in names:
        n.create_subscription(Pose, f"/xtdrone/{v}/cmd_pose_enu", lambda m, v=v: sent.__setitem__(v, m.position), 10)

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
    ego = None
    try:
        if not wait(300, lambda: all(v in st and st[v].connected and v in pose and v in gt for v in names)):
            log(f"FAIL: not ready: {[v for v in names if not (v in st and st[v].connected and v in pose)]}")
            return 1
        ground = {v: gt[v].z for v in names}
        up = lambda: [vel[v].publish(Twist(linear=Vector3(z=0.6))) for v in names]  # noqa: E731
        deadline = time.time() + 120
        while time.time() < deadline and not all(st[v].armed for v in names):
            for v in names:
                cmd[v].publish(String(data="OFFBOARD"))
            wait(1, every=up)
            for v in names:
                cmd[v].publish(String(data="ARM"))
            wait(2, lambda: all(st[v].armed for v in names), every=up)
        if not wait(90, lambda: all(pose[v].position.z > 0.8 for v in names), every=up):
            log(f"FAIL: takeoff ({', '.join(f'{v} {pose[v].position.z:.2f}' for v in names)})")
            return 1
        here = {}
        for v in names:
            p = Pose()
            p.position.x, p.position.y, p.position.z = pose[v].position.x, pose[v].position.y, 0.8
            p.orientation = pose[v].orientation
            here[v] = p
        wait(3, every=lambda: [hold[v].publish(here[v]) for v in names])
        log("all airborne; starting EGO-Planner-Swarm")
        logs = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
        os.makedirs(logs, exist_ok=True)
        ego_log = open(os.path.join(logs, "ego_swarm_launch.log"), "w")
        ego = subprocess.Popen(["ros2", "launch", "xtdrone_planning", "ego_swarm.launch.py", f"count:={a.count}"],
                               stdout=ego_log, stderr=subprocess.STDOUT, start_new_session=True,
                               env={**os.environ, "PYTHONUNBUFFERED": "1"})
        d = {"close": 1e9, "pair": None, "low": 1e9, "low_at": ""}
        t_start = time.time()

        trace = open(os.path.join(logs, "ego_swarm_trace.csv"), "w")
        trace.write("t," + ",".join(f"{v}_x,{v}_y,{v}_z,{v}_cmd_z" for v in names) + "\n")
        last = [0.0]

        def track():
            if time.time() - last[0] > 0.5:  # positions for the planner log (ego_swarm_trace.csv)
                last[0] = time.time()
                trace.write(f"{time.time() - t_start:.1f}," +
                            ",".join(f"{gt[v].x:.2f},{gt[v].y:.2f},{gt[v].z:.2f},"
                                     f"{sent[v].z if v in sent else float('nan'):.2f}" for v in names) + "\n")
            track_pairs()

        def track_pairs():
            for u, w in itertools.combinations(names, 2):
                dist = math.dist((gt[u].x, gt[u].y, gt[u].z), (gt[w].x, gt[w].y, gt[w].z))
                if dist < d["close"]:
                    d["close"], d["pair"] = dist, (u, w)
            for v in names:
                if gt[v].z - ground[v] < d["low"]:
                    d["low"] = gt[v].z - ground[v]
                    d["low_at"] = (f"{v} at ({gt[v].x:.1f}, {gt[v].y:.1f}, {gt[v].z:.2f}), "
                                   f"{time.time() - t_start:.0f} s after the start")

        def err(i, v):
            return math.dist((pose[v].position.x, pose[v].position.y), goals[i][:2])
        t0 = time.time()
        arrived = wait(a.timeout, lambda: all(err(i, v) < 1.0 for i, v in enumerate(names)), every=track)
        for i, v in enumerate(names):
            log(f"{'ok  ' if err(i, v) < 1.0 else 'FAIL'} {v}: at ({gt[v].x:.1f}, {gt[v].y:.1f}, {gt[v].z:.1f}), "
                f"{err(i, v):.2f} m from its goal {goals[i][:2]}")
        log(f"{'ok  ' if arrived else 'FAIL'} all at their goals after {time.time() - t0:.0f} s")
        ok &= arrived
        apart = d["close"] > 0.5
        log(f"{'ok  ' if apart else 'FAIL'} closest approach {d['close']:.2f} m ({d['pair'][0]}, {d['pair'][1]})")
        ok &= apart
        high = d["low"] > 0.1
        log(f"{'ok  ' if high else 'FAIL'} lowest point {d['low']:.2f} m above the start height: {d['low_at']}")
        ok &= high
    finally:
        if ego:
            os.killpg(ego.pid, signal.SIGINT)
            try:
                ego.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(ego.pid, signal.SIGKILL)
        for v in names:
            cmd[v].publish(String(data="AUTO.LAND"))
        wait(90, lambda: all(v in st and not st[v].armed for v in names))
        n.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
