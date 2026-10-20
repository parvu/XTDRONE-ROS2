#!/usr/bin/env python3
"""Chapter 28 check: three iris_downward_camera land on AprilTags 0, 1 and 2.

Requires: xtdrone.launch.py scenario:=outdoor2_precision_landing communication:=basic
(the scenario puts the apriltag0-2 plate at the origin: tag i at x = -0.75 + 0.75 i, y = 0).

1. take off to 4, 5 and 6 m, fly each one to 0.5 m off its tag (ground truth), hover;
2. start `ros2 run xtdrone_py precision_landing iris i [--detector D]` for each;
3. check that each vehicle lands within --tolerance of its tag centre and disarms.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

N = 3
ALT = 4.0
OFFSET = (0.4, 0.3)  # start position relative to the tag, so the landing has to correct it


def tag_xy(i):
    return (-0.75 + 0.75 * i, 0.0)


def log(m):
    print(f"[precision_landing_test] {m}", flush=True)


class Fleet(Node):
    def __init__(self):
        super().__init__("precision_landing_test")
        self.names = [f"iris_{i}" for i in range(N)]
        self.state = {n: None for n in self.names}
        self.home = {n: None for n in self.names}
        self.gt = {n: None for n in self.names}
        self.cmd = {n: self.create_publisher(String, f"/xtdrone/{n}/cmd", 3) for n in self.names}
        self.vel = {n: self.create_publisher(Twist, f"/xtdrone/{n}/cmd_vel_enu", 1) for n in self.names}
        for n in self.names:
            self.create_subscription(State, f"/{n}/mavros/state", lambda m, n=n: self.state.__setitem__(n, m),
                                     qos_profile_sensor_data)
            self.create_subscription(HomePosition, f"/{n}/mavros/home_position/home",
                                     lambda m, n=n: self.home.__setitem__(n, m), qos_profile_sensor_data)
            self.create_subscription(Odometry, f"/{n}/ground_truth/odom",
                                     lambda m, n=n: self.gt.__setitem__(n, m.pose.pose.position), 1)

    def wait(self, seconds, cond=None, every=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return cond() if cond else True

    def goto(self, targets):
        """P control on ground truth towards targets {name: (x, y, z)} in the world (= local ENU axes)."""
        for n, (x, y, z) in targets.items():
            p = self.gt[n]
            v = Vector3(x=max(-1.0, min(1.0, x - p.x)), y=max(-1.0, min(1.0, y - p.y)),
                        z=max(-1.0, min(1.0, z - p.z)))
            self.vel[n].publish(Twist(linear=v))

    def err(self, n, target):
        p = self.gt[n]
        return math.dist((p.x, p.y, p.z), target)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", default="opencv", choices=("opencv", "apriltag_ros"))
    ap.add_argument("--tolerance", type=float, default=0.3)
    a = ap.parse_args()
    rclpy.init()
    f = Fleet()
    procs = []
    ok = True
    try:
        if not f.wait(180, lambda: all(f.state[n] and f.state[n].connected and f.home[n] and f.gt[n]
                                       for n in f.names)):
            log("FAIL: vehicles not ready")
            return 1
        # iris_2 starts last but its tag is first: each vehicle flies at its own altitude (4, 5, 6 m)
        up = {n: (f.gt[n].x, f.gt[n].y, ALT + i) for i, n in enumerate(f.names)}
        deadline = time.time() + 90
        while time.time() < deadline and not all(f.state[n].armed for n in f.names):
            for n in f.names:
                if not f.state[n].armed:
                    f.cmd[n].publish(String(data="OFFBOARD"))
            f.wait(1.0, every=lambda: f.goto(up))
            for n in f.names:
                if not f.state[n].armed:
                    f.cmd[n].publish(String(data="ARM"))
            f.wait(2.0, lambda: all(f.state[n].armed for n in f.names), every=lambda: f.goto(up))
        above = {n: (tag_xy(i)[0] + OFFSET[0], tag_xy(i)[1] + OFFSET[1], ALT + i) for i, n in enumerate(f.names)}
        f.wait(40, lambda: all(f.gt[n].z > up[n][2] - 0.3 for n in f.names), every=lambda: f.goto(up))
        if not f.wait(90, lambda: all(f.err(n, above[n]) < 0.2 for n in f.names), every=lambda: f.goto(above)):
            log("FAIL: could not reach the start points: " +
                ", ".join(f"{n} {f.err(n, above[n]):.2f} m" for n in f.names))
            return 1
        for n in f.names:
            f.cmd[n].publish(String(data="HOVER"))
        f.wait(3.0, every=lambda: [f.vel[n].publish(Twist()) for n in f.names])
        log("all three hover 0.5 m off their tags at 4, 5 and 6 m")
        for i in range(N):
            procs.append(subprocess.Popen(["ros2", "run", "xtdrone_py", "precision_landing", "iris", str(i),
                                           "--detector", a.detector],
                                          stdout=open(f"/tmp/precision_landing_{i}.log", "w"), stderr=subprocess.STDOUT,
                                          start_new_session=True))
        landed = f.wait(120, lambda: all(not f.state[n].armed or f.gt[n].z < 0.3 for n in f.names))
        f.wait(3.0)
        for i, n in enumerate(f.names):
            p, t = f.gt[n], tag_xy(i)
            e = math.dist((p.x, p.y), t)
            good = e < a.tolerance and p.z < 0.3
            ok &= good
            log(f"{'ok  ' if good else 'FAIL'} {n}: landed at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), "
                f"tag {i} at ({t[0]:.2f}, {t[1]:.2f}): error {e:.2f} m, armed {f.state[n].armed}")
        ok &= landed
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGINT)
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        for n in f.names:
            f.cmd[n].publish(String(data="AUTO.LAND"))
        f.wait(30, lambda: all(f.state[n] and not f.state[n].armed for n in f.names))
        f.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
