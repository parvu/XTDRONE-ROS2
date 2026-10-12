#!/usr/bin/env python3
"""Chapter 25 check: the ROS 2 leader/follower formation demo with 6 iris.

Requires: xtdrone.launch.py scenario:=formation_6 ekf:=vision communication:=basic,
and `ros2 run xtdrone_py get_local_pose iris 6` (shared position frame).

1. take off all vehicles to ~8 m (keyboard sequence i, b, t) and hover;
2. start `ros2 launch xtdrone_formation_demo formation.launch.py uav_count:=6`;
3. check the waiting (origin) formation, then 'T' and 'triangle' (sent like the
   keyboard does in leader mode: /xtdrone/leader/cmd);
4. move the leader (/xtdrone/leader/cmd_vel_flu) and check the followers keep formation;
5. stop the demo and land.
Formation errors use the Gazebo ground truth (/iris_N/ground_truth/odom).
"""
import itertools
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../ros2/fomation_demo"))
from xtdrone_formation_demo.formation_dict import formation_dict_6  # noqa: E402

N = 6
TAKEOFF_ALT = 8.0
TOL = 0.8  # m, per-follower position tolerance


def log(m):
    print(f"[formation_test] {m}", flush=True)


class Fleet(Node):
    def __init__(self):
        super().__init__("formation_test")
        self.names = [f"iris_{i}" for i in range(N)]
        self.state = {n: None for n in self.names}
        self.home = {n: None for n in self.names}
        self.gt = {n: None for n in self.names}
        self.cmd = {n: self.create_publisher(String, f"/xtdrone/{n}/cmd", 3) for n in self.names}
        self.vel = {n: self.create_publisher(Twist, f"/xtdrone/{n}/cmd_vel_flu", 1) for n in self.names}
        self.leader_cmd = self.create_publisher(String, "/xtdrone/leader/cmd", 3)
        self.leader_vel = self.create_publisher(Twist, "/xtdrone/leader/cmd_vel_flu", 1)
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

    def z(self, n):
        return self.gt[n].z if self.gt[n] else float("nan")

    def rel(self):
        """Follower positions relative to the leader (3 x 5), ground truth."""
        L = self.gt["iris_0"]
        return np.array([[self.gt[n].x - L.x, self.gt[n].y - L.y, self.gt[n].z - L.z] for n in self.names[1:]]).T


def formation_error(rel, target):
    """Largest follower error under the best one-to-one assignment (the demo reassigns followers)."""
    best = float("inf")
    for perm in itertools.permutations(range(target.shape[1])):
        err = max(np.linalg.norm(rel[:, i] - target[:, j]) for i, j in enumerate(perm))
        best = min(best, err)
    return best


def check(fleet, name, target, timeout):
    ok = fleet.wait(timeout, lambda: formation_error(fleet.rel(), target) < TOL)
    err = formation_error(fleet.rel(), target)
    log(f"{'ok  ' if ok else 'FAIL'} formation '{name}': max follower error {err:.2f} m")
    if not ok:
        rel = fleet.rel()
        for i, n in enumerate(fleet.names[1:]):
            log(f"   {n}: relative to leader ({rel[0, i]:.1f}, {rel[1, i]:.1f}, {rel[2, i]:.1f}), "
                f"absolute z {fleet.z(n):.1f} m, mode {fleet.state[n].mode}, armed {fleet.state[n].armed}")
        log("   targets: " + ", ".join(f"({t[0]:.0f}, {t[1]:.0f}, {t[2]:.0f})" for t in target.T))
    return ok


def main():
    rclpy.init()
    f = Fleet()
    demo = None
    ok = True
    try:
        ready = lambda: all(f.state[n] and f.state[n].connected and f.home[n] and f.gt[n] for n in f.names)  # noqa
        if not f.wait(180, ready):
            log("FAIL: not all vehicles connected with home position")
            return 1
        log("all 6 vehicles connected")
        up = lambda: [f.vel[n].publish(Twist(linear=Twist().linear.__class__(z=1.0))) for n in f.names]  # noqa
        deadline = time.time() + 90
        while time.time() < deadline and not all(f.state[n].armed for n in f.names):
            for n in f.names:
                if not f.state[n].armed:
                    f.cmd[n].publish(String(data="OFFBOARD"))
            f.wait(1.0, every=up)
            for n in f.names:
                if not f.state[n].armed:
                    f.cmd[n].publish(String(data="ARM"))
            f.wait(2.0, lambda: all(f.state[n].armed for n in f.names), every=up)
        if not f.wait(60, lambda: all(f.z(n) > TAKEOFF_ALT for n in f.names), every=up):
            log("FAIL: takeoff, altitudes " + ", ".join(f"{n}={f.z(n):.1f}" for n in f.names))
            return 1
        stop = lambda: [f.vel[n].publish(Twist()) for n in f.names]  # noqa
        for n in f.names:
            f.cmd[n].publish(String(data="HOVER"))
        f.wait(3.0, every=stop)
        log("all vehicles hover at " + ", ".join(f"{f.z(n):.1f}" for n in f.names) + " m")

        demo = subprocess.Popen(["ros2", "launch", "xtdrone_formation_demo", "formation.launch.py",
                                 "uav_type:=iris", "uav_count:=6"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        # the leader must be up (and subscribed) before formation commands are sent
        if not f.wait(60, lambda: f.leader_cmd.get_subscription_count() > 0):
            log("FAIL: formation leader did not start")
            return 1
        ok &= check(f, "waiting (origin)", formation_dict_6["origin"], 60)
        for name in ("T", "triangle"):
            f.leader_cmd.publish(String(data=name))
            f.wait(1.0)
            ok &= check(f, name, formation_dict_6[name], 90)
        # leader moves forward at 0.5 m/s for 10 s; followers keep the triangle
        start = (f.gt["iris_0"].x, f.gt["iris_0"].y)
        worst = 0.0
        end = time.time() + 10
        move = Twist()
        move.linear.x = 0.5
        while time.time() < end:
            f.leader_vel.publish(move)
            f.wait(0.1)
            worst = max(worst, formation_error(f.rel(), formation_dict_6["triangle"]))
        moved = math.dist(start, (f.gt["iris_0"].x, f.gt["iris_0"].y))
        f.leader_vel.publish(Twist())
        log(f"leader moved {moved:.1f} m; max follower error while moving {worst:.2f} m")
        ok &= moved > 2.0 and worst < 2 * TOL
    finally:
        if demo:
            os.killpg(demo.pid, signal.SIGINT)
            try:
                demo.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(demo.pid, signal.SIGKILL)
        for n in f.names:
            f.cmd[n].publish(String(data="AUTO.LAND"))
        landed = f.wait(120, lambda: all(f.state[n] and not f.state[n].armed for n in f.names))
        log("landed and disarmed" if landed else "vehicles still armed after landing timeout")
        f.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
