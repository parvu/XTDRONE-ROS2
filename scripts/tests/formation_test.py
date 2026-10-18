#!/usr/bin/env python3
"""Chapters 24/25 check: the ROS 2 leader/follower formation demo.

  formation_test.py                                   # chapter 25: 6 iris, simple followers
  formation_test.py --count 9 --controller consensus --avoid   # chapter 24

Requires: xtdrone.launch.py scenario:=formation_6 (or world:=empty vehicles:="iris*9")
ekf:=vision communication:=basic, and `ros2 run xtdrone_py get_local_pose iris N`
(shared position frame).

1. take off all vehicles to ~8 m (keyboard sequence i, b, t) and hover;
2. start `ros2 launch xtdrone_formation_demo formation.launch.py uav_count:=6`;
3. check the waiting (origin) formation, then two more formations (sent like the
   keyboard does in leader mode: /xtdrone/leader/cmd);
4. move the leader (/xtdrone/leader/cmd_vel_flu) and check the followers keep formation;
5. stop the demo and land.
Formation errors use the Gazebo ground truth (/iris_N/ground_truth/odom).
"""
import argparse
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
from scipy.optimize import linear_sum_assignment
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../ros2/fomation_demo"))
from xtdrone_formation_demo.formation_dict import formation_dict_6, formation_dict_9  # noqa: E402

FORMATIONS = {6: (formation_dict_6, ("T", "triangle")), 9: (formation_dict_9, ("cube", "pyramid"))}
N = 6
TAKEOFF_ALT = 8.0
TOL = 0.8  # m, per-follower position tolerance


# ArduPilot SITL with many vehicles runs at about a third of real time or less: wall-clock waits are scaled
SLOW = float(os.environ.get("XTDRONE_TEST_SLOW", "2"))


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
        end = time.time() + seconds * SLOW
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
        """Follower positions relative to the leader (3 x N-1), ground truth."""
        L = self.gt["iris_0"]
        return np.array([[self.gt[n].x - L.x, self.gt[n].y - L.y, self.gt[n].z - L.z] for n in self.names[1:]]).T


def formation_error(rel, target):
    """Largest follower error under the closest one-to-one assignment (the demo reassigns followers)."""
    cost = np.linalg.norm(rel[:, :, None] - target[:, None, :], axis=0)
    rows, cols = linear_sum_assignment(cost)
    return float(cost[rows, cols].max())


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
    global N
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=6, choices=sorted(FORMATIONS))
    ap.add_argument("--controller", default="simple", choices=("simple", "consensus"))
    ap.add_argument("--avoid", action="store_true")
    a = ap.parse_args()
    N = a.count
    formations, sequence = FORMATIONS[N]
    rclpy.init()
    f = Fleet()
    demo = None
    ok = True
    try:
        ready = lambda: all(f.state[n] and f.state[n].connected and f.home[n] and f.gt[n] for n in f.names)  # noqa
        if not f.wait(180, ready):
            log("FAIL: not all vehicles connected with home position")
            return 1
        log(f"all {N} vehicles connected")
        # ArduPilot reports a home position as soon as the EKF has an origin; arming then, before the estimator has
        # settled, gave wandering headings and bouncing altitudes. Let it settle first.
        f.wait(20)
        up = lambda: [f.vel[n].publish(Twist(linear=Twist().linear.__class__(z=1.0))) for n in f.names]  # noqa
        deadline = time.time() + 90 * SLOW
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
                                 "uav_type:=iris", f"uav_count:={N}", f"controller:={a.controller}",
                                 f"avoid:={str(a.avoid).lower()}"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        # the leader must be up (and subscribed) before formation commands are sent
        if not f.wait(60, lambda: f.leader_cmd.get_subscription_count() > 0):
            log("FAIL: formation leader did not start")
            return 1
        ok &= check(f, "waiting (origin)", formations["origin"], 60)
        for name in sequence:
            f.leader_cmd.publish(String(data=name))
            f.wait(1.0)
            ok &= check(f, name, formations[name], 90)
        # leader moves forward at 0.5 m/s for 10 s; followers keep the last formation
        start = (f.gt["iris_0"].x, f.gt["iris_0"].y)
        worst = 0.0
        end = time.time() + 10
        move = Twist()
        move.linear.x = 0.5
        while time.time() < end:
            f.leader_vel.publish(move)
            f.wait(0.1)
            worst = max(worst, formation_error(f.rel(), formations[sequence[-1]]))
        moved = math.dist(start, (f.gt["iris_0"].x, f.gt["iris_0"].y))
        f.leader_vel.publish(Twist())
        log(f"leader moved {moved:.1f} m; max follower error while moving {worst:.2f} m")
        # 0.5 m/s for 10 s wall time; with 9+ vehicles the sim runs below real time (RTF ~0.6)
        # consensus followers learn the leader's motion through chains of local leaders and lag more
        moving_tol = (2.5 if a.controller == "consensus" else 2.0) * TOL
        ok &= moved > 1.0 and worst < moving_tol
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
