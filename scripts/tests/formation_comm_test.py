#!/usr/bin/env python3
"""Chapter 27 check: formation control with the SINR communication model, 20 iris.

Requires: xtdrone.launch.py scenario:=formation_comm_20 (no communication nodes; the
formation controllers command MAVROS directly).

1. start `ros2 launch xtdrone_formation_demo formation_communication.launch.py`;
2. AUTO.TAKEOFF (sent like the keyboard in leader mode on /xtdrone/leader/cmd): all reach ~0.6 m;
3. FORM_1 for --form-time s: the leader moves along +y. UAVs 1-12 receive the leader group over
   the modelled channel and follow it; UAV 13 only hears UAVs 14-16 (higher ids), so it has no
   local leader and holds its position;
4. HOVER, AUTO.LAND.
Positions are the MAVROS local positions (each relative to the vehicle's spawn point; only
displacements are compared). Subscribing to 20 ground-truth streams would load this machine more.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

N = 20
FOLLOWERS = range(1, 13)
ISOLATED = 13


# ArduPilot SITL with many vehicles runs at about a third of real time or less: wall-clock waits are scaled
SLOW = float(os.environ.get("XTDRONE_TEST_SLOW", "2"))


def log(m):
    print(f"[formation_comm_test] {m}", flush=True)


class Fleet(Node):
    def __init__(self):
        super().__init__("formation_comm_test")
        self.names = [f"iris_{i}" for i in range(N)]
        self.state = {n: None for n in self.names}
        self.gt = {n: None for n in self.names}
        self.comm = None
        self.seen = set()  # vehicles whose MAVROS reported connected at least once (it flaps under load)
        self.leader_cmd = self.create_publisher(String, "/xtdrone/leader/cmd", 3)
        for n in self.names:
            self.create_subscription(State, f"/{n}/mavros/state", lambda m, n=n: self.on_state(n, m),
                                     qos_profile_sensor_data)
            self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose",
                                     lambda m, n=n: self.gt.__setitem__(n, m.pose.position), qos_profile_sensor_data)
        try:
            from xtdrone_msgs.msg import CommVerify
            self.create_subscription(CommVerify, "/communication_verify", lambda m: setattr(self, "comm", m), 10)
        except ImportError:
            pass

    def on_state(self, n, m):
        self.state[n] = m
        if m.connected:
            self.seen.add(n)

    def wait(self, seconds, cond=None, every=None):
        end = time.time() + seconds * SLOW
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return cond() if cond else True

    def send(self, cmd, repeat=5):
        for _ in range(repeat):
            self.leader_cmd.publish(String(data=cmd))
            self.wait(0.2)

    def keep_sending(self, cmd, seconds, cond=None):
        """Re-send cmd every second (controllers ignore repeats of their current command): under
        load, discovery can finish on the publisher side before a subscriber is ready."""
        last = [0.0]

        def every():
            if time.time() - last[0] > 1.0:
                self.leader_cmd.publish(String(data=cmd))
                last[0] = time.time()
        return self.wait(seconds, cond, every)

    def xy(self, i):
        p = self.gt[self.names[i]]
        return p.x, p.y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--form-time", type=float, default=30.0, help="wall seconds in FORM_1")
    a = ap.parse_args()
    rclpy.init()
    f = Fleet()
    demo = None
    ok = True
    try:
        if not f.wait(300, lambda: all(n in f.seen and f.gt[n] for n in f.names)):
            missing = [n for n in f.names if not (n in f.seen and f.gt[n])]
            log(f"FAIL: not connected: {missing}")
            return 1
        log("all 20 vehicles connected")
        log_dir = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
        demo_log = open(os.path.join(log_dir, "ch27_formation_comm_demo.log"), "w")
        demo = subprocess.Popen(["ros2", "launch", "xtdrone_formation_demo", "formation_communication.launch.py"],
                                stdout=demo_log, stderr=subprocess.STDOUT, start_new_session=True)
        if not f.wait(90, lambda: f.leader_cmd.get_subscription_count() >= N and f.comm is not None):
            log(f"FAIL: controllers not up ({f.leader_cmd.get_subscription_count()} subscribers, "
                f"communication model {'up' if f.comm else 'down'})")
            return 1
        links = list(f.comm.trans_matrix[1].trans_matrix)
        log(f"communication model up; UAV 1 receives {sum(links)} of {len(links)} transmitters")
        if not f.keep_sending("AUTO.TAKEOFF", 240, lambda: all(f.gt[n].z > 0.4 for n in f.names)):
            low = [n for n in f.names if f.gt[n].z <= 0.4]
            log(f"FAIL: takeoff, still low: {low}")
            return 1
        f.wait(5.0)
        log("all 20 airborne at " + ", ".join(f"{f.gt[n].z:.1f}" for n in f.names[:5]) + ", ... m")
        start = [f.xy(i) for i in range(N)]
        f.keep_sending("FORM_1", a.form_time)
        f.keep_sending("HOVER", 5.0)
        end = [f.xy(i) for i in range(N)]
        moved = [(e[0] - s[0], e[1] - s[1]) for s, e in zip(start, end)]
        lead = moved[0]
        log(f"leader moved ({lead[0]:+.1f}, {lead[1]:+.1f}) m")
        ok &= lead[1] > 2.0
        worst = max(math.dist(moved[i], lead) for i in FOLLOWERS)
        log(f"{'ok  ' if worst < 1.5 else 'FAIL'} UAVs 1-12 follow the leader: largest deviation "
            f"from its displacement {worst:.2f} m")
        ok &= worst < 1.5
        iso = math.hypot(*moved[ISOLATED])
        log(f"{'ok  ' if iso < 0.5 else 'FAIL'} UAV 13 (no lower-id neighbour on the channel) holds: "
            f"moved {iso:.2f} m")
        ok &= iso < 0.5
    finally:
        landed = f.keep_sending("AUTO.LAND", 150, lambda: all(f.state[n] and not f.state[n].armed for n in f.names))
        log("landed and disarmed" if landed else "some vehicles still armed after the landing timeout")
        if demo:
            os.killpg(demo.pid, signal.SIGINT)
            try:
                demo.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(demo.pid, signal.SIGKILL)
        f.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
