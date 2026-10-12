#!/usr/bin/env python3
"""Chapter 41 check: does the world wind act on a hovering iris?

Requires: xtdrone.launch.py world:=empty vehicles:=iris communication:=basic, and
`ros2 run xtdrone_py wind_control`. Takes off to 3 m, hovers, measures tilt and drift;
sets a wind of --wind m/s along +x on /wind_xtdrone; measures again. With wind PX4 has
to lean into it (or the vehicle drifts).
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def log(m):
    print(f"[wind_test] {m}", flush=True)


class T(Node):
    def __init__(self):
        super().__init__("wind_test")
        self.state = self.home = self.gt = None
        self.create_subscription(State, "/iris_0/mavros/state", lambda m: setattr(self, "state", m),
                                 qos_profile_sensor_data)
        self.create_subscription(HomePosition, "/iris_0/mavros/home_position/home",
                                 lambda m: setattr(self, "home", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/iris_0/ground_truth/odom", lambda m: setattr(self, "gt", m), 1)
        self.cmd = self.create_publisher(String, "/xtdrone/iris_0/cmd", 3)
        self.vel = self.create_publisher(Twist, "/xtdrone/iris_0/cmd_vel_flu", 1)
        self.wind = self.create_publisher(Twist, "/wind_xtdrone", 1)

    def wait(self, s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return cond() if cond else True

    def tilt_and_pos(self):
        q = self.gt.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        p = self.gt.pose.pose.position
        return math.degrees(roll), math.degrees(pitch), (p.x, p.y, p.z)

    def measure(self, seconds):
        rolls, pitches, start = [], [], None
        end = time.time() + seconds
        while time.time() < end:
            self.wait(0.2)
            r, p, pos = self.tilt_and_pos()
            start = start or pos
            rolls.append(r)
            pitches.append(p)
        drift = math.dist(start[:2], pos[:2])
        return sum(rolls) / len(rolls), sum(pitches) / len(pitches), drift


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wind", type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    t = T()
    if not t.wait(120, lambda: t.state and t.state.connected and t.home and t.gt):
        log("FAIL: not connected")
        return 1
    up = lambda: t.vel.publish(Twist(linear=Vector3(z=1.0)))  # noqa: E731
    deadline = time.time() + 60
    while time.time() < deadline and not t.state.armed:
        t.cmd.publish(String(data="OFFBOARD"))
        t.wait(1, every=up)
        t.cmd.publish(String(data="ARM"))
        t.wait(2, lambda: t.state.armed, every=up)
    t.wait(30, lambda: t.gt.pose.pose.position.z > 3.0, every=up)
    t.cmd.publish(String(data="HOVER"))
    t.wait(5, every=lambda: t.vel.publish(Twist()))
    r0, p0, d0 = t.measure(8)
    log(f"no wind:   roll {r0:+.2f} deg, pitch {p0:+.2f} deg, drift {d0:.2f} m")
    w = Twist(linear=Vector3(x=a.wind))
    for _ in range(5):
        t.wind.publish(w)
        t.wait(0.2)
    t.wait(5)
    r1, p1, d1 = t.measure(8)
    log(f"wind {a.wind:g} m/s +x: roll {r1:+.2f} deg, pitch {p1:+.2f} deg, drift {d1:.2f} m")
    t.wind.publish(Twist())
    t.cmd.publish(String(data="AUTO.LAND"))
    t.wait(40, lambda: not t.state.armed)
    effect = abs(p1 - p0) > 1.0 or abs(r1 - r0) > 1.0 or d1 > d0 + 0.5
    log("wind acts on the vehicle" if effect else "no measurable wind effect on the vehicle")
    t.destroy_node()
    rclpy.try_shutdown()
    return 0 if effect else 1


if __name__ == "__main__":
    sys.exit(main())
