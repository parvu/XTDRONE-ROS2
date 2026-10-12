#!/usr/bin/env python3
"""Chapter 11 check: drive the two CAT Vehicle UGVs like ugv_keyboard_control does.

Requires: xtdrone.launch.py scenario:=outdoor3_ugv.
Both UGVs: 2 m/s straight for 6 s (must move > 4 m along their heading), then 2 m/s with a
0.3 rad steering angle (must turn by > 15 deg), then stop. Also checks the sensor topics.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2

NAMES = ["ugv_0", "ugv_1"]


def log(m):
    print(f"[ugv_test] {m}", flush=True)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__("ugv_test")
        self.odom = {}
        self.seen = set()
        self.cmd = {n: self.create_publisher(Twist, f"/{n}/cmd_vel", 1) for n in NAMES}
        for n in NAMES:
            self.create_subscription(Odometry, f"/{n}/ground_truth/odom", lambda m, n=n: self.odom.__setitem__(n, m), 1)
        for topic, typ in (("/ugv_0/front_laser_points", LaserScan), ("/ugv_0/lidar_points", PointCloud2),
                           ("/ugv_0/camera_left/image_raw_left", Image), ("/ugv_0/triclops/triclops/left/image", Image)):
            self.create_subscription(typ, topic, lambda m, t=topic: self.seen.add(t), qos_profile_sensor_data)

    def wait(self, s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return bool(cond and cond())

    def pose(self, n):
        o = self.odom[n].pose.pose
        return o.position.x, o.position.y, yaw(o.orientation)


def main():
    rclpy.init()
    t = T()
    ok = True
    if not t.wait(120, lambda: all(n in t.odom for n in NAMES)):
        log("FAIL: no ground truth")
        return 1
    t.wait(5)
    start = {n: t.pose(n) for n in NAMES}

    def drive(v, steer):
        return lambda: [t.cmd[n].publish(Twist(linear=Twist().linear.__class__(x=v),
                                               angular=Twist().angular.__class__(z=steer))) for n in NAMES]
    t.wait(6, every=drive(2.0, 0.0))
    for n in NAMES:
        x0, y0, h0 = start[n]
        x1, y1, _ = t.pose(n)
        along = (x1 - x0) * math.cos(h0) + (y1 - y0) * math.sin(h0)
        good = along > 4.0
        ok &= good
        log(f"{'ok  ' if good else 'FAIL'} {n} straight: {along:.1f} m along its heading")
    # turn: accumulate the yaw change while steering left
    last = {n: t.pose(n)[2] for n in NAMES}
    turned = {n: 0.0 for n in NAMES}
    speed = {n: 0.0 for n in NAMES}

    def steer_and_track():
        drive(2.0, 0.3)()
        for n in NAMES:
            h = t.pose(n)[2]
            turned[n] += math.atan2(math.sin(h - last[n]), math.cos(h - last[n]))
            last[n] = h
            v = t.odom[n].twist.twist.linear
            speed[n] = max(speed[n], math.hypot(v.x, v.y))
    t.wait(6, every=steer_and_track)
    t.wait(3, every=drive(0.0, 0.0))
    for n in NAMES:
        deg = math.degrees(turned[n])
        good = deg > 15.0
        ok &= good
        log(f"{'ok  ' if good else 'FAIL'} {n} left steering: turned {deg:.0f} deg (max speed {speed[n]:.1f} m/s)")
    t.wait(20, lambda: len(t.seen) == 4)
    log(f"{'ok  ' if len(t.seen) == 4 else 'FAIL'} sensor topics: {sorted(t.seen)}")
    ok &= len(t.seen) == 4
    t.destroy_node()
    rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
