#!/usr/bin/env python3
"""Chapters 17/18 (and 14/19) check: a vehicle flies on SLAM / odometry only (no GPS, no ground truth).

Requires: xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic and
`ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=<slam>` (pose -> vision_pose).

1. wait until PX4 has a local position from the SLAM pose and a map is published;
2. take off to ~1.5 m, fly a 2 m x 2 m square with position commands in the SLAM frame;
3. compare the SLAM/EKF trajectory with the Gazebo ground truth (both relative to the start);
4. land.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

V = "iris_0"


def log(m):
    print(f"[laser_slam_test] {m}", flush=True)


class T(Node):
    def __init__(self):
        super().__init__("laser_slam_test")
        self.state = self.pose = self.gt = self.map = self.vision = None
        self.create_subscription(State, f"/{V}/mavros/state", lambda m: setattr(self, "state", m), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"/{V}/mavros/local_position/pose", lambda m: setattr(self, "pose", m.pose),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"/{V}/mavros/vision_pose/pose", lambda m: setattr(self, "vision", m.pose), 1)
        self.create_subscription(Odometry, f"/{V}/ground_truth/odom", lambda m: setattr(self, "gt", m.pose.pose), 1)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), latched)
        self.cmd = self.create_publisher(String, f"/xtdrone/{V}/cmd", 3)
        self.vel = self.create_publisher(Twist, f"/xtdrone/{V}/cmd_vel_flu", 1)
        self.target = self.create_publisher(Pose, f"/xtdrone/{V}/cmd_pose_enu", 1)

    def wait(self, s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return bool(cond and cond())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam", default="slam_toolbox",
                    help="slam_toolbox, cartographer (2D, also check /map), kiss_icp, vins (odometry only)")
    ap.add_argument("--tolerance", type=float, default=0.5, help="largest SLAM vs ground truth difference (m)")
    a = ap.parse_args()
    rclpy.init()
    t = T()
    ok = True
    try:
        if not t.wait(180, lambda: t.state and t.state.connected and t.gt and t.vision):
            log("FAIL: no MAVROS connection / ground truth / SLAM pose")
            return 1
        if not t.wait(120, lambda: t.pose is not None):
            log("FAIL: PX4 has no local position from the SLAM pose")
            return 1
        t.wait(10)
        log(f"{a.slam} pose reaches PX4: local position ({t.pose.position.x:.2f}, {t.pose.position.y:.2f})")
        s0 = (t.pose.position.x, t.pose.position.y)
        g0 = (t.gt.position.x, t.gt.position.y)
        up = lambda: t.vel.publish(Twist(linear=Twist().linear.__class__(z=0.6)))  # noqa: E731
        deadline = time.time() + 60
        while time.time() < deadline and not t.state.armed:
            t.cmd.publish(String(data="OFFBOARD"))
            t.wait(1, every=up)
            t.cmd.publish(String(data="ARM"))
            t.wait(2, lambda: t.state.armed, every=up)
        if not t.wait(30, lambda: t.gt.position.z > 1.3, every=up):
            log(f"FAIL: takeoff (z {t.gt.position.z:.2f})")
            return 1
        z = t.pose.position.z
        worst = 0.0
        corners = [(2, 0), (2, 2), (0, 2), (0, 0)]
        for cx, cy in corners:
            goal = Pose()
            goal.position.x, goal.position.y, goal.position.z = s0[0] + cx, s0[1] + cy, z
            goal.orientation.w = 1.0
            sent = [0.0]

            def track():
                nonlocal worst
                if time.time() - sent[0] > 1.0:  # re-send: one message can be lost right after takeoff
                    t.target.publish(goal)
                    sent[0] = time.time()
                if t.pose and t.gt:
                    ds = (t.pose.position.x - s0[0], t.pose.position.y - s0[1])
                    dg = (t.gt.position.x - g0[0], t.gt.position.y - g0[1])
                    worst = max(worst, math.dist(ds, dg))
            reached = t.wait(30, lambda: math.dist((t.pose.position.x, t.pose.position.y),
                                                   (goal.position.x, goal.position.y)) < 0.25, every=track)
            dg = (t.gt.position.x - g0[0], t.gt.position.y - g0[1])
            log(f"{'ok  ' if reached else 'FAIL'} corner ({cx}, {cy}): ground truth ({dg[0]:.2f}, {dg[1]:.2f})")
            ok &= reached
        good = worst < a.tolerance  # slam_toolbox scan matching at 5 cm cells: about +-0.3-0.4 m
        log(f"{'ok  ' if good else 'FAIL'} SLAM trajectory vs ground truth: largest difference {worst:.2f} m")
        ok &= good
        if a.slam not in ("slam_toolbox", "cartographer"):
            log("PASS" if ok else "FAIL")
            return 0 if ok else 1
        has_map = t.wait(10, lambda: t.map is not None)
        if has_map:
            known = sum(1 for c in t.map.data if c >= 0)
            log(f"ok   map {t.map.info.width} x {t.map.info.height} cells at {t.map.info.resolution} m, {known} known")
        else:
            log("FAIL: no /map")
        ok &= has_map
    finally:
        t.cmd.publish(String(data="AUTO.LAND"))
        t.wait(40, lambda: t.state and not t.state.armed)
        t.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
