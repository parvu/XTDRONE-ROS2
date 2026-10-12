#!/usr/bin/env python3
"""Chapter 16 check: RTAB-Map builds a dense map from the iris_realsense_camera.

Requires: xtdrone.launch.py world:=indoor1 vehicles:=iris_realsense_camera communication:=basic and
`ros2 launch xtdrone_slam rtabmap_rgbd.launch.py`.

Takes off, turns a full circle slowly, flies 1.5 m forward; checks that the odometry RTAB-Map uses
follows the ground truth (displacement within 0.5 m) and that /rtabmap/cloud_map has points.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

V = "iris_0"


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def log(m):
    print(f"[rtabmap_test] {m}", flush=True)


class T(Node):
    def __init__(self):
        super().__init__("rtabmap_test")
        self.state = self.gt = self.odom = self.cloud = None
        self.create_subscription(State, f"/{V}/mavros/state", lambda m: setattr(self, "state", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{V}/ground_truth/odom", lambda m: setattr(self, "gt", m.pose.pose), 1)
        # the odometry RTAB-Map uses: its own RGB-D odometry (odom:=visual) or the relayed one
        self.create_subscription(Odometry, "/rtabmap/odom", lambda m: setattr(self, "odom", m.pose.pose), 10)
        self.create_subscription(Odometry, "/xtdrone/odom", lambda m: setattr(self, "odom", m.pose.pose), 10)
        for qos in (QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL), 1):
            self.create_subscription(PointCloud2, "/rtabmap/cloud_map", lambda m: setattr(self, "cloud", m), qos)
        self.cmd = self.create_publisher(String, f"/xtdrone/{V}/cmd", 3)
        self.vel = self.create_publisher(Twist, f"/xtdrone/{V}/cmd_vel_flu", 1)

    def wait(self, s, cond=None, every=None):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return bool(cond and cond())

    def fly(self, vx=0.0, vz=0.0, wz=0.0):
        return lambda: self.vel.publish(Twist(linear=Vector3(x=vx, z=vz), angular=Vector3(z=wz)))


def main():
    rclpy.init()
    t = T()
    ok = True
    try:
        if not t.wait(180, lambda: t.state and t.state.connected and t.gt and t.odom):
            log("FAIL: no MAVROS / ground truth / RTAB-Map odometry")
            return 1
        t.wait(10)
        deadline = time.time() + 60
        while time.time() < deadline and not t.state.armed:
            t.cmd.publish(String(data="OFFBOARD"))
            t.wait(1, every=t.fly(vz=0.5))
            t.cmd.publish(String(data="ARM"))
            t.wait(2, lambda: t.state.armed, every=t.fly(vz=0.5))
        # the RGB-D camera is rendered on the CPU without a GPU: the simulation can run at ~0.2-0.3 of real time
        if not t.wait(120, lambda: t.gt.position.z > 1.0, every=t.fly(vz=0.5)):
            log("FAIL: takeoff")
            return 1
        t.wait(3, every=t.fly())
        o0 = (t.odom.position.x, t.odom.position.y)
        g0 = (t.gt.position.x, t.gt.position.y)
        # full turn, then 1.5 m forward (ground-truth conditions: the sim may run well below real time)
        turned = [0.0, yaw(t.gt.orientation)]

        def turn():
            t.fly(wz=0.3)()
            y = yaw(t.gt.orientation)
            turned[0] += math.atan2(math.sin(y - turned[1]), math.cos(y - turned[1]))
            turned[1] = y
        t.wait(180, lambda: abs(turned[0]) >= 2 * math.pi, every=turn)
        t.wait(2, every=t.fly())
        t.wait(120, lambda: math.dist((t.gt.position.x, t.gt.position.y), g0) > 1.5, every=t.fly(vx=0.25))
        t.wait(3, every=t.fly())
        do = math.dist((t.odom.position.x, t.odom.position.y), o0)
        dg = math.dist((t.gt.position.x, t.gt.position.y), g0)
        good = abs(do - dg) < 0.5 and dg > 1.0
        log(f"{'ok  ' if good else 'FAIL'} odometry moved {do:.2f} m, ground truth {dg:.2f} m")
        ok &= good
        t.wait(60, lambda: t.cloud is not None and t.cloud.width * t.cloud.height > 10000)
        n = t.cloud.width * t.cloud.height if t.cloud else 0
        good = n > 10000
        log(f"{'ok  ' if good else 'FAIL'} /rtabmap/cloud_map: {n} points")
        ok &= good
    finally:
        t.cmd.publish(String(data="AUTO.LAND"))
        t.wait(40, lambda: t.state and not t.state.armed)
        t.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
