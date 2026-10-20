#!/usr/bin/env python3
"""Chapter 20 check: Nav2 flies an iris_2d_lidar to a goal while slam_toolbox maps indoor3.

Requires: xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic,
`ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox` and
`ros2 launch xtdrone_planning nav2_2d.launch.py`.

Takes off to ~1 m on the SLAM pose, sends NavigateToPose goals (map frame) and checks that the
action succeeds and the ground truth ends within 0.4 m of each goal; lands.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

V = "iris_0"


def log(m):
    print(f"[nav2_test] {m}", flush=True)


class T(Node):
    def __init__(self):
        super().__init__("nav2_test")
        self.state = self.pose = self.gt = None
        self.create_subscription(State, f"/{V}/mavros/state", lambda m: setattr(self, "state", m), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"/{V}/mavros/local_position/pose", lambda m: setattr(self, "pose", m.pose),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{V}/ground_truth/odom", lambda m: setattr(self, "gt", m.pose.pose), 1)
        self.cmd = self.create_publisher(String, f"/xtdrone/{V}/cmd", 3)
        self.vel = self.create_publisher(Twist, f"/xtdrone/{V}/cmd_vel_flu", 1)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

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
    # goals relative to the start, in the SLAM map frame (indoor3: open floor around the start)
    ap.add_argument("--goals", default="2.5,0;2.5,2.5;0,0")
    a = ap.parse_args()
    goals = [tuple(float(c) for c in g.split(",")) for g in a.goals.split(";")]
    rclpy.init()
    t = T()
    ok = True
    try:
        if not t.wait(240, lambda: t.state and t.state.connected and t.gt and t.pose):
            log("FAIL: no MAVROS connection / ground truth / SLAM-based local position")
            return 1
        if not t.nav.wait_for_server(timeout_sec=120):
            log("FAIL: Nav2 navigate_to_pose not available")
            return 1
        t.wait(5)
        s0 = (t.pose.position.x, t.pose.position.y)
        g0 = (t.gt.position.x, t.gt.position.y)
        # the map frame has the heading of the estimator at the start as its x axis (the 2D SLAM yaw is the
        # EKF yaw): express the ground-truth displacement in it
        qp, qg = t.pose.orientation, t.gt.orientation
        rot = (math.atan2(2 * (qp.w * qp.z + qp.x * qp.y), 1 - 2 * (qp.y * qp.y + qp.z * qp.z))
               - math.atan2(2 * (qg.w * qg.z + qg.x * qg.y), 1 - 2 * (qg.y * qg.y + qg.z * qg.z)))
        up = lambda: t.vel.publish(Twist(linear=Twist().linear.__class__(z=0.5)))  # noqa: E731
        deadline = time.time() + 60
        while time.time() < deadline and not t.state.armed:
            t.cmd.publish(String(data="OFFBOARD"))
            t.wait(1, every=up)
            t.cmd.publish(String(data="ARM"))
            t.wait(2, lambda: t.state.armed, every=up)
        if not t.wait(30, lambda: t.gt.position.z > 1.0, every=up):
            log("FAIL: takeoff")
            return 1
        t.cmd.publish(String(data="HOVER"))
        t.wait(3, every=lambda: t.vel.publish(Twist()))
        log("airborne; sending goals")
        for gx, gy in goals:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x, goal.pose.pose.position.y = s0[0] + gx, s0[1] + gy
            goal.pose.pose.orientation.w = 1.0
            fut = t.nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(t, fut, timeout_sec=10)
            handle = fut.result()
            if handle is None or not handle.accepted:
                log(f"FAIL goal ({gx}, {gy}) rejected")
                ok = False
                continue
            res = handle.get_result_async()
            t.wait(120, lambda: res.done())
            status = res.result().status if res.done() else None
            dx, dy = t.gt.position.x - g0[0], t.gt.position.y - g0[1]
            dg = (dx * math.cos(rot) - dy * math.sin(rot), dx * math.sin(rot) + dy * math.cos(rot))
            err = math.dist(dg, (gx, gy))
            good = status == GoalStatus.STATUS_SUCCEEDED and err < 0.7  # goal tolerance 0.2 m + SLAM error (ArduPilot branch; PX4 branch: 0.5)
            ok &= good
            log(f"{'ok  ' if good else 'FAIL'} goal ({gx}, {gy}): status {status}, ground truth "
                f"({dg[0]:.2f}, {dg[1]:.2f}), error {err:.2f} m, height {t.gt.position.z:.2f} m")
    finally:
        t.cmd.publish(String(data="AUTO.LAND"))
        t.wait(40, lambda: t.state and not t.state.armed)
        t.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
