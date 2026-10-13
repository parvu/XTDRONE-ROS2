#!/usr/bin/env python3
"""Chapter 32 check: UAVs and a USV (VRX WAM-V) in the sandisland world.

Requires: xtdrone.launch.py scenario:=sandisland communication:=basic and
`ros2 launch xtdrone_bringup usv.launch.py`.

1. the WAM-V floats (stays near the water surface) and drives forward with equal thrust on both
   engines (/wamv/thrusters/{left,right}/thrust) until it moved 5 m (at most --thrust-time s);
2. iris_0 takes off from the island to 5 m above its start and lands.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String


def log(m):
    print(f"[usv_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--thrust", type=float, default=200.0)
    ap.add_argument("--thrust-time", type=float, default=180.0, help="wall-time limit")
    a = ap.parse_args()
    boat = {}
    lock = threading.Lock()

    def on_pose(msg):
        for p in msg.pose:
            if p.name == "wamv":
                with lock:
                    boat["p"] = (p.position.x, p.position.y, p.position.z)

    gz = GzNode()
    # world poses of the models (VRX's /model/wamv/pose carries only a sensor frame)
    gz.subscribe(Pose_V, "/world/sydney_regatta/pose/info", on_pose)
    rclpy.init()
    n = rclpy.create_node("usv_test")
    d = {}
    n.create_subscription(State, "/iris_0/mavros/state", lambda m: d.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(HomePosition, "/iris_0/mavros/home_position/home", lambda m: d.__setitem__("home", m),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, "/iris_0/ground_truth/odom", lambda m: d.__setitem__("gt", m.pose.pose.position), 1)
    left = n.create_publisher(Float64, "/wamv/thrusters/left/thrust", 1)
    right = n.create_publisher(Float64, "/wamv/thrusters/right/thrust", 1)
    cmd = n.create_publisher(String, "/xtdrone/iris_0/cmd", 3)
    vel = n.create_publisher(Twist, "/xtdrone/iris_0/cmd_vel_flu", 1)

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
    if not wait(240, lambda: "p" in boat and d.get("st") and d["st"].connected and "home" in d and "gt" in d):
        log(f"FAIL: not ready (boat pose {'p' in boat}, iris {sorted(d)})")
        return 1
    wait(5)
    with lock:
        b0 = boat["p"]
    log(f"WAM-V at ({b0[0]:.1f}, {b0[1]:.1f}, {b0[2]:.2f})")
    # thrust until the boat moved 5 m: with six PX4 vehicles and the VRX waves on a CPU the simulation
    # runs far below real time
    def moved5():
        with lock:
            return math.dist(b0[:2], boat["p"][:2]) > 5.0
    t0 = time.time()
    wait(a.thrust_time, moved5,
         every=lambda: (left.publish(Float64(data=a.thrust)), right.publish(Float64(data=a.thrust))))
    log(f"thrust phase {time.time() - t0:.0f} s")
    left.publish(Float64(data=0.0))
    right.publish(Float64(data=0.0))
    with lock:
        b1 = boat["p"]
    moved = math.dist(b0[:2], b1[:2])
    good = moved > 5.0 and abs(b1[2]) < 1.0
    ok &= good
    log(f"{'ok  ' if good else 'FAIL'} WAM-V drove {moved:.1f} m, height {b1[2]:.2f} m (floating)")

    z0 = d["gt"].z
    up = lambda: vel.publish(Twist(linear=Vector3(z=1.0)))  # noqa: E731
    deadline = time.time() + 60
    while time.time() < deadline and not d["st"].armed:
        cmd.publish(String(data="OFFBOARD"))
        wait(1, every=up)
        cmd.publish(String(data="ARM"))
        wait(2, lambda: d["st"].armed, every=up)
    climbed = wait(60, lambda: d["gt"].z > z0 + 5.0, every=up)
    ok &= climbed
    log(f"{'ok  ' if climbed else 'FAIL'} iris_0 climbed from {z0:.1f} m to {d['gt'].z:.1f} m above the island")
    cmd.publish(String(data="AUTO.LAND"))
    landed = wait(90, lambda: not d["st"].armed)
    log(f"{'ok  ' if landed else 'FAIL'} iris_0 landed and disarmed")
    ok &= landed
    # stop the gz-transport subscriber before the interpreter exits: left to the interpreter's teardown its
    # threads aborted the process ("terminate called without an active exception") after a passed test
    gz.unsubscribe("/world/sydney_regatta/pose/info")
    del gz
    n.destroy_node()
    rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
