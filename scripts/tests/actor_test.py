#!/usr/bin/env python3
"""Chapter 38 check: ROS-controlled actors in the robocup world walk to commanded points.

Requires: xtdrone.launch.py world:=robocup vehicles:=iris px4:=false (the launch bridges
/actor_<i>/cmd_motion to the xtdrone_gz_plugins ActorMotion system of each actor).

Sends actor_2 to (2, -2) and actor_5 to (0, 0) at 2 m/s (like control_actors.sh) and checks, from
the Gazebo pose of the actors, that they get there.
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Vector3
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode

TARGETS = {"actor_2": (2.0, -2.0, 2.0), "actor_5": (0.0, 0.0, 2.0)}


def log(m):
    print(f"[actor_test] {m}", flush=True)


def main():
    poses = {}
    lock = threading.Lock()

    def on_pose(msg):
        with lock:
            poses[msg.name] = (msg.position.x, msg.position.y)

    gz = GzNode()
    for a in TARGETS:  # published by the ActorMotion system (gz pose topics leave actors out)
        gz.subscribe(Pose, f"/{a}/pose", on_pose)
    rclpy.init()
    node = rclpy.create_node("actor_test")
    pubs = {a: node.create_publisher(Vector3, f"/{a}/cmd_motion", 1) for a in TARGETS}
    deadline = time.time() + 120
    while time.time() < deadline and len(poses) < len(TARGETS):
        time.sleep(0.5)
    if len(poses) < len(TARGETS):
        log(f"FAIL: no actor poses from Gazebo ({sorted(poses)})")
        return 1
    with lock:
        start = dict(poses)
    log("start: " + ", ".join(f"{a} ({p[0]:.1f}, {p[1]:.1f})" for a, p in start.items()))

    def arrived():
        with lock:
            return all(math.dist(poses[a], TARGETS[a][:2]) < 0.3 for a in TARGETS)
    end = time.time() + 120
    while time.time() < end and not arrived():
        for a, (x, y, v) in TARGETS.items():
            pubs[a].publish(Vector3(x=x, y=y, z=v))
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    ok = True
    with lock:
        for a, (x, y, _) in TARGETS.items():
            p = poses[a]
            good = math.dist(p, (x, y)) < 0.3 and math.dist(start[a], (x, y)) > 1.0
            ok &= good
            log(f"{'ok  ' if good else 'FAIL'} {a}: walked from ({start[a][0]:.1f}, {start[a][1]:.1f}) "
                f"to ({p[0]:.2f}, {p[1]:.2f}), target ({x}, {y})")
    node.destroy_node()
    rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
