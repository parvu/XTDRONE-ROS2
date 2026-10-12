#!/usr/bin/env python3
"""Automated XTDrone flight check, driving vehicles like the keyboard control does.

Requires a running simulation (xtdrone.launch.py) and the communication nodes
(communication:=basic|enhanced, or multi_vehicle_communication.launch.py).

  xtdrone_flight_test.py --vehicles iris_0 [iris_1 ...] [--altitude 2.0]
  xtdrone_flight_test.py --vehicles plane_0 --mode plane
  xtdrone_flight_test.py --topics /iris_0/stereo_camera/left/image_raw /iris_0/imu_gazebo

multirotor: wait for MAVROS connection and a local position, send OFFBOARD + ARM
with an upward cmd_vel_flu, check the altitude, HOVER, check it holds, AUTO.LAND,
check that it lands and disarms.
plane: AUTO.TAKEOFF + ARM, check it climbs above 15 m, AUTO.LAND.
rover: OFFBOARD + ARM, forward cmd_vel_flu, check it moves 3 m.
--topics: check that every listed topic delivers at least one message.
Exit code 0 on success.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import HomePosition, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class Vehicle:
    def __init__(self, node, name):
        self.name = name
        self.state = None
        self.pose = None
        self.home = None
        node.create_subscription(State, f"/{name}/mavros/state", self._state, qos_profile_sensor_data)
        node.create_subscription(PoseStamped, f"/{name}/mavros/local_position/pose", self._pose,
                                 qos_profile_sensor_data)
        node.create_subscription(HomePosition, f"/{name}/mavros/home_position/home", self._home,
                                 qos_profile_sensor_data)
        self.cmd = node.create_publisher(String, f"/xtdrone/{name}/cmd", 3)
        self.vel = node.create_publisher(Twist, f"/xtdrone/{name}/cmd_vel_flu", 1)

    def _state(self, msg):
        self.state = msg

    def _home(self, msg):
        self.home = msg

    @property
    def ready(self):
        """Connected, local position and home position set (PX4 is ready to arm)."""
        return bool(self.state and self.state.connected and self.pose and self.home)

    def _pose(self, msg):
        self.pose = msg.pose.position

    @property
    def z(self):
        return self.pose.z if self.pose else float("nan")

    def send(self, text):
        self.cmd.publish(String(data=text))

    def twist(self, vx=0.0, vz=0.0):
        t = Twist()
        t.linear.x, t.linear.z = vx, vz
        self.vel.publish(t)


class FlightTest(Node):
    def __init__(self):
        super().__init__("xtdrone_flight_test")

    def wait(self, seconds, cond=None, every=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every:
                every()
            if cond and cond():
                return True
        return cond() if cond else True


def log(msg):
    print(f"[flight_test] {msg}", flush=True)


def settled_altitudes(node, vehicles, timeout=30.0):
    """Altitudes once every vehicle rests (z changes < 2 cm over 1 s). Scenario poses can
    spawn vehicles above the floor (e.g. indoor1: z = 1 m), and the estimator settles too."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        before = [v.z for v in vehicles]
        node.wait(1.0)
        if all(abs(v.z - b) < 0.02 for v, b in zip(vehicles, before)):
            break
    log("ground altitudes: " + ", ".join(f"{v.name}={v.z:.2f}" for v in vehicles))
    return {v.name: v.z for v in vehicles}


def arm(node, vehicles, mode, every=None, timeout=60):
    """Switch to `mode` and arm. PX4 can still refuse to arm for a few seconds after the
    home position is set (estimator checks); like a user pressing the mode key and t again,
    re-send both. The communication node ignores a command identical to the previous one."""
    deadline = time.time() + timeout
    while time.time() < deadline and not all(v.state.armed for v in vehicles):
        for v in vehicles:
            if not v.state.armed:
                v.send(mode)
        node.wait(1.0, every=every)
        for v in vehicles:
            if not v.state.armed:
                v.send("ARM")
        node.wait(2.0, lambda: all(v.state.armed for v in vehicles), every=every)
    log("armed: " + ", ".join(f"{v.name}={v.state.armed}" for v in vehicles))


def multirotor(node, vehicles, altitude):
    ok = node.wait(120, lambda: all(v.ready for v in vehicles))
    if not ok:
        for v in vehicles:
            log(f"{v.name}: connected={bool(v.state and v.state.connected)} pose={v.pose is not None} "
                f"home={v.home is not None}")
        return False
    log("MAVROS connected, local and home position available: " + ", ".join(v.name for v in vehicles))
    ground = settled_altitudes(node, vehicles)
    # Keyboard sequence from the manual: i (vz > 0.3) -> b (OFFBOARD) -> t (ARM)
    up = lambda: [v.twist(vz=0.5) for v in vehicles]  # noqa: E731
    node.wait(1.0, every=up)
    arm(node, vehicles, "OFFBOARD", up)
    climbed = node.wait(60, lambda: all(v.z - ground[v.name] > altitude for v in vehicles), every=up)
    log("altitudes: " + ", ".join(f"{v.name}={v.z - ground[v.name]:.2f} m" for v in vehicles))
    if not climbed:
        log("FAIL: vehicles did not reach the altitude (mode: "
            + ", ".join(f"{v.name}={v.state.mode}/{'armed' if v.state.armed else 'disarmed'}" for v in vehicles)
            + ")")
        return False
    # s (HOVER): zero velocity, the communication node holds position
    hover = lambda: [v.twist() for v in vehicles]  # noqa: E731
    for v in vehicles:
        v.send("HOVER")
    node.wait(1.0, every=hover)
    held = {v.name: v.z for v in vehicles}
    node.wait(5.0, every=hover)
    drift = max(abs(v.z - held[v.name]) for v in vehicles)
    log(f"hover drift over 5 s: {drift:.2f} m")
    if drift > 0.5:
        log("FAIL: hover drift too large")
        return False
    # n (AUTO.LAND)
    for v in vehicles:
        v.send("AUTO.LAND")
    landed = node.wait(90, lambda: all(not v.state.armed for v in vehicles), every=hover)
    log("landed and disarmed" if landed else "FAIL: vehicles still armed after landing timeout")
    return landed


def plane(node, vehicles):
    if not node.wait(120, lambda: all(v.ready for v in vehicles)):
        log("FAIL: MAVROS not connected")
        return False
    ground = settled_altitudes(node, vehicles)
    # Keyboard sequence from the manual: v (AUTO.TAKEOFF) -> t (ARM)
    arm(node, vehicles, "AUTO.TAKEOFF")
    ok = node.wait(120, lambda: all(v.z - ground[v.name] > 15 for v in vehicles))
    log("altitudes: " + ", ".join(f"{v.name}={v.z - ground[v.name]:.1f} m" for v in vehicles))
    for v in vehicles:
        v.send("AUTO.LAND")
    return ok


def rover(node, vehicles):
    if not node.wait(120, lambda: all(v.ready for v in vehicles)):
        log("FAIL: MAVROS not connected")
        return False
    start = {v.name: (v.pose.x, v.pose.y) for v in vehicles}
    fwd = lambda: [v.twist(vx=2.0) for v in vehicles]  # noqa: E731
    node.wait(1.0, every=fwd)
    arm(node, vehicles, "OFFBOARD", fwd)

    def dist(v):
        return math.hypot(v.pose.x - start[v.name][0], v.pose.y - start[v.name][1])
    ok = node.wait(60, lambda: all(dist(v) > 3.0 for v in vehicles), every=fwd)
    log("distances: " + ", ".join(f"{v.name}={dist(v):.1f} m" for v in vehicles))
    for v in vehicles:
        v.twist()
        v.send("DISARM")
    return ok


def topics(node, names, timeout):
    from rosidl_runtime_py.utilities import get_message
    seen = {}
    deadline = time.time() + timeout
    subs = []
    while time.time() < deadline and len(seen) < len(names):
        types = dict(node.get_topic_names_and_types())
        for t in names:
            if t in types and t not in seen and not any(s.topic_name == t for s in subs):
                cls = get_message(types[t][0])
                subs.append(node.create_subscription(cls, t, lambda m, t=t: seen.setdefault(t, True),
                                                     qos_profile_sensor_data))
        rclpy.spin_once(node, timeout_sec=0.1)
    for t in names:
        log(f"{'ok  ' if t in seen else 'FAIL'} {t}")
    # Unsubscribe again: the sensor bridges are lazy, and image streams kept alive during a
    # flight check slow the whole simulation down on a machine without GPU.
    for sub in subs:
        node.destroy_subscription(sub)
    return len(seen) == len(names)


def exist_topics(node, names, timeout):
    deadline = time.time() + timeout
    found = set()
    while time.time() < deadline and len(found) < len(names):
        found = {t for t, _ in node.get_topic_names_and_types()} & set(names)
        rclpy.spin_once(node, timeout_sec=0.2)
    for t in names:
        log(f"{'ok  ' if t in found else 'FAIL'} {t} (exists)")
    return len(found) == len(names)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", nargs="*", default=[])
    ap.add_argument("--mode", choices=["multirotor", "plane", "rover"], default="multirotor")
    ap.add_argument("--altitude", type=float, default=2.0)
    ap.add_argument("--topics", nargs="*", default=[])
    ap.add_argument("--exist-topics", nargs="*", default=[],
                    help="topics that must exist (e.g. contact sensors only publish on contact)")
    ap.add_argument("--topic-timeout", type=float, default=60.0)
    args = ap.parse_args()
    rclpy.init()
    node = FlightTest()
    ok = True
    try:
        if args.exist_topics:
            ok = exist_topics(node, args.exist_topics, args.topic_timeout) and ok
        if args.topics:
            ok = topics(node, args.topics, args.topic_timeout) and ok
        if args.vehicles:
            vehicles = [Vehicle(node, n) for n in args.vehicles]
            if args.mode == "multirotor":
                ok = multirotor(node, vehicles, args.altitude) and ok
            elif args.mode == "plane":
                ok = plane(node, vehicles) and ok
            else:
                ok = rover(node, vehicles) and ok
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
