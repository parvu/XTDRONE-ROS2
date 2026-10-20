#!/usr/bin/env python3
"""Chapter 29 check: every ArduPilot instance keeps its own mission.

Requires: xtdrone.launch.py world:=outdoor2 vehicles:="iris*3" mavros_plugins:=all (the MAVROS mission plugin).

1. upload a different mission to each iris through MAVROS (as QGroundControl does over MAVLink): take off to 5 m,
   then a waypoint 10, 20 and 30 m north of its home;
2. read the missions back: each vehicle still has its own (each SITL instance keeps its own mission storage);
3. AUTO and arm all three; check that each one ends within 2 m of its own waypoint.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, SetMode, WaypointPull, WaypointPush
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
FRAME_GLOBAL_REL_ALT = 3


def log(m):
    print(f"[mission_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicles", default="iris_0,iris_1,iris_2")
    ap.add_argument("--timeout", type=float, default=150.0)
    a = ap.parse_args()
    names = a.vehicles.split(",")
    rclpy.init()
    n = rclpy.create_node("mission_test")
    d = {v: {} for v in names}
    for v in names:
        ns = f"/{v}/mavros/"
        n.create_subscription(State, ns + "state", lambda m, v=v: d[v].__setitem__("st", m), qos_profile_sensor_data)
        n.create_subscription(NavSatFix, ns + "global_position/global", lambda m, v=v: d[v].__setitem__("fix", m),
                              qos_profile_sensor_data)
        n.create_subscription(PoseStamped, ns + "local_position/pose",
                              lambda m, v=v: d[v].__setitem__("pos", m.pose.position), qos_profile_sensor_data)
    cli = {v: {"push": n.create_client(WaypointPush, f"/{v}/mavros/mission/push"),
               "pull": n.create_client(WaypointPull, f"/{v}/mavros/mission/pull"),
               "mode": n.create_client(SetMode, f"/{v}/mavros/set_mode"),
               "arm": n.create_client(CommandBool, f"/{v}/mavros/cmd/arming")} for v in names}

    def wait(s, cond, every=None):
        end = time.time() + s
        last = 0.0
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
            if cond():
                return True
            if every and time.time() - last > 3.0:
                every()
                last = time.time()
        return cond()

    def call(client, req, timeout=20.0):
        if not client.wait_for_service(timeout_sec=90.0):  # the MAVROS plugins come up some 10-60 s after the link
            return None
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    if not wait(300, lambda: all(d[v].get("st") and d[v]["st"].connected and "fix" in d[v] and "pos" in d[v]
                                 for v in names)):
        log("FAIL: MAVROS, GPS or local position of a vehicle missing")
        return 1
    wait(90, lambda: False)  # home position, EKF origin and MAVROS' first pull of parameters, missions and fences

    # 1. a different mission per vehicle
    target = {}
    origin = {}
    for k, v in enumerate(names):
        fix = d[v]["fix"]
        north = 10.0 * (k + 1)
        lat = fix.latitude + north / 111320.0
        target[v] = north
        origin[v] = (d[v]["pos"].x, d[v]["pos"].y)  # the vehicles spawn side by side: the waypoint is north of the start
        # ArduPilot: item 0 of a mission is the home position
        wps = [Waypoint(frame=0, command=MAV_CMD_NAV_WAYPOINT, autocontinue=True,
                        x_lat=fix.latitude, y_long=fix.longitude, z_alt=0.0),
               Waypoint(frame=FRAME_GLOBAL_REL_ALT, command=MAV_CMD_NAV_TAKEOFF, is_current=True, autocontinue=True,
                        x_lat=fix.latitude, y_long=fix.longitude, z_alt=5.0),
               Waypoint(frame=FRAME_GLOBAL_REL_ALT, command=MAV_CMD_NAV_WAYPOINT, autocontinue=True,
                        param2=1.0, x_lat=lat, y_long=fix.longitude, z_alt=5.0)]
        for _ in range(2):  # under load a push can go unanswered (MAVROS busy with its initial pull)
            r = call(cli[v]["push"], WaypointPush.Request(start_index=0, waypoints=wps), timeout=60.0)
            if r is not None:
                break
        if r is None or not r.success:
            log(f"FAIL: mission upload to {v}: {r}")
            return 1
        log(f"ok   {v}: mission uploaded (waypoint {north:.0f} m north)")

    # 2. each vehicle still has its own mission
    ok = True
    for k, v in enumerate(names):
        r = call(cli[v]["pull"], WaypointPull.Request())
        good = r is not None and r.success and r.wp_received == 3
        log(f"{'ok  ' if good else 'FAIL'} {v}: {r.wp_received if r else 'no'} waypoints read back")
        ok &= good
    if not ok:
        return 1

    # 3. fly them
    def start():
        for v in names:
            if d[v]["st"].mode != "AUTO":
                call(cli[v]["mode"], SetMode.Request(custom_mode="AUTO"))
            elif not d[v]["st"].armed:
                call(cli[v]["arm"], CommandBool.Request(value=True))
    if not wait(90, lambda: all(d[v]["st"].armed and d[v]["st"].mode == "AUTO" for v in names), every=start):
        log("FAIL: AUTO.MISSION / arming: " + ", ".join(f"{v} {d[v]['st'].mode} armed {d[v]['st'].armed}"
                                                       for v in names))
        return 1
    log("ok   all in AUTO and armed")

    def err(v):
        p = d[v]["pos"]  # ENU: the waypoint is target[v] m north (+y) of the start
        return math.hypot(p.x - origin[v][0], p.y - origin[v][1] - target[v])
    wait(a.timeout, lambda: all(err(v) < 2.0 and abs(d[v]["pos"].z - 5.0) < 1.5 for v in names))
    for v in names:
        p = d[v]["pos"]
        good = err(v) < 2.0 and abs(p.z - 5.0) < 1.5
        log(f"{'ok  ' if good else 'FAIL'} {v} at ({p.x:.1f}, {p.y:.1f}, {p.z:.1f}) m from home, "
            f"{err(v):.1f} m from its waypoint ({target[v]:.0f} m north, 5 m up)")
        ok &= good
    log("PASS" if ok else "FAIL")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
