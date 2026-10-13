#!/usr/bin/env python3
"""Chapter 35 check: three standard VTOLs fly a formation whose leader loiters over the UGV.

Requires: xtdrone.launch.py scenario:=outdoor2_fw_formation (ugv_0 at the origin, the
leader standard_vtol_0 at (-600, 0), the followers at (-615, -15) and (-615, 15)); the test starts
`ros2 launch xtdrone_fixed_wing_formation fixed_wing_formation.launch.py`.

1. AUTO.TAKEOFF and arm all three (MIS_TAKEOFF_ALT from the launch), wait for the take-off (AUTO.LOITER);
2. one at a time: transition to fixed-wing (MAV_CMD_DO_VTOL_TRANSITION), then start the formation with switch_fw_mode
   (OFFBOARD; formation_type 2, the triangle: follower 1 30 m north and 30 m west of the leader, follower 2
   30 m south and 30 m west);
3. check that the leader reaches the UGV and loiters over it, and that over the last part of the run the
   followers hold their slots and nobody comes down.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from xtdrone_fixed_wing_formation.msg import FwCmd, FwStates

PREFIX = "standard_vtol_"
SPAWN = {0: (-600.0, 0.0), 1: (-615.0, -15.0), 2: (-615.0, 15.0)}
# formation_controller.cpp: offsets (north, east) of the followers per formation type
SLOTS = {1: {1: (30, 0), 2: (-30, 0)}, 2: {1: (30, -30), 2: (-30, -30)}, 3: {1: (0, 30), 2: (0, -30)}}


def log(m):
    print(f"[fw_formation_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--formation", type=int, default=2)
    ap.add_argument("--duration", type=float, default=300.0, help="formation flight, s (wall time)")
    ap.add_argument("--window", type=float, default=60.0, help="checked final part of the flight, s")
    ap.add_argument("--slot-tol", type=float, default=40.0, help="max mean follower distance from its slot, m")
    ap.add_argument("--ugv-tol", type=float, default=250.0, help="max leader distance from the UGV, m")
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("fw_formation_test")
    d = {i: {} for i in SPAWN}
    d["ugv"] = None
    for i in SPAWN:
        ns = f"/{PREFIX}{i}/mavros/"
        n.create_subscription(State, ns + "state", lambda m, i=i: d[i].__setitem__("state", m), qos_profile_sensor_data)
        n.create_subscription(ExtendedState, ns + "extended_state",
                              lambda m, i=i: d[i].__setitem__("ext", m), qos_profile_sensor_data)
        n.create_subscription(PoseStamped, ns + "local_position/pose",
                              lambda m, i=i: d[i].__setitem__("pose", m.pose.position), qos_profile_sensor_data)
    for i in (1, 2):  # the followers' commands and states, logged at the start of the formation flight
        n.create_subscription(FwCmd, f"/{PREFIX}{i}/fixed_wing_formation_control/fw_cmd",
                              lambda m, i=i: d[i].__setitem__("cmd", m), 10)
        n.create_subscription(FwStates, f"/{PREFIX}{i}/fixed_wing_formation_control/fw_states",
                              lambda m, i=i: d[i].__setitem__("fws", m), 10)
    n.create_subscription(Odometry, "/ugv_0/ground_truth/odom", lambda m: d.__setitem__("ugv", m.pose.pose.position),
                          qos_profile_sensor_data)
    cli = {i: {"mode": n.create_client(SetMode, f"/{PREFIX}{i}/mavros/set_mode"),
               "arm": n.create_client(CommandBool, f"/{PREFIX}{i}/mavros/cmd/arming"),
               "cmd": n.create_client(CommandLong, f"/{PREFIX}{i}/mavros/cmd/command")} for i in SPAWN}

    def spin(s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    def wait(s, cond, every=None):
        end = time.time() + s
        last = 0.0
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
            if cond():
                return True
            if every and time.time() - last > 2.0:
                every()
                last = time.time()
        return cond()

    def call(client, req):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        fut = client.call_async(req)
        end = time.time() + 5
        while not fut.done() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def world(i):  # MAVROS local ENU -> Gazebo world (each local frame starts at the spawn point)
        p = d[i]["pose"]
        return (SPAWN[i][0] + p.x, SPAWN[i][1] + p.y, p.z)

    def mode(i):
        return d[i]["state"].mode if "state" in d[i] else ""

    logs = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
    os.makedirs(logs, exist_ok=True)
    nodes_log = open(os.path.join(logs, "fw_formation_nodes.log"), "w")
    procs = [subprocess.Popen(["ros2", "launch", "xtdrone_fixed_wing_formation", "fixed_wing_formation.launch.py"],
                              stdout=nodes_log, stderr=subprocess.STDOUT, start_new_session=True)]
    try:
        if not wait(300, lambda: all("pose" in d[i] and d[i].get("state") and d[i]["state"].connected for i in SPAWN)
                    and d["ugv"] is not None):
            log("FAIL: MAVROS of the VTOLs or the UGV ground truth not up")
            return 1
        # PX4 needs a home position (GPS fix) before it arms
        spin(20)

        def takeoff():
            for i in SPAWN:
                if mode(i) != "AUTO.TAKEOFF" and not d[i]["state"].armed:
                    call(cli[i]["mode"], SetMode.Request(custom_mode="AUTO.TAKEOFF"))
                if not d[i]["state"].armed:
                    call(cli[i]["arm"], CommandBool.Request(value=True))
        if not wait(120, lambda: all(d[i]["state"].armed for i in SPAWN), every=takeoff):
            log("FAIL: not armed: " + ", ".join(f"{i} {mode(i)}" for i in SPAWN if not d[i]["state"].armed))
            return 1
        log("ok   armed in AUTO.TAKEOFF")
        # transition only after the take-off: commanded during the climb, the VTOLs quad-chuted (back to
        # multicopter) and returned
        if not wait(240, lambda: all(world(i)[2] > 35 and mode(i) == "AUTO.LOITER" for i in SPAWN)):
            log("FAIL: climb: " + ", ".join(f"{world(i)[2]:.1f} m {mode(i)}" for i in SPAWN))
            return 1
        log("ok   climbed: " + ", ".join(f"{world(i)[2]:.1f} m" for i in SPAWN))

        # one at a time, each straight into the formation (OFFBOARD) after its transition. All three transitioning
        # together, one VTOL's airspeed sometimes dropped out and PX4 quad-chuted it back to multicopter for good;
        # transitioned VTOLs left loitering (80 m circles at the same altitude) flew into the ones still to go.
        for i in SPAWN:
            if not wait(60, lambda: d[i]["ext"].vtol_state == ExtendedState.VTOL_STATE_FW,
                        every=lambda: d[i]["ext"].vtol_state == ExtendedState.VTOL_STATE_MC and call(
                            cli[i]["cmd"], CommandLong.Request(command=3000, param1=float(ExtendedState.VTOL_STATE_FW)))):
                log(f"FAIL: transition of {i}: vtol_state {d[i]['ext'].vtol_state}")
                return 1
            procs.append(subprocess.Popen(
                ["ros2", "run", "xtdrone_fixed_wing_formation", "switch_fw_mode", "--ros-args", "-p", f"plane_id:={i}",
                 "-p", "mode:=1", "-p", f"formation_type:={a.formation}", "-p", "use_sim_time:=true"],
                stdout=nodes_log, stderr=subprocess.STDOUT, start_new_session=True))
            if not wait(30, lambda: mode(i) == "OFFBOARD"):
                log(f"FAIL: OFFBOARD of {i}: {mode(i)}")
                return 1
            log(f"ok   {i}: fixed-wing and OFFBOARD at {world(i)[2]:.1f} m")
        log(f"ok   OFFBOARD, formation {a.formation}")

        start = time.time()
        samples = []  # (t, leader-UGV distance, slot errors, altitudes)
        last_log = 0.0
        while time.time() - start < a.duration:
            spin(1.0)
            lw = world(0)
            ugv = d["ugv"]
            dug = math.hypot(lw[0] - ugv.x, lw[1] - ugv.y)
            errs = []
            for f, (dn, de) in SLOTS[a.formation].items():
                fw = world(f)
                errs.append(math.hypot(fw[0] - (lw[0] + de), fw[1] - (lw[1] + dn)))
            alts = [world(i)[2] for i in SPAWN]
            samples.append((time.time() - start, dug, errs, alts))
            if time.time() - start < 30:
                for f in (1, 2):
                    c, st = d[f].get("cmd"), d[f].get("fws")
                    if c and st:
                        log(f"  follower {f}: cmd roll {math.degrees(c.roll_angle_sp):.0f} pitch "
                            f"{math.degrees(c.pitch_angle_sp):.0f} thr {c.throttle_sp:.2f}; state alt {st.relative_alt:.0f} "
                            f"pitch {math.degrees(st.pitch_angle):.0f} roll {math.degrees(st.roll_angle):.0f} "
                            f"airspeed {st.air_speed:.1f} vz {st.ned_vel_z:.1f}")
            if time.time() - last_log > 15:
                last_log = time.time()
                log(f"t {time.time() - start:5.0f} s  leader ({lw[0]:.0f}, {lw[1]:.0f}, {lw[2]:.0f}) "
                    f"{dug:.0f} m from UGV; slot errors {errs[0]:.0f} {errs[1]:.0f} m; alt "
                    + " ".join(f"{z:.0f}" for z in alts) + "; " + " ".join(mode(i) for i in SPAWN))
            if min(alts) < 5:
                log(f"FAIL: a vehicle came down (altitudes {alts})")
                return 1
        tail = [s for s in samples if s[0] > a.duration - a.window]
        ok = True
        max_dug = max(s[1] for s in tail)
        log(f"{'ok  ' if max_dug < a.ugv_tol else 'FAIL'} leader within {max_dug:.0f} m of the UGV over the last "
            f"{a.window:.0f} s (< {a.ugv_tol:.0f})")
        ok &= max_dug < a.ugv_tol
        for k, f in enumerate(SLOTS[a.formation]):
            mean = sum(s[2][k] for s in tail) / len(tail)
            mx = max(s[2][k] for s in tail)
            log(f"{'ok  ' if mean < a.slot_tol else 'FAIL'} follower {f} slot error mean {mean:.1f} m, "
                f"max {mx:.1f} m (mean < {a.slot_tol:.0f})")
            ok &= mean < a.slot_tol
        alive = all(p.poll() is None for p in procs)
        log(f"{'ok  ' if alive else 'FAIL'} formation nodes still running")
        ok &= alive
        log("PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
