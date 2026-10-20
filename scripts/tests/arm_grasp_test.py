#!/usr/bin/env python3
"""Chapter 33 check: the le_arm under iris_0 picks up the green cube and the iris flies away with it.

Requires: xtdrone.launch.py scenario:=grasping_arm (iris_arm at (0.1, 0) next to the table with the cube);
the test starts `ros2 launch xtdrone_robotic_arm robotic_arm.launch.py` (controllers, move_group,
vision_manager, grasping_demo).

1. wait for grasping_demo: object seen, approach, attach and close the gripper, lift ("grasped");
2. check that the cube went up with the arm (it was resting on the table);
3. take off (GUIDED, arm, takeoff command) and check that the cube stays with the iris, above 1.5 m.
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

CUBE = "coke_can_box_model"


def log(m):
    print(f"[arm_grasp_test] {m}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=400.0, help="grasp timeout, s (wall time)")
    a = ap.parse_args()
    cube = {}
    lock = threading.Lock()

    def on_pose(msg):
        for p in msg.pose:
            if p.name == CUBE:
                with lock:
                    cube["p"] = (p.position.x, p.position.y, p.position.z)

    gz = GzNode()
    gz.subscribe(Pose_V, "/world/grasping/pose/info", on_pose)
    rclpy.init()
    n = rclpy.create_node("arm_grasp_test")
    d = {"states": []}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_state(m):
        if not d["states"] or d["states"][-1] != m.data:
            d["states"].append(m.data)
            log(f"grasp state: {m.data}")
            if m.data == "grasping":
                d["c_grasp"] = cube_pos()
    n.create_subscription(String, "/iris_0/le_arm/grasp_state", on_state, latched)
    n.create_subscription(State, "/iris_0/mavros/state", lambda m: d.__setitem__("st", m), qos_profile_sensor_data)
    n.create_subscription(Odometry, "/iris_0/ground_truth/odom", lambda m: d.__setitem__("gt", m.pose.pose.position),
                          qos_profile_sensor_data)
    set_mode = n.create_client(SetMode, "/iris_0/mavros/set_mode")
    arming = n.create_client(CommandBool, "/iris_0/mavros/cmd/arming")
    takeoff_srv = n.create_client(CommandTOL, "/iris_0/mavros/cmd/takeoff")

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

    def cube_pos():
        with lock:
            return cube.get("p")

    logs = os.environ.get("XTDRONE_TEST_LOGS", os.path.expanduser("~/.ros/xtdrone_tests"))
    os.makedirs(logs, exist_ok=True)
    arm_log = open(os.path.join(logs, "arm_grasp_launch.log"), "w")
    arm = subprocess.Popen(["ros2", "launch", "xtdrone_robotic_arm", "robotic_arm.launch.py"], stdout=arm_log,
                           stderr=subprocess.STDOUT, start_new_session=True)
    try:
        if not wait(240, lambda: cube_pos() is not None and "gt" in d and d.get("st") and d["st"].connected):
            log("FAIL: no cube pose, iris ground truth or MAVROS")
            return 1
        c0 = cube_pos()
        log(f"cube at ({c0[0]:.3f}, {c0[1]:.3f}, {c0[2]:.3f}), iris at ({d['gt'].x:.2f}, {d['gt'].y:.2f}, "
            f"{d['gt'].z:.2f})")
        if not wait(a.timeout, lambda: d["states"] and d["states"][-1] in ("grasped", "failed")):
            log(f"FAIL: grasping_demo did not finish: {d['states']}")
            return 1
        ok = d["states"][-1] == "grasped"
        log(f"{'ok  ' if ok else 'FAIL'} grasping_demo: {' -> '.join(d['states'])}")
        if not ok:
            return 1
        wait(3, lambda: False)
        c1 = cube_pos()
        # the cube is already joined to the gripper by the detachable-joint plugin when the model loads (it is pulled
        # off the table at the start): the lift is measured from the moment of the grasp
        c0 = d.get("c_grasp") or c0
        lifted = c1[2] - c0[2]
        ok = lifted > 0.01
        log(f"{'ok  ' if ok else 'FAIL'} cube lifted {lifted * 100:.1f} cm by the arm (> 1 cm)")
        if not ok:
            return 1

        def takeoff():
            # ArduPilot: GUIDED, arm, then the takeoff command (the arm and the cube make the iris heavy: 3 m)
            if d["st"].mode != "GUIDED":
                set_mode.call_async(SetMode.Request(custom_mode="GUIDED"))
            elif not d["st"].armed:
                arming.call_async(CommandBool.Request(value=True))
            elif d["gt"].z < 0.5:
                takeoff_srv.call_async(CommandTOL.Request(altitude=3.0))
        if not wait(90, lambda: d["st"].armed and d["gt"].z > 1.8, every=takeoff):
            log(f"FAIL: take-off: armed {d['st'].armed}, mode {d['st'].mode}, z {d['gt'].z:.2f}")
            return 1
        wait(10, lambda: False)
        c2, g = cube_pos(), d["gt"]
        sep = ((c2[0] - g.x) ** 2 + (c2[1] - g.y) ** 2 + (c2[2] - g.z) ** 2) ** 0.5
        ok = c2[2] > 1.5 and sep < 0.5
        log(f"{'ok  ' if ok else 'FAIL'} in flight: iris at {g.z:.2f} m, cube at {c2[2]:.2f} m, {sep:.2f} m from "
            f"the iris (cube > 1.5 m, < 0.5 m from the iris)")
        alive = arm.poll() is None
        log(f"{'ok  ' if alive else 'FAIL'} arm nodes still running")
        ok &= alive
        log("PASS" if ok else "FAIL")
        return 0 if ok else 1
    finally:
        try:
            os.killpg(arm.pid, signal.SIGINT)
            arm.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(arm.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        n.destroy_node()
        rclpy.shutdown()
        # stop the gz-transport subscriber before the interpreter's teardown (usv_test: abort at exit)
        gz.unsubscribe("/world/grasping/pose/info")
        del gz


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    # skip the interpreter teardown: gz-transport's threads still abort it ("terminate called without an active
    # exception") after the unsubscribe, which turned a passed test into a failed exit code
    os._exit(rc)
