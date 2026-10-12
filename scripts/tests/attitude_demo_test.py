#!/usr/bin/env python3
"""Chapter 7 check: drive the interactive attitude control demo through a pseudo terminal.

Requires a running simulation with iris_0 (e.g. scenario:=indoor1). Starts
`ros2 run xtdrone_py attitude_control_demo iris <mode> [hover]`, presses
't' (arm + offboard position takeoff to 2 m), then 'b' (attitude control towards
the target (1, 0, 2)), and checks that the vehicle reaches and holds the target;
then presses 'l' (land) and 'q'.

  attitude_demo_test.py --mode angular_speed   # or angle
"""
import argparse
import math
import os
import pty
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def log(m):
    print(f"[attitude_test] {m}", flush=True)


class Watch(Node):
    def __init__(self):
        super().__init__("attitude_demo_test")
        self.p = None
        self.state = None
        self.create_subscription(PoseStamped, "/iris_0/mavros/local_position/pose", self._pose, qos_profile_sensor_data)
        self.create_subscription(State, "/iris_0/mavros/state", self._state, qos_profile_sensor_data)
        self.pitch = self.target_pitch = self.thrust = float("nan")
        self.vbody = self.vgt = (float("nan"),) * 3
        self.sent = None
        self.create_subscription(AttitudeTarget, "/iris_0/mavros/setpoint_raw/attitude",
                                 lambda m: setattr(self, "sent", m), 10)
        self.create_subscription(TwistStamped, "/iris_0/mavros/local_position/velocity_body",
                                 lambda m: setattr(self, "vbody", (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/iris_0/ground_truth/odom",
                                 lambda m: (setattr(self, "vgt", (m.twist.twist.linear.x, m.twist.twist.linear.y,
                                                                  m.twist.twist.linear.z)),
                                            setattr(self, "pgt", m.pose.pose.position)), 1)
        self.pgt = None
        self.create_subscription(Odometry, "/iris_0/mavros/local_position/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(AttitudeTarget, "/iris_0/mavros/setpoint_raw/target_attitude", self._target,
                                 qos_profile_sensor_data)

    @staticmethod
    def _pitch(q):
        return math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))

    def _odom(self, m):
        self.pitch = self._pitch(m.pose.pose.orientation)

    def _target(self, m):
        self.target_pitch = self._pitch(m.orientation)
        self.thrust = m.thrust

    def _pose(self, m):
        self.p = m.pose.position

    def _state(self, m):
        self.state = m

    def wait(self, seconds, cond=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond and cond():
                return True
        return cond() if cond else True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["angular_speed", "angle"], default="angular_speed")
    ap.add_argument("--hover", default="0.73")
    a = ap.parse_args()
    rclpy.init()
    w = Watch()
    if not w.wait(120, lambda: w.state and w.state.connected and w.p is not None):
        log("FAIL: MAVROS not connected")
        return 1
    master, slave = pty.openpty()
    demo = subprocess.Popen(["ros2", "run", "xtdrone_py", "attitude_control_demo", "iris", a.mode, a.hover],
                            stdin=slave, stdout=subprocess.DEVNULL,
                            stderr=open(os.environ["XTDRONE_DEBUG_LOG"], "w") if os.environ.get("XTDRONE_DEBUG_LOG")
                            else subprocess.DEVNULL,
                            start_new_session=True)
    os.close(slave)
    ok = False
    try:
        w.wait(5)
        ground = w.p.z
        # 't': arm + offboard position takeoff to (0, 0, 2); PX4 may refuse to arm for a few
        # seconds after start, so press it again until armed (like a user).
        deadline = time.time() + 90
        while time.time() < deadline and not (w.state and w.state.armed):
            os.write(master, b"t")
            w.wait(4, lambda: w.state and w.state.armed)
        if not w.wait(60, lambda: w.p.z - ground > 1.7):
            log(f"FAIL: takeoff, z={w.p.z - ground:.2f} m, armed={w.state.armed}, mode={w.state.mode}")
            return 1
        log(f"takeoff: z={w.p.z - ground:.2f} m, mode={w.state.mode}")
        w.wait(3)
        # 'b': attitude control towards (control_x, control_y, control_z) = (1, 0, 2)
        os.write(master, b"b")
        def err():
            return math.dist((w.p.x, w.p.y, w.p.z - ground), (1.0, 0.0, 2.0))
        reached = w.wait(40, lambda: err() < 0.4)
        log(f"attitude control: position ({w.p.x:.2f}, {w.p.y:.2f}, {w.p.z - ground:.2f}), error {err():.2f} m")
        if reached:
            worst = 0.0
            end = time.time() + 8
            while time.time() < end:
                w.wait(0.2)
                worst = max(worst, err())
            log(f"holding the target for 8 s: max error {worst:.2f} m; pitch {math.degrees(w.pitch):.1f} deg, "
                f"PX4 attitude target pitch {math.degrees(w.target_pitch):.1f} deg, thrust {w.thrust:.2f}; "
                f"velocity_body {tuple(round(v, 2) for v in w.vbody)}, ground-truth body velocity "
                f"{tuple(round(v, 2) for v in w.vgt)}")
            if w.pgt is not None:
                log(f"ground truth position ({w.pgt.x:.2f}, {w.pgt.y:.2f}, {w.pgt.z:.2f}); MAVROS local "
                    f"({w.p.x:.2f}, {w.p.y:.2f}, {w.p.z:.2f})")
            if w.sent is not None:
                q = w.sent.orientation
                log(f"demo publishes: pitch {math.degrees(w._pitch(q)):.1f} deg, q=({q.w:.3f}, {q.x:.3f}, {q.y:.3f}, "
                    f"{q.z:.3f}), type_mask {w.sent.type_mask}, thrust {w.sent.thrust:.2f}")
            ok = worst < 0.6
        os.write(master, b"l")
        w.wait(30, lambda: not w.state.armed)
        os.write(master, b"q")
    finally:
        demo.terminate()
        try:
            demo.wait(5)
        except subprocess.TimeoutExpired:
            demo.kill()
        w.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
