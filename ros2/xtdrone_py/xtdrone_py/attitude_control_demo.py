"""Multirotor attitude control example (port of control/attitude_contorl_demo.py).

  ros2 run xtdrone_py attitude_control_demo iris angular_speed   # or: angle
  ros2 run xtdrone_py attitude_control_demo iris angle 0.73      # optional hover throttle

Controls <type>_0 through /<type>_0/mavros/setpoint_raw/attitude with a PID
(position -> attitude -> body rate / thrust) with a hover-throttle feedforward.
The ROS 1 version used 0.55 (Classic iris). The x500 airframe used on Gazebo
Harmonic hovers at 0.73 (measured), the default here; pass another value as the
third argument for other vehicles. Set XTDRONE_DEBUG=1 to print the controller state.
"""
import math
import os
import sys
import time
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from nav_msgs.msg import Odometry

from xtdrone_py.common import MASK_POSITION, MAVROS_QOS, VehicleNode, q2rpy, rpy2q
from xtdrone_py.keyboard_control import Terminal

HELP = """
4/1 : increase/decrease px_r
5/2 : increase/decrease py_r
6/3 : increase/decrease pz_r
7/8 : increase/decrease yaw_r
a   : arm
d   : disarm
t   : arm and take off (offboard position hold at 2 m)
r   : return to the takeoff point
b   : begin attitude control
h   : hover
l   : land
p   : print key control message
q   : quit
"""


class AttitudeDemo(VehicleNode):

    def __init__(self, uav_type, control_type, hover_thrust):
        super().__init__("attitude_control", uav_type, 0)
        self.control_type = control_type
        self.hover_thrust = hover_thrust
        self.px = self.py = self.pz = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.roll_rate = self.pitch_rate = self.yaw_rate = 0.0
        self.mode = ""
        n = self.name
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(TwistStamped, f"/{n}/mavros/local_position/velocity_body",
                                 self.local_vel_callback, MAVROS_QOS)
        self.create_subscription(Odometry, f"/{n}/mavros/local_position/odom", self.odom_callback, MAVROS_QOS)
        self.create_subscription(State, f"/{n}/mavros/state", self.state_callback, MAVROS_QOS)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 10)
        self.body_target_pub = self.create_publisher(AttitudeTarget, f"/{n}/mavros/setpoint_raw/attitude", 2)

    def local_pose_callback(self, msg):
        self.px, self.py, self.pz = msg.pose.position.x, -msg.pose.position.y, msg.pose.position.z

    def local_vel_callback(self, msg):
        self.vx, self.vy, self.vz = msg.twist.linear.x, -msg.twist.linear.y, msg.twist.linear.z

    def odom_callback(self, msg):
        self.roll, self.pitch, self.yaw = q2rpy(msg.pose.pose.orientation)
        a = msg.twist.twist.angular
        self.roll_rate, self.pitch_rate, self.yaw_rate = a.x, a.y, a.z

    def state_callback(self, msg):
        self.mode = msg.mode

    def offboard(self):
        if self.mode != "OFFBOARD":
            self.set_mode("OFFBOARD")

    def loop(self):
        print("start!\n" + HELP)
        mission = None
        takeoff = (0.0, 0.0, 2.0, 0.0)
        control_x, control_y, control_z, yaw_r = 1.0, 0.0, 2.0, 0.0
        angle_max = 0.3
        steps = {"4": (0, 0.2), "1": (0, -0.2), "5": (1, 0.2), "2": (1, -0.2),
                 "6": (2, 0.2), "3": (2, -0.2), "7": (3, 0.05), "8": (3, -0.05)}
        with Terminal() as term:
            while rclpy.ok():
                key = term.key()
                if key in steps:
                    idx, delta = steps[key]
                    vals = [control_x, control_y, control_z, yaw_r]
                    vals[idx] += delta
                    control_x, control_y, control_z, yaw_r = vals
                    print(f"control_x:{control_x:.2f}  control_y:{control_y:.2f}  "
                          f"control_z:{control_z:.2f}  yaw_r:{yaw_r:.2f}")
                elif key == "a":
                    self.arm(True)
                elif key == "d":
                    self.arm(False)
                elif key == "r":
                    mission = "return"
                    print("return to takeoff point", takeoff[:3])
                elif key == "h":
                    self.set_mode("AUTO.LOITER")
                elif key == "b":
                    mission = "control"
                    print("begin attitude control")
                elif key == "t":
                    self.arm(True)
                    mission = "takeoff"
                    print("takeoff to", takeoff[:3])
                elif key == "l":
                    self.set_mode("AUTO.LAND")
                    mission = "land"
                elif key == "p":
                    print(HELP)
                elif key in ("q", "\x03"):
                    break

                if mission in ("return", "takeoff"):
                    t = PositionTarget()
                    t.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    t.position.x, t.position.y, t.position.z = takeoff[:3]
                    t.yaw = takeoff[3]
                    t.type_mask = MASK_POSITION
                    self.target_motion_pub.publish(t)
                    self.offboard()
                elif mission == "control":
                    self.publish_attitude(control_x, control_y, control_z, yaw_r, angle_max)
                    self.offboard()

    def publish_attitude(self, cx, cy, cz, yaw_r, angle_max):
        roll_r = -0.54236707 * (self.py - cy) - 0.42717549 * self.vy
        pitch_r = -0.54236707 * (self.px - cx) - 0.42717549 * self.vx
        roll_r = max(-angle_max, min(angle_max, roll_r))
        pitch_r = max(-angle_max, min(angle_max, pitch_r))
        if os.environ.get("XTDRONE_DEBUG") and time.time() - getattr(self, "_dbg_t", 0) > 1.0:
            self._dbg_t = time.time()
            print(f"[debug] px={self.px:.2f} py={self.py:.2f} pz={self.pz:.2f} vx={self.vx:.2f} vy={self.vy:.2f} "
                  f"roll_r={roll_r:.3f} pitch_r={pitch_r:.3f} pitch={self.pitch:.3f}", file=sys.stderr, flush=True)
        thrust = (-0.5 * (self.pz - cz) - 0.31774509 * self.vz
                  + self.hover_thrust / math.cos(self.pitch) / math.cos(self.roll))
        att = AttitudeTarget()
        att.header.stamp = self.get_clock().now().to_msg()
        att.thrust = float(max(0.0, min(1.0, thrust)))
        if self.control_type == "angular_speed":
            att.body_rate.x = -5 * (self.roll - roll_r) - 0.1 * self.roll_rate
            att.body_rate.y = -5 * (self.pitch - pitch_r) - 0.1 * self.pitch_rate
            att.body_rate.z = -1 * (self.yaw - yaw_r) - 0.1 * self.yaw_rate
            att.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        else:
            att.orientation = rpy2q(roll_r, pitch_r, yaw_r)
            att.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE
                             | AttitudeTarget.IGNORE_YAW_RATE)
        self.body_target_pub.publish(att)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 2 or argv[1] not in ("angular_speed", "angle"):
        raise SystemExit("usage: attitude_control_demo <uav_type> <angular_speed|angle> [hover_thrust]")
    hover = float(argv[2]) if len(argv) > 2 else 0.73
    rclpy.init()
    node = AttitudeDemo(argv[0], argv[1], hover)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.loop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
