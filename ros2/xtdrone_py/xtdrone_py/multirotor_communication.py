"""XTDrone multirotor communication (port of communication/multirotor_communication.py
and multirotor_communication_enhanced.py).

  ros2 run xtdrone_py multirotor_communication iris 0
  ros2 run xtdrone_py multirotor_communication_enhanced iris 0

Subscribes /xtdrone/<type>_<id>/{cmd, cmd_pose_flu, cmd_pose_enu, cmd_vel_flu, cmd_vel_enu,
cmd_accel_flu, cmd_accel_enu} and streams /<type>_<id>/mavros/setpoint_raw/local at 30 Hz.
"""
import math

from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

from xtdrone_py.common import (FRAME_BODY_NED, FRAME_BODY_OFFSET_NED, FRAME_LOCAL_NED, MAVROS_QOS,
                               MOTION_MASKS, VehicleNode, q2yaw, run)


class MultirotorCommunication(VehicleNode):

    def __init__(self, vehicle_type, vehicle_id):
        super().__init__("communication", vehicle_type, vehicle_id)
        self.current_position = None
        self.current_yaw = 0.0
        self.hover_flag = 0
        self.coordinate_frame = FRAME_LOCAL_NED
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None

        n, x = self.name, f"/xtdrone/{self.name}"
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(String, f"{x}/cmd", self.cmd_callback, 3)
        self.create_subscription(Pose, f"{x}/cmd_pose_flu", self.cmd_pose_flu_callback, 1)
        self.create_subscription(Pose, f"{x}/cmd_pose_enu", self.cmd_pose_enu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_flu", self.cmd_vel_flu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_enu", self.cmd_vel_enu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_accel_flu", self.cmd_accel_flu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_accel_enu", self.cmd_accel_enu_callback, 1)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 1)
        self.create_timer(1.0 / 30.0, self.publish_target)
        self.log("communication initialized")

    def publish_target(self):
        self.target_motion.header.stamp = self.get_clock().now().to_msg()
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.current_position = msg.pose.position
        self.current_yaw = q2yaw(msg.pose.orientation)

    def construct_target(self, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0,
                         afx=0.0, afy=0.0, afz=0.0, yaw=0.0, yaw_rate=0.0):
        t = PositionTarget()
        t.coordinate_frame = self.coordinate_frame
        t.position.x, t.position.y, t.position.z = float(x), float(y), float(z)
        t.velocity.x, t.velocity.y, t.velocity.z = float(vx), float(vy), float(vz)
        t.acceleration_or_force.x = float(afx)
        t.acceleration_or_force.y = float(afy)
        t.acceleration_or_force.z = float(afz)
        t.yaw, t.yaw_rate = float(yaw), float(yaw_rate)
        t.type_mask = MOTION_MASKS[self.motion_type]
        return t

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = FRAME_BODY_OFFSET_NED
        self.motion_type = 0
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z,
                                                   yaw=q2yaw(msg.orientation))

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = FRAME_LOCAL_NED
        self.motion_type = 0
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z,
                                                   yaw=q2yaw(msg.orientation))

    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = FRAME_BODY_NED
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = FRAME_LOCAL_NED
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = FRAME_BODY_NED
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x, afy=msg.linear.y, afz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = FRAME_LOCAL_NED
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x, afy=msg.linear.y, afz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def hover_state_transition(self, x, y, z, w):
        if abs(x) > 0.02 or abs(y) > 0.02 or abs(z) > 0.02 or abs(w) > 0.005:
            self.hover_flag = 0
            self.flight_mode = "OFFBOARD"
        elif self.flight_mode != "HOVER":
            self.hover_flag = 1
            self.flight_mode = "HOVER"
            self.hover()

    def cmd_callback(self, msg):
        if msg.data in (self.last_cmd, "", "stop controlling"):
            return
        if msg.data == "ARM":
            self.arm(True)
        elif msg.data == "DISARM":
            self.arm(False)
        elif msg.data[:-1] == "mission" and msg.data != self.mission:
            self.mission = msg.data
            self.log(msg.data)
        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()
        self.last_cmd = msg.data

    def hover(self):
        if self.current_position is None:
            self.log("HOVER ignored, no local position yet")
            return
        self.coordinate_frame = FRAME_LOCAL_NED
        self.motion_type = 0
        p = self.current_position
        self.target_motion = self.construct_target(x=p.x, y=p.y, z=p.z, yaw=self.current_yaw)
        self.log(self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.hover()
        else:
            self.set_mode(self.flight_mode)


class EnhancedMultirotorCommunication(MultirotorCommunication):
    """Adds single-direction position hold: zero commanded velocity on an axis holds it."""

    def __init__(self, vehicle_type, vehicle_id):
        self.hold_position_x = self.hold_position_y = self.hold_position_z = 0.0
        self.hold_yaw = 0.0
        self.hold_flag = 0
        self.hold_x_flag = self.hold_y_flag = self.hold_z_flag = self.hold_yaw_flag = 0
        self.hold_kp_x = self.hold_kp_y = self.hold_kp_z = self.hold_kp_yaw = 1.0
        super().__init__(vehicle_type, vehicle_id)

    def cmd_vel_flu_callback(self, msg):
        self._vel_with_hold(msg, "flu", FRAME_BODY_NED)

    def cmd_vel_enu_callback(self, msg):
        self._vel_with_hold(msg, "enu", FRAME_LOCAL_NED)

    def _vel_with_hold(self, msg, vel_type, frame):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0 and self.current_position is not None:
            self.hold_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, vel_type)
        if self.hover_flag == 0 and self.hold_flag == 0:
            self.coordinate_frame = frame
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                       yaw_rate=msg.angular.z)

    def _yaw_hold_rate(self):
        return -self.hold_kp_yaw * (self.current_yaw - self.hold_yaw)

    def hold_state_transition(self, x, y, z, w, vel_type):
        p = self.current_position
        if vel_type == "flu":
            if abs(x) < 0.02 and abs(y) < 0.02 and (self.hold_x_flag == 0 or self.hold_y_flag == 0):
                self.hold_position_x, self.hold_position_y = p.x, p.y
                self.hold_x_flag = self.hold_y_flag = 1
            if abs(x) < 0.02:
                if self.hold_x_flag == 0:
                    self.hold_position_x, self.hold_position_y = p.x, p.y
                    self.hold_x_flag = 1
            else:
                self.hold_x_flag = 0
            if abs(y) < 0.02:
                if self.hold_y_flag == 0:
                    self.hold_position_x, self.hold_position_y = p.x, p.y
                    self.hold_y_flag = 1
            else:
                self.hold_y_flag = 0
            if abs(z) < 0.02:
                if self.hold_z_flag == 0:
                    self.hold_position_z = p.z
                    self.hold_z_flag = 1
                z = -self.hold_kp_z * (p.z - self.hold_position_z)
            else:
                self.hold_z_flag = 0
            if abs(w) < 0.005:
                if self.hold_yaw_flag == 0:
                    self.hold_yaw = self.current_yaw
                    self.hold_yaw_flag = 1
            else:
                self.hold_yaw_flag = 0

            ex, ey = p.x - self.hold_position_x, p.y - self.hold_position_y
            c, s = math.cos(self.current_yaw), math.sin(self.current_yaw)
            hold_x = -self.hold_kp_x * ex * c - self.hold_kp_y * ey * s
            hold_y = self.hold_kp_x * ex * s - self.hold_kp_y * ey * c
            if self.hold_x_flag and self.hold_y_flag and self.hold_z_flag != 1:
                rate = w if self.hold_yaw_flag == 0 else self._yaw_hold_rate()
                self._hold_target(FRAME_BODY_NED, 1, hold_x, hold_y, z, yaw_rate=rate)
            elif self.hold_x_flag and self.hold_yaw_flag:
                self._hold_target(FRAME_BODY_NED, 1, hold_x, y, z, yaw_rate=self._yaw_hold_rate())
            elif self.hold_y_flag and self.hold_yaw_flag:
                self._hold_target(FRAME_BODY_NED, 1, x, hold_y, z, yaw_rate=self._yaw_hold_rate())
            elif self.hold_z_flag:
                rate = w if self.hold_yaw_flag == 0 else self._yaw_hold_rate()
                self._hold_target(FRAME_BODY_NED, 1, x, y, z, yaw_rate=rate)
            else:
                self.hold_flag = 0
        else:  # enu
            if abs(x) < 0.02:
                if self.hold_x_flag == 0:
                    self.hold_position_x = p.x
                    self.hold_x_flag = 1
                x = -self.hold_kp_x * (p.x - self.hold_position_x)
            else:
                self.hold_x_flag = 0
            if abs(y) < 0.02:
                if self.hold_y_flag == 0:
                    self.hold_position_y = p.y
                    self.hold_y_flag = 1
                y = -self.hold_kp_y * (p.y - self.hold_position_y)
            else:
                self.hold_y_flag = 0
            if abs(z) < 0.02:
                if self.hold_z_flag == 0:
                    self.hold_position_z = p.z
                    self.hold_z_flag = 1
                z = -self.hold_kp_z * (p.z - self.hold_position_z)
            else:
                self.hold_z_flag = 0
            if abs(w) < 0.005:
                if self.hold_yaw_flag == 0:
                    self.hold_yaw = self.current_yaw
                    self.hold_yaw_flag = 1
            else:
                self.hold_yaw_flag = 0
            if self.hold_x_flag or self.hold_y_flag or self.hold_z_flag:
                if self.hold_yaw_flag == 0:
                    self._hold_target(FRAME_LOCAL_NED, 1, x, y, z, yaw_rate=w)
                else:
                    self._hold_target(FRAME_LOCAL_NED, 3, x, y, z, yaw=self.hold_yaw)
            else:
                self.hold_flag = 0

    def _hold_target(self, frame, motion_type, vx, vy, vz, yaw=0.0, yaw_rate=0.0):
        self.hold_flag = 1
        self.coordinate_frame = frame
        self.motion_type = motion_type
        self.target_motion = self.construct_target(vx=vx, vy=vy, vz=vz, yaw=yaw, yaw_rate=yaw_rate)


def _args(argv, cls):
    if len(argv) < 2:
        raise SystemExit("usage: <vehicle_type> <vehicle_id>   e.g. iris 0")
    return cls(argv[0], argv[1])


def main():
    run(lambda argv: _args(argv, MultirotorCommunication))


def main_enhanced():
    run(lambda argv: _args(argv, EnhancedMultirotorCommunication))
