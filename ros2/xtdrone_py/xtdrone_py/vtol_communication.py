"""XTDrone VTOL communication (port of communication/vtol_communication.py).

  ros2 run xtdrone_py vtol_communication standard_vtol 0

Commands on /xtdrone/<type>_<id>/cmd: 'multirotor' / 'plane' switch the VTOL
transition state, 'loiter' / 'idle' the fixed-wing mission, others set the mode.
"""
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandVtolTransition
from std_msgs.msg import String

from xtdrone_py.common import (FRAME_BODY_NED, FRAME_BODY_OFFSET_NED, FRAME_LOCAL_NED, MAVROS_QOS,
                               MOTION_MASKS, VehicleNode, q2yaw, run)
from xtdrone_py.plane_communication import PLANE_IDLE_MASK, PLANE_MISSION_MASKS

MC_STATE, FW_STATE = 3, 4  # MAV_VTOL_STATE_MC / _FW


class VtolCommunication(VehicleNode):

    def __init__(self, vehicle_type, vehicle_id):
        super().__init__("communication", vehicle_type, vehicle_id, telemetry=True)
        self.local_pose = None
        self.current_yaw = 0.0
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = FRAME_LOCAL_NED  # default 0 (GLOBAL) is rejected by PX4
        self.hover_flag = 0
        self.coordinate_frame = FRAME_LOCAL_NED
        self.motion_type = 0
        self.flight_mode = None
        self.plane_mission = None
        self.transition_state = "multirotor"

        n, x = self.name, f"/xtdrone/{self.name}"
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(Pose, f"{x}/cmd_pose_flu", self.cmd_pose_flu_callback, 1)
        self.create_subscription(Pose, f"{x}/cmd_pose_enu", self.cmd_pose_enu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_flu", self.cmd_vel_flu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_enu", self.cmd_vel_enu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_accel_flu", self.cmd_accel_flu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_accel_enu", self.cmd_accel_enu_callback, 1)
        self.create_subscription(String, f"{x}/cmd", self.cmd_callback, 3)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 1)
        self.transition_client = self.create_client(CommandVtolTransition, f"/{n}/mavros/cmd/vtol_transition")
        self.create_timer(1.0 / 30.0, self.publish_target)
        self.log(self.transition_state)
        self.transition(MC_STATE)
        self.log("communication initialized")

    def transition(self, state):
        req = CommandVtolTransition.Request()
        req.state = state
        label = "multirotor" if state == MC_STATE else "plane"
        self.call(self.transition_client, req, f"transition to {label}", f"transition to {label} failed",
                  check=lambda r: r.success)

    def publish_target(self):
        self.target_motion.header.stamp = self.get_clock().now().to_msg()
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.local_pose = msg.pose
        self.current_yaw = q2yaw(msg.pose.orientation)

    def construct_target(self, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0,
                         afx=0.0, afy=0.0, afz=0.0, yaw=0.0, yaw_rate=0.0):
        t = PositionTarget()
        t.coordinate_frame = self.coordinate_frame
        enu = self.coordinate_frame == FRAME_LOCAL_NED
        t.position.x, t.position.y = (float(x), float(y)) if enu else (float(-y), float(x))
        t.position.z = float(z)
        if self.transition_state == "plane":
            t.type_mask = PLANE_MISSION_MASKS.get(self.plane_mission, PLANE_IDLE_MASK)
            return t
        t.velocity.x, t.velocity.y = (float(vx), float(vy)) if enu else (float(-vy), float(vx))
        t.velocity.z = float(vz)
        ax, ay = (afx, afy) if enu else (-afy, afx)
        t.acceleration_or_force.x, t.acceleration_or_force.y = float(ax), float(ay)
        t.acceleration_or_force.z = float(afz)
        t.yaw, t.yaw_rate = float(yaw), float(yaw_rate)
        t.type_mask = MOTION_MASKS[self.motion_type]
        return t

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = FRAME_BODY_OFFSET_NED
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z,
                                                   yaw=q2yaw(msg.orientation))

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = FRAME_LOCAL_NED
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z,
                                                   yaw=q2yaw(msg.orientation))

    def _motion(self, msg, frame, motion_type):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = frame
            self.motion_type = motion_type
            if motion_type == 1:
                self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
                                                           yaw_rate=msg.angular.z)
            else:
                self.target_motion = self.construct_target(afx=msg.linear.x, afy=msg.linear.y, afz=msg.linear.z,
                                                           yaw_rate=msg.angular.z)

    def cmd_vel_flu_callback(self, msg):
        self._motion(msg, FRAME_BODY_NED, 1)

    def cmd_vel_enu_callback(self, msg):
        self._motion(msg, FRAME_LOCAL_NED, 1)

    def cmd_accel_flu_callback(self, msg):
        self._motion(msg, FRAME_BODY_NED, 2)

    def cmd_accel_enu_callback(self, msg):
        self._motion(msg, FRAME_LOCAL_NED, 2)

    def hover_state_transition(self, x, y, z, w):
        if abs(x) > 0.005 or abs(y) > 0.005 or abs(z) > 0.005 or abs(w) > 0.005:
            self.hover_flag = 0
            self.flight_mode = "OFFBOARD"

    def cmd_callback(self, msg):
        data = msg.data
        if data == "":
            return
        if data == "ARM":
            self.arm(True)
        elif data == "DISARM":
            self.arm(False)
        elif data in ("multirotor", "plane"):
            self.transition_state = data
            self.log(data)
            self.transition(MC_STATE if data == "multirotor" else FW_STATE)
        elif data in ("loiter", "idle"):
            self.plane_mission = data
            self.log(data)
        elif data == "HOVER":
            self.flight_mode = data
            self.hover_flag = 1
            self.hover()
            self.log("Hover")
        else:
            self.flight_mode = data
            self.hover_flag = 0
            self.set_mode(data)

    def hover(self):
        if self.local_pose is None:
            return
        self.coordinate_frame = FRAME_LOCAL_NED
        self.motion_type = 0
        p = self.local_pose.position
        self.target_motion = self.construct_target(x=p.x, y=p.y, z=p.z, yaw=self.current_yaw)


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: vtol_communication <vehicle_type> <vehicle_id>   e.g. standard_vtol 0")
        return VtolCommunication(argv[0], argv[1])
    run(make)
