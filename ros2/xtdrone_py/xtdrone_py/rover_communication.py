"""XTDrone rover communication (port of communication/rover_communication.py).

  ros2 run xtdrone_py rover_communication 0

cmd_vel_*: linear.x is the forward speed and angular.z the steering command.
"""
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String

from xtdrone_py.common import (FRAME_BODY_NED, FRAME_BODY_OFFSET_NED, FRAME_LOCAL_NED, MASK_POSITION,
                               MAVROS_QOS, VehicleNode, run)
ROVER_MASK_VELOCITY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                       | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                       | PositionTarget.IGNORE_YAW_RATE)


class RoverCommunication(VehicleNode):

    def __init__(self, vehicle_id):
        super().__init__("communication", "rover", vehicle_id)
        self.local_pose = None
        self.mavros_state = None
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = FRAME_LOCAL_NED  # default 0 (GLOBAL) is rejected by PX4
        self.mission = None
        self.motion_type = 1
        self.coordinate_frame = FRAME_LOCAL_NED
        n, x = self.name, f"/xtdrone/{self.name}"
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(State, f"/{n}/mavros/state", self.mavros_state_callback, MAVROS_QOS)
        self.create_subscription(Pose, f"{x}/cmd_pose_flu", self.cmd_pose_flu_callback, 1)
        self.create_subscription(Pose, f"{x}/cmd_pose_enu", self.cmd_pose_enu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_flu", self.cmd_vel_flu_callback, 1)
        self.create_subscription(Twist, f"{x}/cmd_vel_enu", self.cmd_vel_enu_callback, 1)
        self.create_subscription(String, f"{x}/cmd", self.cmd_callback, 3)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 1)
        self.create_timer(1.0 / 30.0, self.publish_target)
        self.log("communication initialized")

    def publish_target(self):
        self.target_motion.header.stamp = self.get_clock().now().to_msg()
        self.target_motion_pub.publish(self.target_motion)

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def construct_target(self, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        t = PositionTarget()
        t.coordinate_frame = self.coordinate_frame
        t.position.x, t.position.y, t.position.z = float(x), float(y), float(z)
        t.velocity.x, t.velocity.y, t.velocity.z = float(vx), float(vy), float(vz)
        t.yaw = float(yaw)
        t.type_mask = MASK_POSITION if self.motion_type == 0 else ROVER_MASK_VELOCITY
        return t

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame, self.motion_type = FRAME_BODY_OFFSET_NED, 0
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z)

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame, self.motion_type = FRAME_LOCAL_NED, 0
        self.target_motion = self.construct_target(x=msg.position.x, y=msg.position.y, z=msg.position.z)

    def cmd_vel_flu_callback(self, msg):
        self.coordinate_frame, self.motion_type = FRAME_BODY_NED, 1
        self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.angular.z, vz=msg.linear.z)

    def cmd_vel_enu_callback(self, msg):
        self.coordinate_frame, self.motion_type = FRAME_LOCAL_NED, 1
        self.target_motion = self.construct_target(vx=msg.linear.x, vy=msg.angular.z, vz=msg.linear.z)

    def cmd_callback(self, msg):
        if msg.data == "":
            return
        if msg.data == "ARM":
            self.arm(True)
        elif msg.data == "DISARM":
            self.arm(False)
        elif msg.data[:-1] == "mission" and msg.data != self.mission:
            self.mission = msg.data
            self.log(msg.data)
        else:
            self.set_mode(msg.data)


def main():
    def make(argv):
        if not argv:
            raise SystemExit("usage: rover_communication <vehicle_id>")
        return RoverCommunication(argv[0])
    run(make)
