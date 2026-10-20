"""XTDrone fixed-wing communication (port of communication/plane_communication.py).

  ros2 run xtdrone_py plane_communication 0
"""
import math

from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

from xtdrone_py.common import FRAME_LOCAL_NED, MASK_POSITION, MAVROS_QOS, VehicleNode, q2yaw, run

# ROS 1 mission words -> the XTDrone mode names they stand for (ArduPlane: TAKEOFF, GUIDED loiter, RTL)
MISSION_MODES = {"takeoff": "AUTO.TAKEOFF", "land": "AUTO.LAND", "loiter": "OFFBOARD"}


class PlaneCommunication(VehicleNode):

    def __init__(self, vehicle_id):
        super().__init__("communication", "plane", vehicle_id, telemetry=True)
        self.local_pose = None
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = FRAME_LOCAL_NED  # default 0 (GLOBAL) is rejected by PX4
        self.coordinate_frame = FRAME_LOCAL_NED
        self.mission = None
        self.target_set = False  # a position setpoint sent before the first command would send the plane to the origin
        n, x = self.name, f"/xtdrone/{self.name}"
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(Pose, f"{x}/cmd_pose_flu", self.cmd_pose_flu_callback, 1)
        self.create_subscription(Pose, f"{x}/cmd_pose_enu", self.cmd_pose_enu_callback, 1)
        self.create_subscription(String, f"{x}/cmd", self.cmd_callback, 3)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 1)
        self.create_timer(1.0 / 30.0, self.publish_target)
        self.log("communication initialized")

    def publish_target(self):
        """ArduPlane ignores position setpoints (SET_POSITION_TARGET_LOCAL_NED only changes the altitude): the target
        goes to it as MAV_CMD_DO_REPOSITION, 4 times a second (the plane loiters around it)."""
        self._tick = getattr(self, "_tick", 0) + 1
        if not self.target_set or self.local_pose is None or self._tick % 8:
            return
        t, p = self.target_motion, self.local_pose.pose.position
        x, y, z = t.position.x, t.position.y, t.position.z
        if t.coordinate_frame != FRAME_LOCAL_NED:  # offset from the plane, body FLU
            yaw = q2yaw(self.local_pose.pose.orientation)
            c, s = math.cos(yaw), math.sin(yaw)
            x, y, z = p.x + c * x - s * y, p.y + s * x + c * y, p.z + z
        self.reposition(x, y, max(z, 5.0))

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def construct_target(self, x=0.0, y=0.0, z=0.0):
        t = PositionTarget()
        t.coordinate_frame = self.coordinate_frame
        if self.coordinate_frame == FRAME_LOCAL_NED:
            t.position.x, t.position.y = float(x), float(y)
        else:
            t.position.x, t.position.y = float(-y), float(x)
        t.position.z = float(z)
        t.type_mask = MASK_POSITION
        self.target_set = True
        return t

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        self.target_motion = self.construct_target(msg.position.x, msg.position.y, msg.position.z)

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = FRAME_LOCAL_NED
        self.target_motion = self.construct_target(msg.position.x, msg.position.y, msg.position.z)

    def cmd_callback(self, msg):
        if msg.data == "":
            return
        if msg.data == "ARM":
            self.arm(True)
        elif msg.data == "DISARM":
            self.arm(False)
        elif msg.data in ("takeoff", "land", "loiter", "idle"):
            self.mission = msg.data
            self.log(self.mission)
            if msg.data in MISSION_MODES:
                self.set_mode(MISSION_MODES[msg.data])
        else:
            self.set_mode(msg.data)


def main():
    def make(argv):
        if not argv:
            raise SystemExit("usage: plane_communication <vehicle_id>")
        return PlaneCommunication(argv[0])
    run(make)
