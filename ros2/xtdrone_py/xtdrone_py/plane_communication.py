"""XTDrone fixed-wing communication (port of communication/plane_communication.py).

  ros2 run xtdrone_py plane_communication 0
"""
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

from xtdrone_py.common import FRAME_LOCAL_NED, MAVROS_QOS, VehicleNode, run

# PX4 position setpoint type bits carried in POSITION_TARGET type_mask.
PLANE_MISSION_MASKS = {"takeoff": 4096, "land": 8192, "loiter": 12288}
PLANE_IDLE_MASK = 16384


class PlaneCommunication(VehicleNode):

    def __init__(self, vehicle_id):
        super().__init__("communication", "plane", vehicle_id)
        self.local_pose = None
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = FRAME_LOCAL_NED  # default 0 (GLOBAL) is rejected by PX4
        self.coordinate_frame = FRAME_LOCAL_NED
        self.mission = None
        n, x = self.name, f"/xtdrone/{self.name}"
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose_callback, MAVROS_QOS)
        self.create_subscription(Pose, f"{x}/cmd_pose_flu", self.cmd_pose_flu_callback, 1)
        self.create_subscription(Pose, f"{x}/cmd_pose_enu", self.cmd_pose_enu_callback, 1)
        self.create_subscription(String, f"{x}/cmd", self.cmd_callback, 3)
        self.target_motion_pub = self.create_publisher(PositionTarget, f"/{n}/mavros/setpoint_raw/local", 1)
        self.create_timer(1.0 / 30.0, self.publish_target)
        self.log("communication initialized")

    def publish_target(self):
        self.target_motion.header.stamp = self.get_clock().now().to_msg()
        self.target_motion_pub.publish(self.target_motion)

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
        t.type_mask = PLANE_MISSION_MASKS.get(self.mission, PLANE_IDLE_MASK)
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
            # Re-send the last setpoint with the new mission type.
            p = self.target_motion.position
            if self.coordinate_frame == FRAME_LOCAL_NED:
                self.target_motion = self.construct_target(p.x, p.y, p.z)
            else:
                self.target_motion = self.construct_target(p.y, -p.x, p.z)
        else:
            self.set_mode(msg.data)


def main():
    def make(argv):
        if not argv:
            raise SystemExit("usage: plane_communication <vehicle_id>")
        return PlaneCommunication(argv[0])
    run(make)
