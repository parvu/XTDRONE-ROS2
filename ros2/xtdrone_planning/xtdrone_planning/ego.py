"""EGO-Planner glue for XTDrone (chapters 21 and 22).

  ros2 run xtdrone_planning ego_transfer iris 0       # motion_planning/3d/ego_transfer.py
  ros2 run xtdrone_planning ego_pos_cmd iris 0        # traj_server's pose_cmd in XTDrone's ROS 1 EGO fork

ego_transfer publishes /<v>/ego/odom (MAVROS odometry, reliable QoS) and /<v>/camera_pose: the pose of the depth camera (optical frame) in EGO's world
frame, from the vehicle pose (MAVROS local position, which with ekf:=vision is the ground truth or
the VIO pose). EGO's grid map projects the depth image with it.
ego_pos_cmd turns EGO's trajectory server output (quadrotor_msgs/PositionCommand on
/xtdrone/<v>/planning/pos_cmd) into the XTDrone position command /xtdrone/<v>/cmd_pose_enu (position
and yaw), as XTDrone's modified ROS 1 traj_server did.
"""
import math

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from xtdrone_py.common import run

# optical (x right, y down, z forward) in FLU body: rotation rpy (-pi/2, 0, -pi/2), as ego_transfer.py
Q_OPTICAL = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w
CAMERA_OFFSET = (0.12, 0.0, 0.03)   # realsense_link in base_link (iris_realsense_camera)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    x, y, z, w = q
    u = (x, y, z)
    c1 = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    c1 = (c1[0] + w * v[0], c1[1] + w * v[1], c1[2] + w * v[2])
    c2 = (u[1] * c1[2] - u[2] * c1[1], u[2] * c1[0] - u[0] * c1[2], u[0] * c1[1] - u[1] * c1[0])
    return (v[0] + 2 * c2[0], v[1] + 2 * c2[1], v[2] + 2 * c2[2])


class EgoTransfer(Node):

    def __init__(self, vtype, vid):
        n = f"{vtype}_{vid}"
        super().__init__(f"{n}_ego_transfer")
        self.pub = self.create_publisher(PoseStamped, f"/{n}/camera_pose", 10)
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.callback,
                                 qos_profile_sensor_data)
        # EGO subscribes to its odometry as reliable; MAVROS publishes best effort
        self.odom_pub = self.create_publisher(Odometry, f"/{n}/ego/odom", 10)
        self.create_subscription(Odometry, f"/{n}/mavros/local_position/odom", self.odom_callback,
                                 qos_profile_sensor_data)

    def odom_callback(self, msg):
        msg.header.frame_id = "world"
        self.odom_pub.publish(msg)

    def callback(self, msg):
        o = msg.pose.orientation
        q_body = (o.x, o.y, o.z, o.w)
        off = rotate(q_body, CAMERA_OFFSET)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "world"
        p = msg.pose.position
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p.x + off[0], p.y + off[1], p.z + off[2]
        q = quat_mul(q_body, Q_OPTICAL)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
        self.pub.publish(out)


class EgoPosCmd(Node):

    def __init__(self, vtype, vid):
        from quadrotor_msgs.msg import PositionCommand  # from the EGO-Planner workspace
        n = f"{vtype}_{vid}"
        super().__init__(f"{n}_ego_pos_cmd")
        self.pub = self.create_publisher(Pose, f"/xtdrone/{n}/cmd_pose_enu", 10)
        self.create_subscription(PositionCommand, f"/xtdrone/{n}/planning/pos_cmd", self.callback, 10)

    def callback(self, msg):
        p = Pose()
        p.position.x, p.position.y, p.position.z = msg.position.x, msg.position.y, msg.position.z
        p.orientation.z, p.orientation.w = math.sin(msg.yaw / 2), math.cos(msg.yaw / 2)
        self.pub.publish(p)


def transfer_main():
    run(lambda argv: EgoTransfer(argv[0], argv[1]))


def pos_cmd_main():
    run(lambda argv: EgoPosCmd(argv[0], argv[1]))
