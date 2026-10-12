"""Ground-truth pose for PX4 (port of sensing/pose_ground_truth/get_local_pose.py).

  ros2 run xtdrone_py get_local_pose iris 1

Reads /<type>_<i>/ground_truth/odom (gz OdometryPublisher, bridged by
xtdrone.launch.py) and publishes /<type>_<i>/mavros/vision_pose/pose and
/<type>_<i>/mavros/vision_speed/speed at 30 Hz, like the Gazebo Classic version
did from /gazebo/model_states. Set EKF2_EV_CTRL / EKF2_HGT_REF for vision as
described in the PX4 EKF chapter.
"""
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry

from xtdrone_py.common import run
from rclpy.node import Node


def rotate(q, v):
    """Rotate vector v (x, y, z) by quaternion q."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # t = 2 * cross(q.xyz, v)
    tx, ty, tz = 2 * (qy * z - qz * y), 2 * (qz * x - qx * z), 2 * (qx * y - qy * x)
    return (x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx))


class GroundTruth(Node):

    def __init__(self, vehicle_type, vehicle_num):
        super().__init__(f"{vehicle_type}_get_pose_groundtruth")
        self.names = [f"{vehicle_type}_{i}" for i in range(vehicle_num)]
        self.poses = {n: None for n in self.names}
        self.speeds = {n: None for n in self.names}
        self.pose_pubs, self.speed_pubs = {}, {}
        for n in self.names:
            self.create_subscription(Odometry, f"/{n}/ground_truth/odom",
                                     lambda msg, n=n: self.odom_callback(n, msg), 1)
            self.pose_pubs[n] = self.create_publisher(PoseStamped, f"/{n}/mavros/vision_pose/pose", 1)
            self.speed_pubs[n] = self.create_publisher(Vector3Stamped, f"/{n}/mavros/vision_speed/speed", 1)
            print(f"Get {n} groundtruth pose", flush=True)
        self.create_timer(1.0 / 30.0, self.publish)

    def odom_callback(self, name, msg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        speed = Vector3Stamped()
        speed.header.frame_id = "map"
        # OdometryPublisher twist is in the body frame; MAVROS expects ENU world velocity.
        lin = msg.twist.twist.linear
        speed.vector.x, speed.vector.y, speed.vector.z = rotate(msg.pose.pose.orientation, (lin.x, lin.y, lin.z))
        self.poses[name], self.speeds[name] = pose, speed

    def publish(self):
        now = self.get_clock().now().to_msg()
        for n in self.names:
            if self.poses[n] is None:
                continue
            self.poses[n].header.stamp = now
            self.speeds[n].header.stamp = now
            self.pose_pubs[n].publish(self.poses[n])
            self.speed_pubs[n].publish(self.speeds[n])


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: get_local_pose <vehicle_type> <vehicle_num>   e.g. iris 1")
        return GroundTruth(argv[0], int(argv[1]))
    run(make)
