"""Pairwise collision avoidance for the formation demo (port of
coordination/formation_demo/avoid.py, chapter 24).

  ros2 run xtdrone_formation_demo avoid iris 9 vel

Publishes /xtdrone/<type>_<i>/avoid_vel (or avoid_accel): for every pair closer than
avoid_radius, a push perpendicular to the line between them, scaled by how close they are.
"""
import argparse

import numpy
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

AID_VEC1 = numpy.array([1.0, 0.0, 0.0])
AID_VEC2 = numpy.array([0.0, 1.0, 0.0])


def avoid_controls(positions, avoid_radius=1.5):
    """Avoidance vector for each position (list of 3-arrays, None for unknown)."""
    n = len(positions)
    out = [numpy.zeros(3) for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if positions[i] is None or positions[j] is None:
                continue
            d = positions[i] - positions[j]
            dist = numpy.linalg.norm(d)
            k = 1 - dist / avoid_radius
            if k <= 0 or dist == 0:
                continue
            cos1 = d.dot(AID_VEC1) / dist
            cos2 = d.dot(AID_VEC2) / dist
            aid = AID_VEC1 if abs(cos1) < abs(cos2) else AID_VEC2
            c = numpy.cross(d, aid)
            push = k * c / numpy.linalg.norm(c)
            out[i] += push
            out[j] -= push
    return out


class Avoid(Node):

    def __init__(self, vehicle_type, vehicle_num, control_type):
        super().__init__("avoid")
        self.positions = [None] * vehicle_num
        self.pubs = []
        for i in range(vehicle_num):
            self.create_subscription(PoseStamped, f"{vehicle_type}_{i}/mavros/local_position/pose",
                                     lambda msg, i=i: self.pose_callback(i, msg), qos_profile_sensor_data)
            self.pubs.append(self.create_publisher(Vector3, f"/xtdrone/{vehicle_type}_{i}/avoid_{control_type}", 1))
        self.create_timer(1.0 / 30.0, self.timer_callback)

    def pose_callback(self, i, msg):
        p = msg.pose.position
        self.positions[i] = numpy.array([p.x, p.y, p.z])

    def timer_callback(self):
        for pub, c in zip(self.pubs, avoid_controls(self.positions)):
            pub.publish(Vector3(x=float(c[0]), y=float(c[1]), z=float(c[2])))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vehicle_type")
    parser.add_argument("vehicle_num", type=int)
    parser.add_argument("control_type", nargs="?", default="vel", choices=("vel", "accel"))
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Avoid(args.vehicle_type, args.vehicle_num, args.control_type)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
