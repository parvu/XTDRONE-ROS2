"""Republish an odometry as odom -> base_link (topic and TF) for RTAB-Map (chapter 16).

  ros2 run xtdrone_slam odom_relay /iris_0/ground_truth/odom      # -> /xtdrone/odom + TF odom -> base_link

The Gazebo ground truth carries gz frame names (iris_0/odom, iris_0); RTAB-Map wants one odometry
frame pair that matches the TF tree of the launch (odom -> base_link -> depth_camera_base).
Stamps are kept (simulation time).
"""
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):

    def __init__(self, source, target="/xtdrone/odom"):
        super().__init__("xtdrone_odom_relay")
        self.pub = self.create_publisher(Odometry, target, 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, source, self.callback, 10)

    def callback(self, msg):
        msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
        self.pub.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = "base_link"
        p = msg.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not argv:
        raise SystemExit("usage: odom_relay <odometry topic> [output topic]")
    rclpy.init()
    node = OdomRelay(*argv[:2])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
