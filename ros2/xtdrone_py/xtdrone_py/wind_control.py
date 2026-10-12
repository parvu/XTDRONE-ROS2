"""Change the wind of a running Gazebo Harmonic world from ROS 2
(replaces XTDrone's libgazebo_ros_wind_plugin_xtdrone.so).

  ros2 run xtdrone_py wind_control [--world outdoor2]
  ros2 topic pub --once /wind_xtdrone geometry_msgs/msg/Twist "{linear: {x: 3.0, y: 0.0, z: 0.0}}"

The linear part of /wind_xtdrone (m/s, world ENU frame) becomes the world wind
(gz.msgs.Wind on /world/<world>/wind, applied by the WindEffects system).
"""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from xtdrone_py.gazebo_models import detect_world

try:
    from gz.msgs10.wind_pb2 import Wind
    from gz.transport13 import Node as GzNode
except ImportError:  # pragma: no cover
    GzNode = None


class WindControl(Node):

    def __init__(self, world):
        super().__init__("wind_control")
        if GzNode is None:
            raise SystemExit("gz-transport Python bindings not found (install gz-harmonic)")
        self.gz = GzNode()
        self.topic = f"/world/{world}/wind"
        self.pub = self.gz.advertise(self.topic, Wind)
        self.create_subscription(Twist, "/wind_xtdrone", self.callback, 1)
        print(f"wind_control: /wind_xtdrone -> {self.topic}", flush=True)

    def callback(self, msg):
        wind = Wind()
        wind.linear_velocity.x = msg.linear.x
        wind.linear_velocity.y = msg.linear.y
        wind.linear_velocity.z = msg.linear.z
        wind.enable_wind = True
        self.pub.publish(wind)
        print(f"wind set to ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}) m/s", flush=True)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=None)
    args = ap.parse_args(argv)
    rclpy.init()
    node = WindControl(args.world or detect_world())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
