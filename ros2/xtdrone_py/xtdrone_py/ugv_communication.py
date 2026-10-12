"""XTDrone UGV command -> gz AckermannSteering (replaces catvehicle's cmdvel2gazebo controller).

  ros2 run xtdrone_py ugv_communication 0      (started by xtdrone.launch.py for every ugv)

/ugv_<id>/cmd_vel: linear.x = speed (m/s), angular.z = front steering angle (rad), as sent by
ugv_keyboard_control and XTDGroundControl. AckermannSteering takes a yaw rate instead:
yaw_rate = v * tan(steering) / wheel_base. Published on /ugv_<id>/ackermann_cmd_vel (bridged to
/model/ugv_<id>/cmd_vel).
"""
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node

from xtdrone_py.common import run

WHEEL_BASE = 2.6       # m, CAT Vehicle (generate_ugv_models.py)
STEERING_LIMIT = 0.6   # rad


def steering_to_yaw_rate(speed, steering, wheel_base=WHEEL_BASE, limit=STEERING_LIMIT):
    steering = max(-limit, min(limit, steering))
    return speed * math.tan(steering) / wheel_base


class UGVCommunication(Node):

    def __init__(self, ugv_id):
        super().__init__(f"ugv_{ugv_id}_communication")
        self.pub = self.create_publisher(Twist, f"/ugv_{ugv_id}/ackermann_cmd_vel", 1)
        self.create_subscription(Twist, f"/ugv_{ugv_id}/cmd_vel", self.cmd_vel_callback, 1)
        print(f"ugv_{ugv_id}: communication node started", flush=True)

    def cmd_vel_callback(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = steering_to_yaw_rate(msg.linear.x, msg.angular.z)
        self.pub.publish(out)


def main():
    def make(argv):
        if len(argv) < 1:
            raise SystemExit("usage: ugv_communication <id>")
        return UGVCommunication(int(argv[0]))
    run(make)
