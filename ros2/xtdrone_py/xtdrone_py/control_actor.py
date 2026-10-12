"""Walk an actor to a point (port of control/actor/control_actor.py, chapter 38).

  ros2 run xtdrone_py control_actor 0 -34 34 5          # actor_0 -> (-34, 34) at 5 m/s
  ros2 launch xtdrone_py control_actors.launch.py         # the six robocup actors (control_actors.sh)

Publishes /actor_<id>/cmd_motion (geometry_msgs/Vector3: x, y, z = speed) at 20 Hz. In ROS 1 the
message was ros_actor_cmd_pose_plugin_msgs/ActorMotion (x, y, v). xtdrone.launch.py bridges the
topic to the xtdrone_gz_plugins ActorMotion system of every actor that has it (robocup world).
"""
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from xtdrone_py.common import run


class ControlActor(Node):

    def __init__(self, actor_id, x, y, v):
        super().__init__(f"control_actor_{actor_id}")
        self.msg = Vector3(x=x, y=y, z=v)
        self.pub = self.create_publisher(Vector3, f"/actor_{actor_id}/cmd_motion", 1)
        self.create_timer(1.0 / 20.0, lambda: self.pub.publish(self.msg))


def main():
    def make(argv):
        if len(argv) < 4:
            raise SystemExit("usage: control_actor <actor id> <x> <y> <speed>")
        return ControlActor(argv[0], float(argv[1]), float(argv[2]), float(argv[3]))
    run(make)
