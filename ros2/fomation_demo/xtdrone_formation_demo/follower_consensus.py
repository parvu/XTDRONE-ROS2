"""Distributed formation follower with a consensus protocol (port of
coordination/formation_demo/follower_consensus.py, chapter 24).

  ros2 run xtdrone_formation_demo follower_consensus iris 1 9 vel

Each follower tracks its local leaders from /xtdrone/communication_topology (built by the
leader) instead of only the global leader. See https://arxiv.org/abs/2005.01125.
"""
import argparse

import numpy
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class Follower(Node):

    def __init__(self, uav_type, uav_id, uav_num, control_type):
        super().__init__("follower" + str(uav_id - 1))
        self.uav_type = uav_type
        self.id = uav_id
        self.uav_num = uav_num
        self.control_type = control_type
        self.f = 30
        self.formation_pattern = None
        self.communication_topology = None
        self.local_leader_ids = []
        self.Kp = 1.0
        self.Kp_avoid = 1.0
        self.weight = 1.0
        self.pose = [PoseStamped() for _ in range(self.uav_num)]
        self.create_subscription(Float32MultiArray, "/xtdrone/formation_pattern", self.formation_pattern_callback, 10)
        self.create_subscription(Int32MultiArray, "/xtdrone/communication_topology",
                                 self.communication_topology_callback, 10)
        for i in range(self.uav_num):
            self.create_subscription(PoseStamped, f"{self.uav_type}_{i}/mavros/local_position/pose",
                                     lambda msg, i=i: self.pose.__setitem__(i, msg), qos_profile_sensor_data)
        prefix = f"/xtdrone/{self.uav_type}_{self.id}"
        if self.control_type == "vel":
            self.vel_max = 1.0
            self.avoid = Vector3()
            self.create_subscription(Vector3, prefix + "/avoid_vel", self.avoid_callback, 10)
            self.cmd_pub = self.create_publisher(Twist, prefix + "/cmd_vel_enu", 10)
        else:
            self.accel_max = 1.0
            self.gamma = (4.0 / self.Kp) ** 0.5
            self.avoid = Vector3()
            self.velocity = [TwistStamped() for _ in range(self.uav_num)]
            self.create_subscription(Vector3, prefix + "/avoid_accel", self.avoid_callback, 10)
            self.cmd_pub = self.create_publisher(Twist, prefix + "/cmd_accel_enu", 10)
            for i in range(self.uav_num):
                self.create_subscription(TwistStamped, f"{self.uav_type}_{i}/mavros/local_position/velocity_local",
                                         lambda msg, i=i: self.velocity.__setitem__(i, msg), qos_profile_sensor_data)
        self.create_timer(1.0 / self.f, self.timer_callback)

    def formation_pattern_callback(self, msg):
        self.formation_pattern = numpy.array(msg.data).reshape(3, self.uav_num - 1)

    def communication_topology_callback(self, msg):
        self.communication_topology = numpy.array(msg.data).reshape(self.uav_num, self.uav_num)
        self.local_leader_ids = numpy.flatnonzero(self.communication_topology[self.id, :] == 1)
        self.weight = self.Kp / max(len(self.local_leader_ids), 1)

    def avoid_callback(self, msg):
        self.avoid = msg

    def position(self, i):
        p = self.pose[i].pose.position
        return numpy.array([p.x, p.y, p.z])

    def offset(self, i):
        """Formation offset of vehicle i relative to the global leader (zero for the leader)."""
        return numpy.zeros(3) if i == 0 else self.formation_pattern[:, i - 1]

    def timer_callback(self):
        if self.communication_topology is None or self.formation_pattern is None:
            return
        own = self.position(self.id) - self.offset(self.id)
        u = numpy.zeros(3)
        for j in self.local_leader_ids:
            u += (self.position(j) - self.offset(j)) - own
            if self.control_type == "accel":
                vj, vi = self.velocity[j].twist.linear, self.velocity[self.id].twist.linear
                u += self.gamma * numpy.array([vj.x - vi.x, vj.y - vi.y, vj.z - vi.z])
        a = self.avoid
        cmd = self.weight * u + self.Kp_avoid * numpy.array([a.x, a.y, a.z])
        limit = self.vel_max if self.control_type == "vel" else self.accel_max
        magnitude = numpy.linalg.norm(cmd)
        if magnitude > 3 ** 0.5 * limit:
            cmd = cmd / magnitude * limit
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = (float(c) for c in cmd)
        self.cmd_pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("uav_type")
    parser.add_argument("uav_id", type=int)
    parser.add_argument("uav_count", type=int, choices=(6, 9, 18))
    parser.add_argument("control_type", nargs="?", default="vel", choices=("vel", "accel"))
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.uav_id < args.uav_count:
        parser.error("uav_id must identify a follower between 1 and uav_count - 1")
    rclpy.init(args=ros_args)
    node = Follower(args.uav_type, args.uav_id, args.uav_count, args.control_type)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
