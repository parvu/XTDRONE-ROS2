"""Walk a converted world's script actor once the YOLO chapter is up (chapter 23).

  ros2 run xtdrone_py actor_walker actor iris_gimbal 0 --loop "0,3;40,41,3;..." [--z 2.0]

xtdrone.launch.py starts one per actor whose xtdrone_gz_plugins ActorMotion plugin carries a
<walk_loop> (classic_to_gz.py moved the waypoints of the Classic <script> there; gz-sim would
play the script and fight ActorMotion). The actor waits until the drone is above --z m
(mavros local_position) and yolo_detection and yolo_human_tracking are running for that vehicle,
then walks the loop forever through /<actor>/cmd_motion (geometry_msgs/Vector3 x, y, speed;
chapter 38). The gate checks node names, not publishers: the chapter's own test publishes
cmd_vel_flu from the start.
"""
import argparse
import math
import time

from geometry_msgs.msg import Pose, PoseStamped, Vector3
from rclpy.node import Node

from xtdrone_py.common import MAVROS_QOS, run


class ActorWalker(Node):

    def __init__(self, actor, vtype, vid, loop, z_gate=2.0):
        super().__init__(f"actor_walker_{actor}")
        self.vehicle = f"{vtype}_{vid}"
        self.z_gate = z_gate
        self.z, self.pose, self.open = None, None, False
        pts = [[float(v) for v in leg.split(",")] for leg in loop.split(";") if leg.strip()]
        # legs between consecutive waypoints (the loop closes back to the first); zero-length legs
        # are the turn waypoints of the Classic script. speed = distance / the script's time for
        # the leg, clamped so turns and short legs keep walking.
        self.legs = []
        for (t0, x0, y0), (t1, x1, y1) in zip(pts, pts[1:] + pts[:1]):
            dist = math.dist((x0, y0), (x1, y1))
            if dist < 0.05:
                continue
            self.legs.append((x1, y1, min(max(dist / max(t1 - t0, 1.0), 0.2), 2.0), dist))
        self.pub = self.create_publisher(Vector3, f"/{actor}/cmd_motion", 1)
        self.create_subscription(PoseStamped, f"/{self.vehicle}/mavros/local_position/pose",
                                 lambda m: setattr(self, "z", m.pose.position.z), MAVROS_QOS)
        self.create_subscription(Pose, f"/{actor}/pose", lambda m: setattr(self, "pose", m), 10)
        self.i, self.leg_start = 0, 0.0
        self.create_timer(0.5, self.gate)
        if self.legs:
            self.create_timer(0.05, self.walk)
        print(f"{self.get_name()}: waiting for {self.vehicle} above {z_gate:g} m, yolo_detection and "
              "yolo_human_tracking", flush=True)

    def gate(self):
        if self.open or self.z is None or self.z <= self.z_gate:
            return
        names = self.get_node_names()
        if f"yolo_detection_{self.vehicle}" not in names or f"yolo_human_tracking_{self.vehicle}" not in names:
            return
        self.open = True
        self.leg_start = time.time()
        print(f"{self.get_name()}: gate open, walking {len(self.legs)} legs", flush=True)

    def walk(self):
        if not self.open:
            return
        x, y, v, dist = self.legs[self.i]
        self.pub.publish(Vector3(x=x, y=y, z=v))
        # ActorMotion stops within 0.05 m of the goal; the pose bridge is lazy and can lag, so a
        # leg also ends after its script time + 3 s.
        if (self.pose is not None and math.dist((self.pose.position.x, self.pose.position.y), (x, y)) < 0.4
                or time.time() - self.leg_start > dist / v + 3.0):
            self.i = (self.i + 1) % len(self.legs)
            self.leg_start = time.time()


def main():
    def make(argv):
        ap = argparse.ArgumentParser(prog="actor_walker")
        ap.add_argument("actor_name")
        ap.add_argument("vehicle_type")
        ap.add_argument("vehicle_id", type=int)
        ap.add_argument("--loop", required=True, help="walk_loop waypoints: t,x,y;t,x,y;... (classic_to_gz.py)")
        ap.add_argument("--z", type=float, default=2.0, help="open the gate above this drone height (m)")
        a = ap.parse_args(argv)
        return ActorWalker(a.actor_name, a.vehicle_type, a.vehicle_id, a.loop, a.z)
    run(make)
