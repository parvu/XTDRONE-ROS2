"""UAV contact detection (port of sensing/contact_detect/contact_detect.py).

  ros2 run xtdrone_py contact_detect iris 6

Every XTDrone vehicle model carries a bumper contact sensor, bridged by
xtdrone.launch.py to /<type>_<i>/benchmarker/collision (ros_gz_interfaces/Contacts).
"""
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts

from xtdrone_py.common import run


class ContactDetect(Node):

    def __init__(self, vehicle_type, vehicle_num):
        super().__init__(f"{vehicle_type}_contact_detect")
        for i in range(vehicle_num):
            name = f"{vehicle_type}_{i}"
            self.create_subscription(Contacts, f"/{name}/benchmarker/collision",
                                     lambda msg, name=name: self.callback(name, msg), 1)

    @staticmethod
    def callback(name, msg):
        if not msg.contacts:
            return
        c = msg.contacts[0]
        other = c.collision2.name if name not in c.collision2.name else c.collision1.name
        print(f"{name} contact {other}", flush=True)
        if c.positions:
            p = c.positions[0]
            print(f"One contact position is [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]", flush=True)


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: contact_detect <vehicle_type> <vehicle_num>   e.g. iris 6")
        return ContactDetect(argv[0], int(argv[1]))
    run(make)
