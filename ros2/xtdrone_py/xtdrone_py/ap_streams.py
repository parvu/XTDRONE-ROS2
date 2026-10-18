"""ArduPilot telemetry setup for one MAVROS vehicle that has no communication node.

  ros2 run xtdrone_py ap_streams iris 0

The communication nodes (multirotor_communication, ...) do this themselves; this node is the same thing alone, for
launches with communication:=none. See VehicleNode._start_telemetry in common.py.
"""
from xtdrone_py.common import VehicleNode, run


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: ap_streams <vehicle_type> <vehicle_id>")
        return VehicleNode("ap_streams", argv[0], argv[1], telemetry=True)
    run(make)
