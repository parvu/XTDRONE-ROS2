"""Move Gazebo models directly (ports of control/control_gazebo_models.py and
control/control_gazebo_vehicles.py).

  ros2 run xtdrone_py control_gazebo_models [--world outdoor3]
  ros2 run xtdrone_py control_gazebo_vehicles iris 10 10 [--world empty]

Gazebo Classic used /gazebo/set_model_states. Gazebo Harmonic offers the
/world/<world>/set_pose_vector service (UserCommands system), called here
through the gz-transport Python bindings shipped with Gazebo Harmonic.
"""
import argparse
import subprocess
import sys
import time

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
except ImportError:  # pragma: no cover - only without Gazebo Harmonic python bindings
    Node = None


def detect_world():
    """Name of the running world, from its /world/<name>/clock topic."""
    out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, check=False).stdout
    for line in out.splitlines():
        if line.startswith("/world/") and line.endswith("/clock"):
            return line.split("/")[2]
    raise SystemExit("no running Gazebo world found (gz topic -l shows no /world/<name>/clock)")


class PoseSetter:
    def __init__(self, world):
        if Node is None:
            raise SystemExit("gz-transport Python bindings not found (install gz-harmonic)")
        self.node = Node()
        self.service = f"/world/{world}/set_pose_vector"

    def set(self, poses):
        """poses: iterable of (name, x, y, z)."""
        req = Pose_V()
        for name, x, y, z in poses:
            p = req.pose.add()
            p.name = name
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
        ok, rep = self.node.request(self.service, req, Pose_V, Boolean, 1000)
        return ok and rep.data


def loop(setter, poses, v=0.5, f=20.0):
    period = 1.0 / f
    while True:
        for p in poses:
            p[1] += v / f
        if not setter.set(poses):
            print(f"{setter.service} failed", file=sys.stderr)
        time.sleep(period)


def models_main():
    ap = argparse.ArgumentParser(description="Move rover_static and rover_static_clone along +x at 0.5 m/s.")
    ap.add_argument("--world", default=None)
    args = ap.parse_args()
    setter = PoseSetter(args.world or detect_world())
    try:
        loop(setter, [["rover_static", 70.0, 1.0, 0.0], ["rover_static_clone", 70.0, 5.0, 0.0]])
    except KeyboardInterrupt:
        pass


def vehicles_main():
    ap = argparse.ArgumentParser(description="Teleport a rows x columns grid of vehicles (no PX4) along +x.")
    ap.add_argument("vehicle_type")
    ap.add_argument("row_num", type=int)
    ap.add_argument("column_num", type=int)
    ap.add_argument("--world", default=None)
    args = ap.parse_args()
    n = args.row_num * args.column_num
    poses = [[f"{args.vehicle_type}_{i}", float(i // args.row_num), float(i % args.row_num), 3.0] for i in range(n)]
    setter = PoseSetter(args.world or detect_world())
    try:
        loop(setter, poses)
    except KeyboardInterrupt:
        pass
