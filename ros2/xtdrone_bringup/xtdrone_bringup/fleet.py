"""XTDrone fleet description: vehicle types, ArduPilot SITL ports and layout.

A fleet is a list of Vehicle entries. It comes from a scenario YAML
(config/scenarios/<name>.yaml) or from a compact spec string such as
"iris_stereo_camera*3,plane" (three iris with stereo cameras, one plane),
laid out on a grid like XTDrone's multi-vehicle generator.py.
"""
from dataclasses import dataclass, field
from pathlib import Path
import re

import yaml

# XTDrone vehicle type -> family. The family selects the XTDrone communication script and the ArduPilot SITL
# binary (ardupilot.SITL); "sitl": False vehicles run no autopilot.
VEHICLE_TYPES = {
    "iris": {"sitl": True, "family": "multirotor"},
    "solo": {"sitl": True, "family": "multirotor"},
    "typhoon_h480": {"sitl": True, "family": "multirotor"},
    "plane": {"sitl": True, "family": "plane"},
    "standard_vtol": {"sitl": True, "family": "vtol"},
    "rover": {"sitl": True, "family": "rover"},
    "tailsitter": {"sitl": True, "family": "vtol"},
    "tiltrotor": {"sitl": True, "family": "vtol"},
    # CAT Vehicle UGV (chapter 11): no autopilot / MAVROS, gz AckermannSteering
    "ugv": {"sitl": False, "family": "ugv"},
}

# Gazebo Classic XTDrone model names that map to a different Harmonic model.
MODEL_ALIASES = {
    "plane_gimbal": "plane",
    # XTDrone wrappers of PX4 models with the same name
    "standard_vtol": "standard_vtol_xtd",
    "tiltrotor": "tiltrotor_xtd",
}

# Classic XTDrone models without a Gazebo Harmonic equivalent yet.
UNSUPPORTED = {}

# MAVROS talks to SERIAL0 of the SITL instance N over UDP: MAVROS listens on 14550 + N, SITL (udpclient) sends there.
# (TCP port 5760 left a bound port in TIME_WAIT after every run, and the next SITL could not bind it.)
SITL_PORT_BASE = 14550


@dataclass
class Vehicle:
    model: str            # gz model directory, e.g. iris_stereo_camera
    vtype: str            # XTDrone vehicle type, e.g. iris
    id_in_type: int       # the N in iris_N
    instance: int = 0     # global SITL instance (0..)
    pose: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    @property
    def name(self):
        return f"{self.vtype}_{self.id_in_type}"

    @property
    def family(self):
        return VEHICLE_TYPES[self.vtype]["family"]

    @property
    def has_sitl(self):
        return VEHICLE_TYPES[self.vtype]["sitl"]

    @property
    def fcu_url(self):
        return f"udp://:{self.mavlink_port}@"

    @property
    def mavlink_port(self):
        return SITL_PORT_BASE + self.instance

    @property
    def mav_sys_id(self):
        return self.instance + 1


def vehicle_type_of(model):
    """XTDrone rule: iris_stereo_camera -> iris, typhoon_h480_x -> typhoon_h480."""
    model = MODEL_ALIASES.get(model, model)
    for vtype in sorted(VEHICLE_TYPES, key=len, reverse=True):
        if model == vtype or model.startswith(vtype + "_"):
            return vtype
    for vtype, why in UNSUPPORTED.items():
        if model == vtype or model.startswith(vtype + "_"):
            raise ValueError(f"{model}: {why}")
    raise ValueError(f"unknown XTDrone vehicle model '{model}'")


def parse_spec(spec, spacing=3.0, columns=3):
    """'iris_stereo_camera*3,plane' -> vehicles on generator.py-style rows."""
    vehicles = []
    counters = {}
    column_offset = 0
    for part in [p.strip() for p in spec.split(",") if p.strip()]:
        m = re.fullmatch(r"([A-Za-z0-9_]+)(?:\*(\d+))?", part)
        if not m:
            raise ValueError(f"bad vehicle spec '{part}', expected MODEL or MODEL*COUNT")
        model, count = m.group(1), int(m.group(2) or 1)
        vtype = vehicle_type_of(model)
        model = MODEL_ALIASES.get(model, model)
        rows = min(columns, count)
        for k in range(count):
            idx = counters.get(vtype, 0)
            counters[vtype] = idx + 1
            x = (column_offset + k % rows) * spacing
            y = (k // rows) * spacing
            vehicles.append(Vehicle(model, vtype, idx, pose=[x, y, 0.0, 0.0, 0.0, 0.0]))
        column_offset += rows
    return _number(vehicles)


def load_scenario(path):
    data = yaml.safe_load(Path(path).read_text())
    vehicles = []
    for v in data.get("vehicles", []):
        model = MODEL_ALIASES.get(v["model"], v["model"])
        vtype = v.get("type") or vehicle_type_of(model)
        pose = [float(x) for x in v.get("pose", [0, 0, 0, 0, 0, 0])]
        pose += [0.0] * (6 - len(pose))
        vehicles.append(Vehicle(model, vtype, int(v.get("id", 0)), pose=pose))
    return data.get("world", "default"), _number(vehicles)


def load_scenario_objects(path):
    """Static models of a scenario: objects: [{model: apriltag0-2, name: tags, pose: [x, y, z, R, P, Y]}]."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    objects = []
    for i, o in enumerate(data.get("objects", [])):
        pose = [float(x) for x in o.get("pose", [0, 0, 0, 0, 0, 0])]
        pose += [0.0] * (6 - len(pose))
        objects.append({"model": o["model"], "name": o.get("name", f"{o['model']}_{i}"), "pose": pose})
    return objects


def _number(vehicles):
    """SITL instances 0, 1, ... for the vehicles with an autopilot (UGVs have none and get -1)."""
    names = set()
    i = 0
    for v in vehicles:
        if v.has_sitl:
            v.instance, i = i, i + 1
        else:
            v.instance = -1
        if v.name in names:
            raise ValueError(f"duplicate vehicle name {v.name}")
        names.add(v.name)
    return vehicles


def bridge_config(model_dir, world, vehicle):
    """ros_gz_bridge YAML entries for one vehicle from its xtdrone_bridge.yaml."""
    spec_file = Path(model_dir) / vehicle.model / "xtdrone_bridge.yaml"
    entries = []
    if spec_file.is_file():
        spec = yaml.safe_load(spec_file.read_text()) or {}
        for b in spec.get("bridges", []):
            direction = b.get("direction", "GZ_TO_ROS")
            entry = {
                "ros_topic_name": f"/{vehicle.name}/{b['ros_topic']}",
                "gz_topic_name": b["gz_topic"].format(world=world, name=vehicle.name),
                "ros_type_name": b["ros_type"],
                "gz_type_name": b["gz_type"],
                "direction": direction,
            }
            if direction == "GZ_TO_ROS":
                entry["lazy"] = True  # only convert sensor data somebody subscribed to
            entries.append(entry)
    # Ground truth odometry from the OdometryPublisher system in every model.
    entries.append({
        "ros_topic_name": f"/{vehicle.name}/ground_truth/odom",
        "gz_topic_name": f"/model/{vehicle.name}/odometry",
        "ros_type_name": "nav_msgs/msg/Odometry",
        "gz_type_name": "gz.msgs.Odometry",
        "direction": "GZ_TO_ROS",
        "lazy": True,
    })
    return entries
