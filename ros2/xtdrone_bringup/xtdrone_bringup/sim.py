"""Launch building blocks shared by xtdrone.launch.py and spawn_vehicle.launch.py."""
import functools
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from xtdrone_bringup import ardupilot, fleet


@dataclass
class SimPaths:
    ardupilot_dir: Path
    gz_share: Path

    @classmethod
    def find(cls, ardupilot_dir_arg):
        d = ardupilot.find_ardupilot_dir(ardupilot_dir_arg)
        ardupilot.binary(d, "multirotor")
        ardupilot.gazebo_plugin_dir(d)
        return cls(d, Path(get_package_share_directory("xtdrone_gz")))

    @property
    def plugin_dir(self):
        return ardupilot.gazebo_plugin_dir(self.ardupilot_dir)

    @property
    def model_dirs(self):
        return [self.gz_share / "models", self.gz_share / "classic_models", self.plugin_dir / "models", ardupilot.sitl_models_dir(self.ardupilot_dir)]

    @property
    def world_dirs(self):
        return [self.gz_share / "worlds", self.plugin_dir / "worlds"]

    def plugin_path(self):
        extra = [p for p in os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "").split(os.pathsep) if p]
        return os.pathsep.join([str(self.plugin_dir / "build")] + extra)

    def resource_path(self):
        dirs = [str(d) for d in self.model_dirs + self.world_dirs if d.is_dir()]
        extra = [p for p in os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep) if p]
        return os.pathsep.join(dirs + [p for p in extra if p not in dirs])

    def resolve_world(self, world):
        p = Path(world).expanduser()
        if p.suffix == ".sdf" and p.is_file():
            return p
        for d in self.world_dirs:
            candidate = d / f"{world}.sdf"
            if candidate.is_file():
                return candidate
        raise RuntimeError(f"world '{world}' not found in {', '.join(map(str, self.world_dirs))}. "
                           "Run scripts/build_xtdrone_ros2.sh to convert the XTDrone worlds.")

    def model_sdf(self, model):
        for d in self.model_dirs:
            if (d / model / "model.sdf").is_file():
                return d / model / "model.sdf"
        raise RuntimeError(f"model '{model}' not found in {', '.join(map(str, self.model_dirs))}")


# EKF3 sensor fusion presets (ArduPilot 4.7 parameter names), see the EKF chapter. Source set 1: 3 = GPS,
# 6 = ExternalNav (MAVROS vision_pose), 1 = barometer, 0 = none. Vision presets need the EKF origin: ap_streams sets it.
EKF_PRESETS = {
    "gps": "AHRS_EKF_TYPE=3 EK3_ENABLE=1 EK3_SRC1_POSXY=3 EK3_SRC1_VELXY=3 EK3_SRC1_POSZ=1 EK3_SRC1_VELZ=3 "
           "EK3_SRC1_YAW=1 VISO_TYPE=0 COMPASS_USE2=0 COMPASS_USE3=0",
    # vision: position (and velocity) from the vision pose, yaw from the first compass. Yaw from the vision pose too
    # started the EKF with a wrong heading: the vehicles turned on the ground until the fusion caught up
    "vision": "AHRS_EKF_TYPE=3 EK3_ENABLE=1 VISO_TYPE=1 GPS1_TYPE=0 EK3_SRC1_POSXY=6 EK3_SRC1_VELXY=0 "
              "EK3_SRC1_POSZ=6 EK3_SRC1_VELZ=6 EK3_SRC1_YAW=1 COMPASS_USE2=0 COMPASS_USE3=0",
    # 2D laser SLAM (chapters 17, 18, 20): position and yaw from the SLAM pose, whose height is the
    # downward range finder (laser_transfer, as in ROS 1); the barometer is fused as well
    "laser2d": "AHRS_EKF_TYPE=3 EK3_ENABLE=1 VISO_TYPE=1 GPS1_TYPE=0 EK3_SRC1_POSXY=6 EK3_SRC1_VELXY=0 "
               "EK3_SRC1_POSZ=1 EK3_SRC1_VELZ=0 EK3_SRC1_YAW=6 COMPASS_USE=0 COMPASS_USE2=0 COMPASS_USE3=0",
}


# Applied to every vehicle before the EKF preset and the params argument (which can override them).
# XTDrone vehicles are flown by companion programs through MAVROS, usually without a ground station or RC:
# no GCS/RC failsafes (FS_GCS_ENABLE, FS_THR_ENABLE), and arming checks that need hardware are the SITL defaults.
# ARMING_SKIPCHK 84: skip the compass (4), INS (16) and RC (64) checks (planes wait for an RC receiver): the simulated compasses and gyros disagree by more
# than the limits ("Check mag field", "Gyros inconsistent") when the simulation runs slower than real time, and
# refused to arm.
XTDRONE_DEFAULT_PARAMS = "FS_GCS_ENABLE=0 FS_THR_ENABLE=0 SIM_RC_FAIL=1 ARMING_SKIPCHK=84"


def autopilot_params(ekf, extra=""):
    if ekf not in EKF_PRESETS:
        raise RuntimeError(f"ekf must be one of {', '.join(EKF_PRESETS)}")
    extra = extra.replace(",", " ").strip()  # "A=1 B=2" or "A=1,B=2"
    return " ".join(p for p in (XTDRONE_DEFAULT_PARAMS, EKF_PRESETS[ekf], extra) if p)


def parm_file(params, path):
    """ArduPilot defaults file from 'NAME=value NAME=value'."""
    lines = []
    for item in params.split():
        name, _, value = item.partition("=")
        lines.append(f"{name} {value}")
    Path(path).write_text("\n".join(lines) + "\n")
    return str(path)


def set_real_time_factor(world, factor):
    """Target real-time factor of a running world (the launch does it after the world is up)."""
    return ExecuteProcess(
        cmd=["gz", "service", "-s", f"/world/{world}/set_physics", "--reqtype", "gz.msgs.Physics",
             "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", f"real_time_factor: {factor:g}"],
        output="log", name="set_real_time_factor")


def world_with_step(world_file, step, tmp):
    """A copy of the world with another physics step (and the matching update rate): the tailsitter's attitude loop
    needs 250 Hz, which the 4 ms step of the worlds cannot feed (the gyro must run at 1.8 x the loop rate)."""
    text = Path(world_file).read_text()
    text = re.sub(r"<max_step_size>[^<]*</max_step_size>", f"<max_step_size>{step:g}</max_step_size>", text)
    text = re.sub(r"<real_time_update_rate>[^<]*</real_time_update_rate>",
                  f"<real_time_update_rate>{1.0 / step:g}</real_time_update_rate>", text)
    out = Path(tmp) / Path(world_file).name
    out.write_text(text)
    return out


def world_name(world_file):
    return ET.parse(world_file).getroot().find("world").get("name")


# MAVROS plugins the XTDrone nodes use. Each MAVROS instance otherwise loads ~60 plugins
# that stream and parse data nobody reads, which matters with many vehicles.
# No "param": XTDrone sets the ArduPilot parameters at startup (defaults file), and the plugin's full
# parameter download saturates MAVROS when many vehicles connect at once.
XTDRONE_MAVROS_PLUGINS = [
    "sys_status", "sys_time", "command", "local_position", "global_position", "imu",
    "altitude", "home_position", "setpoint_raw", "setpoint_position", "setpoint_velocity",
    "setpoint_accel", "setpoint_attitude", "vision_pose", "odometry",
    "vfr_hud", "wind_estimation", "waypoint", "rc_io",
]
# mavros_plugins:=swarm (the default above 6 vehicles): only what the XTDrone nodes use
# (state/set_mode, arming, local position, home, setpoint_raw, vision). Fewer plugins = fewer DDS
# endpoints, which matters when 10-20 MAVROS processes discover each other on one machine.
XTDRONE_SWARM_MAVROS_PLUGINS = [
    "sys_status", "sys_time", "command", "local_position", "global_position", "home_position", "setpoint_raw",
    "vision_pose",
]  # global_position: the EKF origin (set_gp_origin) of the vision presets goes through it


SWARM_THRESHOLD = 6  # mavros_plugins:=auto picks "swarm" above this many vehicles


# SITL home (lat, lon, alt, heading): the origin of the GPS the autopilots see
DEFAULT_HOME = (47.397742, 8.545594, 488.0, 0)
SPAWN_CLEARANCE = 0.3  # m, spawn height above the ground for vehicles requested at z <= 0


@functools.lru_cache(maxsize=None)
def ground_level(world):
    """z of the world's ground_plane model (outdoor4 is built 5 m up), 0 if the world has none."""
    try:
        sdf = Path(get_package_share_directory("xtdrone_gz")) / "worlds" / f"{world}.sdf"
        for m in ET.parse(sdf).getroot().find("world").findall("model"):
            if m.get("name") == "ground_plane":
                pose = lambda e: [float(x) for x in (e.findtext("pose") or "0 0 0").split()]  # noqa: E731
                return pose(m)[2] + sum(pose(l)[2] for l in m.findall("link"))
    except (OSError, ET.ParseError, ValueError, IndexError, LookupError):
        pass
    return 0.0


def mavros_params(vehicle, plugins="xtdrone"):
    """MAVROS APM plugin list and config (their keys are /** wildcards) plus the FCU link.
    plugins: 'xtdrone' (XTDRONE_MAVROS_PLUGINS), 'swarm' (XTDRONE_SWARM_MAVROS_PLUGINS) or 'all'
    (MAVROS' APM default set)."""
    share = Path(get_package_share_directory("mavros")) / "launch"
    params = [str(share / f) for f in ("apm_pluginlists.yaml", "apm_config.yaml") if (share / f).is_file()]
    # use_sim_time: SITL follows the simulation clock (lockstep); with MAVROS on wall time their clock offset
    # drifts whenever Gazebo is slower than real time. The stamps of the messages sent to the autopilot
    # (vision pose: get_local_pose, laser_transfer) are simulation time as well.
    params.append({"fcu_url": vehicle.fcu_url, "gcs_url": "", "tgt_system": vehicle.mav_sys_id,
                   "tgt_component": 1, "fcu_protocol": "v2.0", "use_sim_time": True,
                   "system_id": 255})  # the default SYSID_MYGCS: ArduPilot only takes RC overrides from the GCS id
    if plugins in ("xtdrone", "swarm"):
        # allowlist entries override the denylist in MAVROS
        allow = XTDRONE_MAVROS_PLUGINS if plugins == "xtdrone" else XTDRONE_SWARM_MAVROS_PLUGINS
        params.append({"plugin_denylist": ["*"], "plugin_allowlist": allow})
    elif plugins != "all":
        raise RuntimeError("mavros_plugins must be 'xtdrone', 'swarm' or 'all'")
    return params


def communication_node(vehicle, enhanced, origin, vision=False):
    """XTDrone communication node (xtdrone_py) matching the vehicle family."""
    if vehicle.family == "multirotor":
        exe = "multirotor_communication_enhanced" if enhanced else "multirotor_communication"
        args = [vehicle.vtype, str(vehicle.id_in_type)]
    elif vehicle.family in ("vtol", "tailsitter"):
        exe, args = "vtol_communication", [vehicle.vtype, str(vehicle.id_in_type)]
    else:
        exe, args = f"{vehicle.family}_communication", [str(vehicle.id_in_type)]
    return Node(package="xtdrone_py", executable=exe, arguments=args, output="screen",
                # with GPS the local ENU frame of ArduPilot starts at the spawn point (its EKF origin), with a vision EKF
                # at the world origin: the position of the GPS fix of a world point (reposition commands)
                parameters=[{"origin_lat": origin[0], "origin_lon": origin[1], "origin_alt": origin[2],
                             "spawn_x": 0.0 if vision else float(vehicle.pose[0]),
                             "spawn_y": 0.0 if vision else float(vehicle.pose[1])}])


def actor_bridges(world_file):
    """ROS -> gz bridges for the actors of a world that carry the XTDrone ActorMotion system
    (/<actor>/cmd_motion, geometry_msgs/Vector3: x, y, speed; chapter 38)."""
    entries = []
    try:
        root = ET.parse(world_file).getroot()
    except (ET.ParseError, OSError):
        return entries
    for actor in root.iter("actor"):
        if any(pl.get("filename") == "xtdrone_actor_motion" for pl in actor.findall("plugin")):
            topic = f"/{actor.get('name')}/cmd_motion"
            entries.append({"ros_topic_name": topic, "gz_topic_name": topic,
                            "ros_type_name": "geometry_msgs/msg/Vector3", "gz_type_name": "gz.msgs.Vector3d",
                            "direction": "ROS_TO_GZ"})
            pose = f"/{actor.get('name')}/pose"
            entries.append({"ros_topic_name": pose, "gz_topic_name": pose, "ros_type_name": "geometry_msgs/msg/Pose",
                            "gz_type_name": "gz.msgs.Pose", "direction": "GZ_TO_ROS", "lazy": True})
    return entries


def actor_walkers(world_file):
    """(actor name, walk_loop) for the actors whose xtdrone_actor_motion plugin carries a
    <walk_loop>: the waypoints of the Classic script trajectory, kept by the converter for the
    xtdrone_py actor_walker node (chapter 23)."""
    out = []
    try:
        root = ET.parse(world_file).getroot()
    except (ET.ParseError, OSError):
        return out
    for actor in root.iter("actor"):
        for pl in actor.findall("plugin"):
            loop = pl.findtext("walk_loop") if pl.get("filename") == "xtdrone_actor_motion" else None
            if loop:
                out.append((actor.get("name"), loop.strip()))
    return out


def fleet_bridge(vehicles, world, paths, tmp=None, name="xtdrone_gz_bridge", extra=()):
    """One ros_gz_bridge process for the sensors and ground truth of all given vehicles (lazy:
    a topic is converted only while somebody subscribes). One process instead of one per
    vehicle keeps large fleets cheap."""
    tmp = tmp or tempfile.mkdtemp(prefix="xtdrone_launch_")
    entries = [e for v in vehicles for e in fleet.bridge_config(paths.gz_share / "models", world, v)] + list(extra)
    cfg = Path(tmp) / f"{name}.yaml"
    cfg.write_text(yaml.safe_dump(entries))
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name=name,
                output="log", parameters=[{"config_file": str(cfg)}])


def vehicle_actions(v, world, paths, rootfs, mavros=True, bridge=True, communication="none", tmp=None,
                    params=None, speed_factor=1.0, sitl=True, mavros_plugins="xtdrone", home=None):
    """Spawn + ArduPilot SITL + MAVROS + bridge (+ communication) for one vehicle (a UGV: spawn, bridge and
    its ugv_communication node)."""
    sitl = sitl and v.has_sitl
    tmp = tmp or tempfile.mkdtemp(prefix="xtdrone_launch_")
    params = autopilot_params("gps") if params is None else params
    x, y, z, roll, pitch, yaw = v.pose
    if v.model == "plane":
        # the zephyr's nose points to -y (the ArduPilotPlugin transform assumes it): turn it to the requested heading
        yaw += math.pi / 2
    if v.model == "tailsitter":
        # ArduPlane's tailsitter code hovers nose up: the model sits on its tail (four legs)
        pitch, z = pitch - math.pi / 2, max(z, 0.3)
    if z <= 0.0:
        # Start slightly above the ground and let the vehicle drop, so wheels and legs do not start inside
        # terrain meshes (e.g. the outdoor3 raceway).
        z = ground_level(world) + SPAWN_CLEARANCE
    model_sdf = paths.model_sdf(v.model)
    if "ArduPilotPlugin" in model_sdf.read_text():
        text = model_sdf.read_text()
        if sitl:
            # the SITL instance N talks to the plugin on UDP port 9002 + 10 N
            text = text.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{9002 + 10 * v.instance}</fdm_port_in>")
        else:
            # no SITL to answer: the plugin would wait for it at every step (lockstep)
            text = re.sub(r'\s*<plugin name="ArduPilotPlugin".*?</plugin>', "", text, flags=re.S)
        model_sdf = Path(tmp) / f"{v.name}.sdf"
        model_sdf.write_text(text)
    actions = [Node(
        package="ros_gz_sim", executable="create", output="log", name=f"spawn_{v.name}",
        arguments=["-world", world, "-file", str(model_sdf), "-name", v.name,
                   "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)])]
    if sitl:
        spec = ardupilot.SITL[v.family]
        workdir = Path(rootfs).expanduser() / str(v.instance)
        workdir.mkdir(parents=True, exist_ok=True)
        # Fresh parameters every launch: ArduPilot saves its parameters in eeprom.bin of the working directory
        (workdir / "eeprom.bin").unlink(missing_ok=True)
        autotest = paths.ardupilot_dir / "Tools" / "autotest"
        files = [str(autotest / f) for f in spec["defaults"]]
        files += [str(paths.gz_share / "ardupilot" / f) for f in spec["overlay"]]
        files.append(parm_file(f"MAV_SYSID={v.mav_sys_id} {params}", Path(tmp) / f"{v.name}.parm"))
        lat, lon, alt, hdg = home or DEFAULT_HOME
        actions.append(ExecuteProcess(
            cmd=["python3", "-m", "xtdrone_bringup.sitl_run", str(ardupilot.binary(paths.ardupilot_dir, v.family)),
                 "--model", "JSON", "--speedup", f"{speed_factor:g}", "--slave", "0", "-I", str(v.instance),
                 "--sysid", str(v.mav_sys_id), "--sim-address", "127.0.0.1", "--home", f"{lat},{lon},{alt},{hdg}",
                 "--serial0", f"udpclient:127.0.0.1:{v.mavlink_port}", "--defaults", ",".join(files)],
            cwd=str(workdir), output="screen", name=f"sitl_{v.name}"))
    if mavros and sitl:
        # Like mavros' node.launch: namespace only. mavros_node runs several nodes in one
        # process (router, UAS, plugins); a node name override would rename them all.
        # Topics end up in /<name>/mavros/... .
        actions.append(Node(package="mavros", executable="mavros_node", namespace=v.name,
                            output="log", parameters=mavros_params(v, mavros_plugins)))
        if communication == "none":
            # ArduPilot streams only what is asked for (message intervals, EKF origin for the vision presets): the
            # communication nodes do that themselves, this node is the same thing alone
            actions.append(Node(package="xtdrone_py", executable="ap_streams",
                                arguments=[v.vtype, str(v.id_in_type)], output="screen",
                                parameters=[{"origin_lat": (home or DEFAULT_HOME)[0],
                                             "origin_lon": (home or DEFAULT_HOME)[1],
                                             "origin_alt": (home or DEFAULT_HOME)[2]}]))
    if bridge:
        actions.append(fleet_bridge([v], world, paths, tmp, name=f"{v.name}_gz_bridge"))
    if communication != "none" and sitl:
        actions.append(communication_node(v, communication == "enhanced", home or DEFAULT_HOME,
                                          vision="VISO_TYPE=1" in params))
    if v.family == "ugv":
        # XTDrone UGV command (speed, steering angle) -> AckermannSteering (speed, yaw rate);
        # replaces the catvehicle cmd_vel controller, so it always runs
        actions.append(Node(package="xtdrone_py", executable="ugv_communication", arguments=[str(v.id_in_type)],
                            output="screen"))
    return actions
