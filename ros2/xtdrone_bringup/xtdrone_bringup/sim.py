"""Launch building blocks shared by xtdrone.launch.py and spawn_vehicle.launch.py."""
import functools
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from xtdrone_bringup import fleet, px4


@dataclass
class SimPaths:
    px4_dir: Path
    build_dir: Path
    gz_share: Path

    @classmethod
    def find(cls, px4_dir_arg):
        px4_dir = px4.find_px4_dir(px4_dir_arg)
        px4.check_px4(px4_dir)
        build_dir = px4_dir / "build" / "px4_sitl_default"
        if not (build_dir / "bin" / "px4").is_file():
            raise RuntimeError(f"PX4 SITL is not built in {build_dir}. Run scripts/build_xtdrone_ros2.sh.")
        return cls(px4_dir, build_dir, Path(get_package_share_directory("xtdrone_gz")))

    @property
    def model_dirs(self):
        return [self.gz_share / "models", self.gz_share / "classic_models",
                self.px4_dir / "Tools/simulation/gz/models"]

    @property
    def world_dirs(self):
        return [self.gz_share / "worlds", self.px4_dir / "Tools/simulation/gz/worlds"]

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


# EKF2 sensor fusion presets (PX4 v1.16 parameter names), see the EKF chapter.
EKF_PRESETS = {
    "gps": "EKF2_GPS_CTRL=7 EKF2_EV_CTRL=0 EKF2_HGT_REF=1 EKF2_BARO_CTRL=1 EKF2_MAG_TYPE=0",
    "vision": "EKF2_GPS_CTRL=0 EKF2_EV_CTRL=15 EKF2_HGT_REF=3 EKF2_BARO_CTRL=0 EKF2_MAG_TYPE=5",
    # 2D laser SLAM (chapters 17, 18, 20): position and yaw from the SLAM pose, whose height is the
    # downward range finder (laser_transfer, as in ROS 1); the barometer is fused as well
    "laser2d": "EKF2_GPS_CTRL=0 EKF2_EV_CTRL=11 EKF2_HGT_REF=3 EKF2_BARO_CTRL=1 EKF2_MAG_TYPE=5",
}


# Applied to every vehicle before the EKF preset and px4_params (which can override them).
# XTDrone vehicles are flown by companion programs through MAVROS, usually without a
# ground station or RC: PX4 v1.16 airframes set NAV_DLL_ACT=2, which makes a GCS link an
# arming requirement, and COM_RCL_EXCEPT bit 2 keeps Offboard running without RC.
XTDRONE_DEFAULT_PARAMS = "NAV_DLL_ACT=0 COM_RCL_EXCEPT=4"


def px4_params(ekf, extra=""):
    if ekf not in EKF_PRESETS:
        raise RuntimeError(f"ekf must be one of {', '.join(EKF_PRESETS)}")
    extra = extra.replace(",", " ").strip()  # "A=1 B=2" or "A=1,B=2"
    return " ".join(p for p in (XTDRONE_DEFAULT_PARAMS, EKF_PRESETS[ekf], extra) if p)


def set_real_time_factor(world, factor):
    """Target real-time factor of a running world. PX4 v1.16 does this itself only when it
    spawns the model; XTDrone spawns the models, so the launch does it."""
    return ExecuteProcess(
        cmd=["gz", "service", "-s", f"/world/{world}/set_physics", "--reqtype", "gz.msgs.Physics",
             "--reptype", "gz.msgs.Boolean", "--timeout", "10000", "--req", f"real_time_factor: {factor:g}"],
        output="log", name="set_real_time_factor")


def world_name(world_file):
    return ET.parse(world_file).getroot().find("world").get("name")


# MAVROS plugins the XTDrone nodes use. Each MAVROS instance otherwise loads ~60 plugins
# that stream and parse data nobody reads, which matters with many vehicles.
# No "param": XTDrone sets PX4 parameters at startup (XTDRONE_PX4_PARAMS), and the plugin's full
# parameter download (~900 values per vehicle) saturates MAVROS when many vehicles connect at once.
XTDRONE_MAVROS_PLUGINS = [
    "sys_status", "sys_time", "command", "local_position", "global_position", "imu",
    "altitude", "home_position", "setpoint_raw", "setpoint_position", "setpoint_velocity",
    "setpoint_accel", "setpoint_attitude", "vision_pose", "vision_speed", "odometry",
    "vfr_hud", "wind_estimation",  # fixed-wing formation (chapter 35)
]
# mavros_plugins:=swarm (the default for mavlink_profile swarm): only what the XTDrone nodes use
# (state/set_mode, arming, local position, home, setpoint_raw, vision). Fewer plugins = fewer DDS
# endpoints, which matters when 10-20 MAVROS processes discover each other on one machine.
XTDRONE_SWARM_MAVROS_PLUGINS = [
    "sys_status", "sys_time", "command", "local_position", "home_position", "setpoint_raw",
    "vision_pose", "vision_speed",
]


SWARM_THRESHOLD = 6  # mavlink_profile:=auto picks "swarm" above this many vehicles


def mavlink_profile(profile, vehicle_count):
    """full | light | swarm | auto -> full, light or swarm (px4-rc.mavlink stream rates)."""
    if profile not in ("full", "light", "swarm", "auto"):
        raise RuntimeError("mavlink_profile must be full, light, swarm or auto")
    if profile == "auto":
        return "swarm" if vehicle_count > SWARM_THRESHOLD else "full"
    return profile


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
    """MAVROS PX4 plugin list and config (their keys are /** wildcards) plus the FCU link.
    plugins: 'xtdrone' (XTDRONE_MAVROS_PLUGINS), 'swarm' (XTDRONE_SWARM_MAVROS_PLUGINS) or 'all'
    (MAVROS' PX4 default set)."""
    share = Path(get_package_share_directory("mavros")) / "launch"
    params = [str(share / f) for f in ("px4_pluginlists.yaml", "px4_config.yaml") if (share / f).is_file()]
    # use_sim_time: PX4 runs on simulation time (lockstep); with MAVROS on wall time their clock offset drifts
    # whenever Gazebo is slower than real time. A sudden change of simulation speed then made PX4 reset its
    # time sync, drop the vision poses and blind-land (chapter 22). The stamps of the messages sent to PX4
    # (vision pose: get_local_pose, laser_transfer) are simulation time as well.
    params.append({"fcu_url": vehicle.fcu_url, "gcs_url": "", "tgt_system": vehicle.mav_sys_id,
                   "tgt_component": 1, "fcu_protocol": "v2.0", "use_sim_time": True})
    if plugins in ("xtdrone", "swarm"):
        # allowlist entries override the denylist in MAVROS
        allow = XTDRONE_MAVROS_PLUGINS if plugins == "xtdrone" else XTDRONE_SWARM_MAVROS_PLUGINS
        params.append({"plugin_denylist": ["*"], "plugin_allowlist": allow})
    elif plugins != "all":
        raise RuntimeError("mavros_plugins must be 'xtdrone', 'swarm' or 'all'")
    return params


def communication_node(vehicle, enhanced):
    """XTDrone communication node (xtdrone_py) matching the vehicle family."""
    if vehicle.family == "multirotor":
        exe = "multirotor_communication_enhanced" if enhanced else "multirotor_communication"
        args = [vehicle.vtype, str(vehicle.id_in_type)]
    elif vehicle.family == "vtol":
        exe, args = "vtol_communication", [vehicle.vtype, str(vehicle.id_in_type)]
    else:
        exe, args = f"{vehicle.family}_communication", [str(vehicle.id_in_type)]
    return Node(package="xtdrone_py", executable=exe, arguments=args, output="screen")


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
                    params=None, speed_factor=1.0, px4=True, mavros_plugins="xtdrone", mavlink_profile="full",
                    uxrce_dds=False):
    """Spawn + PX4 + MAVROS + bridge (+ communication) for one vehicle (a UGV: spawn, bridge and
    its ugv_communication node)."""
    px4 = px4 and v.has_px4
    tmp = tmp or tempfile.mkdtemp(prefix="xtdrone_launch_")
    params = px4_params("gps") if params is None else params
    if 'type="air_speed"' in paths.model_sdf(v.model).read_text():
        # the model has a gz airspeed sensor (standard_vtol_xtd): use it, not PX4's airspeed simulation
        params = "SENS_EN_ARSPDSIM=0 " + params
    x, y, z, roll, pitch, yaw = v.pose
    if z <= 0.0:
        # Like PX4's own gz spawn: start slightly above the ground and let the vehicle drop,
        # so wheels and legs do not start inside terrain meshes (e.g. the outdoor3 raceway).
        z = ground_level(world) + SPAWN_CLEARANCE
    actions = [Node(
        package="ros_gz_sim", executable="create", output="log", name=f"spawn_{v.name}",
        arguments=["-world", world, "-file", str(paths.model_sdf(v.model)), "-name", v.name,
                   "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)])]
    if px4:
        workdir = Path(rootfs).expanduser() / str(v.instance)
        workdir.mkdir(parents=True, exist_ok=True)
        # Fresh parameters every launch: PX4 saves the parameters set at startup (px4_params) in the working
        # directory and loads them on the next start, so one run's px4_params leaked into later runs of the same
        # vehicle instance (FW_USE_AIRSPD=0 from a test made later VTOL transitions climb away).
        for saved in ("parameters.bson", "parameters_backup.bson"):
            (workdir / saved).unlink(missing_ok=True)
        overlay = paths.gz_share / "px4" / "init.d-posix"
        actions.append(ExecuteProcess(
            cmd=[str(paths.build_dir / "bin" / "px4"), "-i", str(v.instance), "-d", str(paths.build_dir / "etc")],
            cwd=str(workdir), output="screen", name=f"px4_{v.name}", emulate_tty=True,
            additional_env={
                "PX4_SYS_AUTOSTART": str(v.airframe),
                "PX4_SIMULATOR": "gz",
                "PX4_GZ_STANDALONE": "1",
                "PX4_GZ_WORLD": world,
                "PX4_GZ_MODEL_NAME": v.name,
                "XTDRONE_PX4_PARAMS": params,
                "XTDRONE_MAVLINK_PROFILE": mavlink_profile,
                # px4-rc.simulator overlay: PX4's uXRCE-DDS client only on request
                "XTDRONE_UXRCE_DDS": "1" if uxrce_dds else "0",
                # rcS scales the failsafe timeouts with it (chapter 43)
                **({"PX4_SIM_SPEED_FACTOR": f"{speed_factor:g}"} if speed_factor != 1.0 else {}),
                # px4-rc.mavlink / px4-rc.simulator overlays (XTDrone ports, parameters) are found first on PATH
                "PATH": f"{overlay}{os.pathsep}{os.environ.get('PATH', '')}",
            }))
    if mavros and px4:
        # Like mavros' node.launch: namespace only. mavros_node runs several nodes in one
        # process (router, UAS, plugins); a node name override would rename them all.
        # Topics end up in /<name>/mavros/... .
        actions.append(Node(package="mavros", executable="mavros_node", namespace=v.name,
                            output="log", parameters=mavros_params(v, mavros_plugins)))
    if bridge:
        actions.append(fleet_bridge([v], world, paths, tmp, name=f"{v.name}_gz_bridge"))
    if communication != "none" and px4:
        actions.append(communication_node(v, communication == "enhanced"))
    if v.family == "ugv":
        # XTDrone UGV command (speed, steering angle) -> AckermannSteering (speed, yaw rate);
        # replaces the catvehicle cmd_vel controller, so it always runs
        actions.append(Node(package="xtdrone_py", executable="ugv_communication", arguments=[str(v.id_in_type)],
                            output="screen"))
    return actions
