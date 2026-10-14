"""XTDrone simulation on Gazebo Harmonic + PX4 v1.16.2 + MAVROS 2.

Replaces the ROS 1 launch files in XTDrone's sitl_config/launch (indoor1.launch,
outdoor2.launch, multi_vehicle.launch, ...).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
  ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:=plane
  ros2 launch xtdrone_bringup xtdrone.launch.py world:=indoor1 vehicles:="iris*6"

Per vehicle <type>_<id> (e.g. iris_0) it starts:
  * the gz model, spawned with that name,
  * PX4 SITL instance N attached to it (PX4_GZ_MODEL_NAME),
  * MAVROS 2 in /<type>_<id>/mavros (fcu udp://:24540+N@127.0.0.1:34580+N),
  * ros_gz_bridge for the vehicle sensors and ground truth.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from xtdrone_bringup import fleet, px4
from xtdrone_bringup.sim import (EKF_PRESETS, SimPaths, actor_bridges, fleet_bridge, mavlink_profile, px4_params, set_real_time_factor, vehicle_actions,
                                 world_name)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _arg(context, name).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be 'true' or 'false'")
    return value == "true"


def launch_setup(context):
    paths = SimPaths.find(_arg(context, "px4_dir"))
    scenario = _arg(context, "scenario")
    if scenario:
        sc_path = Path(scenario)
        if not sc_path.is_file():
            sc_path = Path(get_package_share_directory("xtdrone_bringup")) / "config/scenarios" / f"{scenario}.yaml"
        world, vehicles = fleet.load_scenario(sc_path)
        objects = fleet.load_scenario_objects(sc_path)
    else:
        world = _arg(context, "world")
        vehicles = fleet.parse_spec(_arg(context, "vehicles"))
        objects = []
    world_file = paths.resolve_world(world)
    world = world_name(world_file)
    communication = _arg(context, "communication")
    if communication not in ("none", "basic", "enhanced"):
        raise RuntimeError("communication must be none, basic or enhanced")
    verbose = _arg(context, "gz_verbose")

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", paths.resource_path()),
        ExecuteProcess(cmd=["gz", "sim", "-r", "-s", "-v", verbose, str(world_file)],
                       output="screen", name="gz_server"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="log"),
    ]
    speed_factor = float(_arg(context, "speed_factor"))
    if speed_factor <= 0:
        raise RuntimeError("speed_factor must be > 0")
    if speed_factor != 1.0:
        actions.append(TimerAction(period=1.5, actions=[set_real_time_factor(world, speed_factor)]))
    if _bool(context, "gui"):
        # Start the GUI after the vehicles are spawned: a GUI still waiting for its initial
        # world state misses entities created meanwhile and never shows them.
        gui_delay = 2.0 + 0.5 * len(vehicles) + 4.0
        actions.append(TimerAction(period=gui_delay, actions=[
            ExecuteProcess(cmd=["gz", "sim", "-g", "-v", verbose], output="log", name="gz_gui")]))
    for o in objects:
        x, y, z, roll, pitch, yaw = o["pose"]
        actions.append(TimerAction(period=2.0, actions=[Node(
            package="ros_gz_sim", executable="create", output="log", name=f"spawn_{o['name']}".replace("-", "_"),
            arguments=["-world", world, "-file", str(paths.model_sdf(o["model"])), "-name", o["name"],
                       "-x", str(x), "-y", str(y), "-z", str(z), "-R", str(roll), "-P", str(pitch),
                       "-Y", str(yaw)])]))
    actors = actor_bridges(world_file)
    if _bool(context, "bridge") and (vehicles or actors):
        actions.append(fleet_bridge(vehicles, world, paths, extra=actors))
    profile = mavlink_profile(_arg(context, "mavlink_profile"), len(vehicles))
    plugins = _arg(context, "mavros_plugins")
    if plugins == "auto":
        plugins = "swarm" if profile == "swarm" else "xtdrone"
    for k, v in enumerate(vehicles):
        # Stagger vehicles a little so gz is up and spawns do not pile up.
        actions.append(TimerAction(period=2.0 + 0.5 * k, actions=vehicle_actions(
            v, world, paths, _arg(context, "rootfs"), mavros=_bool(context, "mavros"),
            bridge=False, communication=communication,
            params=px4_params(_arg(context, "ekf"), _arg(context, "px4_params")), speed_factor=speed_factor,
            px4=_bool(context, "px4"), mavros_plugins=plugins, mavlink_profile=profile,
            uxrce_dds=_bool(context, "uxrce_dds"))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="",
                              description="Scenario name in xtdrone_bringup/config/scenarios or a YAML path. "
                                          "Overrides world and vehicles."),
        DeclareLaunchArgument("world", default_value="indoor1",
                              description="World name (xtdrone_gz/worlds or PX4 gz worlds) or .sdf path."),
        DeclareLaunchArgument("vehicles", default_value="iris_stereo_camera",
                              description="Fleet spec, e.g. 'iris_stereo_camera*3,plane'."),
        DeclareLaunchArgument("gui", default_value="true", description="Start the Gazebo GUI."),
        DeclareLaunchArgument("mavros", default_value="true", description="Start MAVROS 2 per vehicle."),
        DeclareLaunchArgument("bridge", default_value="true", description="Bridge sensors and ground truth."),
        DeclareLaunchArgument("communication", default_value="none",
                              description="Also start XTDrone communication nodes: none, basic or enhanced."),
        DeclareLaunchArgument("uxrce_dds", default_value="false",
                              description="Start PX4's uXRCE-DDS client (for px4_msgs / ROS 2 offboard "
                                          "code next to MAVROS; needs MicroXRCEAgent udp4 -p 8888)."),
        DeclareLaunchArgument("mavlink_profile", default_value="auto",
                              description="PX4 MAVLink stream rates: full, light (unused streams off), swarm "
                                          "(lower rates for many vehicles) or auto (swarm above 6 vehicles)."),
        DeclareLaunchArgument("mavros_plugins", default_value="auto",
                              description="MAVROS plugins: 'xtdrone' (those XTDrone uses), 'swarm' (the minimum "
                                          "for the XTDrone nodes), 'all', or 'auto' (swarm with the swarm "
                                          "MAVLink profile, else xtdrone)."),
        DeclareLaunchArgument("px4", default_value="true",
                              description="Start PX4 (and MAVROS) per vehicle; false spawns models only (chapter 43)."),
        DeclareLaunchArgument("speed_factor", default_value="1.0",
                              description="Target real-time factor (> 1 runs faster than real time if the CPU allows)."),
        DeclareLaunchArgument("ekf", default_value="gps",
                              description=f"EKF2 fusion preset: {', '.join(EKF_PRESETS)}."),
        DeclareLaunchArgument("px4_params", default_value="",
                              description="Extra PX4 parameters, 'NAME=value NAME=value'."),
        DeclareLaunchArgument("gz_verbose", default_value="1", description="Gazebo verbosity 0-4."),
        DeclareLaunchArgument("px4_dir", default_value=px4.default_px4_dir(),
                              description="Pinned PX4-Autopilot v1.16.2 checkout."),
        DeclareLaunchArgument("rootfs", default_value=str(Path.home() / ".ros" / "xtdrone_px4"),
                              description="Working directories of the PX4 instances."),
        OpaqueFunction(function=launch_setup),
    ])
