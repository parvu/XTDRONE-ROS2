"""XTDrone simulation on Gazebo Harmonic + ArduPilot 4.7.2 SITL + MAVROS 2.

Replaces the ROS 1 launch files in XTDrone's sitl_config/launch (indoor1.launch,
outdoor2.launch, multi_vehicle.launch, ...).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1
  ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor2 vehicles:=plane
  ros2 launch xtdrone_bringup xtdrone.launch.py world:=indoor1 vehicles:="iris*6"

Per vehicle <type>_<id> (e.g. iris_0) it starts:
  * the gz model, spawned with that name,
  * ArduPilot SITL instance N, linked to it by the ArduPilot Gazebo plugin (UDP port 9002 + 10 N),
  * MAVROS 2 in /<type>_<id>/mavros (fcu udp://:14550+N@),
  * ros_gz_bridge for the vehicle sensors and ground truth.
"""
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from xtdrone_bringup import ardupilot, fleet
from xtdrone_bringup.sim import (EKF_PRESETS, SWARM_THRESHOLD, SimPaths, actor_bridges, actor_walkers, autopilot_params,
                                 fleet_bridge, set_real_time_factor, vehicle_actions, world_name, world_with_step)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _arg(context, name).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be 'true' or 'false'")
    return value == "true"


def launch_setup(context):
    paths = SimPaths.find(_arg(context, "ardupilot_dir"))
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
    # vehicles that need a finer physics step (tailsitter: 2 ms); the argument step:= overrides
    step = float(_arg(context, "step") or (0.002 if any(v.vtype == "tailsitter" for v in vehicles) else 0))
    if step > 0:
        world_file = world_with_step(world_file, step, tempfile.mkdtemp(prefix="xtdrone_world_"))
    communication = _arg(context, "communication")
    if communication not in ("none", "basic", "enhanced"):
        raise RuntimeError("communication must be none, basic or enhanced")
    verbose = _arg(context, "gz_verbose")

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", paths.resource_path()),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", paths.plugin_path()),
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
    # script actors (outdoor1, outdoor4) wait for the drone and the YOLO nodes before walking
    for name, loop in actor_walkers(world_file):
        if vehicles:
            actions.append(Node(package="xtdrone_py", executable="actor_walker", output="screen",
                                arguments=[name, vehicles[0].vtype, str(vehicles[0].id_in_type), "--loop", loop]))
    plugins = _arg(context, "mavros_plugins")
    if plugins == "auto":
        plugins = "swarm" if len(vehicles) > SWARM_THRESHOLD else "xtdrone"
    for k, v in enumerate(vehicles):
        # Stagger vehicles a little so gz is up and spawns do not pile up.
        actions.append(TimerAction(period=2.0 + 0.5 * k, actions=vehicle_actions(
            v, world, paths, _arg(context, "rootfs"), mavros=_bool(context, "mavros"),
            bridge=False, communication=communication,
            params=autopilot_params(_arg(context, "ekf"), _arg(context, "params")), speed_factor=speed_factor,
            sitl=_bool(context, "sitl"), mavros_plugins=plugins)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="",
                              description="Scenario name in xtdrone_bringup/config/scenarios or a YAML path. "
                                          "Overrides world and vehicles."),
        DeclareLaunchArgument("world", default_value="indoor1",
                              description="World name (xtdrone_gz/worlds or ardupilot_gazebo worlds) or .sdf path."),
        DeclareLaunchArgument("vehicles", default_value="iris_stereo_camera",
                              description="Fleet spec, e.g. 'iris_stereo_camera*3,plane'."),
        DeclareLaunchArgument("gui", default_value="true", description="Start the Gazebo GUI."),
        DeclareLaunchArgument("mavros", default_value="true", description="Start MAVROS 2 per vehicle."),
        DeclareLaunchArgument("bridge", default_value="true", description="Bridge sensors and ground truth."),
        DeclareLaunchArgument("communication", default_value="none",
                              description="Also start XTDrone communication nodes: none, basic or enhanced."),
        DeclareLaunchArgument("mavros_plugins", default_value="auto",
                              description="MAVROS plugins: 'xtdrone' (those XTDrone uses), 'swarm' (the minimum "
                                          "for the XTDrone nodes), 'all', or 'auto' (swarm above 6 vehicles, "
                                          "else xtdrone)."),
        DeclareLaunchArgument("sitl", default_value="true",
                              description="Start ArduPilot SITL (and MAVROS) per vehicle; false spawns models only "
                                          "(chapter 43)."),
        DeclareLaunchArgument("step", default_value="",
                              description="Physics step of the world in s (default: the world's, 0.004; 0.002 with "
                                          "a tailsitter)."),
        DeclareLaunchArgument("speed_factor", default_value="1.0",
                              description="Target real-time factor (> 1 runs faster than real time if the CPU allows)."),
        DeclareLaunchArgument("ekf", default_value="gps",
                              description=f"EKF3 fusion preset: {', '.join(EKF_PRESETS)}."),
        DeclareLaunchArgument("params", default_value="",
                              description="Extra ArduPilot parameters, 'NAME=value NAME=value'."),
        DeclareLaunchArgument("gz_verbose", default_value="1", description="Gazebo verbosity 0-4."),
        DeclareLaunchArgument("ardupilot_dir", default_value=ardupilot.default_ardupilot_dir(),
                              description="ArduPilot 4.7.2 checkout (built by scripts/build_ardupilot.sh)."),
        DeclareLaunchArgument("rootfs", default_value=str(Path.home() / ".ros" / "xtdrone_sitl"),
                              description="Working directories of the SITL instances."),
        OpaqueFunction(function=launch_setup),
    ])
