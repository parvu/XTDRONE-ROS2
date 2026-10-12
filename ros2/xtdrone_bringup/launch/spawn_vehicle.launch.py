"""Add one XTDrone vehicle (gz model + PX4 + MAVROS + bridge) to a running simulation
(replaces single_vehicle_spawn_xtd.launch for the 'add UAV simulation node dynamically' chapter).

  ros2 launch xtdrone_bringup spawn_vehicle.launch.py model:=iris id:=1 instance:=1 x:=3 y:=0

instance is the global PX4 instance (MAVLink ports 24540+instance / 34580+instance)
and must not be used by another running vehicle.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from xtdrone_bringup import fleet, px4
from xtdrone_bringup.sim import EKF_PRESETS, SimPaths, mavlink_profile, px4_params, vehicle_actions
from xtdrone_bringup.gz_util import running_world


def launch_setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    paths = SimPaths.find(arg("px4_dir"))
    model = fleet.MODEL_ALIASES.get(arg("model"), arg("model"))
    v = fleet.Vehicle(model, fleet.vehicle_type_of(model), int(arg("id")), instance=int(arg("instance")),
                      pose=[float(arg(k)) for k in ("x", "y", "z", "R", "P", "Y")])
    world = arg("world") or running_world()
    return vehicle_actions(v, world, paths, arg("rootfs"), communication=arg("communication"),
                           params=px4_params(arg("ekf"), arg("px4_params")),
                           mavros_plugins=arg("mavros_plugins"),
                           mavlink_profile=mavlink_profile(arg("mavlink_profile"), 1),
                           uxrce_dds=arg("uxrce_dds").lower() in ("1", "true", "yes"))


def generate_launch_description():
    args = [
        DeclareLaunchArgument("model", default_value="iris", description="XTDrone vehicle model"),
        DeclareLaunchArgument("id", default_value="1", description="N in <type>_N"),
        DeclareLaunchArgument("instance", default_value="1", description="global PX4 instance"),
        DeclareLaunchArgument("world", default_value="", description="running world name (auto-detected)"),
        DeclareLaunchArgument("communication", default_value="none", description="none, basic or enhanced"),
        DeclareLaunchArgument("ekf", default_value="gps", description=", ".join(EKF_PRESETS)),
        DeclareLaunchArgument("mavros_plugins", default_value="xtdrone", description="xtdrone or all"),
        DeclareLaunchArgument("uxrce_dds", default_value="false", description="start PX4's uXRCE-DDS client"),
        DeclareLaunchArgument("mavlink_profile", default_value="full",
                              description="full or swarm (use swarm when added to a large fleet)"),
        DeclareLaunchArgument("px4_params", default_value="", description="'NAME=value ...'"),
        DeclareLaunchArgument("px4_dir", default_value=px4.default_px4_dir()),
        DeclareLaunchArgument("rootfs", default_value=str(Path.home() / ".ros" / "xtdrone_px4")),
    ]
    args += [DeclareLaunchArgument(k, default_value="0") for k in ("x", "y", "z", "R", "P", "Y")]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
