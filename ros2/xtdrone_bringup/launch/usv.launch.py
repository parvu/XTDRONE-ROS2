"""Add VRX's WAM-V unmanned surface vehicle to a running sandisland world (chapter 32).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=sandisland
  ros2 launch xtdrone_bringup usv.launch.py                 # WAM-V at the ROS 1 pose (158, 108)
  ros2 topic pub -r 10 /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 100.0"

Uses VRX's spawn launch (vrx_gz spawn.launch.py) with a WAM-V that has only its thrusters
(sensors:=true for VRX's full sensor set: cameras, lidar, GPS, IMU - slow without a GPU).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def spawn(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    vrx = Path(get_package_share_directory("vrx_gz")) / "launch" / "spawn.launch.py"
    largs = {"world": "sydney_regatta", "name": arg("name"), "model": "wam-v",
             "x": arg("x"), "y": arg("y"), "z": arg("z"), "Y": arg("Y")}
    if arg("sensors").lower() not in ("1", "true", "yes"):
        largs["urdf"] = str(Path(get_package_share_directory("xtdrone_gz")) / "usv" / "xtdrone_wamv.urdf.xacro")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vrx)), launch_arguments=largs.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="wamv"),
        DeclareLaunchArgument("x", default_value="158"),
        DeclareLaunchArgument("y", default_value="108"),
        DeclareLaunchArgument("z", default_value="0.1"),
        DeclareLaunchArgument("Y", default_value="-2.76"),
        DeclareLaunchArgument("sensors", default_value="false"),
        OpaqueFunction(function=spawn),
    ])
