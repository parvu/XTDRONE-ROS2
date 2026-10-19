"""Start XTDrone communication nodes for many vehicles
(replaces communication/multi_vehicle_communication.sh).

  ros2 launch xtdrone_py multi_vehicle_communication.launch.py iris_num:=3
  ros2 launch xtdrone_py multi_vehicle_communication.launch.py iris_gimbal_num:=6 enhanced:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MULTIROTORS = ("iris", "iris_gimbal", "solo")
VTOLS = ("standard_vtol",)


def setup(context):
    enhanced = LaunchConfiguration("enhanced").perform(context).lower() == "true"
    nodes = []
    for vtype in MULTIROTORS:
        exe = "multirotor_communication_enhanced" if enhanced else "multirotor_communication"
        for i in range(int(LaunchConfiguration(f"{vtype}_num").perform(context))):
            nodes.append(Node(package="xtdrone_py", executable=exe, arguments=[vtype, str(i)], output="screen"))
    for vtype in VTOLS:
        for i in range(int(LaunchConfiguration(f"{vtype}_num").perform(context))):
            nodes.append(Node(package="xtdrone_py", executable="vtol_communication",
                              arguments=[vtype, str(i)], output="screen"))
    for i in range(int(LaunchConfiguration("plane_num").perform(context))):
        nodes.append(Node(package="xtdrone_py", executable="plane_communication", arguments=[str(i)], output="screen"))
    for i in range(int(LaunchConfiguration("rover_num").perform(context))):
        nodes.append(Node(package="xtdrone_py", executable="rover_communication", arguments=[str(i)], output="screen"))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument(f"{t}_num", default_value="0", description=f"number of {t} vehicles")
            for t in MULTIROTORS + VTOLS + ("plane", "rover")]
    args.append(DeclareLaunchArgument("enhanced", default_value="false",
                                      description="use multirotor_communication_enhanced (position hold)"))
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
