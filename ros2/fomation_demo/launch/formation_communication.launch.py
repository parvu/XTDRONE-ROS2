"""Chapter 27: formation control with a communication model.

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=formation_comm_20
  ros2 launch xtdrone_formation_demo formation_communication.launch.py
  ros2 run xtdrone_py multirotor_keyboard_control iris 20 vel   # g (leader), v takeoff, 1/2 FORM_1/FORM_2

One consensus controller node per UAV (instead of mpiexec), the SINR communication model
(/communication_verify) and the UAV-state aggregator (/xtdrone_gcs/all_uav_data). By default
all of them share one process (single_process:=false starts one process per node, like MPI).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    vtype, num = arg("uav_type"), int(arg("uav_count"))
    if arg("single_process").lower() in ("1", "true", "yes"):
        return [Node(package="xtdrone_formation_demo", executable="formation_comm_all", output="screen",
                     arguments=[vtype, str(num), "--leaders", arg("leaders"), "--pos-file", arg("pos_file")])]
    nodes = [
        Node(package="xtdrone_formation_demo", executable="uav_data_aggregator",
             arguments=[vtype, str(num)], output="screen"),
        Node(package="xtdrone_formation_demo", executable="communication_verify",
             arguments=[str(num), "--leaders", arg("leaders")], output="screen"),
    ]
    nodes += [Node(package="xtdrone_formation_demo", executable="formation_comm_controller",
                   arguments=[vtype, str(i), str(num), "--pos-file", arg("pos_file")], output="screen")
              for i in range(num)]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("uav_type", default_value="iris"),
        DeclareLaunchArgument("uav_count", default_value="20"),
        DeclareLaunchArgument("leaders", default_value="all",
                              description="'all' or comma-separated transmitting UAV ids, e.g. 0,13,17"),
        DeclareLaunchArgument("pos_file", default_value="UAV_pos1",
                              description="desired formation table: UAV_pos1, UAV_pos2, UAV_pos3 or a path"),
        DeclareLaunchArgument("single_process", default_value="true",
                              description="run all nodes in one process (lighter with 20 UAVs)"),
        OpaqueFunction(function=launch_nodes),
    ])
