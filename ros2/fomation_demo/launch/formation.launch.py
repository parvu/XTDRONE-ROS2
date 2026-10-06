from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context, *args, **kwargs):
    uav_type = LaunchConfiguration("uav_type").perform(context)
    uav_count = int(LaunchConfiguration("uav_count").perform(context))
    if uav_count not in (6, 9, 18):
        raise RuntimeError("uav_count must be 6, 9, or 18")

    nodes = [
        Node(
            package="xtdrone_formation_demo",
            executable="leader",
            name="leader",
            arguments=[uav_type, str(uav_count)],
            output="screen",
        )
    ]
    nodes.extend(
        Node(
            package="xtdrone_formation_demo",
            executable="follower",
            name="follower{}".format(uav_id - 1),
            arguments=[uav_type, str(uav_id), str(uav_count)],
            output="screen",
        )
        for uav_id in range(1, uav_count)
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_type", default_value="iris"),
            DeclareLaunchArgument("uav_count", default_value="6"),
            OpaqueFunction(function=launch_nodes),
        ]
    )
