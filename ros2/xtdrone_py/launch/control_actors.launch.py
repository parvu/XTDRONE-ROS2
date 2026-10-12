"""The six robocup actors of control/actor/control_actors.sh (chapter 38)."""
from launch import LaunchDescription
from launch_ros.actions import Node

TARGETS = [(0, -34, 34, 5), (1, -38, -36, 5), (2, 2, -2, 5), (3, -2, 2, 5), (4, -2, -2, 5), (5, 0, 0, 5)]


def generate_launch_description():
    return LaunchDescription([
        Node(package="xtdrone_py", executable="control_actor", arguments=[str(a) for a in t], output="log")
        for t in TARGETS
    ])
