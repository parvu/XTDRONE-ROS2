"""Leader + followers of the XTDrone formation demo (chapters 24 and 25).

  ros2 launch xtdrone_formation_demo formation.launch.py uav_count:=6
  ros2 launch xtdrone_formation_demo formation.launch.py uav_count:=9 controller:=consensus avoid:=true

controller:=simple   followers track the global leader (follower.py, chapter 25)
controller:=consensus followers track their local leaders (follower_consensus.py, chapter 24)
control_type:=vel|accel applies to the consensus controller and avoid.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    uav_type = arg("uav_type")
    uav_count = int(arg("uav_count"))
    controller = arg("controller")
    control_type = arg("control_type")
    if uav_count not in (6, 9, 18):
        raise RuntimeError("uav_count must be 6, 9, or 18")
    if controller not in ("simple", "consensus"):
        raise RuntimeError("controller must be simple or consensus")
    if control_type not in ("vel", "accel"):
        raise RuntimeError("control_type must be vel or accel")
    if controller == "simple" and control_type != "vel":
        raise RuntimeError("the simple controller supports control_type:=vel only")

    nodes = [
        Node(
            package="xtdrone_formation_demo",
            executable="leader",
            name="leader",
            arguments=[uav_type, str(uav_count)],
            output="screen",
        )
    ]
    if arg("avoid").lower() in ("1", "true", "yes"):
        nodes.append(Node(
            package="xtdrone_formation_demo",
            executable="avoid",
            name="avoid",
            arguments=[uav_type, str(uav_count), control_type],
            output="screen",
        ))
    for uav_id in range(1, uav_count):
        if controller == "simple":
            executable, extra = "follower", []
        else:
            executable, extra = "follower_consensus", [control_type]
        nodes.append(Node(
            package="xtdrone_formation_demo",
            executable=executable,
            name="follower{}".format(uav_id - 1),
            arguments=[uav_type, str(uav_id), str(uav_count)] + extra,
            output="screen",
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_type", default_value="iris"),
            DeclareLaunchArgument("uav_count", default_value="6"),
            DeclareLaunchArgument("controller", default_value="simple",
                                  description="simple (chapter 25) or consensus (chapter 24)"),
            DeclareLaunchArgument("control_type", default_value="vel", description="vel or accel"),
            DeclareLaunchArgument("avoid", default_value="false",
                                  description="start the pairwise collision avoidance node"),
            OpaqueFunction(function=launch_nodes),
        ]
    )
