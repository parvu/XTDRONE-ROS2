"""2D motion planning with Nav2 (chapter 20; replaces move_base).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic
  ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox
  # take off with the keyboard (about 1 m), switch to OFFBOARD, then close the keyboard
  ros2 launch xtdrone_planning nav2_2d.launch.py rviz:=true      # "2D Goal Pose" in RViz

Planner (NavFn, Dijkstra), controller (DWB, holonomic), behaviors and the BT navigator, with
cmd_vel -> /xtdrone/<vehicle>/cmd_vel_flu and the MAVROS odometry for the velocity feedback.
The map and the map -> base_link pose come from the laser SLAM (chapters 17/18).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    share = Path(get_package_share_directory("xtdrone_planning"))
    params = arg("params_file") or str(share / "config" / "nav2_2d.yaml")
    v = arg("vehicle")
    remaps = [("cmd_vel", f"/xtdrone/{v}/cmd_vel_flu"), ("odom", f"/{v}/mavros/local_position/odom")]
    pkg = {"controller_server": "nav2_controller", "planner_server": "nav2_planner",
           "behavior_server": "nav2_behaviors", "bt_navigator": "nav2_bt_navigator"}
    out = [Node(package=pkg[s], executable=s, name=s, output="screen", parameters=[params], remappings=remaps)
           for s in SERVERS]
    out.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
                    output="screen", parameters=[{"use_sim_time": True, "autostart": True, "node_names": SERVERS}]))
    if arg("rviz").lower() in ("1", "true", "yes"):
        out.append(Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": True}],
                        arguments=["-d", str(Path(get_package_share_directory("nav2_bringup")) / "rviz" /
                                         "nav2_default_view.rviz")]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("params_file", default_value="", description="default: config/nav2_2d.yaml"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
