"""Hybrid A* planning and pure pursuit for the CAT Vehicle UGV (chapter 31).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_ugv
  ros2 launch xtdrone_ugv_planning ugv_planning.launch.py [rviz:=true]
  # goal: RViz "2D Goal Pose" (/goal_pose), in the map frame

slam_toolbox (Gmapping in ROS 1) maps outdoor1 with the front SICK lidar (/ugv_0/front_laser_points) and the
ground-truth odometry (odom_to_tf: TF ugv_0/odom -> ugv_0/base_footprint, the CAT Vehicle's wheel odometry in
ROS 1). hybrid_astar_node plans on /map from the vehicle pose (TF map -> ugv_0/base_footprint) to the goal and
publishes /front_end_path; pure_pursuit_node follows it with speed and front steering angle on /ugv_0/cmd_vel.
"""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def slam_params(ugv, scan_matching):
    """xtdrone_slam's slam_toolbox_2d.yaml for the UGV's frames and its 80 m SICK lidar."""
    params = yaml.safe_load((Path(get_package_share_directory("xtdrone_slam")) / "config" /
                             "slam_toolbox_2d.yaml").read_text())
    p = params["slam_toolbox"]["ros__parameters"]
    p.update({"odom_frame": f"{ugv}/odom", "base_frame": f"{ugv}/base_footprint", "map_frame": "map",
              "scan_topic": f"/{ugv}/front_laser_points", "resolution": 0.1, "max_laser_range": 40.0,
              "use_sim_time": True, "use_scan_matching": scan_matching})
    # scan_matching:=false (default): the odometry is Gazebo's ground truth (the CAT Vehicle's wheel odometry in
    # ROS 1), and in the sparse outdoor scans scan matching only added error: map -> odom drifted and turned by
    # metres, and the vehicle, following the plan in the map frame, missed its goal in the world
    # minimum_travel_distance / heading stay 0 (every scan is used): with Gmapping's 0.5 m the map of a
    # standing vehicle had one scan, and Karto marks a cell only after two passes, so the map stayed nearly empty
    f = tempfile.NamedTemporaryFile("w", prefix="xtdrone_ugv_slam_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, f)
    f.close()
    return f.name


def nodes(context, *args, **kwargs):
    ugv = LaunchConfiguration("ugv").perform(context)
    base = f"{ugv}/base_footprint"
    sim = {"use_sim_time": True}
    return [
        Node(package="xtdrone_ugv_planning", executable="odom_to_tf", output="log",
             parameters=[sim, {"odom_topic": f"/{ugv}/ground_truth/odom"}]),
        # SICK LMS291 of the CAT Vehicle (generate_ugv_models.py)
        Node(package="tf2_ros", executable="static_transform_publisher", output="log", parameters=[sim],
             arguments=["--x", "2.4662", "--z", "0.926879", "--frame-id", base, "--child-frame-id",
                        "front_laser_link"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py")),
            launch_arguments={"slam_params_file": slam_params(
                ugv, LaunchConfiguration("scan_matching").perform(context).lower() in ("1", "true", "yes")),
                "use_sim_time": "true"}.items()),
        Node(package="xtdrone_ugv_planning", executable="hybrid_astar_node", output="screen",
             prefix=LaunchConfiguration("planner_prefix").perform(context) or None,
             parameters=[sim, {"base_frame": base}]),
        Node(package="xtdrone_ugv_planning", executable="pure_pursuit_node", output="screen",
             parameters=[sim, {"base_frame": base, "cmd_topic": f"/{ugv}/cmd_vel", "wheel_base": 2.6,
                               "speed": float(LaunchConfiguration("speed").perform(context))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ugv", default_value="ugv_0"),
        DeclareLaunchArgument("speed", default_value="3.0", description="cruise speed of pure pursuit (m/s)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("scan_matching", default_value="false",
                              description="slam_toolbox scan matching (with ground-truth odometry: off)"),
        DeclareLaunchArgument("planner_prefix", default_value="", description="e.g. 'gdb -batch -ex run -ex bt --args'"),
        OpaqueFunction(function=nodes),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             parameters=[{"use_sim_time": True}]),
    ])
