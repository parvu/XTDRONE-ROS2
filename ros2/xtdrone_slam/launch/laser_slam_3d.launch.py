"""3D laser odometry for an iris_3d_lidar (chapter 19; A-LOAM in ROS 1, KISS-ICP here).

  ros2 launch xtdrone_bringup xtdrone.launch.py world:=outdoor1 vehicles:=iris_3d_lidar ekf:=vision communication:=basic
  ros2 launch xtdrone_slam laser_slam_3d.launch.py

KISS-ICP (submodule ros2/kiss-icp, v1.2.3) on /<v>/velodyne_points with base_link as base frame
(static TF base_link -> laser_3d), and laser_transfer (kiss_icp) forwarding /kiss/odometry to
/<v>/mavros/vision_pose/pose (position, height and attitude from the lidar odometry).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    vehicle = arg("vehicle")
    vtype, vid = vehicle.rsplit("_", 1)
    kiss = Path(get_package_share_directory("kiss_icp")) / "launch" / "odometry.launch.py"
    return [
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_to_laser_3d",
             arguments=["--z", "0.12", "--frame-id", "base_link", "--child-frame-id", "laser_3d"],
             parameters=[{"use_sim_time": True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(kiss)), launch_arguments={
            "topic": f"/{vehicle}/velodyne_points", "base_frame": "base_link", "use_sim_time": "true",
            "visualize": arg("rviz")}.items()),
        Node(package="xtdrone_slam", executable="laser_transfer", arguments=[vtype, vid, "kiss_icp"], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
