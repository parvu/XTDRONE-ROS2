"""Stereo visual-inertial odometry with VINS-Fusion (chapter 14).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor1 ekf:=vision communication:=basic
  ros2 launch xtdrone_slam vio.launch.py

VINS-Fusion (submodule ros2/vins-fusion, ROS 2 Jazzy fork) with config/vins/xtdrone_stereo_imu.yaml
(stereo camera + Gazebo IMU of iris_stereo_camera), and laser_transfer (vins) forwarding the VINS
odometry to /<v>/mavros/vision_pose/pose (ROS 1: vins_transfer.py). loop:=true adds loop_fusion.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    vehicle = arg("vehicle")
    vtype, vid = vehicle.rsplit("_", 1)
    config = arg("config") or str(Path(get_package_share_directory("xtdrone_slam")) / "config" / "vins" /
                                  "xtdrone_stereo_imu.yaml")
    out = [
        Node(package="vins", executable="vins_node", arguments=[config], output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="xtdrone_slam", executable="laser_transfer", arguments=[vtype, vid, "vins"], output="screen"),
    ]
    if arg("loop").lower() in ("1", "true", "yes"):
        out.append(Node(package="loop_fusion", executable="loop_fusion_node", arguments=[config], output="screen",
                        parameters=[{"use_sim_time": True}]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("config", default_value="", description="default: config/vins/xtdrone_stereo_imu.yaml"),
        DeclareLaunchArgument("loop", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
