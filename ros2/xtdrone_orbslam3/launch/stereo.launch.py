"""Stereo visual SLAM with ORB-SLAM3 (chapter 13; ROS 1: xtdrone_run_orbslam_stereo.sh).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=outdoor1_stereo ekf:=vision communication:=basic
  ros2 launch xtdrone_orbslam3 stereo.launch.py [viewer:=true]

The ORB-SLAM3 pose (/orbslam3/vision_pose/pose, FLU) goes to /<v>/mavros/vision_pose/pose through
laser_transfer (orbslam3), which stamps it with its sensor time for EKF2's delay compensation.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ORB = Path(os.environ.get("ORB_SLAM3_DIR", Path.home() / ".cache/xtdrone/orbslam3/ORB_SLAM3"))


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    v = arg("vehicle")
    vtype, vid = v.rsplit("_", 1)
    settings = arg("settings") or str(Path(get_package_share_directory("xtdrone_orbslam3")) / "config" /
                                      "xtdrone_stereo.yaml")
    return [
        Node(package="xtdrone_orbslam3", executable="stereo_node", output="screen",
             arguments=[str(ORB / "Vocabulary" / "ORBvoc.txt"), settings, arg("viewer")],
             parameters=[{"use_sim_time": True}],
             remappings=[("/camera/left/image_raw", f"/{v}/stereo_camera/left/image_raw"),
                         ("/camera/right/image_raw", f"/{v}/stereo_camera/right/image_raw")]),
        Node(package="xtdrone_slam", executable="laser_transfer", arguments=[vtype, vid, "orbslam3"], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("settings", default_value="", description="default: config/xtdrone_stereo.yaml"),
        DeclareLaunchArgument("viewer", default_value="false", description="ORB-SLAM3's Pangolin viewer"),
        OpaqueFunction(function=nodes),
    ])
