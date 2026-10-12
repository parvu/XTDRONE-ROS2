"""2D laser SLAM for an iris_2d_lidar (chapters 17 and 18).

  ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=indoor3 ekf:=laser2d communication:=basic
  ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=slam_toolbox      # chapter 17 (HectorSLAM in ROS 1)
  ros2 launch xtdrone_slam laser_slam_2d.launch.py slam:=cartographer      # chapter 18

Starts the static TFs odom -> base_link (no odometry: pure scan matching, like HectorSLAM) and
base_link -> laser_2d, the SLAM node, laser_transfer (pose -> /<v>/mavros/vision_pose/pose) and,
with rviz:=true, RViz.
"""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def slam_toolbox_params(share, vehicle):
    """config/slam_toolbox_2d.yaml with the vehicle's scan topic, as a temporary file."""
    params = yaml.safe_load((share / "config" / "slam_toolbox_2d.yaml").read_text())
    params["slam_toolbox"]["ros__parameters"]["scan_topic"] = f"/{vehicle}/scan"
    f = tempfile.NamedTemporaryFile("w", prefix="xtdrone_slam_toolbox_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, f)
    f.close()
    return f.name


def nodes(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)

    share = Path(get_package_share_directory("xtdrone_slam"))
    vehicle, slam = arg("vehicle"), arg("slam")
    vtype, vid = vehicle.rsplit("_", 1)
    sim = {"use_sim_time": True}
    out = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_to_laser_2d",
             arguments=["--z", "0.1", "--frame-id", "base_link", "--child-frame-id", "laser_2d"], parameters=[sim]),
    ]
    if slam == "slam_toolbox":
        out += [
            Node(package="tf2_ros", executable="static_transform_publisher", name="odom_to_base_link",
                 arguments=["--frame-id", "odom", "--child-frame-id", "base_link"], parameters=[sim]),
            # slam_toolbox's own launch: in Jazzy the node is a lifecycle node that it configures and activates
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py")),
                launch_arguments={"slam_params_file": slam_toolbox_params(share, vehicle),
                                  "use_sim_time": "true"}.items()),
        ]
    elif slam == "cartographer":
        out += [
            # the bridged Gazebo IMU (x500 base_link IMU) carries the gz scoped frame name (:: -> /)
            Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_to_imu",
                 arguments=["--frame-id", "base_link", "--child-frame-id", f"{vehicle}/base_link/imu_sensor"],
                 parameters=[sim]),
            Node(package="cartographer_ros", executable="cartographer_node", output="screen", parameters=[sim],
                 arguments=["-configuration_directory", str(share / "config"),
                            "-configuration_basename", "cartographer_2d.lua"],
                 remappings=[("scan", f"/{vehicle}/scan"), ("imu", f"/{vehicle}/imu_gazebo")]),
            Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node", output="log",
                 parameters=[sim], arguments=["-resolution", "0.05"]),
        ]
    else:
        raise RuntimeError("slam must be slam_toolbox or cartographer")
    # wall-clock stamps on purpose: MAVROS maps them to PX4 time with its (wall-clock) timesync
    out.append(Node(package="xtdrone_slam", executable="laser_transfer", arguments=[vtype, vid, slam],
                    output="screen"))
    if arg("rviz").lower() in ("1", "true", "yes"):
        out.append(Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config" / "laser_slam_2d.rviz")],
                        parameters=[sim]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("slam", default_value="slam_toolbox", description="slam_toolbox or cartographer"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
