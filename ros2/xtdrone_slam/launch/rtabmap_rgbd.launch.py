"""3D dense reconstruction with RTAB-Map on an iris_realsense_camera (chapter 16).

  ros2 launch xtdrone_bringup xtdrone.launch.py world:=indoor1 vehicles:=iris_realsense_camera communication:=basic
  ros2 launch xtdrone_slam rtabmap_rgbd.launch.py                 # ground-truth odometry (default)
  ros2 launch xtdrone_slam rtabmap_rgbd.launch.py odom:=visual    # RTAB-Map RGB-D visual odometry
  ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel        # fly around slowly

The ROS 1 chapter used VINS-Fusion as odometry (rtabmap_vins.launch). Until the VINS-Fusion port
(chapter 14) the default is the Gazebo ground truth (odom:=groundtruth, through odom_relay).
odom:=visual runs RTAB-Map's RGB-D visual odometry (needs textured scenes; it loses track on the
plain walls of indoor1), odom:=external with odom_topic:=... uses another nav_msgs/Odometry. Outputs: /rtabmap/cloud_map, /rtabmap/octomap_full,
/rtabmap/map (2D grid), the database in ~/.ros/rtabmap.db.
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

    v = arg("vehicle")
    cam = f"/{v}/realsense/depth_camera"
    mode = arg("odom")
    if mode not in ("groundtruth", "visual", "external"):
        raise RuntimeError("odom must be groundtruth, visual or external")
    visual = mode == "visual"
    extra = []
    if not visual:
        source = f"/{v}/ground_truth/odom" if mode == "groundtruth" else arg("odom_topic")
        extra.append(Node(package="xtdrone_slam", executable="odom_relay", arguments=[source],
                          parameters=[{"use_sim_time": True}], output="log"))
    rtabmap = Path(get_package_share_directory("rtabmap_launch")) / "launch" / "rtabmap.launch.py"
    return extra + [
        # realsense_link sits at (0.12, 0, 0.03) in base_link; the images carry the frame
        # depth_camera_base, which is the optical frame (z forward, x right, y down).
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_to_depth_camera",
             arguments=["--x", "0.12", "--z", "0.03", "--roll", "-1.5707963", "--yaw", "-1.5707963",
                        "--frame-id", "base_link", "--child-frame-id", "depth_camera_base"],
             parameters=[{"use_sim_time": True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(rtabmap)), launch_arguments={
            "use_sim_time": "true",
            "rtabmap_args": "--delete_db_on_start",
            "frame_id": "base_link",
            "rgb_topic": f"{cam}/color/image_raw",
            "depth_topic": f"{cam}/depth/image_raw",
            "camera_info_topic": f"{cam}/color/camera_info",
            # one gz rgbd_camera: colour and depth carry the same stamp; approximate sync paired frames
            # 0.05-0.1 s apart when the bridge dropped one, which broke the odometry in turns
            "approx_sync": "false",
            "visual_odometry": "true" if visual else "false",
            "odom_topic": "odom" if visual else "/xtdrone/odom",
            "rviz": arg("rviz"),
            "rtabmap_viz": arg("rtabmap_viz"),
            "queue_size": "30",
        }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="iris_0"),
        DeclareLaunchArgument("odom", default_value="groundtruth",
                              description="groundtruth, visual (RTAB-Map RGB-D) or external"),
        DeclareLaunchArgument("odom_topic", default_value="/vins_estimator/odometry",
                              description="odometry topic for odom:=external"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),
        OpaqueFunction(function=nodes),
    ])
