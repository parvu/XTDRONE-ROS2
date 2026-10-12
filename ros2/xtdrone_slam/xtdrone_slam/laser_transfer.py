"""Laser SLAM pose -> PX4 vision pose (port of sensing/slam/laser_slam/script/laser_transfer.py).

  ros2 run xtdrone_slam laser_transfer iris 0 slam_toolbox    # chapter 17 (HectorSLAM in ROS 1)
  ros2 run xtdrone_slam laser_transfer iris 0 cartographer    # chapter 18
  ros2 run xtdrone_slam laser_transfer iris 0 kiss_icp        # chapter 19 (A-LOAM in ROS 1)
  ros2 run xtdrone_slam laser_transfer iris 0 vins            # chapter 14 (vins_transfer.py in ROS 1)
  ros2 run xtdrone_slam laser_transfer iris 0 orbslam3        # chapter 13
  ros2 run xtdrone_slam laser_transfer iris 0 aloam           # an A-LOAM port (camera_init -> aft_mapped)

2D SLAM: the pose is the TF map -> base_link, the height the downward range finder
(/<v>/distance, as in ROS 1). Published at 30 Hz on /<v>/mavros/vision_pose/pose; use
ekf:=vision so that PX4 fuses it (chapter 6).
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

FRAMES = {"slam_toolbox": ("map", "base_link"), "hector": ("map", "base_link"),
          "cartographer": ("map", "base_link"), "aloam": ("camera_init", "aft_mapped"),
          "kiss_icp": ("odom_lidar", "base_link"), "vins": ("world", "body"), "orbslam3": ("map", "camera")}


class LaserTransfer(Node):

    def __init__(self, vehicle_type, vehicle_id, slam):
        super().__init__(f"{vehicle_type}{vehicle_id}_{slam}_laser_transfer")
        if slam not in FRAMES:
            raise SystemExit(f"unknown SLAM '{slam}', use one of {', '.join(FRAMES)}")
        n = f"{vehicle_type}_{vehicle_id}"
        self.parent, self.child = FRAMES[slam]
        self.use_height = slam not in ("aloam", "kiss_icp", "vins", "orbslam3")  # 3D odometry has its own height
        self.height = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseStamped, f"/{n}/mavros/vision_pose/pose", 1)
        if self.use_height:
            self.create_subscription(LaserScan, f"/{n}/distance", self.distance_callback, qos_profile_sensor_data)
        self.tracked = None
        # The SLAM pose describes a moment in the past (sensor time, simulation clock). MAVROS needs ROS
        # (wall) time stamps, so the pose is stamped "now - its age" for EKF2's delay compensation.
        self.sim_now = None
        self.create_subscription(Clock, "/clock", lambda m: setattr(self, "sim_now", m.clock), 10)
        if slam == "cartographer":
            # Cartographer's /tracked_pose (map frame) instead of a TF lookup: the latest map -> base_link
            # in the TF buffer was not always the newest pose (stale entries), which made PX4's estimate jump.
            self.create_subscription(PoseStamped, "/tracked_pose", lambda m: setattr(self, "tracked", m), 10)
        elif slam == "vins":
            # VINS-Fusion's body (IMU) odometry in its gravity-aligned world frame (ROS 1: vins_transfer.py)
            self.create_subscription(Odometry, "/odometry",
                                     lambda m: setattr(self, "tracked", PoseStamped(header=m.header, pose=m.pose.pose)),
                                     10)
        elif slam == "orbslam3":
            # xtdrone_orbslam3 stereo_node publishes the camera pose in a FLU world frame
            self.create_subscription(PoseStamped, "/orbslam3/vision_pose/pose", lambda m: setattr(self, "tracked", m), 10)
        elif slam == "kiss_icp":
            # KISS-ICP's odometry topic, as PoseStamped
            self.create_subscription(Odometry, "/kiss/odometry",
                                     lambda m: setattr(self, "tracked", PoseStamped(header=m.header, pose=m.pose.pose)),
                                     10)
        self.create_timer(1.0 / 30.0, self.publish)
        self.warned = False

    def distance_callback(self, msg):
        # median of the valid rays (the range finder is a narrow fan of a few rays)
        valid = sorted(r for r in msg.ranges if msg.range_min <= r <= msg.range_max)
        self.height = valid[len(valid) // 2] if valid else 0.0

    def stamp(self, sensor_stamp):
        now = self.get_clock().now()
        if self.sim_now is None or (sensor_stamp.sec == 0 and sensor_stamp.nanosec == 0):
            return now.to_msg()
        age = (self.sim_now.sec - sensor_stamp.sec) + (self.sim_now.nanosec - sensor_stamp.nanosec) * 1e-9
        age = min(max(age, 0.0), 0.5)
        return (now - Duration(seconds=age)).to_msg()

    def publish(self):
        if self.tracked is not None:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = self.stamp(self.tracked.header.stamp)
            p.pose = self.tracked.pose
            if self.use_height:
                p.pose.position.z = self.height
            self.pub.publish(p)
            return
        try:
            t = self.tf.lookup_transform(self.parent, self.child, Time())
        except Exception as e:  # noqa: BLE001 - SLAM not started yet
            if not self.warned:
                self.get_logger().info(f"waiting for TF {self.parent} -> {self.child} ({type(e).__name__})")
                self.warned = True
            return
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.stamp(t.header.stamp)
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = self.height if self.use_height else t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        self.pub.publish(p)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 3:
        raise SystemExit("usage: laser_transfer <vehicle_type> <id> slam_toolbox|cartographer|aloam")
    rclpy.init()
    node = LaserTransfer(argv[0], argv[1], argv[2])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
