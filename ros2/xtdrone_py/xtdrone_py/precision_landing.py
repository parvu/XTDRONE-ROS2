"""AprilTag precision landing (port of control/precision_landing.py, chapter 28).

  ros2 run xtdrone_py precision_landing iris 0                 # OpenCV AprilTag detector
  ros2 run xtdrone_py precision_landing iris 0 --detector apriltag_ros

The vehicle <type>_<id> lands on the tag with the same id. The ROS 1 version looked up
map -> tag_<id> in a TF tree that MAVROS and apriltag_ros published with per-vehicle frame
names. Here the tag position in the camera frame comes from either
  * opencv: cv2.aruco (DICT_APRILTAG_36h11) + solvePnP on /<v>/camera/image_raw, or
  * apriltag_ros: the TF <camera frame> -> <tag frame> that apriltag_ros (ROS 2) publishes,
and is turned into the vehicle's local ENU frame with its own pose (downward camera of
iris_downward_camera: image top = vehicle front). The command is the same as before:
cmd_vel_enu = Kp * (tag - position) horizontally, -land_vel vertically.
"""
import argparse

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from xtdrone_py.common import run

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

CAMERA_OFFSET = np.array([0.0, 0.0, -0.08])  # camera_link in base_link (iris_downward_camera)


def quat_rotate(q, v):
    """Rotate v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def optical_to_body(p):
    """Downward camera optical frame (x right, y down = vehicle back, z forward = down) -> FLU body."""
    return np.array([-p[1], -p[0], -p[2]])


def tag_in_camera(gray, K, tag_id, tag_size=0.5, detector=None):
    """Position (optical frame) of tag tag_id in a grayscale image, or None."""
    detector = detector or make_detector()
    corners, ids, _ = detector(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.ravel()):
        if int(i) != tag_id:
            continue
        h = tag_size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, _, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, None,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            return tvec.ravel()
    return None


def make_detector():
    """gray -> (corners, ids, rejected) for tag36h11, with the OpenCV >= 4.7 or the older aruco API
    (Ubuntu 24.04 python3-opencv is 4.6)."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters()).detectMarkers
    params = cv2.aruco.DetectorParameters_create()
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def image_to_gray(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        return a.reshape(msg.height, msg.step)[:, :msg.width]
    ch = 4 if msg.encoding in ("rgba8", "bgra8") else 3
    a = a.reshape(msg.height, msg.step)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    code = cv2.COLOR_RGB2GRAY if msg.encoding.startswith("rgb") else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(a[:, :, :3], code)


class PrecisionLanding(Node):

    def __init__(self, vehicle_type, vehicle_id, detector="opencv", tag_id=None, tag_size=0.5,
                 tag_frame=None, rate=10.0):
        n = f"{vehicle_type}_{vehicle_id}"
        super().__init__(f"{n}_precision_landing")
        self.tag_id = int(vehicle_id) if tag_id is None else tag_id
        self.tag_size = tag_size
        self.Kp = 1.0
        self.land_vel = 0.5
        self.pose = None
        self.K = None
        self.camera_frame = None
        self.image = None
        self.tag_enu = None  # last tag estimate in the local ENU frame (the tag does not move)
        self.mode = detector
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"/{n}/camera/camera_info", self.info_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, f"/xtdrone/{n}/cmd_vel_enu", 1)
        if detector == "opencv":
            if cv2 is None or not hasattr(cv2, "aruco"):
                raise SystemExit("OpenCV with the aruco module is required (python3-opencv)")
            self.detector = make_detector()
            self.create_subscription(Image, f"/{n}/camera/image_raw",
                                     lambda m: setattr(self, "image", m), qos_profile_sensor_data)
        else:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tag_frame = tag_frame or f"tag36h11:{self.tag_id}"
        self.create_timer(1.0 / rate, self.detect)
        self.create_timer(1.0 / 50.0, self.control)
        print(f"{n}: precision landing on tag {self.tag_id} ({detector})", flush=True)

    def info_callback(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def tag_optical(self):
        if self.mode == "opencv":
            if self.image is None or self.K is None:
                return None
            img, self.image = self.image, None
            return tag_in_camera(image_to_gray(img), self.K, self.tag_id, self.tag_size, self.detector)
        if not self.camera_frame:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.camera_frame, self.tag_frame, rclpy_time_zero())
        except Exception:  # noqa: BLE001 - not seen yet
            return None
        tr = t.transform.translation
        return np.array([tr.x, tr.y, tr.z])

    def detect(self):
        if self.pose is None:
            return
        p_opt = self.tag_optical()
        if p_opt is None:
            return
        q = self.pose.pose.orientation
        pos = self.pose.pose.position
        p_body = optical_to_body(p_opt) + CAMERA_OFFSET
        self.tag_enu = quat_rotate((q.x, q.y, q.z, q.w), p_body) + np.array([pos.x, pos.y, pos.z])

    def control(self):
        if self.tag_enu is None or self.pose is None:
            return
        pos = self.pose.pose.position
        cmd = Twist()
        cmd.linear.x = self.Kp * (self.tag_enu[0] - pos.x)
        cmd.linear.y = self.Kp * (self.tag_enu[1] - pos.y)
        cmd.linear.z = -self.land_vel
        self.cmd_pub.publish(cmd)


def rclpy_time_zero():
    from rclpy.time import Time
    return Time()


def main():
    def make(argv):
        ap = argparse.ArgumentParser(prog="precision_landing")
        ap.add_argument("vehicle_type")
        ap.add_argument("vehicle_id", type=int)
        ap.add_argument("--detector", choices=("opencv", "apriltag_ros"), default="opencv")
        ap.add_argument("--tag-id", type=int, default=None, help="default: the vehicle id")
        ap.add_argument("--tag-size", type=float, default=0.5, help="black square edge in m (tags.yaml)")
        ap.add_argument("--tag-frame", default=None, help="apriltag_ros TF frame (default tag36h11:<id>)")
        a = ap.parse_args(argv)
        return PrecisionLanding(a.vehicle_type, a.vehicle_id, a.detector, a.tag_id, a.tag_size, a.tag_frame)
    run(make)


if __name__ == "__main__":
    main()
