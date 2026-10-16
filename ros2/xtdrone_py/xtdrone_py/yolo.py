"""Target detection and tracking with a gimbal camera (chapter 23).

  ros2 run xtdrone_py gimbal_control typhoon_h480 0 [--pitch -60]   # sensing/gimbal/gimbal_control.py;
      keeps the camera on the first "person" of /<v>/detections as the drone tilts and rolls
  ros2 run xtdrone_py yolo_detection typhoon_h480 0                  # darknet_ros (YOLO) -> vision_msgs
  ros2 run xtdrone_py yolo_human_tracking typhoon_h480 0             # control/yolo_human_tracking.py

yolo_detection runs YOLOv4-tiny (COCO, the Darknet network family of darknet_ros) with OpenCV's DNN
module on the CPU: no darknet build, no GPU. Weights: ~/.cache/xtdrone/yolo (downloaded on first use,
or --weights/--cfg). Detections: /<v>/detections (vision_msgs/Detection2DArray, class names from
coco.names, e.g. "person"), annotated image /<v>/detections/image.
"""
import argparse
import math
import os
import urllib.request

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from xtdrone_py.common import run

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

YOLO_DIR = os.path.expanduser("~/.cache/xtdrone/yolo")
YOLO_FILES = {
    "yolov4-tiny.weights": "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights",
    "yolov4-tiny.cfg": "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg",
    "coco.names": "https://raw.githubusercontent.com/AlexeyAB/darknet/master/data/coco.names",
}
CAMERA = {"typhoon_h480": "cgo3_camera/image_raw", "iris": "camera/image_raw"}


def yolo_files():
    os.makedirs(YOLO_DIR, exist_ok=True)
    for name, url in YOLO_FILES.items():
        path = os.path.join(YOLO_DIR, name)
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            print(f"downloading {url}", flush=True)
            urllib.request.urlretrieve(url, path)
    return [os.path.join(YOLO_DIR, n) for n in ("yolov4-tiny.cfg", "yolov4-tiny.weights", "coco.names")]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def pitch_of(q):
    x, y, z, w = q
    return math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))


class Yolo:
    """YOLOv4-tiny on a BGR image: list of (class_name, score, x_min, y_min, x_max, y_max)."""

    def __init__(self, cfg, weights, names, conf=0.4, nms=0.4, size=(960, 544)):
        self.net = cv2.dnn.readNetFromDarknet(cfg, weights)
        self.model = cv2.dnn_DetectionModel(self.net)
        # Network input. The typhoon's gimbal camera is 640 x 360 with a 115 deg field of view: a pedestrian
        # 10 m away is about 45 px tall, too small for YOLOv4-tiny at the usual 416 x 416 (no detection);
        # 960 x 544 (16:9, multiples of 32) finds it with score 0.95 in about 70 ms on the CPU.
        self.model.setInputParams(size=tuple(size), scale=1 / 255.0, swapRB=True)
        self.names = [n.strip() for n in open(names) if n.strip()]
        self.conf, self.nms = conf, nms

    def detect(self, bgr):
        classes, scores, boxes = self.model.detect(bgr, self.conf, self.nms)
        out = []
        for c, s, (x, y, w, h) in zip(np.array(classes).ravel(), np.array(scores).ravel(), boxes):
            out.append((self.names[int(c)], float(s), int(x), int(y), int(x + w), int(y + h)))
        return out


class YoloDetection(Node):

    def __init__(self, vtype, vid, cfg=None, weights=None, names=None, rate=5.0, size=(960, 544)):
        n = f"{vtype}_{vid}"
        super().__init__(f"yolo_detection_{n}")
        if cfg is None:
            cfg, weights, names = yolo_files()
        self.yolo = Yolo(cfg, weights, names, size=size)
        self.image = None
        self.create_subscription(Image, f"/{n}/{CAMERA.get(vtype, 'camera/image_raw')}",
                                 lambda m: setattr(self, "image", m), qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection2DArray, f"/{n}/detections", 1)
        self.image_pub = self.create_publisher(Image, f"/{n}/detections/image", 1)
        self.create_timer(1.0 / rate, self.process)
        print(f"{n}: YOLOv4-tiny on {CAMERA.get(vtype, 'camera/image_raw')}", flush=True)

    def process(self):
        if self.image is None:
            return
        m, self.image = self.image, None
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = np.ascontiguousarray(a[:, :, ::-1]) if m.encoding == "rgb8" else a.copy()
        out = Detection2DArray(header=m.header)
        for name, score, x0, y0, x1, y1 in self.yolo.detect(bgr):
            d = Detection2D(header=m.header)
            d.bbox.center.position.x, d.bbox.center.position.y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            d.bbox.size_x, d.bbox.size_y = float(x1 - x0), float(y1 - y0)
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = name, score
            d.results.append(h)
            out.detections.append(d)
            cv2.rectangle(bgr, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2.putText(bgr, f"{name} {score:.2f}", (x0, max(0, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        self.pub.publish(out)
        self.image_pub.publish(Image(header=m.header, height=bgr.shape[0], width=bgr.shape[1], encoding="bgr8",
                                     step=bgr.shape[1] * 3, data=bgr.tobytes()))


def gimbal_focus(err, base, body_roll, kp=0.002):
    """Gimbal joint targets (pitch, roll, yaw) that keep the person bbox centered.

    err is the bbox center minus the image center (u - 320, v - 180), None when no person is in
    view. The camera joint turns about -y (positive = up), the yaw joint about +z (positive = left)
    and the roll joint about -x upstream of the pitch, so roll_cmd = body roll cancels the body
    roll's disturbance on the view directly; the yaw/pitch P terms on the image error absorb the
    body pitch and yaw as the drone tilts to follow."""
    roll = max(-0.785, min(0.785, body_roll))
    if err is None:
        return base, roll, 0.0
    u, v = err
    yaw = max(-math.pi, min(math.pi, -kp * u))
    pitch = max(-1.05, min(2.09, base - kp * v))
    return pitch, roll, yaw


class GimbalControl(Node):
    """Keeps the gimbal camera on the detected person as the drone tilts and rolls to follow it,
    and publishes the camera orientation (/xtdrone/<v>/cam_pose).

    Holds the --pitch angle while no person is in view; when one is, gimbal_focus() turns the
    joints to keep its bbox centered. ROS 1 used MAVROS mount control and read the camera link
    from Gazebo; here the gimbal joints are commanded directly (/<v>/gimbal/*_cmd) and the camera
    orientation is body attitude x the live gimbal pitch."""

    def __init__(self, vtype, vid, pitch_deg=-60.0, kp=0.002):
        n = f"{vtype}_{vid}"
        super().__init__(f"gimbal_control_{n}")
        # XTDrone's pitch -60 = 60 deg down. The cgo3 camera joint turns about -y, so a positive
        # angle tilts the camera up and the joint takes minus the downward angle (limits -1.05 .. 2.09)
        self.base = max(-1.05, min(2.09, math.radians(pitch_deg)))
        self.kp = kp
        self.pitch, self.roll, self.yaw = self.base, 0.0, 0.0
        self.err, self.last_seen = None, None
        self.pitch_pub = self.create_publisher(Float64, f"/{n}/gimbal/pitch_cmd", 1)
        self.roll_pub = self.create_publisher(Float64, f"/{n}/gimbal/roll_cmd", 1)
        self.yaw_pub = self.create_publisher(Float64, f"/{n}/gimbal/yaw_cmd", 1)
        self.cam_pub = self.create_publisher(PoseStamped, f"/xtdrone/{n}/cam_pose", 1)
        self.pose = None
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, f"/{n}/detections", self.detections, 1)
        self.create_timer(1.0 / 30.0, self.loop)
        print(f"{n}: Gimbal control", flush=True)

    def detections(self, msg):
        for d in msg.detections:
            if d.results and d.results[0].hypothesis.class_id == "person":
                self.err = (d.bbox.center.position.x - 320.0, d.bbox.center.position.y - 180.0)
                self.last_seen = self.get_clock().now()
                break

    def loop(self):
        body_roll = 0.0
        if self.pose is not None:
            o = self.pose.pose.orientation
            body_roll = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x * o.x + o.y * o.y))
        if self.last_seen is not None and (self.get_clock().now() - self.last_seen).nanoseconds < 1e9:
            err = self.err
        else:
            err = None
        self.pitch, self.roll, self.yaw = gimbal_focus(err, self.base, body_roll, self.kp)
        self.pitch_pub.publish(Float64(data=self.pitch))
        self.roll_pub.publish(Float64(data=self.roll))
        self.yaw_pub.publish(Float64(data=self.yaw))
        if self.pose is None:
            return
        o = self.pose.pose.orientation
        d = -self.pitch  # pitch down about +y (FLU)
        g = (0.0, math.sin(d / 2), 0.0, math.cos(d / 2))
        q = quat_mul((o.x, o.y, o.z, o.w), g)
        cam = PoseStamped()
        cam.header.stamp = self.get_clock().now().to_msg()
        cam.header.frame_id = "map"
        cam.pose.position = self.pose.pose.position
        cam.pose.orientation.x, cam.pose.orientation.y, cam.pose.orientation.z, cam.pose.orientation.w = q
        self.cam_pub.publish(cam)


class HumanTracking(Node):
    """Image-based visual servoing towards the first "person" detection (yolo_human_tracking.py)."""

    def __init__(self, vtype, vid, width=640, height=360, fx=205.46963709898583, fy=205.46963709898583):
        n = f"{vtype}_{vid}"
        super().__init__(f"yolo_human_tracking_{n}")
        self.u_center, self.v_center, self.fx, self.fy = width / 2, height / 2, fx, fy
        self.Kp_xy, self.Kp_z = 0.5, 1.0
        self.height = None
        self.target_height = None
        self.theta = math.radians(60)
        self.twist = Twist()
        self.cmd = ""
        self.last_seen = None
        self.create_subscription(Detection2DArray, f"/{n}/detections", self.detections, 1)
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self.local_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f"/xtdrone/{n}/cam_pose", self.cam_pose, 1)
        self.vel_pub = self.create_publisher(Twist, f"/xtdrone/{n}/cmd_vel_flu", 1)
        self.cmd_pub = self.create_publisher(String, f"/xtdrone/{n}/cmd", 1)
        self.create_timer(1.0 / 60.0, self.loop)

    def local_pose(self, m):
        self.height = m.pose.position.z
        if self.target_height is None:
            self.target_height = self.height

    def cam_pose(self, m):
        o = m.pose.orientation
        self.theta = pitch_of((o.x, o.y, o.z, o.w))

    def detections(self, msg):
        for d in msg.detections:
            if not d.results or d.results[0].hypothesis.class_id != "person" or self.height is None:
                continue
            theta = max(self.theta, 0.1)
            z = self.height / math.sin(theta)
            u_ = d.bbox.center.position.x - self.u_center
            v_ = d.bbox.center.position.y - self.v_center
            u_vel, v_vel = -self.Kp_xy * u_, -self.Kp_xy * v_
            x_vel = v_vel * z / (v_ * math.cos(theta) + self.fy * math.sin(theta))
            y_vel = (z * u_vel - u_ * math.cos(theta) * x_vel) / self.fx
            self.twist = Twist()
            self.twist.linear.x, self.twist.linear.y = x_vel, y_vel
            self.twist.linear.z = self.Kp_z * (self.target_height - self.height)
            self.cmd = ""
            self.last_seen = self.get_clock().now()
            print("find human", flush=True)
            break

    def loop(self):
        if self.last_seen is not None and (self.get_clock().now() - self.last_seen).nanoseconds > 2e9:
            self.twist, self.cmd, self.last_seen = Twist(), "HOVER", None
            print("HOVER", flush=True)
        self.vel_pub.publish(self.twist)
        self.cmd_pub.publish(String(data=self.cmd))
        self.cmd = ""


def _args(argv, prog, extra=None):
    ap = argparse.ArgumentParser(prog=prog)
    ap.add_argument("vehicle_type")
    ap.add_argument("vehicle_id", type=int)
    if extra:
        extra(ap)
    return ap.parse_args(argv)


def detection_main():
    def make(argv):
        a = _args(argv, "yolo_detection", lambda ap: (ap.add_argument("--cfg"), ap.add_argument("--weights"),
                                                     ap.add_argument("--names"), ap.add_argument("--rate", type=float,
                                                                                                 default=5.0),
                                                     ap.add_argument("--size", default="960x544",
                                                                     help="network input WxH, multiples of 32")))
        w, h = (int(v) for v in a.size.lower().split("x"))
        return YoloDetection(a.vehicle_type, a.vehicle_id, a.cfg, a.weights, a.names, a.rate, (w, h))
    run(make)


def gimbal_main():
    def make(argv):
        a = _args(argv, "gimbal_control", lambda ap: (ap.add_argument("--pitch", type=float, default=-60.0),
                                                      ap.add_argument("--kp", type=float, default=0.002,
                                                                      help="rad of joint per px of image error")))
        return GimbalControl(a.vehicle_type, a.vehicle_id, a.pitch, a.kp)
    run(make)


def tracking_main():
    def make(argv):
        a = _args(argv, "yolo_human_tracking")
        return HumanTracking(a.vehicle_type, a.vehicle_id)
    run(make)
