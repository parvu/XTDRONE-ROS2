"""Lane detection and lane keeping (chapter 30), ports of sensing/lane_detection/lane_detection_{rover,ugv}.py
and control/{rover,ugv}_self_driving.py.

  ros2 run xtdrone_py lane_detection rover 0      # /rover_0/front_cam/left/image_raw
  ros2 run xtdrone_py self_driving rover 0        # -> /xtdrone/rover_0/cmd_vel_flu
  ros2 run xtdrone_py lane_detection ugv 0        # /ugv_0/triclops/triclops/left/image
  ros2 run xtdrone_py self_driving ugv 0          # -> /ugv_0/cmd_vel

lane_detection keeps the yellow pixels (HSV threshold), finds line segments (Canny + Hough),
fits one line to the segments with negative and one to those with positive slope, and publishes
the horizontal offset of the lane centre from the image centre on /<v>/lane_mid_error (Int16,
1000 = fewer than two lines found) and the annotated image on /<v>/image_lane.
self_driving drives at Vx and steers with -Kp * error; it stops when the error is 1000.
"""
import argparse

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int16

from xtdrone_py.common import run

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

NO_LANE = 1000

# per vehicle family: camera topic, HSV range of the yellow lines, part of the image used (rows), Kp
PROFILES = {
    # ROS 1 rover: front stereo camera, rows 0 .. 3/5 of the image
    "rover": {"image": "/{n}/front_cam/left/image_raw", "lower": (30, 150, 101), "upper": (100, 255, 255),
              "rows": (0.0, 0.6), "kp": 0.003, "cmd": "/xtdrone/{n}/cmd_vel_flu"},
    # ROS 1 ugv used a realsense camera that the catvehicle model does not have; the triclops
    # front camera looks ahead the same way. Lower half of the image.
    "ugv": {"image": "/{n}/triclops/triclops/left/image", "lower": (26, 43, 46), "upper": (30, 255, 255),
            "rows": (0.5, 1.0), "kp": 0.002, "cmd": "/{n}/cmd_vel"},
}


def lane_error(bgr, lower, upper, rows=(0.5, 1.0)):
    """(error in pixels or NO_LANE, annotated image) for a BGR image."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.uint8(lower), np.uint8(upper))
    yellow = cv2.bitwise_and(bgr, bgr, mask=mask)
    height, width = yellow.shape[:2]
    edges = cv2.Canny(cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY), 100, 200)
    lines = cv2.HoughLinesP(edges, rho=6, theta=np.pi / 60, threshold=200, lines=np.array([]),
                            minLineLength=50, maxLineGap=50)
    if lines is None:
        return NO_LANE, yellow
    left_x, left_y, right_x, right_y = [], [], [], []
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        if x2 == x1:
            continue
        slope = float(y2 - y1) / float(x2 - x1)
        if slope <= 0:
            left_x += [x1, x2]
            left_y += [y1, y2]
        else:
            right_x += [x1, x2]
            right_y += [y1, y2]
    if not left_x or not right_x:
        return NO_LANE, yellow
    min_y, max_y = int(height * rows[0]), int(height * rows[1])
    left = np.poly1d(np.polyfit(left_y, left_x, deg=1))
    right = np.poly1d(np.polyfit(right_y, right_x, deg=1))
    error = (left(min_y) + right(min_y)) / 2.0 - width / 2.0
    out = yellow.copy()
    for poly in (left, right):
        cv2.line(out, (int(poly(max_y)), max_y), (int(poly(min_y)), min_y), (0, 0, 255), 5)
    return int(max(-999, min(999, error))), out


def steer(error, kp, vx=5.0):
    """(speed, steering) from the lane error, as rover/ugv_self_driving.py."""
    if error == NO_LANE:
        return 0.0, 0.0
    angle = -kp * error if abs(error) > 20 else 0.0
    return vx * (1 - abs(angle)), angle


class LaneDetection(Node):

    def __init__(self, family, vid):
        n = f"{family}_{vid}"
        super().__init__(f"lane_detection_{n}")
        self.p = PROFILES[family]
        self.error_pub = self.create_publisher(Int16, f"/{n}/lane_mid_error", 1)
        self.image_pub = self.create_publisher(Image, f"/{n}/image_lane", 1)
        self.error = NO_LANE
        self.create_subscription(Image, self.p["image"].format(n=n), self.image_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / 50.0, lambda: self.error_pub.publish(Int16(data=self.error)))

    def image_callback(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        bgr = a[:, :, ::-1].copy() if msg.encoding == "rgb8" else a.copy()
        self.error, out = lane_error(bgr, self.p["lower"], self.p["upper"], self.p["rows"])
        img = Image(header=msg.header, height=out.shape[0], width=out.shape[1], encoding="bgr8",
                    step=out.shape[1] * 3, data=out.tobytes())
        self.image_pub.publish(img)


class SelfDriving(Node):

    def __init__(self, family, vid, vx):
        n = f"{family}_{vid}"
        super().__init__(f"{family}_self_driving_{vid}")
        self.p = PROFILES[family]
        self.vx = vx
        self.twist = Twist()
        self.pub = self.create_publisher(Twist, self.p["cmd"].format(n=n), 1)
        self.create_subscription(Int16, f"/{n}/lane_mid_error", self.error_callback, 1)
        self.create_timer(1.0 / 50.0, lambda: self.pub.publish(self.twist))

    def error_callback(self, msg):
        self.twist.linear.x, self.twist.angular.z = steer(msg.data, self.p["kp"], self.vx)


def _args(argv, prog):
    ap = argparse.ArgumentParser(prog=prog)
    ap.add_argument("family", choices=sorted(PROFILES))
    ap.add_argument("id", type=int)
    ap.add_argument("--speed", type=float, default=5.0)
    return ap.parse_args(argv)


def detection_main():
    def make(argv):
        a = _args(argv, "lane_detection")
        return LaneDetection(a.family, a.id)
    run(make)


def driving_main():
    def make(argv):
        a = _args(argv, "self_driving")
        return SelfDriving(a.family, a.id, a.speed)
    run(make)
