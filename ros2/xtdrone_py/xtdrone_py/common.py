"""Shared helpers for the XTDrone ROS 2 nodes (rclpy ports of the ROS 1 scripts)."""
import math
import sys

import rclpy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# MAVROS 2 publishes state/pose topics with the sensor-data QoS (best effort).
MAVROS_QOS = qos_profile_sensor_data

MASK_POSITION = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
                 | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW_RATE)
MASK_VELOCITY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                 | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW)
MASK_ACCEL = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
              | PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
              | PositionTarget.IGNORE_YAW)
MASK_VELOCITY_YAW = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                     | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                     | PositionTarget.IGNORE_YAW_RATE)
MOTION_MASKS = {0: MASK_POSITION, 1: MASK_VELOCITY, 2: MASK_ACCEL, 3: MASK_VELOCITY_YAW}

FRAME_LOCAL_NED = PositionTarget.FRAME_LOCAL_NED          # 1 (MAVROS: ENU <-> NED handled)
FRAME_BODY_NED = PositionTarget.FRAME_BODY_NED            # 8
FRAME_BODY_OFFSET_NED = PositionTarget.FRAME_BODY_OFFSET_NED  # 9


# XTDrone keeps PX4's mode names on its topics and keyboards; these are the ArduPilot (Copter) modes behind them.
AP_MODES = {
    "OFFBOARD": "GUIDED",
    "AUTO.TAKEOFF": "GUIDED",   # the communication node sends the takeoff command after arming
    "AUTO.LAND": "LAND",
    "AUTO.RTL": "RTL",
    "AUTO.LOITER": "LOITER",
    "AUTO.MISSION": "AUTO",
    "POSCTL": "LOITER",
    "ALTCTL": "ALT_HOLD",
    "STABILIZED": "STABILIZE",
    "MANUAL": "STABILIZE",
}


def ap_mode(name):
    """ArduPilot mode for an XTDrone (PX4-style) mode name; ArduPilot names pass through."""
    return AP_MODES.get(name, name)


def q2yaw(q):
    """Yaw (rad) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def q2rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch, q2yaw(q)


def rpy2q(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class VehicleNode(Node):
    """Node bound to one XTDrone vehicle <type>_<id> with MAVROS arming/mode clients."""

    def __init__(self, node_suffix, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = str(vehicle_id)
        self.name = f"{vehicle_type}_{self.vehicle_id}"
        super().__init__(f"{self.name}_{node_suffix}")
        self.arm_client = self.create_client(CommandBool, f"/{self.name}/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"/{self.name}/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{self.name}/mavros/cmd/takeoff")

    def log(self, text):
        print(f"{self.name}: {text}", flush=True)

    def call(self, client, request, ok_text, fail_text, check=None):
        """Asynchronous service call that reports the outcome like the ROS 1 scripts."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.log(f"{fail_text} ({client.srv_name} not available)")
            return

        def done(future):
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001 - report any transport error
                self.log(f"{fail_text} ({exc})")
                return
            ok = check(result) if check else True
            self.log(ok_text if ok else fail_text)

        client.call_async(request).add_done_callback(done)

    def arm(self, value=True):
        req = CommandBool.Request()
        req.value = value
        word = "Armed" if value else "Disarmed"
        self.call(self.arm_client, req, f"{word} True", f"{word} False (failed)",
                  check=lambda r: r.success)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = ap_mode(mode)
        self.call(self.mode_client, req, mode, mode + " failed", check=lambda r: r.mode_sent)

    def takeoff(self, altitude):
        """ArduPilot GUIDED does not leave the ground on setpoints: it needs the takeoff command first."""
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        self.call(self.takeoff_client, req, f"takeoff to {altitude:g} m", "takeoff failed", check=lambda r: r.success)


def run(node_factory, args=None):
    """rclpy main loop with clean Ctrl+C handling."""
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory(rclpy.utilities.remove_ros_args(sys.argv)[1:])
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
