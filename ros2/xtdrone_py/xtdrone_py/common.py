"""Shared helpers for the XTDrone ROS 2 nodes (rclpy ports of the ROS 1 scripts)."""
import math
import sys

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandInt, CommandTOL, MessageInterval, SetMode
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


# XTDrone keeps PX4's mode names on its topics and keyboards; these are the ArduPilot modes behind them.
AP_MODES = {
    "OFFBOARD": "GUIDED",
    "AUTO.TAKEOFF": "GUIDED",   # copter: the communication node sends the takeoff command after arming
    "AUTO.LAND": "LAND",
    "AUTO.RTL": "RTL",
    "AUTO.LOITER": "LOITER",
    "AUTO.MISSION": "AUTO",
    "POSCTL": "LOITER",
    "ALTCTL": "ALT_HOLD",
    "STABILIZED": "STABILIZE",
    "MANUAL": "STABILIZE",
}
# ArduPlane: MAVROS 2 has no names for TAKEOFF (13), LOITER (12) or RTL (11); it takes the mode number as a string
AP_MODES_PLANE = {**AP_MODES, "AUTO.TAKEOFF": "13", "AUTO.LAND": "11", "AUTO.RTL": "11", "AUTO.LOITER": "12",
                  "POSCTL": "FBWA", "ALTCTL": "FBWB", "STABILIZED": "FBWA", "MANUAL": "MANUAL"}
AP_MODES_ROVER = {**AP_MODES, "AUTO.TAKEOFF": "GUIDED", "AUTO.LAND": "HOLD", "AUTO.LOITER": "HOLD",
                  "POSCTL": "HOLD", "ALTCTL": "HOLD", "STABILIZED": "MANUAL", "MANUAL": "MANUAL"}


# ArduPlane QuadPlane (standard_vtol, tailsitter): GUIDED with Q_GUIDED_MODE flies like a multicopter; QLAND is mode 20,
# QRTL 21, QHOVER 18, QLOITER 19, QSTABILIZE 17 (MAVROS 2 knows them by number only)
AP_MODES_VTOL = {**AP_MODES, "OFFBOARD": "19", "AUTO.TAKEOFF": "19", "AUTO.LAND": "20", "AUTO.RTL": "21", "POSCTL": "19", "ALTCTL": "18",
                 "STABILIZED": "17", "MANUAL": "17"}


def ap_mode(name, vehicle_type="iris"):
    """ArduPilot mode for an XTDrone (PX4-style) mode name; ArduPilot names pass through."""
    modes = (AP_MODES_PLANE if vehicle_type == "plane" else AP_MODES_ROVER if vehicle_type == "rover"
             else AP_MODES_VTOL if vehicle_type in ("standard_vtol", "tailsitter") else AP_MODES)
    return modes.get(name, name)


# ArduPilot sends only the MAVLink messages it is asked for, and MAVROS does not ask. Message id -> rate (Hz).
AP_MESSAGE_INTERVALS = {
    1: 2,      # SYS_STATUS
    24: 2,     # GPS_RAW_INT
    27: 5,     # RAW_IMU
    29: 2,     # SCALED_PRESSURE
    31: 20,    # ATTITUDE_QUATERNION
    32: 20,    # LOCAL_POSITION_NED
    33: 5,     # GLOBAL_POSITION_INT
    74: 5,     # VFR_HUD
    242: 0.5,  # HOME_POSITION
}


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

    def __init__(self, node_suffix, vehicle_type, vehicle_id, telemetry=False):
        self.vehicle_type = vehicle_type
        self.vehicle_id = str(vehicle_id)
        self.name = f"{vehicle_type}_{self.vehicle_id}"
        super().__init__(f"{self.name}_{node_suffix}")
        self.arm_client = self.create_client(CommandBool, f"/{self.name}/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, f"/{self.name}/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{self.name}/mavros/cmd/takeoff")
        if telemetry:
            self._start_telemetry()

    def _start_telemetry(self):
        """Ask the autopilot for the MAVLink streams (each time the vehicle connects, and again until the local
        position arrives: a request is lost while the autopilot or MAVROS is still starting, e.g. with many vehicles
        at once) and publish the EKF origin (global_position/set_gp_origin), which the vision EKF presets need before
        the autopilot arms. With GPS the origin is already set and ArduPilot ignores it."""
        self._tm_origin = GeoPointStamped()
        p = self._tm_origin.position
        p.latitude = self.declare_parameter("origin_lat", 47.397742).value
        p.longitude = self.declare_parameter("origin_lon", 8.545594).value
        p.altitude = self.declare_parameter("origin_alt", 488.0).value
        # where the local ENU frame starts in the world: at the spawn point with GPS (ArduPilot's EKF origin is the
        # first fix), at the world origin with a vision EKF
        self._spawn = (self.declare_parameter("spawn_x", 0.0).value, self.declare_parameter("spawn_y", 0.0).value)
        self._reposition_client = self.create_client(CommandInt, f"/{self.name}/mavros/cmd/command_int")
        n = self.name
        self._tm_client = self.create_client(MessageInterval, f"/{n}/mavros/set_message_interval")
        self._tm_origin_pub = self.create_publisher(GeoPointStamped, f"/{n}/mavros/global_position/set_gp_origin", 1)
        self.create_subscription(State, f"/{n}/mavros/state", self._tm_state, MAVROS_QOS)
        self.create_subscription(PoseStamped, f"/{n}/mavros/local_position/pose", self._tm_pose, MAVROS_QOS)
        self._tm_connected = False
        self._tm_pose_seen = False
        self._tm_since = None
        self._tm_pending = []
        self._tm_origin_at = self.get_clock().now()
        self.create_timer(0.25, self._tm_step)

    def _tm_state(self, msg):
        if msg.connected and not self._tm_connected:
            self._tm_pending = list(AP_MESSAGE_INTERVALS.items())
            self._tm_since = self.get_clock().now()
            self.log("connected, requesting the MAVLink streams")
        self._tm_connected = msg.connected

    def _tm_pose(self, msg):
        self._tm_pose_seen = True

    def _tm_step(self):
        if not self._tm_connected:
            return
        now = self.get_clock().now()
        if not self._tm_pending and not self._tm_pose_seen and (now - self._tm_since).nanoseconds > 5e9:
            self._tm_pending = list(AP_MESSAGE_INTERVALS.items())
            self._tm_since = now
        if self._tm_pending and self._tm_client.service_is_ready():
            mid, hz = self._tm_pending.pop(0)
            req = MessageInterval.Request()
            req.message_id, req.message_rate = mid, float(hz)
            self._tm_client.call_async(req)
        # the EKF origin, once a second until the local position streams (the EKF may need a while to start)
        if not self._tm_pose_seen and (now - self._tm_origin_at).nanoseconds > 1e9:
            self._tm_origin.header.stamp = now.to_msg()
            self._tm_origin_pub.publish(self._tm_origin)
            self._tm_origin_at = now

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
        req.custom_mode = ap_mode(mode, self.vehicle_type)
        self.call(self.mode_client, req, mode, mode + " failed", check=lambda r: r.mode_sent)

    def reposition(self, x, y, height, speed=-1.0):
        """MAV_CMD_DO_REPOSITION (GUIDED): ArduPlane ignores position setpoints. x, y: local ENU frame, height above the
        takeoff point. The GPS positions are world coordinates around the SITL home (origin_lat, origin_lon)."""
        if not self._reposition_client.service_is_ready():
            return
        lat0, lon0 = self._tm_origin.position.latitude, self._tm_origin.position.longitude
        req = CommandInt.Request()
        req.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        req.command = 192  # MAV_CMD_DO_REPOSITION
        req.param1 = float(speed)
        req.param2 = 1.0  # change mode to GUIDED
        req.x = int((lat0 + (y + self._spawn[1]) / 111320.0) * 1e7)
        req.y = int((lon0 + (x + self._spawn[0]) / (111320.0 * math.cos(math.radians(lat0)))) * 1e7)
        req.z = float(height)
        self._reposition_client.call_async(req)

    def takeoff(self, altitude):
        """ArduPilot GUIDED does not leave the ground on setpoints: it needs the takeoff command first."""
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.log("takeoff failed (service not available)")
            return

        def done(future):
            r = future.result()
            self.log(f"takeoff to {altitude:g} m" if r.success else f"takeoff failed (result {r.result})")
        self.takeoff_client.call_async(req).add_done_callback(done)


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
