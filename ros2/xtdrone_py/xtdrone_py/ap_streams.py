"""ArduPilot telemetry setup for one MAVROS vehicle.

  ros2 run xtdrone_py ap_streams iris 0

ArduPilot sends only the MAVLink messages it is asked for, and MAVROS does not ask: without this node the MAVROS
topics (local_position/pose, imu/data, ...) stay silent. It sets the message intervals each time the vehicle
connects, and publishes the EKF origin (global_position/set_gp_origin), which the vision EKF presets need before
the autopilot arms (with GPS it is already set and ArduPilot ignores it).
"""
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval

from xtdrone_py.common import MAVROS_QOS, VehicleNode, run

# MAVLink message id -> rate (Hz)
INTERVALS = {
    1: 5,      # SYS_STATUS
    24: 5,     # GPS_RAW_INT
    27: 10,    # RAW_IMU
    29: 5,     # SCALED_PRESSURE
    30: 30,    # ATTITUDE
    31: 30,    # ATTITUDE_QUATERNION
    32: 30,    # LOCAL_POSITION_NED
    33: 10,    # GLOBAL_POSITION_INT
    74: 10,    # VFR_HUD
    116: 10,   # SCALED_IMU2
    65: 2,     # RC_CHANNELS
    242: 0.5,  # HOME_POSITION
}


class APStreams(VehicleNode):

    def __init__(self, vehicle_type, vehicle_id):
        super().__init__("ap_streams", vehicle_type, vehicle_id)
        lat = self.declare_parameter("origin_lat", 47.397742).value
        lon = self.declare_parameter("origin_lon", 8.545594).value
        alt = self.declare_parameter("origin_alt", 488.0).value
        self.origin = GeoPointStamped()
        self.origin.position.latitude, self.origin.position.longitude, self.origin.position.altitude = lat, lon, alt
        n = self.name
        self.interval_client = self.create_client(MessageInterval, f"/{n}/mavros/set_message_interval")
        self.origin_pub = self.create_publisher(GeoPointStamped, f"/{n}/mavros/global_position/set_gp_origin", 1)
        self.create_subscription(State, f"/{n}/mavros/state", self.state_callback, MAVROS_QOS)
        self.connected = False
        self.pending = list(INTERVALS.items())
        self.create_timer(0.25, self.step)
        self.origin_sent = 0

    def state_callback(self, msg):
        if msg.connected and not self.connected:
            self.pending = list(INTERVALS.items())
            self.origin_sent = 0
            self.log("connected, requesting the MAVLink streams")
        self.connected = msg.connected

    def step(self):
        if not self.connected:
            return
        if self.pending and self.interval_client.service_is_ready():
            mid, hz = self.pending.pop(0)
            req = MessageInterval.Request()
            req.message_id, req.message_rate = mid, float(hz)
            self.interval_client.call_async(req)
        if self.origin_sent < 20:  # a few times: the EKF may not be up yet
            self.origin.header.stamp = self.get_clock().now().to_msg()
            self.origin_pub.publish(self.origin)
            self.origin_sent += 1


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: ap_streams <vehicle_type> <vehicle_id>")
        return APStreams(argv[0], argv[1])
    run(make)
