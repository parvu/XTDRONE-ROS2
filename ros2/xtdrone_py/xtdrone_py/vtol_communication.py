"""XTDrone VTOL communication (port of communication/vtol_communication.py) for ArduPlane QuadPlanes.

  ros2 run xtdrone_py vtol_communication standard_vtol 0

ArduPlane ignores position and velocity setpoints from a companion computer (SET_POSITION_TARGET_LOCAL_NED only
changes the altitude) and its GUIDED mode flies to a point as a fixed-wing plane. The VTOL therefore hovers in QLOITER
and is flown with RC overrides: the sticks are the velocity (pitch, roll: Q_LOIT_SPEED_MS, throttle: climb rate, yaw:
yaw rate); a position command (cmd_pose_*) is a proportional controller on that velocity.
'multirotor' / 'plane' on /xtdrone/<type>_<id>/cmd switch the flight state: in 'plane' state the vehicle is in
GUIDED fixed-wing flight and the position commands are the points it loiters around (MAV_CMD_DO_REPOSITION).
"""
import math

from mavros_msgs.msg import OverrideRCIn

from xtdrone_py.common import run
from xtdrone_py.multirotor_communication import MultirotorCommunication

# stick full scale (xtdrone-vtol.parm sets Q_LOIT_SPEED_MS, Q_PILOT_SPD_UP/DN, Q_YAW_RATE_MAX)
MAX_HORIZONTAL, MAX_CLIMB, MAX_YAW_RATE = 5.0, 2.5, math.radians(75.0)
POSITION_KP = 0.8
NO_CHANGE = 0  # OverrideRCIn: leave the channel alone


def stick(value, full_scale):
    """PWM of a stick that stands for value (full_scale at the ends)."""
    return int(max(1000, min(2000, 1500 + 500.0 * value / full_scale)))


class VtolCommunication(MultirotorCommunication):

    def __init__(self, vehicle_type, vehicle_id):
        super().__init__(vehicle_type, vehicle_id)
        self.transition_state = "multirotor"
        self.velocity = (0.0, 0.0, 0.0, 0.0)  # ENU velocity and yaw rate
        self.target_position = None           # position command: (x, y, z) in the local ENU frame
        self.rc_pub = self.create_publisher(OverrideRCIn, f"/{self.name}/mavros/rc/override", 1)
        self.create_timer(0.05, self.control)

    # ---- commands --------------------------------------------------------------------------------------------
    def set_velocity(self, vx, vy, vz, w, body):
        self.target_position = None
        if body:
            c, s = math.cos(self.current_yaw), math.sin(self.current_yaw)
            vx, vy = c * vx - s * vy, s * vx + c * vy
        self.velocity = (vx, vy, vz, w)
        self.hover_flag = 0

    def cmd_vel_flu_callback(self, msg):
        self.set_velocity(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, True)

    def cmd_vel_enu_callback(self, msg):
        self.set_velocity(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z, False)

    def cmd_accel_flu_callback(self, msg):
        pass  # not supported by ArduPlane

    cmd_accel_enu_callback = cmd_accel_flu_callback

    def set_position(self, x, y, z, flu):
        if flu and self.current_position is not None:
            c, s = math.cos(self.current_yaw), math.sin(self.current_yaw)
            p = self.current_position
            x, y, z = p.x + c * x - s * y, p.y + s * x + c * y, p.z + z
        self.target_position = (x, y, z)
        self.hover_flag = 0

    def cmd_pose_flu_callback(self, msg):
        self.set_position(msg.position.x, msg.position.y, msg.position.z, True)

    def cmd_pose_enu_callback(self, msg):
        self.set_position(msg.position.x, msg.position.y, msg.position.z, False)

    def hover(self):
        self.target_position = None
        self.velocity = (0.0, 0.0, 0.0, 0.0)

    def cmd_callback(self, msg):
        data = msg.data
        if data in ("multirotor", "plane"):
            self.transition_state = data
            self.log(data)
            self.set_mode("OFFBOARD" if data == "multirotor" else "GUIDED")
            self.last_cmd = data
        elif data in ("loiter", "idle"):
            self.log(data)
        else:
            super().cmd_callback(msg)

    # ---- output ----------------------------------------------------------------------------------------------
    def check_takeoff(self):
        """No GUIDED takeoff: the vehicle lifts off on the throttle stick."""
        if self.arm_pending and self.settled():
            self.arm_pending = False
            self.arm(True)

    def publish_target(self):
        """Replaced by control()."""

    def control(self):
        if not self.armed or self.current_position is None:
            return
        if self.transition_state == "plane":
            self.reposition_tick()
            return
        vx, vy, vz, w = self.velocity
        if self.target_position is not None:
            p = self.current_position
            vx = self.limit(POSITION_KP * (self.target_position[0] - p.x), MAX_HORIZONTAL)
            vy = self.limit(POSITION_KP * (self.target_position[1] - p.y), MAX_HORIZONTAL)
            vz = self.limit(POSITION_KP * (self.target_position[2] - p.z), MAX_CLIMB)
        c, s = math.cos(self.current_yaw), math.sin(self.current_yaw)
        forward, left = c * vx + s * vy, -s * vx + c * vy
        msg = OverrideRCIn()
        msg.channels = [NO_CHANGE] * 18
        msg.channels[0] = stick(-left, MAX_HORIZONTAL)     # roll: right stick = right
        msg.channels[1] = stick(-forward, MAX_HORIZONTAL)  # pitch: forward stick = low PWM
        msg.channels[2] = stick(vz, MAX_CLIMB)             # throttle: climb rate
        msg.channels[3] = stick(-w, MAX_YAW_RATE)          # yaw: right stick = clockwise
        self.rc_pub.publish(msg)

    @staticmethod
    def limit(value, bound):
        return max(-bound, min(bound, value))

    def reposition_tick(self):
        if self.target_position is None:
            return
        self._tick = getattr(self, "_tick", 0) + 1
        if self._tick % 5:  # 4 Hz
            return
        x, y, z = self.target_position
        self.reposition(x, y, max(z - (self.ground_z or 0.0), 1.0))


def main():
    def make(argv):
        if len(argv) < 2:
            raise SystemExit("usage: vtol_communication <vehicle_type> <vehicle_id>   e.g. standard_vtol 0")
        return VtolCommunication(argv[0], argv[1])
    run(make)
