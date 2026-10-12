"""Formation control with a communication model (chapter 27).

Port of coordination/formation_communication_demo/control/script:
communication_verify_1.py (the SINR channel model) and sim_addleader_1.py (one consensus
controller per UAV, started with mpiexec in ROS 1, here one node per UAV). A small
aggregator replaces the UAV-state collection that the modified keyboard / xtdrone_qt did.

  ros2 launch xtdrone_formation_demo formation_communication.launch.py
"""
import argparse
import copy
import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from xtdrone_msgs.msg import AllUAVData, CommVerify, UavComm, UAVDataMsg

# Spawn positions of the 20 iris (iris20_exam1.launch / uav_bias of sim_addleader_1.py).
# Each UAV adds its spawn position to its local pose to get the shared "global" pose.
UAV_POSITIONS = [(0, 0), (1.2, -1.2), (-1.2, -1.2), (1.2, 1.2), (-1.2, 1.2), (2.4, -2.4), (0, -2.4),
                 (-2.4, -2.4), (2.4, 0), (-2.4, 0), (2.4, 2.4), (0, 2.4), (-2.4, 2.4), (-7, -7), (-6, -7),
                 (-5, -7), (-4, -7), (3, 6), (4, 6), (5, 6)]
FORMATION_KEYS = ["waiting", "FORM_1", "FORM_2", "FORM_3", "AUTO.TAKEOFF", "OFFBOARD", "HOVER", "AUTO.LAND"]
FRAME_LOCAL_NED = 1


def spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


def link_matrix(positions, leaders, rate=15.0, bandwidth=1e6, noise=1e-9, power=0.5):
    """trans[i][k] = 1 when UAV i receives leader k (index into leaders) above the needed rate.

    Channel gain h = 1 / (1 + d^2); SINR = P h_k / (N + P sum_{other leaders} h);
    capacity B log2(1 + SINR) must exceed R = 128 bytes * n UAVs * rate. UAV 0 receives nothing.
    """
    n = len(positions)
    required = 128 * n * rate
    trans = [[] for _ in range(n)]
    for i in range(1, n):
        gains = [0.0 if i == k else 1.0 / (1.0 + float(np.sum((positions[i] - positions[k]) ** 2)))
                 for k in leaders]
        total = sum(gains)
        for g in gains:
            sinr = power * g / (noise + power * (total - g))
            trans[i].append(1 if bandwidth * math.log2(1 + sinr) > required else 0)
    return trans


class CommunicationVerify(Node):
    """communication_verify_1.py: publishes /communication_verify at 15 Hz."""

    def __init__(self, uav_num, leaders):
        super().__init__("communication_verify")
        self.uav_num = uav_num
        self.leaders = leaders
        self.positions = None
        self.count = 0
        self.create_subscription(AllUAVData, "/xtdrone_gcs/all_uav_data", self.all_uav_data_callback, 10)
        self.pub = self.create_publisher(CommVerify, "/communication_verify", 10)
        self.create_timer(1.0 / 15.0, self.timer_callback)

    def all_uav_data_callback(self, msg):
        if len(msg.data) >= self.uav_num:
            self.positions = [np.array([d.pose.position.x, d.pose.position.y, d.pose.position.z])
                              for d in msg.data[:self.uav_num]]

    def timer_callback(self):
        if self.positions is None:
            return
        trans = link_matrix(self.positions, self.leaders)
        msg = CommVerify()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.leader = self.leaders
        msg.trans_matrix = [UavComm(trans_matrix=row) for row in trans]
        self.pub.publish(msg)
        self.count += 1
        if self.count % 30 == 0 and self.uav_num > 1:
            self.get_logger().info(f"UAV 1 receives leaders: {trans[1]}")


class UAVDataAggregator(Node):
    """Collects /<type>_<i>/formation/uav_data into /xtdrone_gcs/all_uav_data (15 Hz)."""

    def __init__(self, vehicle_type, uav_num):
        super().__init__("uav_data_aggregator")
        self.all = AllUAVData()
        self.all.data = [UAVDataMsg(uav_id=i) for i in range(uav_num)]
        for i in range(uav_num):
            self.create_subscription(UAVDataMsg, f"/{vehicle_type}_{i}/formation/uav_data", self.uav_callback, 10)
        self.pub = self.create_publisher(AllUAVData, "/xtdrone_gcs/all_uav_data", 10)
        self.create_timer(1.0 / 15.0, self.timer_callback)

    def uav_callback(self, msg):
        if 0 <= msg.uav_id < len(self.all.data):
            self.all.data[msg.uav_id] = msg

    def timer_callback(self):
        self.all.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.all)


def limit(value, maximum):
    return max(-maximum, min(maximum, value))


class FormationController(Node):
    """sim_addleader_1.py for one UAV: takeoff, hover, FORM_1/FORM_2 consensus, land."""

    def __init__(self, vehicle_type, uav_id, uav_num, pos_file):
        super().__init__(f"{vehicle_type}_{uav_id}_formation_comm")
        self.uav_id = uav_id
        self.uav_num = uav_num
        self.ns = f"/{vehicle_type}_{uav_id}"
        self.bias = UAV_POSITIONS[uav_id] if uav_id < len(UAV_POSITIONS) else (0.0, 0.0)
        self.desired = np.loadtxt(pos_file)
        self.takeoff_height = 0.6
        self.Kp = 0.5
        self.Kpangz = 1.0
        self.velxy_max = 1.5
        self.velz_max = 1.5
        self.velangz_max = 0.5
        self.gamma = 1.0
        self.target_yaw = 0.0

        self.local_pose = None
        self.velocity = TwistStamped()
        self.heading = None
        self.armed = False
        self.mode = ""
        self.gcs_cmd = ""
        self.last_gcs_cmd = ""
        self.formation_config = "waiting"
        self.takeoff_target = None
        self.hover_target = None
        self.leader_desire = None
        self.first_form = {"FORM_1": True, "FORM_2": True}
        self.neighbors = []
        self.all_uav_data = None
        self.last_request = 0.0

        self.create_subscription(PoseStamped, self.ns + "/mavros/local_position/pose", self.pose_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(TwistStamped, self.ns + "/mavros/local_position/velocity_local",
                                 lambda m: setattr(self, "velocity", m), qos_profile_sensor_data)
        self.create_subscription(State, self.ns + "/mavros/state", self.state_callback, qos_profile_sensor_data)
        self.create_subscription(String, "/xtdrone_gcs/cmd", lambda m: setattr(self, "gcs_cmd", m.data), 10)
        self.create_subscription(String, "/xtdrone/leader/cmd", self.cmd_callback, 10)
        self.create_subscription(String, f"/xtdrone/{vehicle_type}_1/cmd", self.cmd_callback, 10)
        self.create_subscription(AllUAVData, "/xtdrone_gcs/all_uav_data",
                                 lambda m: setattr(self, "all_uav_data", m), 10)
        self.create_subscription(CommVerify, "/communication_verify", self.comm_callback, 10)
        self.target_pub = self.create_publisher(PositionTarget, self.ns + "/mavros/setpoint_raw/local", 10)
        self.uav_data_pub = self.create_publisher(UAVDataMsg, self.ns + "/formation/uav_data", 10)
        self.arm_client = self.create_client(CommandBool, self.ns + "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, self.ns + "/mavros/set_mode")
        self.create_timer(1.0 / 15.0, self.loop)

    # --- callbacks
    def pose_callback(self, msg):
        self.local_pose = msg
        q = msg.pose.orientation
        self.heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # The targets follow the vehicle until the command starts (and are seeded by the first pose
        # in case the command arrived before MAVROS published one).
        if self.gcs_cmd != "AUTO.TAKEOFF" or self.takeoff_target is None:
            self.takeoff_target = msg.pose.position
        if self.gcs_cmd != "HOVER" or self.hover_target is None:
            self.hover_target = msg.pose.position
        for form in self.first_form:
            if self.gcs_cmd != form:
                self.first_form[form] = True

    def state_callback(self, msg):
        self.armed, self.mode = msg.armed, msg.mode

    def cmd_callback(self, msg):
        if msg.data in FORMATION_KEYS and msg.data != self.formation_config:
            self.formation_config = msg.data
            self.gcs_cmd = msg.data
            self.get_logger().info(f"command {msg.data}")

    def comm_callback(self, msg):
        if self.uav_id >= len(msg.trans_matrix):
            return
        received = list(msg.trans_matrix[self.uav_id].trans_matrix)
        leaders = list(msg.leader)
        is_leader = self.uav_id in leaders
        self.neighbors = [k for k, ok in zip(leaders, received)
                          if ok == 1 and (not is_leader or k < self.uav_id)]

    # --- helpers
    def uav_data(self):
        d = UAVDataMsg()
        d.header.stamp = self.get_clock().now().to_msg()
        d.uav_id = self.uav_id
        d.pose = copy.deepcopy(self.local_pose.pose)
        d.pose.position.x += self.bias[0]
        d.pose.position.y += self.bias[1]
        d.velocity = self.velocity.twist
        d.heading = float(self.heading or 0.0)
        d.is_sim = 1
        return d

    def request(self, arm=False, mode=None):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_request < 1.0:
            return
        self.last_request = now
        if arm:
            if self.arm_client.service_is_ready():
                self.arm_client.call_async(CommandBool.Request(value=arm))
            else:
                self.get_logger().warn("arming service not available yet", throttle_duration_sec=5.0)
        if mode:
            if self.mode_client.service_is_ready():
                self.mode_client.call_async(SetMode.Request(custom_mode=mode))
            else:
                self.get_logger().warn("set_mode service not available yet", throttle_duration_sec=5.0)

    def position_target(self, x, y, z):
        t = PositionTarget(coordinate_frame=FRAME_LOCAL_NED)
        t.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
                       | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                       | PositionTarget.IGNORE_YAW_RATE)
        t.position.x, t.position.y, t.position.z = float(x), float(y), float(z)
        t.yaw = float(self.heading)
        return t

    def velocity_target(self, v, yaw_rate):
        t = PositionTarget(coordinate_frame=FRAME_LOCAL_NED)
        t.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                       | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                       | PositionTarget.IGNORE_YAW)
        t.velocity.x, t.velocity.y, t.velocity.z = (float(c) for c in v)
        t.yaw_rate = float(yaw_rate)
        return t

    def yaw_rate(self):
        return limit(self.Kpangz * self.Kpangz * (self.target_yaw - self.heading), self.velangz_max)

    def leader_control(self, me):
        p, d = me.pose.position, self.leader_desire
        v = (limit(0.7 * (d[0] - p.x), self.velxy_max),
             limit(0.7 * (d[1] - p.y), self.velxy_max),
             limit(0.7 * (self.takeoff_height - p.z), self.velz_max))
        return self.velocity_target(v, self.yaw_rate())

    def formation_control(self, me):
        n = len(self.neighbors)
        weight = 4.0 / (self.gamma ** 2 * n) if n else 0.0
        own = self.desired[self.uav_id]
        p, v = me.pose.position, me.velocity.linear
        delta = np.zeros(3)
        delta_v = np.zeros(3)
        for k in self.neighbors:
            other = self.all_uav_data.data[k]
            op, ov = other.pose.position, other.velocity.linear
            delta += np.array([op.x - p.x, op.y - p.y, op.z - p.z]) - self.desired[k] + own
            delta_v += np.array([ov.x - v.x, ov.y - v.y, ov.z - v.z])
        accel = weight * (delta + self.gamma * delta_v)
        cmd = np.array([v.x, v.y, v.z]) + accel * 0.5 * self.Kp
        cmd = (limit(cmd[0], self.velxy_max), limit(cmd[1], self.velxy_max), limit(cmd[2], self.velz_max))
        return self.velocity_target(cmd, self.yaw_rate())

    # --- main loop (15 Hz)
    def loop(self):
        if self.local_pose is None or self.heading is None:
            if self.gcs_cmd:
                self.get_logger().warn("no local position from MAVROS yet", throttle_duration_sec=5.0)
            return
        me = self.uav_data()
        self.uav_data_pub.publish(me)
        cmd = self.gcs_cmd
        if cmd == "AUTO.TAKEOFF":
            self.last_gcs_cmd = cmd
            self.target_pub.publish(self.position_target(self.takeoff_target.x, self.takeoff_target.y,
                                                         self.takeoff_height))
            if not self.armed or self.mode != "OFFBOARD":
                self.request(arm=not self.armed, mode="OFFBOARD" if self.mode != "OFFBOARD" else None)
        elif cmd in ("FORM_1", "FORM_2"):
            self.last_gcs_cmd = cmd
            if self.mode != "OFFBOARD":
                self.request(mode="OFFBOARD")
            data_ready = self.all_uav_data is not None and len(self.all_uav_data.data) > max(self.neighbors, default=0)
            if self.uav_id == 0 or not self.neighbors or not data_ready:
                p = me.pose.position
                if self.first_form[cmd] or self.leader_desire is None:
                    self.leader_desire = [p.x, p.y, p.z]
                    self.first_form[cmd] = False
                if self.uav_id == 0:
                    self.leader_desire[1] += 0.04 if cmd == "FORM_1" else -0.04
                self.target_pub.publish(self.leader_control(me))
            else:
                self.target_pub.publish(self.formation_control(me))
        elif cmd == "AUTO.LAND":
            self.last_gcs_cmd = cmd
            if self.mode != "AUTO.LAND":
                self.request(mode="AUTO.LAND")
        elif cmd == "HOVER":
            self.last_gcs_cmd = cmd
            self.target_pub.publish(self.position_target(self.hover_target.x, self.hover_target.y,
                                                         self.hover_target.z))
            if not self.armed or self.mode != "OFFBOARD":
                self.request(arm=not self.armed, mode="OFFBOARD" if self.mode != "OFFBOARD" else None)
        else:
            self.gcs_cmd = self.last_gcs_cmd


def all_main():
    """Aggregator, communication model and every UAV controller as separate nodes in one process
    (one executor). With 20 vehicles on one machine this is much lighter than 22 processes."""
    from rclpy.executors import SingleThreadedExecutor
    ap = argparse.ArgumentParser(description="Chapter 27: all formation-communication nodes in one process.")
    ap.add_argument("vehicle_type")
    ap.add_argument("uav_num", type=int)
    ap.add_argument("--leaders", default="all")
    ap.add_argument("--pos-file", default="UAV_pos1")
    a, ros_args = ap.parse_known_args()
    leaders = list(range(a.uav_num)) if a.leaders == "all" else [int(x) for x in a.leaders.split(",")]
    path = a.pos_file if os.path.sep in a.pos_file else default_pos_file(a.pos_file)
    rclpy.init(args=ros_args)
    nodes = [UAVDataAggregator(a.vehicle_type, a.uav_num), CommunicationVerify(a.uav_num, leaders)]
    nodes += [FormationController(a.vehicle_type, i, a.uav_num, path) for i in range(a.uav_num)]
    executor = SingleThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    for n in nodes:
        n.destroy_node()
    rclpy.try_shutdown()


def default_pos_file(name):
    return os.path.join(get_package_share_directory("xtdrone_formation_demo"), "config", name + ".txt")


def controller_main():
    ap = argparse.ArgumentParser(description="Chapter 27 consensus controller for one UAV.")
    ap.add_argument("vehicle_type")
    ap.add_argument("uav_id", type=int)
    ap.add_argument("uav_num", type=int)
    ap.add_argument("--pos-file", default="UAV_pos1", help="UAV_pos1/2/3 or a path to a desired-position table")
    a, ros_args = ap.parse_known_args()
    path = a.pos_file if os.path.sep in a.pos_file else default_pos_file(a.pos_file)
    rclpy.init(args=ros_args)
    spin(FormationController(a.vehicle_type, a.uav_id, a.uav_num, path))


def verify_main():
    ap = argparse.ArgumentParser(description="Chapter 27 SINR communication model.")
    ap.add_argument("uav_num", type=int, nargs="?", default=20)
    ap.add_argument("--leaders", default="all",
                    help="'all' or comma-separated UAV ids that transmit (e.g. 0,13,17)")
    a, ros_args = ap.parse_known_args()
    leaders = list(range(a.uav_num)) if a.leaders == "all" else [int(x) for x in a.leaders.split(",")]
    rclpy.init(args=ros_args)
    spin(CommunicationVerify(a.uav_num, leaders))


def aggregator_main():
    ap = argparse.ArgumentParser(description="Collect formation/uav_data into /xtdrone_gcs/all_uav_data.")
    ap.add_argument("vehicle_type")
    ap.add_argument("uav_num", type=int)
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    spin(UAVDataAggregator(a.vehicle_type, a.uav_num))
