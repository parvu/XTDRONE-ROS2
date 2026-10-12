"""XTDrone keyboard control (ports of control/keyboard/*_keyboard_control.py).

  ros2 run xtdrone_py multirotor_keyboard_control iris 1 vel
  ros2 run xtdrone_py plane_keyboard_control 1
  ros2 run xtdrone_py vtol_keyboard_control standard_vtol 1 vel
  ros2 run xtdrone_py rover_keyboard_control 1

Keys and topics are the same as in the ROS 1 scripts. Run it in a real terminal.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String

KEYS_COMMON = """
   1   2   3   4   5   6   7   8   9   0
        w       r    t   y        i
   a    s    d       g       j    k    l
        x       v    b   n        ,
"""

MULTIROTOR_HELP = """
w/x : increase/decrease forward
a/d : increase/decrease leftward
i/, : increase/decrease upward
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s/k : hover and remove the mask of keyboard control
0   : mask the keyboard control (for single UAV)
0~9 : extendable mission (e.g. different formation configuration)
      this will mask the keyboard control
g   : control the leader / all drones
CTRL-C to quit
"""

PLANE_HELP = """
w/x : increase/decrease north setpoint
a/d : increase/decrease east setpoint
i/, : increase/decrease upward setpoint
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s   : loiter
k   : idle
0~9 : extendable mission
g   : control the leader / all drones
o   : send setpoint
CTRL-C to quit
"""

VTOL_HELP = """
w/x : forward velocity (multirotor) / north setpoint (plane)
a/d : leftward velocity (multirotor) / east setpoint (plane)
i/, : upward velocity (multirotor) / upward setpoint (plane)
j/l : increase/decrease yaw
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
s   : hover (multirotor mode), loiter (plane mode)
k   : hover (multirotor mode), idle (plane mode)
0~9 : extendable mission
g   : control the leader / all drones
o   : transition
CTRL-C to quit
"""

ROVER_HELP = """
w/x : increase/decrease forward velocity
a/d : increase/decrease steering angle
r   : return home
t/y : arm/disarm
b   : offboard
s   : stop (and remove the mask of keyboard control)
0~9 : extendable mission (masks the keyboard control)
g   : control the leader / all vehicles
CTRL-C to quit
"""

UGV_HELP = """
w/x : increase/decrease forward velocity
a/d : increase/decrease steering angle
s   : stop (and remove the mask of keyboard control)
0~9 : extendable mission (masks the keyboard control)
g   : control the leader / all vehicles
CTRL-C to quit
"""

FORMATIONS = {20: ["waiting", "FORM_1", "FORM_2", "FORM_3"],  # chapter 27 communication demo
              18: ["waiting", "cuboid", "sphere", "diamond"],
              9: ["waiting", "cube", "pyramid", "triangle"],
              6: ["waiting", "T", "diamond", "triangle"],
              1: ["stop controlling"]}

FLIGHT_KEYS = {"r": ("AUTO.RTL", "Returning home"), "t": ("ARM", "Arming"), "y": ("DISARM", "Disarming"),
               "v": ("AUTO.TAKEOFF", "AUTO.TAKEOFF"), "b": ("OFFBOARD", "Offboard"),
               "n": ("AUTO.LAND", "AUTO.LAND")}


def clamp(v, limit):
    return max(-limit, min(limit, v))


class Terminal:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise SystemExit("keyboard control needs an interactive terminal")
        self.settings = termios.tcgetattr(sys.stdin)
        return self

    def key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def __exit__(self, *exc):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def banner(title, help_text, leader):
    target = "To the leader (press g to control all)" if leader else "To all vehicles (press g to control the leader)"
    print(f"\nControl Your XTDrone!  [{title}]\n{target}\n---------------------------{KEYS_COMMON}{help_text}")


def _usage(text):
    raise SystemExit("usage: " + text)


def multirotor_main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 3:
        _usage("multirotor_keyboard_control <type> <num> <vel|accel>   e.g. iris 1 vel")
    vtype, num, control_type = argv[0], int(argv[1]), argv[2]
    formation_configs = FORMATIONS.get(num, ["waiting"])
    step, max_linear, max_ang = 0.01, 20.0, 3.0
    rclpy.init()
    node = rclpy.create_node(f"{vtype}_multirotor_keyboard_control")
    suffix = "cmd_vel_flu" if control_type == "vel" else "cmd_accel_flu"
    twist_pubs = [node.create_publisher(Twist, f"/xtdrone/{vtype}_{i}/{suffix}", 1) for i in range(num)]
    cmd_pubs = [node.create_publisher(String, f"/xtdrone/{vtype}_{i}/cmd", 3) for i in range(num)]
    leader_twist = node.create_publisher(Twist, f"/xtdrone/leader/{suffix}", 1)
    leader_cmd = node.create_publisher(String, "/xtdrone/leader/cmd", 3)

    forward = leftward = upward = angular = 0.0
    ctrl_leader, cmd_vel_mask = False, False
    banner(f"{vtype} x{num}, {control_type}", MULTIROTOR_HELP, ctrl_leader)
    with Terminal() as term:
        while rclpy.ok():
            key = term.key()
            cmd = ""
            if key in ("w", "x"):
                forward += step if key == "w" else -step
            elif key in ("a", "d"):
                leftward += step if key == "a" else -step
            elif key in ("i", ","):
                upward += step if key == "i" else -step
            elif key in ("j", "l"):
                angular += step if key == "j" else -step
            elif key in FLIGHT_KEYS:
                cmd, text = FLIGHT_KEYS[key]
                print(text)
            elif key == "g":
                ctrl_leader = not ctrl_leader
                banner(f"{vtype} x{num}, {control_type}", MULTIROTOR_HELP, ctrl_leader)
            elif key in ("k", "s"):
                cmd_vel_mask = False
                forward = leftward = upward = angular = 0.0
                cmd = "HOVER"
                print("Hover")
            elif key.isdigit():
                idx = int(key)
                if idx < len(formation_configs):
                    cmd = formation_configs[idx]
                    print(cmd)
                    cmd_vel_mask = True
            elif key == "\x03":
                break
            if key in ("w", "x", "a", "d", "i", ",", "j", "l", "k", "s"):
                print(f"currently:\t forward {forward:.2f}\t leftward {leftward:.2f}\t "
                      f"upward {upward:.2f}\t angular {angular:.2f}")
            forward, leftward, upward = (clamp(v, max_linear) for v in (forward, leftward, upward))
            angular = clamp(angular, max_ang)
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = forward, leftward, upward
            twist.angular.z = angular
            msg = String(data=cmd)
            if ctrl_leader:
                leader_twist.publish(twist)
                leader_cmd.publish(msg)
            else:
                for i in range(num):
                    if not cmd_vel_mask:
                        twist_pubs[i].publish(twist)
                    cmd_pubs[i].publish(msg)
    node.destroy_node()
    rclpy.try_shutdown()


def plane_main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 1:
        _usage("plane_keyboard_control <num>")
    num = int(argv[0])
    step, ang_step = 0.1, 0.01
    rclpy.init()
    node = rclpy.create_node("plane_keyboard_control")
    pose_pubs = [node.create_publisher(Pose, f"/xtdrone/plane_{i}/cmd_pose_enu", 1) for i in range(num)]
    cmd_pubs = [node.create_publisher(String, f"/xtdrone/plane_{i}/cmd", 1) for i in range(num)]
    leader_pose = node.create_publisher(Pose, "/xtdrone/leader/cmd_pose", 1)
    leader_cmd = node.create_publisher(String, "/xtdrone/leader_cmd", 3)
    forward = leftward = upward = angular = 0.0
    ctrl_leader, send_flag = False, False
    banner(f"plane x{num}", PLANE_HELP, ctrl_leader)
    with Terminal() as term:
        while rclpy.ok():
            key = term.key()
            cmd = ""
            if key in ("w", "x"):
                forward += step if key == "w" else -step
            elif key in ("a", "d"):
                leftward += step if key == "a" else -step
            elif key in ("i", ","):
                upward += step if key == "i" else -step
            elif key in ("j", "l"):
                angular += ang_step if key == "j" else -ang_step
            elif key in FLIGHT_KEYS:
                cmd, text = FLIGHT_KEYS[key]
                print(text)
            elif key == "g":
                ctrl_leader = not ctrl_leader
                banner(f"plane x{num}", PLANE_HELP, ctrl_leader)
            elif key == "s":
                cmd = "loiter"
                print(cmd)
            elif key == "k":
                cmd = "idle"
                print(cmd)
            elif key == "o":
                send_flag = True
                print("send setpoint")
            elif key.isdigit():
                cmd = "mission" + key
                print(cmd)
            elif key == "\x03":
                break
            if key in ("w", "x", "a", "d", "i", ",", "j", "l"):
                print(f"currently:\t north {forward:.2f}\t east {leftward:.2f}\t "
                      f"upward {upward:.2f}\t angular {angular:.2f}")
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = forward, leftward, upward
            pose.orientation.z = angular
            msg = String(data=cmd)
            if ctrl_leader:
                if send_flag:
                    leader_pose.publish(pose)
                leader_cmd.publish(msg)
            else:
                for i in range(num):
                    if send_flag:
                        pose_pubs[i].publish(pose)
                    cmd_pubs[i].publish(msg)
    node.destroy_node()
    rclpy.try_shutdown()


def vtol_main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 3:
        _usage("vtol_keyboard_control <type> <num> <vel|accel>   e.g. standard_vtol 1 vel")
    vtype, num, control_type = argv[0], int(argv[1]), argv[2]
    step, ang_step, max_linear, max_ang = 0.1, 0.01, 1000.0, 0.5
    rclpy.init()
    node = rclpy.create_node(f"{vtype}_vtol_keyboard_control")
    suffix = "cmd_vel_flu" if control_type == "vel" else "cmd_accel_flu"
    pose_pubs = [node.create_publisher(Pose, f"/xtdrone/{vtype}_{i}/cmd_pose_enu", 1) for i in range(num)]
    twist_pubs = [node.create_publisher(Twist, f"/xtdrone/{vtype}_{i}/{suffix}", 1) for i in range(num)]
    cmd_pubs = [node.create_publisher(String, f"/xtdrone/{vtype}_{i}/cmd", 3) for i in range(num)]
    leader_pose = node.create_publisher(Pose, "/xtdrone/leader/cmd_pose_enu", 1)
    leader_twist = node.create_publisher(Twist, f"/xtdrone/leader/{suffix}", 1)
    leader_cmd = node.create_publisher(String, "/xtdrone/leader_cmd", 3)
    forward = leftward = upward = angular = 0.0
    ctrl_leader, transition_state = False, "multirotor"
    banner(f"{vtype} x{num}, {control_type}", VTOL_HELP, ctrl_leader)
    with Terminal() as term:
        while rclpy.ok():
            key = term.key()
            cmd = ""
            if key in ("w", "x"):
                forward += step if key == "w" else -step
            elif key in ("a", "d"):
                leftward += step if key == "a" else -step
            elif key in ("i", ","):
                upward += step if key == "i" else -step
            elif key in ("j", "l"):
                angular += ang_step if key == "j" else -ang_step
            elif key in FLIGHT_KEYS:
                cmd, text = FLIGHT_KEYS[key]
                print(text)
            elif key == "g":
                ctrl_leader = not ctrl_leader
                banner(f"{vtype} x{num}, {control_type}", VTOL_HELP, ctrl_leader)
            elif key in ("s", "k"):
                if transition_state == "multirotor":
                    forward = leftward = upward = angular = 0.0
                    cmd = "HOVER"
                else:
                    cmd = "loiter" if key == "s" else "idle"
                print(cmd)
            elif key == "o":
                transition_state = "plane" if transition_state == "multirotor" else "multirotor"
                cmd = transition_state
                print(cmd)
            elif key.isdigit():
                cmd = "mission" + key
                print(cmd)
            elif key == "\x03":
                break
            if key in ("w", "x", "a", "d", "i", ",", "j", "l"):
                print(f"currently:\t forward {forward:.2f}\t leftward {leftward:.2f}\t "
                      f"upward {upward:.2f}\t angular {angular:.2f}")
            forward, leftward, upward = (clamp(v, max_linear) for v in (forward, leftward, upward))
            angular = clamp(angular, max_ang)
            msg = String(data=cmd)
            pose, twist = Pose(), Twist()
            if transition_state == "plane":
                pose.position.x, pose.position.y, pose.position.z = forward, leftward, upward
                pose.orientation.z = angular
            else:
                twist.linear.x, twist.linear.y, twist.linear.z = forward, leftward, upward
                twist.angular.z = angular
            if ctrl_leader:
                (leader_pose.publish(pose) if transition_state == "plane" else leader_twist.publish(twist))
                leader_cmd.publish(msg)
            else:
                for i in range(num):
                    if transition_state == "plane":
                        pose_pubs[i].publish(pose)
                    else:
                        twist_pubs[i].publish(twist)
                    cmd_pubs[i].publish(msg)
    node.destroy_node()
    rclpy.try_shutdown()


def rover_main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 1:
        _usage("rover_keyboard_control <num>")
    num = int(argv[0])
    lin_step, ang_step, max_lin, max_angle = 0.1, 0.1, 20.0, 1.0
    rclpy.init()
    node = rclpy.create_node("rover_keyboard_control")
    twist_pubs = [node.create_publisher(Twist, f"/xtdrone/rover_{i}/cmd_vel_flu", 1) for i in range(num)]
    cmd_pubs = [node.create_publisher(String, f"/xtdrone/rover_{i}/cmd", 1) for i in range(num)]
    leader_twist = node.create_publisher(Twist, "/xtdrone/leader/cmd_vel", 1)
    leader_cmd = node.create_publisher(String, "/xtdrone/leader_cmd", 3)
    forward = angle = 0.0
    ctrl_leader, cmd_vel_mask = False, False
    banner(f"rover x{num}", ROVER_HELP, ctrl_leader)
    with Terminal() as term:
        while rclpy.ok():
            key = term.key()
            cmd = ""
            if key in ("w", "x"):
                forward += lin_step if key == "w" else -lin_step
            elif key in ("a", "d"):
                angle += ang_step if key == "a" else -ang_step
            elif key in ("r", "t", "y", "b"):
                cmd, text = FLIGHT_KEYS[key]
                print(text)
            elif key == "g":
                ctrl_leader = not ctrl_leader
                banner(f"rover x{num}", ROVER_HELP, ctrl_leader)
            elif key == "s":
                cmd_vel_mask = False
                forward = angle = 0.0
            elif key.isdigit():
                cmd = "mission" + key
                print(cmd)
                cmd_vel_mask = True
            elif key == "\x03":
                break
            if key in ("w", "x", "a", "d", "s"):
                print(f"currently:\t forward vel {forward:.2f}\t steering {angle:.2f}")
            forward, angle = clamp(forward, max_lin), clamp(angle, max_angle)
            twist = Twist()
            twist.linear.x, twist.angular.z = forward, angle
            msg = String(data=cmd)
            if ctrl_leader:
                leader_twist.publish(twist)
                leader_cmd.publish(msg)
            else:
                for i in range(num):
                    if not cmd_vel_mask:
                        twist_pubs[i].publish(twist)
                    cmd_pubs[i].publish(msg)
    node.destroy_node()
    rclpy.try_shutdown()


def ugv_main():
    """control/keyboard/ugv_keyboard_control.py: /ugv_<i>/cmd_vel (linear.x speed, angular.z steering angle)."""
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(argv) < 1:
        _usage("ugv_keyboard_control <num>")
    num = int(argv[0])
    lin_step, ang_step, max_lin, max_angle = 0.1, 0.03, 20.0, 1.0
    rclpy.init()
    node = rclpy.create_node("ugv_keyboard_control")
    twist_pubs = [node.create_publisher(Twist, f"/ugv_{i}/cmd_vel", 1) for i in range(num)]
    cmd_pubs = [node.create_publisher(String, f"/ugv_{i}/cmd", 1) for i in range(num)]
    leader_twist = node.create_publisher(Twist, "/ugv/leader/cmd_vel", 1)
    leader_cmd = node.create_publisher(String, "/ugv/leader_cmd", 3)
    forward = angle = 0.0
    ctrl_leader, cmd_vel_mask = False, False
    banner(f"ugv x{num}", UGV_HELP, ctrl_leader)
    with Terminal() as term:
        while rclpy.ok():
            key = term.key()
            cmd = ""
            if key in ("w", "x"):
                forward += lin_step if key == "w" else -lin_step
            elif key in ("a", "d"):
                angle += ang_step if key == "a" else -ang_step
            elif key == "g":
                ctrl_leader = not ctrl_leader
                banner(f"ugv x{num}", UGV_HELP, ctrl_leader)
            elif key == "s":
                cmd_vel_mask = False
                forward = angle = 0.0
            elif key.isdigit():
                cmd = "mission" + key
                print(cmd)
                cmd_vel_mask = True
            elif key == "\x03":
                break
            forward, angle = clamp(forward, max_lin), clamp(angle, max_angle)
            if key in ("w", "x", "a", "d", "s"):
                print(f"currently:\t forward vel {forward:.2f}\t steering angle {angle:.2f}")
            twist = Twist()
            twist.linear.x, twist.angular.z = forward, angle
            msg = String(data=cmd)
            if ctrl_leader:
                leader_twist.publish(twist)
                leader_cmd.publish(msg)
            else:
                for i in range(num):
                    if not cmd_vel_mask:
                        twist_pubs[i].publish(twist)
                    cmd_pubs[i].publish(msg)
    node.destroy_node()
    rclpy.try_shutdown()
