"""XTDGroundControl on ROS 2 (port of control/XTDGroundControl/python: main.py, send.py, receive.py).

  ros2 run xtdrone_groundcontrol xtdgroundcontrol

The PyQt5 window is unchanged. ROS 1 ran the sender in a multiprocessing.Process; here it is a
thread with its own rclpy node, fed by the same queues. The receiver (a QThread) spins a second
node for the positions (Gazebo ground truth /<type>_<i>/ground_truth/odom) and velocities.
"""
import random
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from xtdrone_groundcontrol import xtd_ui
from xtdrone_groundcontrol.plotcanvas import PlotCanvas

# The UI lists 'quadplane'; the PX4 v1.16 port calls that vehicle standard_vtol.
ROS2_TYPE = {"quadplane": "standard_vtol"}
UGV_INDEX = 7


def vehicle_type(ui_type):
    return ROS2_TYPE.get(ui_type, ui_type)


class Ros2Gui(QThread):
    """receive.py: positions and velocities of all vehicles, emitted once per second."""
    update_text = pyqtSignal(str)
    plot_array = pyqtSignal(list)

    def __init__(self, multi_select, multi_num, multi_type):
        super().__init__()
        self.node = Node("multirotor_pyqt5_receiver")
        self.executor = SingleThreadedExecutor()  # own executor: the global one may be used elsewhere
        self.executor.add_node(self.node)
        self.multirotor_num = sum(multi_num[i] for i in multi_select)
        self.multirotor_type = multi_type
        self.local_pose = [PoseStamped().pose for _ in range(self.multirotor_num)]
        self.local_vel = [TwistStamped() for _ in range(self.multirotor_num)]
        self.plot_all = [[[], []] for _ in range(self.multirotor_num)]
        self.time_map_x = 1
        self.time_map_y = 875 / 587
        self.running = True
        count = 0
        for i in multi_select:
            for k in range(multi_num[i]):
                name = f"{vehicle_type(self.multirotor_type[count])}_{k}"
                self.node.create_subscription(Odometry, f"/{name}/ground_truth/odom",
                                              lambda m, c=count: self.local_pose.__setitem__(c, m.pose.pose), 1)
                self.node.create_subscription(TwistStamped, f"/{name}/mavros/local_position/velocity_body",
                                              lambda m, c=count: self.local_vel.__setitem__(c, m),
                                              qos_profile_sensor_data)
                count += 1

    def run(self):
        last = time.monotonic()
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.03)
            if time.monotonic() - last < 1.0:
                continue
            last = time.monotonic()
            if self.multirotor_num >= 10:
                continue
            text = ""
            for i in range(self.multirotor_num):
                p, v = self.local_pose[i].position, self.local_vel[i].twist.linear
                t = self.multirotor_type[i]
                text += (f"{t} pose:\nuav{i}:\nx:{p.x:.3f}    y:{p.y:.3f}    z:{p.z:.3f}\n"
                         f"{t} vel:\nuav{i}:\nx:{v.x:.3f}    y:{v.y:.3f}    z:{v.z:.3f}\n")
                self.plot_all[i][0].append(p.x * self.time_map_x)
                self.plot_all[i][1].append(p.y * self.time_map_y)
            self.plot_array.emit(self.plot_all)
            self.update_text.emit(text)

    def stop(self):
        self.running = False
        self.wait(2000)
        self.executor.shutdown()
        self.node.destroy_node()


class Gui2Ros(QMainWindow, xtd_ui.Ui_MainWindow):
    """send.py: the main window; publishes commands like the keyboard does."""

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.map = "indoor1"
        self.comboBox_maps.currentIndexChanged.connect(self.initplot)
        self.button_run.clicked.connect(self.startrun)
        self.m = PlotCanvas(self, self.map)
        self.m.move(180, 0)
        self.text_thread = None
        self.sender = None

    def initplot(self):
        self.map = self.comboBox_maps.currentText()
        self.m.canvas_update(self.map)

    def startrun(self):
        print("start run!")
        self.init_controller()
        self.sender = threading.Thread(target=self.run_process, daemon=True)
        self.sender.start()
        if self.text_thread:
            self.text_thread.stop()
        self.text_thread = Ros2Gui(self.multirotor_select, self.multirotor_num, self.multi_type)
        self.text_thread.update_text.connect(self.display)
        self.text_thread.plot_array.connect(self.plot)
        self.text_thread.start()

    def init_controller(self):
        self.text_show_info.setPlainText("data")
        self.multi_num = 0
        self.multi_type = []
        for j in self.multirotor_select:
            self.multi_num += self.multirotor_num[j]
            self.multi_type += [self.multirotor_type[j]] * self.multirotor_num[j]
        self.color_plot = ["#%02x%02x%02x" % tuple(random.randint(16, 255) for _ in range(3))
                           for _ in range(self.multi_num)]

    def make_publishers(self, node):
        suffix = "cmd_vel_flu" if self.control_type == "vel" else "cmd_accel_flu"
        twist_pubs, cmd_pubs = [], []
        for i in self.multirotor_select:
            for k in range(self.multirotor_num[i]):
                if i == UGV_INDEX:
                    twist_pubs.append(node.create_publisher(Twist, f"/ugv_{k}/cmd_vel", 1))
                    cmd_pubs.append(node.create_publisher(String, f"/ugv_{k}/cmd", 1))
                else:
                    name = f"{vehicle_type(self.multirotor_type[i])}_{k}"
                    twist_pubs.append(node.create_publisher(Twist, f"/xtdrone/{name}/{suffix}", 1))
                    cmd_pubs.append(node.create_publisher(String, f"/xtdrone/{name}/cmd", 3))
        leader_twist = node.create_publisher(Twist, f"/xtdrone/leader/{suffix}", 1)
        leader_cmd = node.create_publisher(String, "/xtdrone/leader/cmd", 3)
        return twist_pubs, cmd_pubs, leader_twist, leader_cmd

    @staticmethod
    def latest(q, default):
        """Newest value put into a queue since the last call (or default)."""
        value, got = default, False
        while not q.empty():
            value, got = q.get(), True
        return value, got

    def run_process(self):
        """The 30 Hz sender loop of send.py, fed by the UI queues."""
        node = Node("multirotor_pyqt5_control")
        twist_pubs, cmd_pubs, leader_twist, leader_cmd = self.make_publishers(node)
        n = self.multi_num
        twist = [Twist() for _ in range(n)]
        cmd = ["" for _ in range(n)]
        get_control = [0] * n
        ctrl_leader = False
        cmd_vel_mask = False
        print("StartRun!")
        self.q_start_control_flag.get()  # wait for the Control button
        while rclpy.ok():
            get_control, _ = self.latest(self.q_multirotor_get_control, get_control)
            if len(get_control) < n:
                get_control = list(get_control) + [0] * (n - len(get_control))
            for q, attr in ((self.q_forward, ("linear", "x")), (self.q_upward, ("linear", "z")),
                            (self.q_leftward, ("linear", "y")), (self.q_orientation, ("angular", "z"))):
                value, got = self.latest(q, None)
                if got:
                    for i in range(n):
                        if get_control[i]:
                            setattr(getattr(twist[i], attr[0]), attr[1], float(value))
            ctrl_leader, _ = self.latest(self.q_ctrl_leader, ctrl_leader)
            new_cmd, got = self.latest(self.q_cmd, None)
            for i in range(n):
                if get_control[i]:
                    cmd[i] = ""
            if got:
                for i in range(n):
                    if ctrl_leader:
                        cmd[i] = new_cmd if i == min(1, n - 1) else ""
                    elif get_control[i]:
                        cmd[i] = new_cmd
            cmd_vel_mask, _ = self.latest(self.q_cmd_vel_mask, cmd_vel_mask)
            stop, _ = self.latest(self.q_stop_flag, False)
            if stop:
                cmd = ["AUTO.RTL"] * n
            if ctrl_leader:
                lead = min(1, n - 1)  # the ROS 1 GUI steers the leader with the second vehicle's values
                leader_twist.publish(twist[lead])
                leader_cmd.publish(String(data=cmd[lead]))
            else:
                for i in range(n):
                    if not cmd_vel_mask:
                        twist_pubs[i].publish(twist[i])
                    cmd_pubs[i].publish(String(data=cmd[i]))
            time.sleep(1.0 / 30.0)
            if stop:
                break
        node.destroy_node()

    def display(self, data):
        self.text_show_info.setPlainText(data)

    def plot(self, data):
        for i in range(min(self.multi_num, len(data))):
            self.m.ax.plot(data[i][0], data[i][1], color=self.color_plot[i])
        self.m.draw()

    def closeEvent(self, event):
        if self.text_thread:
            self.text_thread.stop()
        super().closeEvent(event)


def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    ui = Gui2Ros()
    ui.show()
    code = app.exec_()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
