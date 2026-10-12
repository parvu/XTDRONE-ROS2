#!/usr/bin/env python3
"""Chapter 42 check: fly two iris with XTDGroundControl (offscreen Qt), like a user clicking.

Requires: xtdrone.launch.py world:=empty vehicles:="iris*2" communication:=basic.
Steps in the window: iris number 2, Run, Control, tick "iris", command "control all",
"offboard", up 1 m/s, "arm" -> both climb above 2 m -> "hover" -> "land" -> both disarm.
Also checks that the state panel shows the vehicles' poses.
"""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rclpy  # noqa: E402
from mavros_msgs.msg import State  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from xtdrone_groundcontrol.ground_control import Gui2Ros  # noqa: E402

NAMES = ["iris_0", "iris_1"]


def log(m):
    print(f"[groundcontrol_test] {m}", flush=True)


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    mon = rclpy.create_node("groundcontrol_test")
    state, z = {}, {}
    for n in NAMES:
        mon.create_subscription(State, f"/{n}/mavros/state", lambda m, n=n: state.__setitem__(n, m),
                                qos_profile_sensor_data)
        mon.create_subscription(Odometry, f"/{n}/ground_truth/odom",
                                lambda m, n=n: z.__setitem__(n, m.pose.pose.position.z), 1)

    def wait(seconds, cond=None):
        end = time.time() + seconds
        while time.time() < end:
            app.processEvents()
            rclpy.spin_once(mon, timeout_sec=0.05)
            if cond and cond():
                return True
        return bool(cond and cond())

    def command(text):
        ui.box_command.setCurrentText(text)  # currentIndexChanged -> get_command()
        wait(1.5)

    ok = True
    ui = Gui2Ros()
    ui.show()
    try:
        if not wait(180, lambda: all(n in state and state[n].connected and n in z for n in NAMES)):
            log("FAIL: vehicles not connected")
            return 1
        wait(10)  # home position
        ui.box_iris_num.setValue(2)
        ui.button_run.click()
        wait(2)
        ui.button_control.click()
        ui.checkBox_iris.setChecked(True)
        wait(1)
        command("control all")
        command("offboard")
        ui.box_up_and_down_2.setValue(1.0)
        deadline = time.time() + 60
        while time.time() < deadline and not all(state[n].armed for n in NAMES):
            command("hover")  # leave "arm" so that selecting it again re-sends ARM
            ui.box_up_and_down_2.setValue(1.0)
            command("offboard")
            command("arm")
        climbed = wait(60, lambda: all(z[n] > 2.0 for n in NAMES))
        log(f"{'ok  ' if climbed else 'FAIL'} climbed with the GUI: " + ", ".join(f"{n} {z[n]:.1f} m" for n in NAMES))
        ok &= climbed
        text = ui.text_show_info.toPlainText()
        shown = "iris pose" in text and "uav1" in text
        log(f"{'ok  ' if shown else 'FAIL'} state panel shows the poses")
        ok &= shown
        command("hover")
        wait(3)
        command("land")
        landed = wait(90, lambda: all(not state[n].armed for n in NAMES))
        log(f"{'ok  ' if landed else 'FAIL'} landed and disarmed")
        ok &= landed
    finally:
        ui.close()
        app.processEvents()
        mon.destroy_node()
        rclpy.try_shutdown()
    log("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
