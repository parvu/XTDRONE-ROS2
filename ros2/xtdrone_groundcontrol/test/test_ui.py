import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
pytest.importorskip("matplotlib")


def test_window_builds_and_selects_vehicles():
    import rclpy
    from xtdrone_groundcontrol.ground_control import Gui2Ros, vehicle_type
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init()
    try:
        ui = Gui2Ros()
        ui.box_iris_num.setValue(3)
        ui.on_button_run_clicked()
        ui.init_controller()
        assert ui.multi_type == ["iris"] * 3
        twist, cmd, _, _ = ui.make_publishers(rclpy.create_node("t"))
        assert len(twist) == len(cmd) == 3
        assert vehicle_type("quadplane") == "standard_vtol"
        ui.close()
    finally:
        rclpy.try_shutdown()
    del app
