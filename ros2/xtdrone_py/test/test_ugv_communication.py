import math

from xtdrone_py.ugv_communication import steering_to_yaw_rate


def test_straight_and_turning():
    assert steering_to_yaw_rate(5.0, 0.0) == 0.0
    # bicycle model: yaw rate = v tan(delta) / L, left steering is positive
    assert math.isclose(steering_to_yaw_rate(2.6, 0.3), math.tan(0.3))
    assert steering_to_yaw_rate(-1.0, 0.3) < 0  # reversing turns the other way


def test_steering_limited():
    assert steering_to_yaw_rate(2.6, 2.0) == steering_to_yaw_rate(2.6, 0.6)
