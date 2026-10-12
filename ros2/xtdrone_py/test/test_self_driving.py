import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from xtdrone_py.self_driving import NO_LANE, lane_error, steer  # noqa: E402

YELLOW_BGR = (0, 220, 230)  # HSV hue ~28


def road(offset=0):
    """Two yellow lane lines converging towards the top, centre shifted by offset pixels."""
    img = np.zeros((480, 752, 3), np.uint8)
    c = 376 + offset
    cv2.line(img, (c - 300, 479), (c - 60, 240), YELLOW_BGR, 12)
    cv2.line(img, (c + 300, 479), (c + 60, 240), YELLOW_BGR, 12)
    return img


def test_centred_lane_has_small_error():
    err, _ = lane_error(road(), (26, 43, 46), (30, 255, 255), rows=(0.5, 1.0))
    assert err != NO_LANE and abs(err) < 20


def test_shifted_lane_sign_and_no_lane():
    err, _ = lane_error(road(80), (26, 43, 46), (30, 255, 255), rows=(0.5, 1.0))
    assert 50 < err < 120
    err, _ = lane_error(np.zeros((480, 752, 3), np.uint8), (26, 43, 46), (30, 255, 255))
    assert err == NO_LANE


def test_steer():
    assert steer(NO_LANE, 0.002) == (0.0, 0.0)
    v, a = steer(100, 0.002)
    assert a == pytest.approx(-0.2) and v == pytest.approx(4.0)
    assert steer(10, 0.002) == (5.0, 0.0)
