import os

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from xtdrone_py.precision_landing import optical_to_body, quat_rotate, tag_in_camera  # noqa: E402

TEXTURE = os.path.join(os.path.dirname(__file__), "../../xtdrone_gz/models/apriltag0-2/materials/textures/apriltag0-2.png")


def synthetic_view(tag_px=100, center=(740, 360)):
    """1280x720 image with tag 1 (from the model texture) tag_px wide (black square) at center."""
    # the model stores the texture rotated (tags along the plate's x axis); turn it back
    tex = np.ascontiguousarray(np.rot90(cv2.imread(TEXTURE, cv2.IMREAD_GRAYSCALE), -1))
    h, w = tex.shape
    # the middle third of the texture holds tag 1; its black square is 0.2 of the texture width
    tag = tex[:, w // 3: 2 * w // 3]
    black = 0.2 * w
    scale = tag_px / black
    tag = cv2.resize(tag, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    img = np.full((720, 1280), 255, np.uint8)
    th, tw = tag.shape
    y0, x0 = center[1] - th // 2, center[0] - tw // 2
    img[y0:y0 + th, x0:x0 + tw] = tag
    return img


def test_tag_position_from_image():
    K = np.array([[369.5, 0, 640], [0, 369.5, 360], [0, 0, 1]])
    p = tag_in_camera(synthetic_view(), K, tag_id=1, tag_size=0.5)
    assert p is not None
    z = 369.5 * 0.5 / 100
    np.testing.assert_allclose(p, [100 * z / 369.5, 0.0, z], atol=0.05)
    assert tag_in_camera(synthetic_view(), K, tag_id=2) is None


def test_downward_camera_axes():
    # tag ahead of the vehicle appears at the image top (optical -y), below it (optical +z)
    np.testing.assert_allclose(optical_to_body([0, -1, 2]), [1, 0, -2])
    # image right (optical +x) is the vehicle's right (body -y)
    np.testing.assert_allclose(optical_to_body([1, 0, 0]), [0, -1, 0])


def test_quat_rotate_yaw90():
    s = np.sqrt(0.5)
    np.testing.assert_allclose(quat_rotate((0, 0, s, s), np.array([1.0, 0, 0])), [0, 1, 0], atol=1e-9)
