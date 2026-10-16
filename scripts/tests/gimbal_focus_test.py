#!/usr/bin/env python3
"""Offline check of the chapter 23 gimbal focus law (no simulation): the joints must turn the
camera towards the person bbox and cancel the body roll, within the cgo3 joint limits."""
import math
import sys

from xtdrone_py.yolo import gimbal_focus

BASE = math.radians(-60)

p, r, y = gimbal_focus(None, BASE, 0.0)
assert (p, r, y) == (BASE, 0.0, 0.0)                      # no person: hold the base pose

p, r, y = gimbal_focus((100.0, 0.0), BASE, 0.0)
assert y < 0 and abs(y - (-0.002 * 100)) < 1e-9           # person right of center: yaw right
p, r, y = gimbal_focus((0.0, 100.0), BASE, 0.0)
assert p < BASE                                          # person below center: tilt down
p, r, y = gimbal_focus((0.0, -100.0), BASE, 0.0)
assert p > BASE                                          # person above center: tilt up

p, r, y = gimbal_focus((1e6, 1e6), BASE, 0.0)                 # joint limits hold
assert -1.05 <= p <= 2.09 and -math.pi <= y <= math.pi

p, r, y = gimbal_focus(None, BASE, 0.5)
assert r == 0.5                                          # roll cancels the body roll
p, r, y = gimbal_focus(None, BASE, 2.0)
assert r == 0.785                                        # cgo3 roll limit +-45 deg

print("PASS gimbal_focus")
sys.exit(0)
