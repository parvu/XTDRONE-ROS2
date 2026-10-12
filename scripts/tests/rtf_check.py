#!/usr/bin/env python3
"""Check the real time factor Gazebo reaches (chapter 43, speed_factor)."""
import argparse
import re
import subprocess
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--expect", type=float, default=1.0)
ap.add_argument("--tolerance", type=float, default=0.15)
a = ap.parse_args()
time.sleep(30)  # let the vehicles spawn and PX4 settle
values = []
for _ in range(3):
    out = subprocess.run(["gz", "topic", "-e", "-t", "/stats", "-n", "1"], capture_output=True, text=True,
                         timeout=10).stdout
    m = re.search(r"^real_time_factor: ([0-9.eE+-]+)", out, re.M)
    if m:
        values.append(float(m.group(1)))
    time.sleep(2)
rtf = sum(values) / len(values) if values else 0.0
ok = abs(rtf - a.expect) <= a.tolerance * a.expect
print(f"[rtf_check] real time factor {rtf:.2f} (expected {a.expect:g})")
print(f"[rtf_check] {'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
