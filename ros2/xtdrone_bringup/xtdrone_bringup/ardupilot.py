"""Locate the ArduPilot SITL build (submodule ArduPilot, release 4.7.2) and the ArduPilot Gazebo plugin."""
import os
from pathlib import Path

ARDUPILOT_VERSION = "4.7.2"

# XTDrone vehicle family -> SITL binary, ArduPilot default parameter files (relative to ArduPilot/Tools/autotest)
# and the XTDrone overlay (xtdrone_gz/ardupilot)
SITL = {
    "multirotor": {"binary": "arducopter", "defaults": ["default_params/copter.parm", "default_params/gazebo-iris.parm"],
                   "overlay": ["xtdrone.parm"]},
    "plane": {"binary": "arduplane", "defaults": ["models/plane.parm", "default_params/gazebo-zephyr.parm"],
              "overlay": ["xtdrone-plane.parm"]},
    # r1_rover.param: the tuned parameters of SITL_Models' R1 rover (skid steering), see ardupilot/SITL_Models.NOTICE
    "rover": {"binary": "ardurover", "defaults": ["default_params/rover.parm"],
              "overlay": ["r1_rover.param", "xtdrone-rover.parm"]},
}


def default_root():
    if os.environ.get("XTDRONE_ROOT"):
        return Path(os.environ["XTDRONE_ROOT"]).expanduser()
    for start in (Path.cwd(), Path(__file__).resolve().parent):
        for parent in (start, *start.parents):
            if (parent / "ArduPilot" / "wscript").is_file():
                return parent
    return Path.cwd()


def default_ardupilot_dir():
    return os.environ.get("ARDUPILOT_DIR") or str(default_root() / "ArduPilot")


def find_ardupilot_dir(path):
    d = Path(path).expanduser().resolve()
    if not (d / "wscript").is_file():
        raise RuntimeError(
            f"ardupilot_dir must point to the ArduPilot source directory: {d}. Run scripts/build_ardupilot.sh "
            "or set ARDUPILOT_DIR / XTDRONE_ROOT.")
    return d


def binary(ardupilot_dir, family):
    path = Path(ardupilot_dir) / "build" / "sitl" / "bin" / SITL[family]["binary"]
    if not path.is_file():
        raise RuntimeError(f"ArduPilot SITL is not built: {path} is missing. Run scripts/build_ardupilot.sh.")
    return path


def gazebo_plugin_dir(ardupilot_dir):
    d = Path(ardupilot_dir).parent / "ardupilot_gazebo"
    if not (d / "build" / "libArduPilotPlugin.so").is_file():
        raise RuntimeError(f"ArduPilot Gazebo plugin is not built in {d}. Run scripts/build_ardupilot.sh.")
    return d
