"""Locate and verify the repository-pinned PX4-Autopilot checkout."""
import os
import subprocess
from pathlib import Path

PX4_REQUIRED_COMMIT = "54f0455ffcd755534539a7cf33a09a20bf71d29d"  # v1.16.2


def default_px4_dir():
    if os.environ.get("PX4_DIR"):
        return os.environ["PX4_DIR"]
    if os.environ.get("XTDRONE_ROOT"):
        return str(Path(os.environ["XTDRONE_ROOT"]).expanduser() / "PX4-Autopilot")
    starts = (Path.cwd(), Path(__file__).resolve().parent)
    for start in starts:
        for parent in (start, *start.parents):
            candidate = parent / "PX4-Autopilot"
            if (candidate / "Makefile").is_file():
                return str(candidate)
    return str(Path.cwd() / "PX4-Autopilot")


def find_px4_dir(path):
    px4_dir = Path(path).expanduser().resolve()
    if not (px4_dir / "Makefile").is_file():
        raise RuntimeError(
            f"px4_dir must point to a PX4-Autopilot source directory: {px4_dir}. Initialize it with "
            "'git submodule update --init --recursive PX4-Autopilot' or set PX4_DIR / XTDRONE_ROOT.")
    return px4_dir


def check_px4(px4_dir):
    try:
        commit = subprocess.run(["git", "-C", str(px4_dir), "rev-parse", "HEAD"], check=True,
                                capture_output=True, text=True).stdout.strip()
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"cannot read the PX4 commit in {px4_dir}") from exc
    if commit != PX4_REQUIRED_COMMIT and not os.environ.get("XTDRONE_ALLOW_ANY_PX4"):
        raise RuntimeError(
            f"PX4 v1.16.2 ({PX4_REQUIRED_COMMIT}) is required; found {commit} in {px4_dir}. "
            "Set XTDRONE_ALLOW_ANY_PX4=1 to skip this check.")
