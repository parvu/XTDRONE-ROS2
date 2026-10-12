"""Small Gazebo helpers."""
import subprocess


def running_world():
    """Name of the running gz world, from its /world/<name>/clock topic."""
    out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, check=False, timeout=20).stdout
    for line in out.splitlines():
        if line.startswith("/world/") and line.endswith("/clock"):
            return line.split("/")[2]
    raise RuntimeError("no running Gazebo world found (gz topic -l shows no /world/<name>/clock)")
