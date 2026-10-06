import os
import re
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


PX4_REQUIRED_COMMIT = "99c40407ffd7ac184e2d7b4b293f36f10fe561ef"


def launch_px4(context, *args, **kwargs):
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser()
    if not (px4_dir / "Makefile").is_file():
        raise RuntimeError(
            "px4_dir must point to a PX4-Autopilot source directory containing Makefile: "
            f"{px4_dir}. Initialize the repository submodule with "
            "'git submodule update --init --recursive PX4-Autopilot' or set PX4_DIR."
        )

    try:
        px4_commit = subprocess.run(
            ["git", "-C", str(px4_dir), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except FileNotFoundError as exc:
        raise RuntimeError("git is required to verify the pinned PX4 version") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"px4_dir must be the XTDrone PX4 v1.15.4 checkout at "
            f"{PX4_REQUIRED_COMMIT}: {px4_dir}"
        ) from exc
    if px4_commit != PX4_REQUIRED_COMMIT:
        raise RuntimeError(
            f"PX4 v1.15.4 at {PX4_REQUIRED_COMMIT} is required; "
            f"found {px4_commit} in {px4_dir}"
        )

    vehicle = LaunchConfiguration("vehicle").perform(context)
    if not re.fullmatch(r"[A-Za-z0-9_]+", vehicle):
        raise RuntimeError("vehicle must contain only letters, numbers, and underscores")

    port_text = LaunchConfiguration("agent_port").perform(context)
    try:
        agent_port = int(port_text)
    except ValueError as exc:
        raise RuntimeError("agent_port must be an integer from 1 to 65535") from exc
    if not 1 <= agent_port <= 65535:
        raise RuntimeError("agent_port must be an integer from 1 to 65535")

    start_agent = LaunchConfiguration("start_agent").perform(context).lower()
    if start_agent not in ("true", "false"):
        raise RuntimeError("start_agent must be 'true' or 'false'")

    actions = []
    if start_agent == "true":
        actions.append(
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", str(agent_port)],
                output="screen",
                emulate_tty=True,
            )
        )

    actions.append(
        ExecuteProcess(
            cmd=["make", "px4_sitl", f"gz_{vehicle}"],
            cwd=str(px4_dir),
            output="screen",
            emulate_tty=True,
        )
    )
    return actions


def generate_launch_description():
    default_px4_dir = os.environ.get("PX4_DIR")
    if default_px4_dir is None:
        xtdrone_root = os.environ.get("XTDRONE_ROOT")
        if xtdrone_root:
            default_px4_dir = str(Path(xtdrone_root).expanduser() / "PX4-Autopilot")
        else:
            starts = (Path.cwd(), Path(__file__).resolve().parent)
            for start in starts:
                for parent in (start, *start.parents):
                    candidate = parent / "PX4-Autopilot"
                    if (candidate / "Makefile").is_file():
                        default_px4_dir = str(candidate)
                        break
                if default_px4_dir is not None:
                    break
    if default_px4_dir is None:
        default_px4_dir = str(Path.cwd() / "PX4-Autopilot")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value=default_px4_dir,
                description="Path to the pinned in-repository PX4 v1.15.4 checkout.",
            ),
            DeclareLaunchArgument(
                "vehicle",
                default_value="x500",
                description="PX4 Gazebo vehicle target, for example x500.",
            ),
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description="Start the Micro XRCE-DDS Agent for PX4 ROS 2 topics.",
            ),
            DeclareLaunchArgument(
                "agent_port",
                default_value="8888",
                description="UDP port used by the Micro XRCE-DDS Agent.",
            ),
            OpaqueFunction(function=launch_px4),
        ]
    )
