"""Run an ArduPilot SITL binary with a stdin that stays open.

ArduPilot SITL quits when its standard input reaches end-of-file, which is what it gets from a launch file or a
background job. This wrapper starts it with a pipe that stays open and passes termination on to it.

  ros2 run xtdrone_bringup sitl_run <arducopter> [SITL arguments]
"""
import os
import signal
import subprocess
import sys


def main():
    cmd = sys.argv[1:]
    if os.environ.get("XTDRONE_SITL_GDB"):  # a backtrace of a crash (SIGFPE...) in the log
        cmd = ["gdb", "-batch", "-ex", "run", "-ex", "bt", "--args"] + cmd
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def stop(signum, _frame):
        proc.send_signal(signal.SIGTERM)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        sys.exit(proc.wait())
    finally:
        if proc.poll() is None:
            proc.kill()


if __name__ == "__main__":
    main()
