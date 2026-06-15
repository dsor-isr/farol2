"""Careful ROS 2 process cleanup."""

import argparse
import os
import re
import signal
import subprocess
import time

from farol2_cli.utils import run


PATTERNS = (
    "ros2",
    "ros2 launch",
    "launch_ros",
    "rviz2",
    "rqt",
    "component_container",
    "robot_state_publisher",
    "joint_state_publisher",
    "/opt/ros/.*/lib/",
    "/install/.*/lib/",
)
EXCLUSIONS = (
    "plotjuggler",
    "PlotJuggler",
    "rqt",
    "foxglove_bridge",
    r"foxglove[-_ ]bridge",
    "bag_record",
)


def _cmdline(pid: int) -> str:
    try:
        return subprocess.run(
            ["ps", "-p", str(pid), "-o", "args="],
            check=False,
            capture_output=True,
            text=True,
        ).stdout
    except FileNotFoundError:
        return ""


def _matching_pids() -> list[int]:
    pids: set[int] = set()
    for pattern in PATTERNS:
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern], check=False, capture_output=True, text=True
            )
        except FileNotFoundError:
            return []
        for value in result.stdout.split():
            pid = int(value)
            if pid != os.getpid() and not any(re.search(pattern, _cmdline(pid)) for pattern in EXCLUSIONS):
                pids.add(pid)
    return sorted(pids)


def _alive(pids: list[int]) -> list[int]:
    alive = []
    for pid in pids:
        try:
            os.kill(pid, 0)
            alive.append(pid)
        except ProcessLookupError:
            pass
        except PermissionError:
            alive.append(pid)
    return alive


def _signal(pids: list[int], sig: signal.Signals) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def cmd_ros(_args: argparse.Namespace) -> int:
    pids = _matching_pids()
    if not pids:
        print("No ROS 2 processes found.")
        return 0

    print("Found ROS 2-related processes to kill:")
    subprocess.run(["ps", "-fp", *map(str, pids)], check=False)
    print("\nProtected process patterns:")
    for pattern in EXCLUSIONS:
        print(f"  - {pattern}")

    for sig, label in (
        (signal.SIGINT, "SIGINT"),
        (signal.SIGTERM, "SIGTERM"),
        (signal.SIGKILL, "SIGKILL"),
    ):
        pids = _alive(pids)
        if not pids:
            break
        print(f"Sending {label}...")
        _signal(pids, sig)
        if sig != signal.SIGKILL:
            time.sleep(2)

    print("Stopping ROS 2 daemon...")
    run(["ros2", "daemon", "stop"], echo=False)
    print("Done.")
    return 0


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("kill", help="stop ROS-related processes")
    parser.set_defaults(func=lambda _args: parser.print_help() or 0)
    commands = parser.add_subparsers(dest="kill_command", metavar="COMMAND")
    ros = commands.add_parser("ros", help="stop ROS 2 processes while preserving known tools")
    ros.set_defaults(func=cmd_ros)
