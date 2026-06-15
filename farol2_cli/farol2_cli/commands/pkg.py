"""ROS package path commands."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

from farol2_cli.utils import die, find_workspace_root, workspace_root_or_die


def _subprocess_env() -> dict[str, str]:
    env = os.environ.copy()
    for name in list(env):
        if name.startswith("_ARGCOMPLETE") or name in {
            "COMP_LINE",
            "COMP_POINT",
            "COMP_TYPE",
            "COMP_WORDBREAKS",
        }:
            env.pop(name)
    return env


def _run_colcon(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    env = _subprocess_env()
    with tempfile.TemporaryDirectory(prefix="farol_colcon_log_") as log_dir:
        env["COLCON_LOG_PATH"] = log_dir
        return subprocess.run(
            cmd, cwd=cwd, env=env, check=False, capture_output=True, text=True
        )


def complete_packages(**_kwargs: object) -> list[str]:
    names: set[str] = set()
    root = find_workspace_root()
    if root is not None:
        try:
            result = _run_colcon(["colcon", "list", "--names-only"], root)
            if result.returncode == 0:
                names.update(result.stdout.split())
        except FileNotFoundError:
            pass
    try:
        result = subprocess.run(
            ["ros2", "pkg", "list"],
            env=_subprocess_env(),
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            names.update(result.stdout.split())
    except FileNotFoundError:
        pass
    return sorted(names)


def source_path(package: str) -> tuple[int, str]:
    root = workspace_root_or_die()
    if root is None:
        return 1, ""
    try:
        result = _run_colcon(
            ["colcon", "list", "--paths-only", "--packages-select", package], root
        )
    except FileNotFoundError:
        return 127, ""
    path = result.stdout.strip().splitlines()
    if result.returncode != 0 or not path:
        return result.returncode or 1, ""
    selected = Path(path[0])
    return 0, str(selected if selected.is_absolute() else (root / selected).resolve())


def cmd_src(args: argparse.Namespace) -> int:
    code, path = source_path(args.package)
    if code:
        return die(f"workspace package not found: {args.package}", code)
    print(path)
    return 0


def cmd_cd_src(args: argparse.Namespace) -> int:
    code, path = source_path(args.package)
    if code:
        return die(f"workspace package not found: {args.package}", code)
    print(f"cd -- {shlex.quote(path)}")
    return 0


def cmd_share(args: argparse.Namespace) -> int:
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", args.package],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return die("required executable not found: ros2", 127)
    if result.returncode != 0 or not result.stdout.strip():
        return die(f"installed ROS package not found: {args.package}", result.returncode or 1)
    print(Path(result.stdout.strip()) / "share" / args.package)
    return 0


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("pkg", help="ROS package path commands")
    parser.set_defaults(func=lambda _args: parser.print_help() or 0)
    commands = parser.add_subparsers(dest="pkg_command", metavar="COMMAND")
    for name, help_text, handler in (
        ("src", "print a workspace package source path", cmd_src),
        ("share", "print an installed package share path", cmd_share),
        ("cd-src", "print a shell command that changes to a package source path", cmd_cd_src),
    ):
        command = commands.add_parser(name, help=help_text)
        package = command.add_argument("package")
        package.completer = complete_packages
        command.set_defaults(func=handler)
