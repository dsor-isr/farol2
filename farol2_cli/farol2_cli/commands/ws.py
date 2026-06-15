"""Colcon workspace commands."""

import argparse
import os
from pathlib import Path
import shutil

from farol2_cli.utils import die, run, strip_separator, workspace_root_or_die

CLI_PACKAGE = "farol2_cli"


def _default_cores() -> int:
    total = os.cpu_count() or 1
    if total > 4:
        return total - 2
    return max(1, total - 1)


def cmd_build(args: argparse.Namespace) -> int:
    root = workspace_root_or_die()
    if root is None:
        return 1

    cmd = [
        "colcon",
        "build",
        "--symlink-install",
        "--parallel-workers",
        str(args.cores or _default_cores()),
    ]
    if args.packages:
        cmd.extend(["--packages-select", *args.packages])
    cmd.extend(["--cmake-args", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"])
    if args.release:
        cmd.append("-DCMAKE_BUILD_TYPE=Release")
    cmd.extend(strip_separator(args.colcon_args))
    print(f"Building in: {root}")
    return run(cmd, cwd=root)


def _clean_targets(root: Path) -> tuple[list[Path], list[Path]]:
    targets: list[Path] = []
    preserved: list[Path] = []

    build = root / "build"
    if build.is_dir():
        for path in build.iterdir():
            if path.name == CLI_PACKAGE:
                preserved.append(path)
            else:
                targets.append(path)

    install = root / "install"
    if install.is_dir():
        installed_cli = install / CLI_PACKAGE
        if not installed_cli.is_dir():
            raise RuntimeError(
                f"{installed_cli} does not exist; refusing to clean an install "
                "space where farol2_cli cannot be preserved"
            )
        preserved.append(installed_cli)
        for path in install.iterdir():
            if path.name == CLI_PACKAGE:
                continue
            # Keep workspace setup scripts so future shells can still source
            # the preserved farol2_cli package after cleaning.
            if path.is_dir() and not path.is_symlink():
                targets.append(path)

    log = root / "log"
    if log.exists() or log.is_symlink():
        targets.append(log)

    return sorted(targets), sorted(preserved)


def _remove_path(path: Path) -> None:
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink()


def cmd_clean(args: argparse.Namespace) -> int:
    root = workspace_root_or_die()
    if root is None:
        return 1
    if root in {Path("/"), Path.home(), Path("/home"), Path("/usr"), Path("/opt"), Path("/tmp")}:
        return die(f"refusing to clean dangerous workspace root: {root}")

    try:
        targets, preserved = _clean_targets(root)
    except RuntimeError as error:
        return die(str(error))

    if not targets:
        print("No build/install/log artifacts found to remove.")
        return 0

    print("The following build/install/log artifacts will be removed:")
    for target in targets:
        print(f"  {target}")
    if preserved:
        print("\nThe FAROL2 CLI will be preserved:")
        for path in preserved:
            print(f"  {path}")
    if not args.yes and input("Are you sure? [y/N] ").strip().lower() not in {"y", "yes"}:
        print("Aborted.")
        return 1

    for target in targets:
        print(f"Removing: {target}")
        _remove_path(target)
    print("Done.")
    return 0


def cmd_root(_args: argparse.Namespace) -> int:
    root = workspace_root_or_die()
    if root is None:
        return 1
    print(root)
    return 0


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("ws", help="colcon workspace commands")
    parser.set_defaults(func=lambda _args: parser.print_help() or 0)
    commands = parser.add_subparsers(dest="ws_command", metavar="COMMAND")

    build = commands.add_parser("build", help="build the workspace")
    build.add_argument("-p", "--packages", nargs="+", metavar="PACKAGE")
    build.add_argument("-j", "--cores", type=int, metavar="N")
    build.add_argument("--release", action="store_true")
    build.add_argument("colcon_args", nargs=argparse.REMAINDER, help="arguments after -- go to colcon")
    build.set_defaults(func=cmd_build)

    clean = commands.add_parser(
        "clean", help="remove workspace artifacts while preserving farol2_cli"
    )
    clean.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")
    clean.set_defaults(func=cmd_clean)

    root = commands.add_parser("root", help="print the detected workspace root")
    root.set_defaults(func=cmd_root)
