"""Resolve convenient workspace destinations for shell directory changes."""

import argparse
import os
from pathlib import Path

from farol2_cli.commands.pkg import complete_packages, source_path
from farol2_cli.utils import die, find_workspace_root, workspace_root_or_die


def _source_directories(root: Path) -> list[Path]:
    src = root / "src"
    return sorted(path for path in src.iterdir() if path.is_dir())


def _directory_aliases(root: Path) -> dict[str, Path]:
    aliases: dict[str, Path] = {}
    for path in _source_directories(root):
        if path.name != "farol2" and not path.name.startswith("farol2_"):
            continue
        aliases[path.name] = path
        if path.name.startswith("farol2_"):
            aliases[path.name.removeprefix("farol2_")] = path
    return aliases


def complete_destinations(**_kwargs: object) -> list[str]:
    destinations = {"root"}
    packages = {
        package for package in complete_packages() if package.startswith("farol2_")
    }
    destinations.update(packages)
    destinations.update(package.removeprefix("farol2_") for package in packages)

    root = find_workspace_root()
    if root is not None:
        destinations.update(_directory_aliases(root))
    return sorted(destinations)


def resolve_destination(target: str | None) -> tuple[int, Path | None]:
    root = workspace_root_or_die()
    if root is None:
        return 1, None

    if target == "root":
        return 0, root

    aliases = _directory_aliases(root)
    if target is None:
        destination = aliases.get("farol2")
        if destination is None:
            return 1, None
        return 0, destination

    code, package_path = source_path(target)
    if code == 0:
        return 0, Path(package_path)

    destination = aliases.get(target)
    if destination is not None:
        return 0, destination

    code, package_path = source_path(f"farol2_{target}")
    if code == 0:
        return 0, Path(package_path)
    return 1, None


def cmd_cd(args: argparse.Namespace) -> int:
    code, destination = resolve_destination(args.target)
    if code or destination is None:
        return die(f"workspace package or folder not found: {args.target or 'farol2'}")
    print(destination)
    return 0


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "cd", help="change directory using the installed Bash integration"
    )
    target = parser.add_argument(
        "target",
        nargs="?",
        help="package, source-folder alias, or 'root' (default: farol2)",
    )
    # Older argcomplete versions lose custom completers while introspecting a
    # positional subcommand argument, but preserve dynamically supplied choices.
    if os.environ.get("_ARGCOMPLETE"):
        target.choices = complete_destinations()
    else:
        target.completer = complete_destinations
    parser.set_defaults(func=cmd_cd)
