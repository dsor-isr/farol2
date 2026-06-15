"""Shared command-line utilities."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Optional, Sequence


def print_command(cmd: Sequence[str]) -> None:
    print("+ " + shlex.join(str(part) for part in cmd))


def run(cmd: list[str], *, cwd: Optional[Path] = None, echo: bool = True) -> int:
    if echo:
        print_command(cmd)
    try:
        return subprocess.run(cmd, cwd=cwd, check=False).returncode
    except FileNotFoundError:
        return die(f"required executable not found: {cmd[0]}", 127)


def run_check(cmd: list[str], *, cwd: Optional[Path] = None) -> None:
    print_command(cmd)
    subprocess.run(cmd, cwd=cwd, check=True)


def die(message: str, code: int = 1) -> int:
    print(f"Error: {message}", file=sys.stderr)
    return code


def require_executable(name: str) -> bool:
    if shutil.which(name):
        return True
    die(f"required executable not found: {name}", 127)
    return False


def looks_like_workspace(root: Path) -> bool:
    src = root / "src"
    if not src.is_dir():
        return False
    return any(src.glob("*/package.xml")) or any(src.glob("*/*/package.xml"))


def find_workspace_root(start: Optional[Path] = None) -> Optional[Path]:
    configured = os.environ.get("COLCON_ROOT")
    if configured:
        root = Path(configured).expanduser().resolve()
        if looks_like_workspace(root):
            return root

    current = (start or Path.cwd()).resolve()
    for candidate in (current, *current.parents):
        if looks_like_workspace(candidate):
            return candidate
        if candidate.name == "src" and looks_like_workspace(candidate.parent):
            return candidate.parent
    return None


def workspace_root_or_die() -> Optional[Path]:
    root = find_workspace_root()
    if root is None:
        die("could not find a colcon workspace root; run inside one or set COLCON_ROOT")
    return root


def strip_separator(values: list[str]) -> list[str]:
    return values[1:] if values and values[0] == "--" else values
