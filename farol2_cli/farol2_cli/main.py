#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import argparse

import argcomplete

from farol2_cli.commands import bag, cd, kill, pkg, serial, ws


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="farol2", description="FAROL2 development and ROS 2 helpers"
    )
    subparsers = parser.add_subparsers(dest="command", metavar="COMMAND")
    for module in (ws, bag, pkg, cd, serial, kill):
        module.add_parser(subparsers)
    return parser


def main() -> int:
    parser = build_parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 0

    try:
        result = args.func(args)
        return 0 if result is None else result
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
