"""Serial-port helpers."""

import argparse
import glob

from farol2_cli.utils import die, run


def complete_devices(prefix: str, **_kwargs: object) -> list[str]:
    patterns = ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyAMA*", "/dev/serial/by-id/*")
    return sorted(path for pattern in patterns for path in glob.glob(pattern) if path.startswith(prefix))


def cmd_bridge(args: argparse.Namespace) -> int:
    if not glob.glob(args.device):
        return die(f"serial device does not exist: {args.device}")
    return run(
        [
            "socat",
            "-d",
            "-d",
            f"pty,raw,echo=0,link={args.link}",
            f"{args.device},b{args.baud},raw,echo=0",
        ]
    )


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("serial", help="serial-port helpers")
    parser.set_defaults(func=lambda _args: parser.print_help() or 0)
    commands = parser.add_subparsers(dest="serial_command", metavar="COMMAND")
    bridge = commands.add_parser("bridge", help="bridge a real device to a virtual serial port")
    device = bridge.add_argument("device")
    device.completer = complete_devices
    bridge.add_argument("--link", default="/tmp/mcu")
    bridge.add_argument("--baud", type=int, default=115200)
    bridge.set_defaults(func=cmd_bridge)
