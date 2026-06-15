"""ROS 2 bag commands."""

import argparse
import fnmatch
import json
from pathlib import Path
import re
import subprocess
import tempfile

from argcomplete.completers import DirectoriesCompleter

from farol2_cli.utils import die, print_command, run, strip_separator


START_PATTERN = re.compile(r"Start:.*\(([0-9]+(?:\.[0-9]+)?)\)")
TOPIC_PATTERN = re.compile(r"Topic: ([^ ]+)")


def _bag_info(path: str) -> tuple[int, str]:
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", path], check=False, capture_output=True, text=True
        )
    except FileNotFoundError:
        return 127, ""
    return result.returncode, result.stdout


def cmd_info(args: argparse.Namespace) -> int:
    return run(["ros2", "bag", "info", args.bag])


def cmd_play(args: argparse.Namespace) -> int:
    return run(["ros2", "bag", "play", args.bag, *strip_separator(args.ros_args)])


def cmd_crop(args: argparse.Namespace) -> int:
    bag = Path(args.bag)
    if not bag.exists():
        return die(f"input bag does not exist: {bag}")
    if args.t2 <= args.t1:
        return die("t2 must be greater than t1")
    if not args.all_topics and not args.topics:
        return die("specify topics/patterns or use --all for all topics")

    code, info = _bag_info(str(bag))
    if code:
        return die("could not read bag information", code)
    start_match = START_PATTERN.search(info)
    if not start_match:
        return die("could not extract bag start time from ros2 bag info")

    topics: list[str] = []
    if not args.all_topics:
        available = TOPIC_PATTERN.findall(info)
        for requested in args.topics:
            if any(character in requested for character in "*?["):
                topics.extend(fnmatch.filter(available, requested))
            else:
                topics.append(requested)
        topics = sorted(set(topics))
        if not topics:
            return die("no topics matched")

    output = args.output or f"{args.bag}_crop_{args.t1:g}_{args.t2:g}"
    start = float(start_match.group(1))
    start_ns = int((start + args.t1) * 1e9)
    end_ns = int((start + args.t2) * 1e9)

    print("Cropping bag:")
    print(f"  input:   {bag}")
    print(f"  output:  {output}")
    print(f"  storage: {args.storage}")
    print(f"  window:  {args.t1:g}s -> {args.t2:g}s relative to bag start")
    print("  topics:  all" if args.all_topics else "  topics:")
    for topic in topics:
        print(f"    {topic}")

    with tempfile.TemporaryDirectory(prefix="farol_bag_crop_") as temp_dir:
        config = Path(temp_dir) / "output.yaml"
        lines = [
            "output_bags:",
            f"- uri: {json.dumps(output)}",
            f"  storage_id: {args.storage}",
            f"  start_time_ns: {start_ns}",
            f"  end_time_ns: {end_ns}",
        ]
        if args.all_topics:
            lines.extend(("  all_topics: true", "  all_services: true", "  all_actions: true"))
        else:
            lines.append("  topics:")
            lines.extend(f"  - {json.dumps(topic)}" for topic in topics)
        config.write_text("\n".join(lines) + "\n", encoding="utf-8")
        cmd = ["ros2", "bag", "convert", "-i", str(bag), args.storage, "-o", str(config)]
        print_command(cmd)
        try:
            return subprocess.run(cmd, check=False).returncode
        except FileNotFoundError:
            return die("required executable not found: ros2", 127)


def add_parser(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("bag", help="ROS 2 bag commands")
    parser.set_defaults(func=lambda _args: parser.print_help() or 0)
    commands = parser.add_subparsers(dest="bag_command", metavar="COMMAND")

    info = commands.add_parser("info", help="show bag information")
    info_bag = info.add_argument("bag")
    info_bag.completer = DirectoriesCompleter()
    info.set_defaults(func=cmd_info)

    play = commands.add_parser("play", help="play a bag")
    play_bag = play.add_argument("bag")
    play_bag.completer = DirectoriesCompleter()
    play.add_argument("ros_args", nargs=argparse.REMAINDER, help="arguments after -- go to ros2 bag play")
    play.set_defaults(func=cmd_play)

    crop = commands.add_parser("crop", help="crop a bag using times relative to its start")
    crop_bag = crop.add_argument("bag")
    crop_bag.completer = DirectoriesCompleter()
    crop.add_argument("t1", type=float, help="crop start in seconds relative to bag start")
    crop.add_argument("t2", type=float, help="crop end in seconds relative to bag start")
    crop.add_argument("topics", nargs="*", help="topics or wildcard topic patterns")
    crop.add_argument("-a", "--all", dest="all_topics", action="store_true")
    crop.add_argument("-o", "--output")
    crop.add_argument("--storage", default="mcap")
    crop.set_defaults(func=cmd_crop)
