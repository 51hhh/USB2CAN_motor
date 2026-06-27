#!/usr/bin/env python3
"""Record configured ROS 2 topics into a rosbag."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List

import yaml


PACKAGE_NAME = "rviz_bag_tools"


def default_config_path() -> Path:
    script = Path(__file__).resolve()
    source_cfg = script.parents[1] / "config" / "chassis_bag.yaml"
    if source_cfg.exists():
        return source_cfg
    install_cfg = script.parents[2] / "share" / PACKAGE_NAME / "config" / "chassis_bag.yaml"
    return install_cfg


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"config not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"config must be a YAML mapping: {path}")
    return data


def normalize_topic_name(name: str) -> str:
    name = str(name).strip()
    if not name:
        raise ValueError("empty topic name")
    return name if name.startswith("/") else f"/{name}"


def configured_topics(config: Dict[str, Any]) -> List[str]:
    record = config.get("record", {}) or {}
    topics_cfg = record.get("topics", []) or []
    topics: List[str] = []

    for item in topics_cfg:
        if isinstance(item, str):
            topics.append(normalize_topic_name(item))
            continue
        if not isinstance(item, dict):
            raise ValueError(f"invalid topic entry: {item!r}")
        enabled = item.get("enabled", True) and item.get("record", True)
        if not enabled:
            continue
        name = item.get("name") or item.get("source") or item.get("topic")
        if not name:
            raise ValueError(f"topic entry missing name/source/topic: {item!r}")
        topics.append(normalize_topic_name(name))

    # Preserve order while removing duplicates.
    seen = set()
    unique = []
    for topic in topics:
        if topic not in seen:
            unique.append(topic)
            seen.add(topic)
    return unique


def build_output_path(config: Dict[str, Any], args: argparse.Namespace) -> Path:
    if args.output:
        return Path(args.output).expanduser()

    record = config.get("record", {}) or {}
    output_root = Path(str(record.get("output_root", "bags"))).expanduser()
    if not output_root.is_absolute():
        output_root = Path.cwd() / output_root
    name = args.name or time.strftime("bag_%Y%m%d_%H%M%S")
    return output_root / name


def build_command(config: Dict[str, Any], args: argparse.Namespace) -> List[str]:
    record = config.get("record", {}) or {}
    output_path = build_output_path(config, args)
    storage_id = args.storage or str(record.get("storage_id", "sqlite3"))
    include_hidden = bool(record.get("include_hidden_topics", False))
    record_all = bool(record.get("all", False)) or args.all

    cmd = ["ros2", "bag", "record", "-o", str(output_path)]

    if storage_id:
        cmd.extend(["--storage", storage_id])
    if include_hidden or args.include_hidden_topics:
        cmd.append("--include-hidden-topics")

    if args.max_bag_size:
        cmd.extend(["--max-bag-size", str(args.max_bag_size)])
    if args.max_bag_duration:
        cmd.extend(["--max-bag-duration", str(args.max_bag_duration)])

    if record_all:
        cmd.append("--all")
    else:
        topics = configured_topics(config)
        topics.extend(normalize_topic_name(t) for t in args.topic)
        if not topics:
            raise ValueError("no topics configured for recording")
        cmd.extend(topics)

    return cmd


def run_with_optional_timeout(cmd: List[str], timeout_s: float) -> int:
    proc = subprocess.Popen(cmd)
    try:
        proc.wait(timeout=timeout_s if timeout_s and timeout_s > 0 else None)
        return proc.returncode
    except subprocess.TimeoutExpired:
        print(f"\nrecord duration reached: {timeout_s:.1f}s, stopping rosbag...")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        print("\nCtrl-C received, stopping rosbag...")
        proc.send_signal(signal.SIGINT)
        proc.wait()
        return proc.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Record YAML-configured topics to rosbag")
    parser.add_argument("--config", "-c", default=str(default_config_path()), help="YAML config path")
    parser.add_argument("--output", "-o", default=None, help="Output bag directory")
    parser.add_argument("--name", default=None, help="Bag directory name under record.output_root")
    parser.add_argument("--duration", "-d", type=float, default=0.0, help="Recording duration in seconds")
    parser.add_argument("--topic", "-t", action="append", default=[], help="Extra topic to record")
    parser.add_argument("--all", action="store_true", help="Record all topics")
    parser.add_argument("--storage", default=None, help="rosbag storage id, default from config")
    parser.add_argument("--include-hidden-topics", action="store_true", help="Record hidden topics")
    parser.add_argument("--max-bag-size", type=int, default=0, help="ros2 bag --max-bag-size")
    parser.add_argument("--max-bag-duration", type=int, default=0, help="ros2 bag --max-bag-duration")
    parser.add_argument("--dry-run", action="store_true", help="Print command without running it")
    args = parser.parse_args()

    try:
        config = load_yaml(Path(args.config).expanduser())
        cmd = build_command(config, args)
    except Exception as exc:
        print(f"bag_record: {exc}", file=sys.stderr)
        return 2

    print("rosbag record command:")
    print("  " + " ".join(cmd))
    if args.dry_run:
        return 0

    return run_with_optional_timeout(cmd, args.duration)


if __name__ == "__main__":
    sys.exit(main())
