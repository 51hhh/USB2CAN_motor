#!/usr/bin/env python3
"""Interactive rosbag player with safe topic remapping for RViz replay."""

import argparse
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
import select
import sys
import termios
import time
import tty
from typing import Any, Dict, List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from tf2_ros import TransformBroadcaster


PACKAGE_NAME = "rviz_bag_tools"


@dataclass
class BagEvent:
    topic: str
    type_name: str
    timestamp_ns: int


@dataclass
class TopicRule:
    source: str
    target: str
    type_name: str
    frame_id: Optional[str] = None
    child_frame_id: Optional[str] = None
    generate_path: bool = False
    path_topic: Optional[str] = None
    max_path_poses: int = 5000


@dataclass
class BagAccess:
    bag_path: Path
    storage_id: str
    events: List[BagEvent]
    allowed_topics: set
    reader: Optional[Any] = None
    next_index: int = 0
    has_seek: bool = False


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


def topic_name(name: str) -> str:
    name = str(name).strip()
    if not name:
        raise ValueError("empty topic name")
    return name if name.startswith("/") else f"/{name}"


def prefixed_topic(prefix: str, source: str) -> str:
    prefix = topic_name(prefix or "/replay").rstrip("/")
    return f"{prefix}{topic_name(source)}"


def time_from_ns(ns: int) -> Time:
    msg = Time()
    msg.sec = int(ns // 1_000_000_000)
    msg.nanosec = int(ns % 1_000_000_000)
    return msg


def make_reader(bag_path: Path, storage_id: str) -> Any:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def get_storage_id(config: Dict[str, Any]) -> str:
    replay_cfg = config.get("replay", {}) or {}
    record_cfg = config.get("record", {}) or {}
    return str(replay_cfg.get("storage_id") or record_cfg.get("storage_id") or "sqlite3")


def read_bag_index(bag_path: Path, config: Dict[str, Any]) -> Tuple[List[BagEvent], Dict[str, str], str]:
    storage_id = get_storage_id(config)
    reader = make_reader(bag_path, storage_id)
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    rules = build_topic_rules(config, topic_types)
    allowed = set(rules.keys())

    events: List[BagEvent] = []
    while reader.has_next():
        topic, _data, timestamp_ns = reader.read_next()
        if topic not in allowed:
            continue
        events.append(BagEvent(topic, topic_types[topic], int(timestamp_ns)))

    events.sort(key=lambda e: e.timestamp_ns)
    return events, topic_types, storage_id


def lower_bound_event_index(events: List[BagEvent], timestamp_ns: int) -> int:
    timestamps = [event.timestamp_ns for event in events]
    return bisect_left(timestamps, timestamp_ns)


class BagDataSource:
    def __init__(self, bag_path: Path, storage_id: str, events: List[BagEvent]):
        self.access = BagAccess(
            bag_path=bag_path,
            storage_id=storage_id,
            events=events,
            allowed_topics={event.topic for event in events},
        )
        self._open_reader()

    def _open_reader(self) -> None:
        self.access.reader = make_reader(self.access.bag_path, self.access.storage_id)
        self.access.next_index = 0
        self.access.has_seek = hasattr(self.access.reader, "seek")

    def _seek_to_index(self, index: int) -> None:
        if index <= 0:
            self._open_reader()
            return

        event = self.access.events[index]
        if self.access.has_seek and self.access.reader is not None:
            self.access.reader.seek(event.timestamp_ns)
            self.access.next_index = lower_bound_event_index(self.access.events, event.timestamp_ns)
        else:
            self._open_reader()

    def read_serialized(self, index: int) -> Optional[bytes]:
        if index < 0 or index >= len(self.access.events):
            return None
        if self.access.reader is None or index < self.access.next_index:
            self._seek_to_index(index)

        reader = self.access.reader
        assert reader is not None

        while reader.has_next() and self.access.next_index <= index:
            topic, data, timestamp_ns = reader.read_next()
            if topic not in self.access.allowed_topics:
                continue

            # The index was created from the same bag order. If seek lands on a
            # timestamp, align the index to the first selected event at or after
            # that timestamp.
            while (
                self.access.next_index < len(self.access.events)
                and self.access.events[self.access.next_index].timestamp_ns < int(timestamp_ns)
            ):
                self.access.next_index += 1

            if self.access.next_index >= len(self.access.events):
                return None

            expected = self.access.events[self.access.next_index]
            if expected.topic != topic or expected.timestamp_ns != int(timestamp_ns):
                # Rare but possible if storage ordering differs after seek.
                match_index = self._find_matching_index(topic, int(timestamp_ns), self.access.next_index)
                if match_index is None:
                    continue
                self.access.next_index = match_index
                expected = self.access.events[self.access.next_index]

            if self.access.next_index == index and expected.topic == topic:
                self.access.next_index += 1
                return data

            self.access.next_index += 1

        return None

    def _find_matching_index(self, topic: str, timestamp_ns: int, start: int) -> Optional[int]:
        end = min(len(self.access.events), start + 64)
        for i in range(start, end):
            event = self.access.events[i]
            if event.timestamp_ns != timestamp_ns:
                if event.timestamp_ns > timestamp_ns:
                    return None
                continue
            if event.topic == topic:
                return i
        return None


def build_topic_rules(config: Dict[str, Any], bag_topic_types: Dict[str, str]) -> Dict[str, TopicRule]:
    replay_cfg = config.get("replay", {}) or {}
    prefix = replay_cfg.get("remap_prefix", "/replay")
    skip_topics = set(topic_name(t) for t in replay_cfg.get("skip_topics", []) or [])
    topics_cfg = replay_cfg.get("topics", []) or []

    rules: Dict[str, TopicRule] = {}

    if topics_cfg:
        for item in topics_cfg:
            if isinstance(item, str):
                item = {"source": item}
            if not isinstance(item, dict):
                raise ValueError(f"invalid replay topic entry: {item!r}")
            if not item.get("enabled", True):
                continue
            source = topic_name(item.get("source") or item.get("name") or item.get("topic"))
            if source in skip_topics:
                continue
            if source not in bag_topic_types:
                print(f"warn: configured replay topic not in bag: {source}")
                continue
            target = topic_name(item.get("target") or prefixed_topic(prefix, source))
            type_name = item.get("type") or bag_topic_types[source]
            rules[source] = TopicRule(
                source=source,
                target=target,
                type_name=type_name,
                frame_id=item.get("frame_id"),
                child_frame_id=item.get("child_frame_id"),
                generate_path=bool(item.get("generate_path", False)),
                path_topic=topic_name(item["path_topic"]) if item.get("path_topic") else None,
                max_path_poses=int(item.get("max_path_poses", 5000)),
            )
    else:
        for source, type_name in bag_topic_types.items():
            if source in skip_topics:
                continue
            rules[source] = TopicRule(
                source=source,
                target=prefixed_topic(prefix, source),
                type_name=type_name,
            )

    return rules


class InteractiveBagPlayer(Node):
    def __init__(
        self,
        events: List[BagEvent],
        data_source: BagDataSource,
        rules: Dict[str, TopicRule],
        speed: float,
        loop: bool,
        stamp_policy: str,
        publish_tf_from_odom: bool,
        start_paused: bool,
        step_mode: str,
        step_topic: str,
        step_time_sec: float,
    ):
        super().__init__("interactive_bag_player")
        self.events = events
        self.event_timestamps = [event.timestamp_ns for event in events]
        self.data_source = data_source
        self.rules = rules
        self.speed = max(speed, 0.01)
        self.loop = loop
        self.stamp_policy = stamp_policy
        self.publish_tf_from_odom = publish_tf_from_odom
        self.paused = start_paused
        self.idx = 0
        self.current_index = -1
        self.last_wall_time = time.monotonic()
        self.step_mode = step_mode
        self.step_topic = topic_name(step_topic)
        self.step_time_sec = max(step_time_sec, 0.001)
        self.path_poses: Dict[str, List[PoseStamped]] = {}
        self.old_terminal_settings = None

        self.publishers = {}
        self.path_publishers = {}
        self.msg_types = {}
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for rule in self.rules.values():
            try:
                msg_type = get_message(rule.type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                print(f"warn: cannot load message type {rule.type_name} for {rule.source}: {exc}")
                continue
            self.msg_types[rule.source] = msg_type
            self.publishers[rule.source] = self.create_publisher(msg_type, rule.target, qos)
            if rule.generate_path:
                path_topic = rule.path_topic or prefixed_topic("/replay", f"{rule.source}/path")
                self.path_publishers[rule.source] = self.create_publisher(PathMsg, path_topic, qos)
                self.path_poses[rule.source] = []

        self.stdin_is_tty = sys.stdin.isatty()
        if self.stdin_is_tty:
            self.old_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def destroy_node(self):
        if self.old_terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        super().destroy_node()

    def _stamp_for_event(self, event: BagEvent, msg: Any) -> Optional[Time]:
        if self.stamp_policy == "now":
            return self.get_clock().now().to_msg()
        if self.stamp_policy == "bag":
            return time_from_ns(event.timestamp_ns)
        if self.stamp_policy == "preserve":
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                return msg.header.stamp
            return time_from_ns(event.timestamp_ns)
        return self.get_clock().now().to_msg()

    def _apply_header_policy(self, event: BagEvent, msg: Any, rule: TopicRule) -> Optional[Time]:
        stamp = self._stamp_for_event(event, msg)
        if hasattr(msg, "header"):
            if hasattr(msg.header, "stamp") and stamp is not None and self.stamp_policy != "preserve":
                msg.header.stamp = stamp
            if hasattr(msg.header, "frame_id") and rule.frame_id:
                msg.header.frame_id = rule.frame_id
        if isinstance(msg, Odometry) and rule.child_frame_id:
            msg.child_frame_id = rule.child_frame_id
        return stamp

    def _deserialize_event(self, index: int) -> Optional[Any]:
        event = self.events[index]
        msg_type = self.msg_types.get(event.topic)
        if msg_type is None:
            return None
        data = self.data_source.read_serialized(index)
        if data is None:
            print(f"\nwarn: failed to read bag event {index} {event.topic}")
            return None
        try:
            return deserialize_message(data, msg_type)
        except Exception as exc:
            print(f"\nwarn: failed to deserialize {event.topic}: {exc}")
            return None

    def _pose_from_odom(self, odom: Odometry) -> PoseStamped:
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        return pose

    def _publish_generated_path(self, source: str, stamp: Optional[Time]) -> None:
        if source not in self.path_publishers:
            return
        rule = self.rules[source]
        poses = self.path_poses.get(source, [])
        path = PathMsg()
        if poses:
            path.header = poses[-1].header
        else:
            path.header.frame_id = rule.frame_id or "replay_odom"
            if stamp is not None:
                path.header.stamp = stamp
        path.poses = poses
        self.path_publishers[source].publish(path)

    def _append_path_pose(self, source: str, odom: Odometry) -> None:
        rule = self.rules[source]
        poses = self.path_poses.setdefault(source, [])
        poses.append(self._pose_from_odom(odom))
        max_len = max(1, int(rule.max_path_poses))
        if len(poses) > max_len:
            del poses[: len(poses) - max_len]

    def _publish_tf_from_odom(self, odom: Odometry) -> None:
        if not self.publish_tf_from_odom:
            return
        if not odom.header.frame_id or not odom.child_frame_id:
            return
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def publish_event(self, index: int, update_state: bool = True) -> None:
        if index < 0 or index >= len(self.events):
            return
        event = self.events[index]
        rule = self.rules.get(event.topic)
        pub = self.publishers.get(event.topic)
        if rule is None or pub is None:
            return
        msg = self._deserialize_event(index)
        if msg is None:
            return
        stamp = self._apply_header_policy(event, msg, rule)
        pub.publish(msg)
        self.current_index = index

        if isinstance(msg, Odometry):
            if update_state and rule.generate_path:
                self._append_path_pose(event.topic, msg)
            self._publish_generated_path(event.topic, stamp)
            self._publish_tf_from_odom(msg)

    def rebuild_state_before(self, next_index: int) -> None:
        for source in list(self.path_poses.keys()):
            self.path_poses[source] = []
        upper = max(0, min(next_index, len(self.events)))
        for i in range(upper):
            event = self.events[i]
            rule = self.rules.get(event.topic)
            if not rule or not rule.generate_path:
                continue
            msg = self._deserialize_event(i)
            if isinstance(msg, Odometry):
                self._apply_header_policy(event, msg, rule)
                self._append_path_pose(event.topic, msg)

    def get_key(self) -> Optional[str]:
        if not self.stdin_is_tty:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == "\x1b":
                ch2 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ""
                ch3 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ""
                if ch2 == "[":
                    if ch3 == "C":
                        return "RIGHT"
                    if ch3 == "D":
                        return "LEFT"
                return None
            return ch
        return None

    def seek_to(self, index: int) -> None:
        if not self.events:
            return
        index = max(0, min(index, len(self.events) - 1))
        self.rebuild_state_before(index)
        self.publish_event(index, update_state=True)
        self.idx = min(index + 1, len(self.events))
        self.last_wall_time = time.monotonic()

    def seek_step(self, delta: int) -> None:
        if not self.events:
            return
        if self.step_mode == "topic":
            self.seek_to(self._topic_step_index(delta))
        elif self.step_mode == "time":
            self.seek_to(self._time_step_index(delta))
        else:
            self.seek_to((self.current_index if self.current_index >= 0 else -1) + delta)

    def _topic_step_index(self, delta: int) -> int:
        topic = self.step_topic
        if topic not in self.rules:
            return (self.current_index if self.current_index >= 0 else -1) + delta

        current = self.current_index if self.current_index >= 0 else -1
        if delta >= 0:
            found = current
            count = 0
            while found + 1 < len(self.events):
                found += 1
                if self.events[found].topic == topic:
                    count += 1
                    if count >= delta:
                        return found
            return len(self.events) - 1

        found = current
        count = 0
        while found - 1 >= 0:
            found -= 1
            if self.events[found].topic == topic:
                count -= 1
                if count <= delta:
                    return found
        return 0

    def _time_step_index(self, delta: int) -> int:
        current = self.current_index if self.current_index >= 0 else 0
        current = max(0, min(current, len(self.events) - 1))
        target_ns = self.events[current].timestamp_ns + int(delta * self.step_time_sec * 1_000_000_000)
        index = bisect_left(self.event_timestamps, target_ns)
        return max(0, min(index, len(self.events) - 1))

    def handle_input(self) -> bool:
        key = self.get_key()
        if key is None:
            return True
        if key == "q":
            return False
        if key == " ":
            self.paused = not self.paused
            self.last_wall_time = time.monotonic()
        elif key in ("RIGHT", "d"):
            self.seek_step(1)
        elif key in ("LEFT", "a"):
            self.seek_step(-1)
        elif key in (".", "w"):
            self.seek_step(10)
        elif key in (",", "s"):
            self.seek_step(-10)
        elif key in ("+", "="):
            self.speed = min(self.speed * 2.0, 64.0)
        elif key == "-":
            self.speed = max(self.speed / 2.0, 0.03125)
        elif key == "r":
            self.idx = 0
            self.current_index = -1
            self.rebuild_state_before(0)
            self.last_wall_time = time.monotonic()
        return True

    def _wait_for_next_event(self) -> bool:
        if self.idx <= 0:
            return True
        dt_ns = self.events[self.idx].timestamp_ns - self.events[self.idx - 1].timestamp_ns
        wait_s = max(0.0, dt_ns / 1_000_000_000.0 / self.speed)
        elapsed = time.monotonic() - self.last_wall_time
        return elapsed >= wait_s

    def status_line(self) -> str:
        total = len(self.events)
        if total == 0:
            return "no events"
        display_idx = self.current_index if self.current_index >= 0 else min(max(self.idx, 0), total - 1)
        event = self.events[display_idx]
        state = "PAUSE" if self.paused else "PLAY "
        rel_s = (event.timestamp_ns - self.events[0].timestamp_ns) / 1_000_000_000.0
        return (
            f"{state} [{display_idx + 1:>6}/{total}] {self.speed:>6.2f}x "
            f"step={self.step_mode} t={rel_s:>8.3f}s "
            f"{event.topic} -> {self.rules[event.topic].target}"
        )

    def run(self) -> None:
        if not self.events:
            print("no configured events found in bag")
            return

        print("")
        print("=" * 72)
        print(f"interactive bag replay: {len(self.events)} events")
        print("Space pause/resume | d/right +1 | a/left -1 | w/. +10 | s/, -10")
        print("+/- speed | r restart | q quit")
        print("=" * 72)

        try:
            while rclpy.ok():
                if not self.handle_input():
                    break

                if self.paused:
                    sys.stdout.write("\r" + self.status_line() + " " * 12)
                    sys.stdout.flush()
                    time.sleep(0.03)
                    continue

                if self.idx >= len(self.events):
                    if self.loop:
                        self.idx = 0
                        self.current_index = -1
                        self.rebuild_state_before(0)
                        self.last_wall_time = time.monotonic()
                        print("\nloop restart")
                    else:
                        self.paused = True
                        print("\nreplay complete, press r to restart or q to quit")
                    continue

                if self._wait_for_next_event():
                    self.publish_event(self.idx, update_state=True)
                    self.last_wall_time = time.monotonic()
                    self.idx += 1
                    sys.stdout.write("\r" + self.status_line() + " " * 12)
                    sys.stdout.flush()

                time.sleep(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            print("\nstop replay")


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive rosbag replay for RViz")
    parser.add_argument("--bag", "-b", required=True, help="Bag directory")
    parser.add_argument("--config", "-c", default=str(default_config_path()), help="YAML config path")
    parser.add_argument("--speed", type=float, default=None, help="Playback speed override")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    parser.add_argument("--start-paused", action="store_true", help="Start paused")
    parser.add_argument("--step-mode", choices=["message", "topic", "time"], default=None)
    parser.add_argument("--step-topic", default=None, help="Topic used when --step-mode topic")
    parser.add_argument("--step-time", type=float, default=None, help="Seconds used when --step-mode time")
    args = parser.parse_args()

    try:
        config = load_yaml(Path(args.config).expanduser())
        replay_cfg = config.get("replay", {}) or {}
        events, topic_types, storage_id = read_bag_index(Path(args.bag).expanduser(), config)
        rules = build_topic_rules(config, topic_types)
    except Exception as exc:
        print(f"interactive_bag_player: {exc}", file=sys.stderr)
        return 2

    speed = args.speed if args.speed is not None else float(replay_cfg.get("speed", 1.0))
    loop = args.loop or bool(replay_cfg.get("loop", False))
    stamp_policy = str(replay_cfg.get("stamp_policy", "now")).lower()
    if stamp_policy not in {"now", "bag", "preserve"}:
        print(f"warn: unsupported stamp_policy={stamp_policy!r}, using now")
        stamp_policy = "now"
    publish_tf = bool(replay_cfg.get("publish_tf_from_odom", True))
    step_mode = args.step_mode or str(replay_cfg.get("step_mode", "topic")).lower()
    if step_mode not in {"message", "topic", "time"}:
        print(f"warn: unsupported step_mode={step_mode!r}, using topic")
        step_mode = "topic"
    step_topic = args.step_topic or str(replay_cfg.get("step_topic", "/odom"))
    step_time = args.step_time if args.step_time is not None else float(replay_cfg.get("step_time_sec", 1.0))

    print("bag topics:")
    for name, type_name in sorted(topic_types.items()):
        mark = "*" if name in rules else " "
        target = f" -> {rules[name].target}" if name in rules else ""
        print(f"  {mark} {name}: {type_name}{target}")

    if len(events) > 500_000:
        print(f"warn: large bag index contains {len(events)} selected events; seeking may take time")

    rclpy.init()
    data_source = BagDataSource(Path(args.bag).expanduser(), storage_id, events)
    node = InteractiveBagPlayer(
        events=events,
        data_source=data_source,
        rules=rules,
        speed=speed,
        loop=loop,
        stamp_policy=stamp_policy,
        publish_tf_from_odom=publish_tf,
        start_paused=args.start_paused,
        step_mode=step_mode,
        step_topic=step_topic,
        step_time_sec=step_time,
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
