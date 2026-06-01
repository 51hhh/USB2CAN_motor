#!/usr/bin/env python3
"""
GM6020 电机角度归零工具 (ROS2 版, 纯文本写入, 无额外依赖)
读取电机当前角度，计算 offset 写入 motors.yaml，使当前位置变为 0°

原理:
  getAngleDegrees() = (raw_position * direction - offset_rad) * 180/PI
  当 offset_rad = current_angle_deg * PI/180 时 → 输出 0°

用法:
  终端1: ros2 run motor_control_ros2 motor_control_node
  终端2: python3 tools/calibrate_dji6020_zero.py
         python3 tools/calibrate_dji6020_zero.py --dry-run   # 仅查看
"""
import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import DJIMotorState
import sys
import math
import re
from collections import defaultdict
from pathlib import Path

CONFIG = Path(__file__).resolve().parent.parent / "src" / "motor_control_ros2" / "config" / "motors.yaml"


class ZeroCalibrator(Node):
    def __init__(self, config_path: Path, dry_run: bool = False):
        super().__init__("dji6020_zero_calibrator")
        self.config_path = config_path
        self.dry_run = dry_run
        self.samples: dict[str, list[float]] = defaultdict(list)
        self.target = 60

        self.sub = self.create_subscription(
            DJIMotorState, "dji_motor_states", self.cb, 10
        )
        self._last_print = 0

    def cb(self, msg):
        if msg.model != "GM6020" or not msg.online:
            return
        self.samples[msg.joint_name].append(msg.angle)
        now = self.get_clock().now().nanoseconds
        if now - self._last_print > 1e9:
            total = sum(len(v) for v in self.samples.values())
            print(f"\r  已采样 {total} 条...", end="", flush=True)
            self._last_print = now

    def run(self):
        print("=" * 60)
        print("  GM6020 角度归零工具 (ROS2)")
        print(f"  配置: {self.config_path}")
        if self.dry_run:
            print("  模式: --dry-run (仅查看)")
        print(f"  采样目标: {self.target} 条/电机")
        print("  确保 motor_control_node 正在运行...")
        print("=" * 60)

        expected = self.target * max(1, len(self.samples))
        while sum(len(v) for v in self.samples.values()) < expected:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"\r  采样完成")
        self.destroy_subscription(self.sub)

        cal = {}
        # 先读 YAML 中已有的 offset（因为 topic 中的 angle 已减过 offset）
        existing_offsets = {}
        cur = None
        with open(self.config_path) as f:
            for line in f:
                m = re.match(r'\s*-?\s*name:\s*(\S+)', line)
                if m: cur = m.group(1)
                if cur:
                    om = re.match(r'\s+offset:\s*([-\d.]+)', line)
                    if om: existing_offsets[cur] = float(om.group(1))

        print()
        for name in sorted(self.samples):
            angles = self.samples[name]
            avg_deg = sum(angles) / len(angles) % 360.0
            old = existing_offsets.get(name, 0.0)
            # new_offset = old_offset + angle_rad（还原为 raw_position 后作为新 offset）
            offset_rad = old + math.radians(avg_deg)
            cal[name] = {"deg": avg_deg, "rad": offset_rad}
            print(f"  {name:15s}: {avg_deg:7.2f}°  (旧offset={old:.4f})  →  新offset = {offset_rad:.6f} rad")

        if self.dry_run:
            print("\n  [--dry-run] 未写入")
            return

        print()
        ans = input("  写入以上 offset? (y/N): ")
        if ans.lower() != "y":
            print("  已取消")
            return

        # 纯文本方式更新配置文件，不依赖任何 YAML 库
        with open(self.config_path) as f:
            lines = f.readlines()

        result = []
        current_name = None

        for i, line in enumerate(lines):
            m = re.match(r'(\s*)-?\s*name:\s*(\S+)', line)
            if m:
                current_name = m.group(2)

            # 跳过要更新电机的旧 offset 行（非注释行）
            if current_name in cal and re.match(r'\s+offset:', line):
                continue

            result.append(line)

            # 在 id: 行后面插入新 offset（检查 result 末尾，而非原始 lines）
            if current_name in cal and re.match(r'\s+id:\s+\d+', line):
                indent = re.match(r'(\s+)', line).group(1)
                # 检查 result 中上一条是否是刚插入的 offset（避免重复插入）
                if not (len(result) >= 2 and re.match(r'\s+offset:', result[-2])):
                    result.append(f"{indent}offset: {cal[current_name]['rad']:.6f}\n")

        self.config_path.write_text("".join(result))
        print(f"\n  ✅ 已更新 {len(cal)} 个电机 offset")
        print(f"     重启 motor_control_node 使生效")


def main():
    dry_run = "--dry-run" in sys.argv
    config_path = CONFIG
    for i, a in enumerate(sys.argv):
        if a == "--config" and i + 1 < len(sys.argv):
            config_path = Path(sys.argv[i + 1])

    rclpy.init()
    c = ZeroCalibrator(config_path, dry_run)
    c.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
