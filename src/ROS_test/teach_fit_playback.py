#!/usr/bin/env python3
"""
曲线拟合回放脚本 — 用多项式拟合示教数据，生成完美平滑轨迹

原理:
  读取 teach_data.json → 用 Chebyshev 多项式拟合每个电机的角度-时间曲线
  → 位置、速度、加速度全部由多项式解析计算，零噪声
  → 低阶多项式天然滤除所有抖动和毛刺

参数:
  --input        示教数据文件 (默认 teach_data.json)
  --order        多项式阶数 (默认 7, 越低越平滑, 越高越贴合原始数据)
  --kp           转子侧位置增益 (默认 1.0)
  --kd           转子侧阻尼 (默认 0.15)
  --kp-hold      保持阶段 kp (默认 1.0)
  --kd-hold      保持阶段 kd (默认 0.15)
  --speed        回放速度倍率 (默认 1.0)
  --j-rotor      电机转子惯量 kg·m² (默认 3.3e-5, 用于惯性前馈)
  --hold-time    保持时间 s (默认 3.0)
  --rate         控制频率 Hz (默认 200)

用法:
  python3 teach_fit_playback.py
  python3 teach_fit_playback.py --order 5 --speed 1.5
  python3 teach_fit_playback.py --order 10 --kp 2.0
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import json
import numpy as np

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}
OUTER_IDS = {1, 3}
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85


class PolyTrajectory:
    """用 Chebyshev 多项式拟合每个电机的角度轨迹"""

    def __init__(self, frames, order=7):
        # 解析原始数据
        raw_t = np.array([f['t'] for f in frames])
        raw_t = raw_t - raw_t[0]  # 从 0 开始
        self.duration = raw_t[-1]

        self.coeffs = {}  # mid → Chebyshev 系数
        self.fit_err = {}

        for mid in ALL_IDS:
            key = f'm{mid}'
            raw_pos = np.array([f[key] for f in frames])

            # Chebyshev 多项式拟合 (数值稳定, 比 np.polyfit 好)
            # 将时间映射到 [-1, 1]
            t_norm = 2.0 * raw_t / self.duration - 1.0
            coeffs = np.polynomial.chebyshev.chebfit(t_norm, raw_pos, order)
            self.coeffs[mid] = coeffs

            # 拟合误差
            fitted = np.polynomial.chebyshev.chebval(t_norm, coeffs)
            self.fit_err[mid] = np.sqrt(np.mean((raw_pos - fitted) ** 2))

    def eval(self, t):
        """返回 t 时刻的 (pos_dict, vel_dict, acc_dict)"""
        t = max(0.0, min(t, self.duration))
        t_norm = 2.0 * t / self.duration - 1.0
        scale = 2.0 / self.duration

        pos = {}
        vel = {}
        acc = {}
        for mid in ALL_IDS:
            c = self.coeffs[mid]
            pos[mid] = float(np.polynomial.chebyshev.chebval(t_norm, c))
            dc = np.polynomial.chebyshev.chebder(c)
            vel[mid] = float(np.polynomial.chebyshev.chebval(t_norm, dc)) * scale
            ddc = np.polynomial.chebyshev.chebder(dc)
            acc[mid] = float(np.polynomial.chebyshev.chebval(t_norm, ddc)) * scale * scale

        return pos, vel, acc

    def print_summary(self):
        print(f"\n  多项式拟合摘要:")
        print(f"    时长: {self.duration:.2f}s")
        print(f"    阶数: {len(self.coeffs[0]) - 1}")
        # 在密集时间点上评估以找到最大速度/加速度
        t_eval = np.linspace(0, self.duration, 500)
        for mid in ALL_IDS:
            c = self.coeffs[mid]
            p0 = float(np.polynomial.chebyshev.chebval(-1.0, c))
            pf = float(np.polynomial.chebyshev.chebval(1.0, c))
            label = '大臂' if mid in INNER_IDS else '小臂'
            # 最大速度
            scale = 2.0 / self.duration
            dc = np.polynomial.chebyshev.chebder(c)
            t_norm_eval = 2.0 * t_eval / self.duration - 1.0
            vel_all = np.polynomial.chebyshev.chebval(t_norm_eval, dc) * scale
            max_vel = np.max(np.abs(vel_all))
            print(f"    M{mid}({label}): {math.degrees(p0):+.1f}° → {math.degrees(pf):+.1f}° "
                  f"| RMSE={math.degrees(self.fit_err[mid]):.2f}° "
                  f"| Vmax={math.degrees(max_vel):.0f}°/s")


class FitPlayback:
    def __init__(self, args):
        self.kp = args.kp
        self.kd = args.kd
        self.kp_hold = args.kp_hold
        self.kd_hold = args.kd_hold
        self.speed = args.speed
        self.hold_time = args.hold_time
        self.rate_hz = args.rate
        self.j_rotor = args.j_rotor

        # 加载示教数据
        print(f"[INFO] 加载示教数据: {args.input}")
        with open(args.input, 'r') as f:
            raw = json.load(f)

        # 兼容多录制 / 旧单录制格式
        if 'recordings' in raw:
            recs = raw['recordings']
            ri = args.rec if args.rec is not None else -1
            data = recs[ri]
            print(f"[INFO] 共 {len(recs)} 条录制, 使用第 {ri % len(recs)} 条"
                  f" ({data.get('timestamp', '?')}  {data.get('duration', 0):.1f}s  {data.get('n_frames', '?')}帧)")
        else:
            data = raw

        # tau 从 JSON 读取
        self.tau_inner = data.get('tau_inner', 0.0)
        self.tau_outer = data.get('tau_outer', 0.0)
        self.gravity_offset = args.gravity_offset
        print(f"[INFO] 重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")

        # 多项式拟合
        self.traj = PolyTrajectory(data['frames'], order=args.order)
        self.traj.print_summary()

        # ROS 初始化
        rclpy.init()
        self.node = rclpy.create_node('teach_fit_playback')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.velocities = {}
        self.temperatures = {}
        self.last_update = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        self._stop = False
        signal.signal(signal.SIGINT, lambda *_: setattr(self, '_stop', True))

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.temperatures[msg.motor_id] = msg.temperature
        self.last_update[msg.motor_id] = time.time()

    def wait_for_motors(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for _ in range(16):
                rclpy.spin_once(self.node, timeout_sec=0)
            if len(self.positions) >= 4:
                return True
            time.sleep(0.05)
        return len(self.positions) >= 4

    def _send_cmd(self, motor_id, torque_ff, kp, kd, position_target, velocity_target=0.0):
        cmd = UnitreeGO8010Command()
        cmd.id = motor_id
        cmd.mode = 1
        cmd.position_target = position_target
        cmd.velocity_target = velocity_target
        cmd.kp = kp
        cmd.kd = kd
        cmd.torque_ff = torque_ff
        self.pub.publish(cmd)

    def _brake_all(self):
        for mid in ALL_IDS:
            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 0
            self.pub.publish(cmd)

    def get_theta_pair(self):
        t1 = (self.positions.get(0, 0.0) + self.positions.get(2, 0.0)) / 2.0
        t2 = (self.positions.get(1, 0.0) + self.positions.get(3, 0.0)) / 2.0
        return t1, t2

    def gravity_torque_rotor(self, mid, theta1, theta2):
        if mid in INNER_IDS:
            return self.tau_inner * math.cos(theta1 + self.gravity_offset) / GEAR_RATIO
        else:
            abs_angle = theta1 + theta2
            return -self.tau_outer * math.cos(abs_angle + self.gravity_offset) / GEAR_RATIO

    def check_safety(self):
        for mid in ALL_IDS:
            temp = self.temperatures.get(mid, 0)
            if temp > TEMP_CRITICAL:
                print(f"\n[WARN] M{mid} 温度过高: {temp}°C")
                return False
        return True

    def run(self):
        if not self.wait_for_motors():
            print("[FAIL] 无电机响应")
            self.shutdown()
            return

        # 轨迹起终点
        start_pos, _, _ = self.traj.eval(0.0)
        end_pos, _, _ = self.traj.eval(self.traj.duration)
        playback_duration = self.traj.duration / self.speed

        print(f"\n{'='*60}")
        print(f"  曲线拟合回放")
        print(f"{'='*60}")
        print(f"  轨迹时长: {self.traj.duration:.2f}s × {self.speed:.1f}x = {playback_duration:.2f}s")
        print(f"  控制参数: kp={self.kp} kd={self.kd} | 保持: kp={self.kp_hold} kd={self.kd_hold}")
        print(f"  惯性前馈: J_rotor={self.j_rotor} kg·m²")
        print(f"  重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")
        print(f"  回放{playback_duration:.1f}s → 保持{self.hold_time}s")
        print(f"  Ctrl+C 安全退出\n")

        dt = 1.0 / self.rate_hz
        t_start = time.time()
        phase = 'playback'
        t_phase_start = t_start
        loop_count = 0

        while not self._stop:
            t_now = time.time()
            elapsed = t_now - t_start
            elapsed_phase = t_now - t_phase_start

            for _ in range(16):
                rclpy.spin_once(self.node, timeout_sec=0)
            theta1, theta2 = self.get_theta_pair()

            if loop_count % (self.rate_hz * 5) == 0 and loop_count > 0:
                if not self.check_safety():
                    self._brake_all()
                    break

            if phase == 'playback':
                t_traj = elapsed_phase * self.speed

                if t_traj >= self.traj.duration:
                    phase = 'hold'
                    t_phase_start = time.time()
                    print(f"  [{elapsed:.1f}s] 回放完成 → 保持")
                else:
                    pos_d, vel_d, acc_d = self.traj.eval(t_traj)
                    for mid in ALL_IDS:
                        tau_g = self.gravity_torque_rotor(mid, theta1, theta2)
                        # 惯性前馈: J_rotor * α_output * gear * speed²
                        tau_inertia = self.j_rotor * acc_d[mid] * GEAR_RATIO * self.speed * self.speed
                        tau_ff = tau_g + tau_inertia
                        self._send_cmd(mid, tau_ff, self.kp, self.kd,
                                       pos_d[mid], vel_d[mid] * self.speed)

            elif phase == 'hold':
                if elapsed_phase >= self.hold_time:
                    print(f"  [{elapsed:.1f}s] 保持结束")
                    self._brake_all()
                    break

                for mid in ALL_IDS:
                    tau_g = self.gravity_torque_rotor(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp_hold, self.kd_hold, end_pos[mid])

            loop_count += 1
            if loop_count % self.rate_hz == 0:
                parts = []
                for mid in sorted(self.positions):
                    pos = self.positions[mid]
                    temp = self.temperatures.get(mid, -1)
                    parts.append(f"M{mid}:{math.degrees(pos):+6.1f}° {temp}C")
                print(f"  [{elapsed:5.1f}s] {phase:10s} | {' '.join(parts)}")

            sleep_time = dt - (time.time() - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 报告
        print(f"\n{'='*60}")
        print(f"  回放报告")
        for mid in sorted(self.positions):
            label = '大臂' if mid in INNER_IDS else '小臂'
            pos = self.positions.get(mid, 0.0)
            temp = self.temperatures.get(mid, -1)
            print(f"  M{mid}({label}): {math.degrees(pos):+.1f}° | {temp}C")
        print(f"{'='*60}")

        self.shutdown()

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='曲线拟合回放')
    parser.add_argument('--input', type=str, default='teach_data.json',
                        help='示教数据文件 (默认 teach_data.json)')
    parser.add_argument('--rec', type=int, default=None,
                        help='选择第几条录制 (默认 -1 最新, 0=第一条)')
    parser.add_argument('--order', type=int, default=7,
                        help='多项式阶数 (默认 7, 越低越平滑)')

    parser.add_argument('--kp', type=float, default=1.0,
                        help='轨迹跟踪 kp (转子侧, 默认 1.0)')
    parser.add_argument('--kd', type=float, default=0.15,
                        help='轨迹跟踪 kd (转子侧, 默认 0.15)')
    parser.add_argument('--kp-hold', type=float, default=1.0,
                        help='保持阶段 kp (默认 1.0)')
    parser.add_argument('--kd-hold', type=float, default=0.15,
                        help='保持阶段 kd (默认 0.15)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='回放速度倍率 (默认 1.0)')
    parser.add_argument('--j-rotor', type=float, default=3.3e-5,
                        help='电机转子惯量 kg·m² (默认 3.3e-5, GO-M8010-6)')
    parser.add_argument('--hold-time', type=float, default=3.0,
                        help='保持时间 s (默认 3.0)')
    parser.add_argument('--gravity-offset', type=float, default=-1.5708,
                        help='零位到水平的角度偏移 rad (默认 -π/2, 零位=下垂)')
    parser.add_argument('--rate', type=int, default=200,
                        help='控制频率 Hz (默认 200)')

    args = parser.parse_args()
    player = FitPlayback(args)
    player.run()


if __name__ == '__main__':
    main()
