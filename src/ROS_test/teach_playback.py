#!/usr/bin/env python3
"""
示教回放脚本 — 载入示教数据，平滑处理后跟踪回放

数据处理流水线:
  1. 载入 JSON 示教数据
  2. 毛刺检测与去除（基于速度阈值）
  3. 均匀时间重采样（消除采样抖动）
  4. Savitzky-Golay 平滑（保留轨迹形状，消除高频噪声）
  5. 计算平滑后的速度/加速度（用于前馈）

回放控制:
  τ = τ_gravity + τ_ff_vel + kp*(p_des - p) + kd*(ω_des - ω)

  - τ_gravity: 重力补偿
  - τ_ff_vel: 前馈速度补偿 (kd * ω_des 已隐含，此处用速度前馈信号)
  - kp/kd: 位置+阻尼跟踪

阶段 (可选):
  1. ramp: 重力补偿斜坡 (默认跳过, --ramp 设>0启用)
  2. goto: 平滑移动到轨迹起点 (默认跳过, --goto-time 设>0启用)
  3. playback: 跟踪回放平滑后的示教轨迹
  4. hold: 保持在轨迹终点

参数:
  --input        示教数据文件 (默认 teach_data.json)
  --kp           转子侧位置增益 (默认 1.0)
  --kd           转子侧阻尼 (默认 0.15)
  --speed        回放速度倍率 (默认 1.0, >1 加快, <1 减慢)
  --smooth       Savitzky-Golay 窗口大小 (默认 101, 奇数)
  --spike-thresh 毛刺检测速度阈值 rad/s (默认 20.0)
  --ramp         斜坡时间 s (默认 1.5)
  --goto-time    移动到起点时间 s (默认 2.0)
  --hold-time    保持时间 s (默认 3.0)
  --tau-inner    大臂重力矩 Nm (自动从示教文件读取)
  --tau-outer    小臂重力矩 Nm (自动从示教文件读取)
  --rate         回放控制频率 Hz (默认 200)

用法:
  python3 teach_playback.py
  python3 teach_playback.py --input teach_data.json --kp 3.0 --speed 0.8
  python3 teach_playback.py --smooth 101 --speed 1.5
"""

import rclpy
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
import time
import math
import argparse
import signal
import json
import numpy as np
from scipy.signal import savgol_filter

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}
OUTER_IDS = {1, 3}
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85


class TrajectoryProcessor:
    """示教数据处理: 毛刺去除 → 重采样 → 平滑 → 速度/加速度"""

    def __init__(self, frames, rate_hz, smooth_window=31, spike_thresh=20.0):
        self.rate_hz = rate_hz
        self.smooth_window = smooth_window
        self.spike_thresh = spike_thresh

        # 解析数据
        self.raw_t = np.array([f['t'] for f in frames])
        self.raw_pos = {}
        for mid in ALL_IDS:
            self.raw_pos[mid] = np.array([f[f'm{mid}'] for f in frames])

        self.n_raw = len(frames)
        self.duration = self.raw_t[-1] - self.raw_t[0]

        # 处理流水线
        self._remove_spikes()
        self._resample()
        self._smooth()
        self._compute_derivatives()

    def _remove_spikes(self):
        """基于速度阈值检测并插值替换毛刺点"""
        n_spikes_total = 0
        for mid in ALL_IDS:
            pos = self.raw_pos[mid].copy()
            n = len(pos)
            spike_mask = np.zeros(n, dtype=bool)

            for i in range(1, n):
                dt = self.raw_t[i] - self.raw_t[i-1]
                if dt <= 0:
                    spike_mask[i] = True
                    continue
                vel = abs(pos[i] - pos[i-1]) / dt
                if vel > self.spike_thresh:
                    spike_mask[i] = True

            n_spikes = np.sum(spike_mask)
            n_spikes_total += n_spikes

            if n_spikes > 0:
                # 用线性插值替换毛刺点
                good_idx = np.where(~spike_mask)[0]
                if len(good_idx) >= 2:
                    self.raw_pos[mid] = np.interp(
                        self.raw_t, self.raw_t[good_idx], pos[good_idx])

        if n_spikes_total > 0:
            print(f"  [处理] 去除 {n_spikes_total} 个毛刺点")

    def _resample(self):
        """均匀时间重采样"""
        n_samples = max(int(self.duration * self.rate_hz), 10)
        self.t = np.linspace(0.0, self.duration, n_samples)
        self.pos = {}
        for mid in ALL_IDS:
            self.pos[mid] = np.interp(self.t, self.raw_t - self.raw_t[0], self.raw_pos[mid])
        self.n = n_samples
        print(f"  [处理] 重采样: {self.n_raw}帧 → {self.n}帧 ({self.rate_hz}Hz × {self.duration:.2f}s)")

    def _smooth(self):
        """Savitzky-Golay 平滑滤波"""
        window = self.smooth_window
        if window % 2 == 0:
            window += 1
        if window > self.n:
            window = max(self.n // 2 * 2 - 1, 5)

        poly_order = min(3, window - 2)

        for mid in ALL_IDS:
            self.pos[mid] = savgol_filter(self.pos[mid], window, poly_order, mode='mirror')

        print(f"  [处理] Savitzky-Golay 平滑: 窗口={window}, 阶数={poly_order}")

    def _compute_derivatives(self):
        """SG 滤波器直接计算平滑导数"""
        dt = self.t[1] - self.t[0] if self.n > 1 else 1.0 / self.rate_hz
        window = self.smooth_window
        if window % 2 == 0:
            window += 1
        if window > self.n:
            window = max(self.n // 2 * 2 - 1, 5)
        poly_order = min(3, window - 2)

        self.vel = {}
        self.acc = {}
        for mid in ALL_IDS:
            self.vel[mid] = savgol_filter(self.pos[mid], window, poly_order, deriv=1, delta=dt, mode='mirror')
            self.acc[mid] = savgol_filter(self.pos[mid], window, poly_order, deriv=2, delta=dt, mode='mirror')

    def get_state(self, t):
        """插值获取 t 时刻的 (pos, vel) for each motor"""
        t = max(0.0, min(t, self.duration))
        pos = {}
        vel = {}
        for mid in ALL_IDS:
            pos[mid] = float(np.interp(t, self.t, self.pos[mid]))
            vel[mid] = float(np.interp(t, self.t, self.vel[mid]))
        return pos, vel

    def print_summary(self):
        print(f"\n  轨迹摘要:")
        print(f"    时长: {self.duration:.2f}s")
        print(f"    帧数: {self.n}")
        for mid in ALL_IDS:
            p0 = math.degrees(self.pos[mid][0])
            pf = math.degrees(self.pos[mid][-1])
            v_max = math.degrees(np.max(np.abs(self.vel[mid])))
            a_max = math.degrees(np.max(np.abs(self.acc[mid])))
            label = '大臂' if mid in INNER_IDS else '小臂'
            print(f"    M{mid}({label}): {p0:+.1f}° → {pf:+.1f}° | v_max={v_max:.1f}°/s | a_max={a_max:.0f}°/s²")


class TeachPlayback:
    def __init__(self, args):
        self.kp = args.kp
        self.kd = args.kd
        self.kp_hold = args.kp_hold
        self.kd_hold = args.kd_hold
        self.speed = args.speed
        self.ramp_time = args.ramp
        self.goto_time = args.goto_time
        self.hold_time = args.hold_time
        self.rate_hz = args.rate

        # 加载并处理示教数据
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

        # tau 优先使用 JSON 中录制时保存的值，保证录制/回放一致
        self.tau_inner = data.get('tau_inner', args.tau_inner)
        self.tau_outer = data.get('tau_outer', args.tau_outer)
        self.gravity_offset = args.gravity_offset
        print(f"[INFO] 重力补偿来自示教文件: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")

        self.traj = TrajectoryProcessor(
            data['frames'],
            rate_hz=self.rate_hz,
            smooth_window=args.smooth,
            spike_thresh=args.spike_thresh,
        )
        self.traj.print_summary()

        # ROS 初始化
        rclpy.init()
        self.node = rclpy.create_node('teach_playback')
        self.pub = self.node.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.positions = {}
        self.velocities = {}
        self.temperatures = {}
        self.errors = {}
        self.last_update = {}
        self.node.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        self._stop = False
        signal.signal(signal.SIGINT, self._sig_handler)

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.temperatures[msg.motor_id] = msg.temperature
        self.errors[msg.motor_id] = msg.error
        self.last_update[msg.motor_id] = time.time()

    def _sig_handler(self, signum, frame):
        print("\n[STOP] Ctrl+C")
        self._stop = True

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

    def smoothstep(self, t):
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def gravity_torque_rotor(self, mid, theta1, theta2, ramp=1.0):
        """重力补偿力矩 (转子侧)"""
        if mid in INNER_IDS:
            tau = self.tau_inner * math.cos(theta1 + self.gravity_offset) / GEAR_RATIO * ramp
        else:
            tau = -self.tau_outer * math.cos(theta1 + theta2 + self.gravity_offset) / GEAR_RATIO * ramp
        return tau

    def check_node_alive(self):
        now = time.time()
        for mid in ALL_IDS:
            last = self.last_update.get(mid)
            if last is not None and (now - last) <= NODE_TIMEOUT:
                return True
        return False

    def check_safety(self):
        for mid in ALL_IDS:
            temp = self.temperatures.get(mid)
            err = self.errors.get(mid, 0)
            if temp is not None and temp >= TEMP_CRITICAL:
                print(f"\n[CRITICAL] M{mid} 温度={temp}C")
                return False
            if err in (1, 2, 3, 4):
                return False
        return True

    def wait_for_motors(self, timeout=15.0):
        print("[INFO] 等待电机上线...")
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            online = [mid for mid in ALL_IDS if mid in self.positions]
            if len(online) >= 1 and (time.time() - start) >= 2.0:
                missing = [mid for mid in ALL_IDS if mid not in self.positions]
                if missing:
                    print(f"[WARN] 电机 {missing} 未响应，{len(online)}/4 启动")
                else:
                    print("[OK] 所有电机已上线")
                return True
        return False

    def get_theta_pair(self):
        """获取 (θ₁, θ₂) 取对称电机均值"""
        t1 = (self.positions.get(0, 0.0) + self.positions.get(2, 0.0)) / 2.0
        t2 = (self.positions.get(1, 0.0) + self.positions.get(3, 0.0)) / 2.0
        return t1, t2

    def run(self):
        if not self.wait_for_motors():
            print("[FAIL] 无电机响应")
            self.shutdown()
            return

        # 轨迹起点
        start_pos, _ = self.traj.get_state(0.0)
        end_pos, _ = self.traj.get_state(self.traj.duration)

        playback_duration = self.traj.duration / self.speed

        print(f"\n{'='*60}")
        print(f"  示教回放")
        print(f"{'='*60}")
        print(f"  轨迹时长: {self.traj.duration:.2f}s × 速度倍率 {self.speed:.1f}x = {playback_duration:.2f}s")
        print(f"  控制参数: kp={self.kp} kd={self.kd} (轨迹) | kp={self.kp_hold} kd={self.kd_hold} (保持)")
        print(f"  重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm")
        print(f"  阶段: 斜坡{self.ramp_time}s → 移至起点{self.goto_time}s → 回放{playback_duration:.1f}s → 保持{self.hold_time}s")
        print(f"  Ctrl+C 安全退出\n")

        self._run_loop(start_pos, end_pos, playback_duration)

    def _run_loop(self, start_pos, end_pos, playback_duration):
        dt = 1.0 / self.rate_hz
        t_global_start = time.time()
        phase = 'ramp'

        # 各阶段起始时间
        t_phase_start = t_global_start
        initial_pos = dict(self.positions)  # 当前实际位置
        loop_count = 0

        while not self._stop:
            t_now = time.time()
            elapsed_global = t_now - t_global_start
            elapsed_phase = t_now - t_phase_start

            if not self.check_node_alive():
                print("\n[WARN] 节点无响应")
                self._brake_all()
                break

            if loop_count % (self.rate_hz * 5) == 0 and loop_count > 0:
                if not self.check_safety():
                    self._brake_all()
                    break

            for _ in range(16):
                rclpy.spin_once(self.node, timeout_sec=0)
            theta1, theta2 = self.get_theta_pair()

            # === 状态机 ===
            if phase == 'ramp':
                ramp_frac = self.smoothstep(elapsed_phase / self.ramp_time) if self.ramp_time > 0 else 1.0

                for mid in ALL_IDS:
                    pos = self.positions.get(mid, 0.0)
                    tau_g = self.gravity_torque_rotor(mid, theta1, theta2, ramp_frac)
                    self._send_cmd(mid, tau_g, 0.0, self.kd * ramp_frac, pos)

                if elapsed_phase >= self.ramp_time:
                    phase = 'goto'
                    t_phase_start = time.time()
                    initial_pos = dict(self.positions)
                    print(f"  [{elapsed_global:.1f}s] 斜坡完成 → 移动到起点")

            elif phase == 'goto':
                # smoothstep 从当前位置移动到轨迹起点
                frac = self.smoothstep(elapsed_phase / self.goto_time) if self.goto_time > 0 else 1.0

                for mid in ALL_IDS:
                    p0 = initial_pos.get(mid, 0.0)
                    pf = start_pos[mid]
                    p_des = p0 + (pf - p0) * frac

                    tau_g = self.gravity_torque_rotor(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp, self.kd, p_des)

                if elapsed_phase >= self.goto_time:
                    phase = 'playback'
                    t_phase_start = time.time()
                    print(f"  [{elapsed_global:.1f}s] 到达起点 → 开始回放")

            elif phase == 'playback':
                # 按速度倍率索引时间
                t_traj = elapsed_phase * self.speed

                if t_traj >= self.traj.duration:
                    phase = 'hold'
                    t_phase_start = time.time()
                    print(f"  [{elapsed_global:.1f}s] 回放完成 → 保持")
                else:
                    pos_d, vel_d = self.traj.get_state(t_traj)
                    for mid in ALL_IDS:
                        tau_g = self.gravity_torque_rotor(mid, theta1, theta2)
                        # 速度前馈: vel_d 已经是转子侧 (示教记录的就是转子侧)
                        self._send_cmd(mid, tau_g, self.kp, self.kd,
                                       pos_d[mid], vel_d[mid] * self.speed)

            if phase == 'hold':
                t_hold = time.time() - t_phase_start
                if t_hold >= self.hold_time:
                    print(f"  [{elapsed_global:.1f}s] 保持结束")
                    self._brake_all()
                    break

                for mid in ALL_IDS:
                    tau_g = self.gravity_torque_rotor(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp_hold, self.kd_hold, end_pos[mid])

            loop_count += 1
            if loop_count % self.rate_hz == 0:
                self._print_status(elapsed_global, phase)

            sleep_time = dt - (time.time() - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._print_report()
        self.shutdown()

    def _print_status(self, elapsed, phase):
        parts = []
        for mid in sorted(self.positions):
            pos = self.positions[mid]
            temp = self.temperatures.get(mid, -1)
            parts.append(f"M{mid}:{math.degrees(pos):+6.1f}° {temp}C")
        print(f"  [{elapsed:5.1f}s] {phase:10s} | {' '.join(parts)}")

    def _print_report(self):
        print(f"\n{'='*60}")
        print(f"  回放报告")
        for mid in sorted(self.positions):
            label = '大臂' if mid in INNER_IDS else '小臂'
            pos = self.positions.get(mid, 0.0)
            temp = self.temperatures.get(mid, -1)
            print(f"  M{mid}({label}): {math.degrees(pos):+.1f}° | {temp}C")
        print(f"{'='*60}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='示教回放')
    parser.add_argument('--input', type=str, default='teach_data.json',
                        help='示教数据文件 (默认 teach_data.json)')
    parser.add_argument('--rec', type=int, default=None,
                        help='选择第几条录制 (默认 -1 最新, 0=第一条)')

    parser.add_argument('--kp', type=float, default=1.0,
                        help='轨迹跟踪 kp (转子侧, 默认 1.0)')
    parser.add_argument('--kd', type=float, default=0.15,
                        help='轨迹跟踪 kd (转子侧, 默认 0.15)')
    parser.add_argument('--kp-hold', type=float, default=1.0,
                        help='保持阶段 kp (默认 1.0)')
    parser.add_argument('--kd-hold', type=float, default=0.15,
                        help='保持阶段 kd (默认 0.15)')
    parser.add_argument('--speed', type=float, default=2.0,
                        help='回放速度倍率 (默认 1.0)')

    parser.add_argument('--smooth', type=int, default=101,
                        help='Savitzky-Golay 窗口大小 (默认 101, 奇数)')
    parser.add_argument('--spike-thresh', type=float, default=20.0,
                        help='毛刺检测速度阈值 rad/s (默认 20.0)')

    parser.add_argument('--tau-inner', type=float, default=3.1,
                        help='大臂重力矩 Nm (默认 3.1)')
    parser.add_argument('--tau-outer', type=float, default=1.5,
                        help='小臂重力矩 Nm (默认 1.5)')
    parser.add_argument('--gravity-offset', type=float, default=-1.5708,
                        help='零位到水平的角度偏移 rad (默认 -π/2, 零位=下垂)')
    parser.add_argument('--ramp', type=float, default=0.0,
                        help='重力补偿斜坡时间 s (默认 0, 跳过)')
    parser.add_argument('--goto-time', type=float, default=0.0,
                        help='移动到起点时间 s (默认 0, 跳过)')
    parser.add_argument('--hold-time', type=float, default=0.0,
                        help='保持时间 s (默认 3.0)')
    parser.add_argument('--rate', type=int, default=200,
                        help='控制频率 Hz (默认 200)')

    args = parser.parse_args()
    player = TeachPlayback(args)
    player.run()


if __name__ == '__main__':
    main()
