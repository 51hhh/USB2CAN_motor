#!/usr/bin/env python3
"""
示教回放控制节点

状态机: CALIBRATING → IDLE → GOTO_START → PLAYBACK → RETURN → IDLE
触发器: 手柄 RB 键 (button 5) 或红外传感器 /ir_trigger (std_msgs/Bool)

启动后自动校准零位，然后等待触发信号。
收到信号后执行示教轨迹回放，完成后自动回零。

参数 (ROS param):
  teach_file       示教数据 JSON 文件路径
  rec_index        选择第几条录制 (-1=最新)
  kp / kd          轨迹跟踪增益 (转子侧)
  kp_hold / kd_hold 保持阶段增益
  speed            回放速度倍率
  smooth_window    SG 平滑窗口
  spike_thresh     毛刺检测阈值 rad/s
  tau_inner / tau_outer  重力矩 Nm (自动从 JSON 读取优先)
  goto_time        移至起点时间 s
  return_time      回零时间 s
  hold_time        击球后保持时间 s
  rate_hz          控制频率
  trigger_source   'joy' 或 'ir'
  joy_button       手柄触发按钮编号
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010Command, UnitreeGO8010State
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time
import math
import json
import os
import re
import subprocess
import yaml
import numpy as np
from scipy.signal import savgol_filter

GEAR_RATIO = 6.33
ALL_IDS = [0, 1, 2, 3]
INNER_IDS = {0, 2}
OUTER_IDS = {1, 3}
NODE_TIMEOUT = 8.0
TEMP_CRITICAL = 85

MOTOR_NAMES = ['strike_motor_L1', 'strike_motor_L2',
               'strike_motor_R1', 'strike_motor_R2']
CONFIG_PATH = os.path.expanduser(
    '~/USB2CAN_motor/src/motor_control_ros2/config/motors.yaml')
WORKSPACE = os.path.expanduser('~/USB2CAN_motor')


# ── 轨迹处理（从 teach_playback.py 提取） ──────────────────────────

class TrajectoryProcessor:
    """毛刺去除 → 重采样 → SG 平滑 → 速度/加速度"""

    def __init__(self, frames, rate_hz, smooth_window=101, spike_thresh=20.0):
        self.rate_hz = rate_hz
        self.smooth_window = smooth_window
        self.spike_thresh = spike_thresh

        self.raw_t = np.array([f['t'] for f in frames])
        self.raw_pos = {}
        for mid in ALL_IDS:
            self.raw_pos[mid] = np.array([f[f'm{mid}'] for f in frames])

        self.n_raw = len(frames)
        self.duration = self.raw_t[-1] - self.raw_t[0]

        self._remove_spikes()
        self._resample()
        self._smooth()
        self._compute_derivatives()

    def _remove_spikes(self):
        for mid in ALL_IDS:
            pos = self.raw_pos[mid].copy()
            n = len(pos)
            spike_mask = np.zeros(n, dtype=bool)
            for i in range(1, n):
                dt = self.raw_t[i] - self.raw_t[i - 1]
                if dt <= 0:
                    spike_mask[i] = True
                    continue
                if abs(pos[i] - pos[i - 1]) / dt > self.spike_thresh:
                    spike_mask[i] = True
            if np.any(spike_mask):
                good = np.where(~spike_mask)[0]
                if len(good) >= 2:
                    self.raw_pos[mid] = np.interp(
                        self.raw_t, self.raw_t[good], pos[good])

    def _resample(self):
        n = max(int(self.duration * self.rate_hz), 10)
        self.t = np.linspace(0.0, self.duration, n)
        self.pos = {}
        for mid in ALL_IDS:
            self.pos[mid] = np.interp(
                self.t, self.raw_t - self.raw_t[0], self.raw_pos[mid])
        self.n = n

    def _smooth(self):
        w = self.smooth_window
        if w % 2 == 0:
            w += 1
        if w > self.n:
            w = max(self.n // 2 * 2 - 1, 5)
        poly = min(3, w - 2)
        for mid in ALL_IDS:
            self.pos[mid] = savgol_filter(self.pos[mid], w, poly, mode='mirror')

    def _compute_derivatives(self):
        dt = self.t[1] - self.t[0] if self.n > 1 else 1.0 / self.rate_hz
        w = self.smooth_window
        if w % 2 == 0:
            w += 1
        if w > self.n:
            w = max(self.n // 2 * 2 - 1, 5)
        poly = min(3, w - 2)
        self.vel = {}
        self.acc = {}
        for mid in ALL_IDS:
            self.vel[mid] = savgol_filter(
                self.pos[mid], w, poly, deriv=1, delta=dt, mode='mirror')
            self.acc[mid] = savgol_filter(
                self.pos[mid], w, poly, deriv=2, delta=dt, mode='mirror')

    def get_state(self, t):
        t = max(0.0, min(t, self.duration))
        pos, vel = {}, {}
        for mid in ALL_IDS:
            pos[mid] = float(np.interp(t, self.t, self.pos[mid]))
            vel[mid] = float(np.interp(t, self.t, self.vel[mid]))
        return pos, vel


# ── 零位校准 ────────────────────────────────────────────────────

def calibrate_zero(logger):
    """读取当前位置 → 更新 offset → 重启 motor_control_node → 验证"""
    logger.info('开始零位校准...')

    # 读取电机当前位置
    rclpy.init()
    node = rclpy.create_node('_zero_cal_tmp')
    positions = {}

    def cb(msg):
        if msg.joint_name in MOTOR_NAMES:
            positions[msg.joint_name] = msg.position

    node.create_subscription(UnitreeGO8010State, '/unitree_go8010_states', cb, 10)
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(positions) >= len(MOTOR_NAMES):
            break
    node.destroy_node()
    rclpy.shutdown()

    if len(positions) < len(MOTOR_NAMES):
        logger.error(f'校准失败: 仅 {len(positions)}/{len(MOTOR_NAMES)} 电机响应')
        return False

    # 读取旧 offset
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    old_offsets = {}
    for iface in config.get('serial_interfaces', []):
        for motor in iface.get('motors', []):
            name = motor.get('name', '')
            if name in MOTOR_NAMES:
                old_offsets[name] = motor.get('offset', 0.0)

    # 写入新 offset
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        content = f.read()

    for name in MOTOR_NAMES:
        new_offset = old_offsets.get(name, 0.0) + positions[name]
        pattern = (
            r'(- name: ' + re.escape(name) + r'\s*\n'
            r'(?:    \w+:.*\n)*?'
            r'    offset: )'
            r'[-+]?\d*\.?\d+'
        )
        match = re.search(pattern, content)
        if match:
            content = (content[:match.start()] +
                       match.group(1) + f'{new_offset:.6f}' +
                       content[match.end():])
            logger.info(f'  {name}: offset {old_offsets.get(name, 0):.4f} → {new_offset:.4f}'
                        f' (当前位置 {math.degrees(positions[name]):+.1f}°)')

    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        f.write(content)

    # 重启 motor_control_node
    logger.info('重启 motor_control_node ...')
    subprocess.run(['pkill', '-9', '-f', 'motor_control_ros2/motor_control_node'],
                   stderr=subprocess.DEVNULL)
    time.sleep(2)
    env = os.environ.copy()
    setup_cmd = (f'source {WORKSPACE}/install/setup.bash && '
                 f'ros2 run motor_control_ros2 motor_control_node')
    subprocess.Popen(['bash', '-c', setup_cmd], env=env,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     preexec_fn=os.setpgrp)
    logger.info('等待节点初始化 (8s) ...')
    time.sleep(8)

    # 验证
    rclpy.init()
    node = rclpy.create_node('_zero_verify_tmp')
    verify_pos = {}

    def cb2(msg):
        if msg.joint_name in MOTOR_NAMES:
            verify_pos[msg.joint_name] = msg.position

    node.create_subscription(UnitreeGO8010State, '/unitree_go8010_states', cb2, 10)
    t0 = time.time()
    while time.time() - t0 < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(verify_pos) >= len(MOTOR_NAMES):
            break
    node.destroy_node()
    rclpy.shutdown()

    ok = True
    for name in MOTOR_NAMES:
        if name in verify_pos:
            deg = math.degrees(verify_pos[name])
            if abs(deg) > 2.0:
                logger.warn(f'  验证: {name} = {deg:+.2f}° ⚠️')
                ok = False
            else:
                logger.info(f'  验证: {name} = {deg:+.2f}° ✓')
        else:
            logger.error(f'  验证: {name} 无数据')
            ok = False

    if ok:
        logger.info('零位校准完成 ✓')
    else:
        logger.warn('零位校准部分失败，请检查')
    return ok


# ── 主控制节点 ──────────────────────────────────────────────────

class TeachPlaybackControlNode(Node):
    # 状态枚举
    CALIBRATING = 'CALIBRATING'
    IDLE = 'IDLE'
    GOTO_START = 'GOTO_START'
    PLAYBACK = 'PLAYBACK'
    HOLD = 'HOLD'
    RETURN = 'RETURN'

    def __init__(self):
        super().__init__('teach_playback_control_node')

        # 声明参数
        self.declare_parameter('teach_file', 'teach_data.json')
        self.declare_parameter('rec_index', -1)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('kp_hold', 1.0)
        self.declare_parameter('kd_hold', 0.15)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('smooth_window', 101)
        self.declare_parameter('spike_thresh', 20.0)
        self.declare_parameter('tau_inner', 3.1)
        self.declare_parameter('tau_outer', 1.5)
        self.declare_parameter('goto_time', 2.0)
        self.declare_parameter('return_time', 2.0)
        self.declare_parameter('hold_time', 1.0)
        self.declare_parameter('rate_hz', 200)
        self.declare_parameter('trigger_source', 'joy')
        self.declare_parameter('joy_button', 5)  # RB

        # 读取参数
        p = self.get_parameter
        self.teach_file = p('teach_file').value
        self.rec_index = p('rec_index').value
        self.kp = p('kp').value
        self.kd = p('kd').value
        self.kp_hold = p('kp_hold').value
        self.kd_hold = p('kd_hold').value
        self.speed = p('speed').value
        self.smooth_window = p('smooth_window').value
        self.spike_thresh = p('spike_thresh').value
        self.tau_inner = p('tau_inner').value
        self.tau_outer = p('tau_outer').value
        self.goto_time = p('goto_time').value
        self.return_time = p('return_time').value
        self.hold_time = p('hold_time').value
        self.rate_hz = p('rate_hz').value
        self.trigger_source = p('trigger_source').value
        self.joy_button = p('joy_button').value

        # 加载示教数据
        self._load_trajectory()

        # 电机状态
        self.positions = {}
        self.velocities = {}
        self.temperatures = {}
        self.errors = {}
        self.last_update = {}

        # 状态机
        self.state = self.CALIBRATING
        self._trigger_pending = False
        self._phase_start = 0.0
        self._initial_pos = {}

        # ROS 接口
        self.cmd_pub = self.create_publisher(
            UnitreeGO8010Command, '/unitree_go8010_command', 10)
        self.state_sub = self.create_subscription(
            UnitreeGO8010State, '/unitree_go8010_states', self._state_cb, 10)

        # 触发器订阅
        if self.trigger_source == 'joy':
            self.create_subscription(Joy, '/joy', self._joy_cb, 10)
            self.get_logger().info(f'触发源: 手柄 RB 键 (button {self.joy_button})')
        else:
            self.create_subscription(Bool, '/ir_trigger', self._ir_cb, 10)
            self.get_logger().info('触发源: 红外传感器 /ir_trigger')

        # 状态发布
        self.state_pub = self.create_publisher(Bool, '/strike_busy', 10)

        self.get_logger().info('节点已创建，开始校准...')

    def _load_trajectory(self):
        """加载示教 JSON 并构建轨迹处理器"""
        self.get_logger().info(f'加载示教数据: {self.teach_file}')
        with open(self.teach_file, 'r') as f:
            raw = json.load(f)

        if 'recordings' in raw:
            recs = raw['recordings']
            ri = self.rec_index
            data = recs[ri]
            self.get_logger().info(
                f'共 {len(recs)} 条录制, 使用第 {ri % len(recs)} 条'
                f' ({data.get("timestamp", "?")}  {data.get("duration", 0):.1f}s)')
        else:
            data = raw

        # tau 优先从 JSON 读取
        json_tau_inner = data.get('tau_inner')
        json_tau_outer = data.get('tau_outer')
        if json_tau_inner is not None and json_tau_inner != 0:
            self.tau_inner = json_tau_inner
        if json_tau_outer is not None and json_tau_outer != 0:
            self.tau_outer = json_tau_outer
        self.get_logger().info(
            f'重力补偿: 大臂={self.tau_inner}Nm 小臂={self.tau_outer}Nm')

        self.traj = TrajectoryProcessor(
            data['frames'],
            rate_hz=self.rate_hz,
            smooth_window=self.smooth_window,
            spike_thresh=self.spike_thresh,
        )
        self.start_pos, _ = self.traj.get_state(0.0)
        self.end_pos, _ = self.traj.get_state(self.traj.duration)
        self.playback_duration = self.traj.duration / self.speed

        self.get_logger().info(
            f'轨迹: {self.traj.duration:.2f}s × {self.speed:.1f}x = {self.playback_duration:.2f}s')

    def _state_cb(self, msg):
        self.positions[msg.motor_id] = msg.position
        self.velocities[msg.motor_id] = msg.velocity
        self.temperatures[msg.motor_id] = msg.temperature
        self.errors[msg.motor_id] = msg.error
        self.last_update[msg.motor_id] = time.time()

    def _joy_cb(self, msg):
        if (len(msg.buttons) > self.joy_button and
                msg.buttons[self.joy_button] and
                self.state == self.IDLE):
            self._trigger_pending = True
            self.get_logger().info('收到手柄触发信号')

    def _ir_cb(self, msg):
        if msg.data and self.state == self.IDLE:
            self._trigger_pending = True
            self.get_logger().info('收到红外触发信号')

    # ── 控制辅助 ─────────────────────────────────────────────

    def _send_cmd(self, mid, torque_ff, kp, kd, pos_target, vel_target=0.0):
        cmd = UnitreeGO8010Command()
        cmd.id = mid
        cmd.mode = 1
        cmd.position_target = pos_target
        cmd.velocity_target = vel_target
        cmd.kp = kp
        cmd.kd = kd
        cmd.torque_ff = torque_ff
        self.cmd_pub.publish(cmd)

    def _brake_all(self):
        for mid in ALL_IDS:
            cmd = UnitreeGO8010Command()
            cmd.id = mid
            cmd.mode = 0
            self.cmd_pub.publish(cmd)

    def _smoothstep(self, t):
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    def _gravity_torque(self, mid, theta1, theta2):
        if mid in INNER_IDS:
            return self.tau_inner * math.cos(theta1) / GEAR_RATIO
        else:
            return -self.tau_outer * math.cos(theta1 + theta2) / GEAR_RATIO

    def _get_theta_pair(self):
        t1 = (self.positions.get(0, 0.0) + self.positions.get(2, 0.0)) / 2.0
        t2 = (self.positions.get(1, 0.0) + self.positions.get(3, 0.0)) / 2.0
        return t1, t2

    def _check_safety(self):
        for mid in ALL_IDS:
            temp = self.temperatures.get(mid)
            if temp is not None and temp >= TEMP_CRITICAL:
                self.get_logger().error(f'M{mid} 温度={temp}C 超限!')
                return False
            if self.errors.get(mid, 0) in (1, 2, 3, 4):
                self.get_logger().error(f'M{mid} 错误码={self.errors[mid]}')
                return False
        return True

    def _motors_online(self):
        return len(self.positions) >= 4

    def _publish_busy(self, busy):
        msg = Bool()
        msg.data = busy
        self.state_pub.publish(msg)

    # ── 状态机主循环 ──────────────────────────────────────────

    def run(self):
        """阻塞式主循环"""

        # ── Phase 0: 零位校准 ──
        self.get_logger().info('='*50)
        self.get_logger().info('  Phase 0: 零位校准')
        self.get_logger().info('='*50)
        # 校准需要独立的 rclpy context（calibrate_zero 内部 init/shutdown）
        # 所以在 rclpy.init 之前执行
        # 但本节点已处于 rclpy context 中，需要先 shutdown
        # → 方案: 使用 subprocess 调用校准脚本
        cal_script = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '..', '..', 'ROS_test', 'calibrate_zero_position.py')
        cal_script = os.path.normpath(cal_script)
        if os.path.exists(cal_script):
            result = subprocess.run(
                ['python3', cal_script],
                cwd=os.path.dirname(cal_script),
                timeout=60,
            )
            if result.returncode != 0:
                self.get_logger().warn('零位校准返回非零，继续运行...')
        else:
            self.get_logger().warn(f'校准脚本不存在: {cal_script}，跳过校准')

        self.state = self.IDLE
        self.get_logger().info('进入 IDLE，等待触发信号...')

        # ── 200Hz 控制循环 ──
        dt = 1.0 / self.rate_hz
        loop_count = 0

        while rclpy.ok():
            t_now = time.time()

            # 排空消息队列
            for _ in range(16):
                rclpy.spin_once(self, timeout_sec=0)

            if not self._motors_online():
                if loop_count % self.rate_hz == 0:
                    self.get_logger().warn('等待电机上线...')
                loop_count += 1
                time.sleep(dt)
                continue

            # 安全检查 (每秒一次)
            if loop_count % self.rate_hz == 0:
                if not self._check_safety():
                    self._brake_all()
                    self.state = self.IDLE
                    self.get_logger().error('安全检查失败, 刹车!')
                    time.sleep(1.0)
                    continue

            theta1, theta2 = self._get_theta_pair()

            # ── IDLE: 重力补偿保持 + 等待触发 ──
            if self.state == self.IDLE:
                self._publish_busy(False)

                for mid in ALL_IDS:
                    tau_g = self._gravity_torque(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp_hold, self.kd_hold, 0.0)

                if self._trigger_pending:
                    self._trigger_pending = False
                    self.state = self.GOTO_START
                    self._phase_start = time.time()
                    self._initial_pos = dict(self.positions)
                    self._publish_busy(True)
                    self.get_logger().info('触发! → GOTO_START')

            # ── GOTO_START: 移动到轨迹起点 ──
            elif self.state == self.GOTO_START:
                elapsed = time.time() - self._phase_start
                frac = self._smoothstep(elapsed / self.goto_time) if self.goto_time > 0 else 1.0

                for mid in ALL_IDS:
                    p0 = self._initial_pos.get(mid, 0.0)
                    pf = self.start_pos[mid]
                    p_des = p0 + (pf - p0) * frac
                    tau_g = self._gravity_torque(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp, self.kd, p_des)

                if elapsed >= self.goto_time:
                    self.state = self.PLAYBACK
                    self._phase_start = time.time()
                    self.get_logger().info('到达起点 → PLAYBACK')

            # ── PLAYBACK: 轨迹回放 ──
            elif self.state == self.PLAYBACK:
                elapsed = time.time() - self._phase_start
                t_traj = elapsed * self.speed

                if t_traj >= self.traj.duration:
                    self.state = self.HOLD
                    self._phase_start = time.time()
                    self.get_logger().info('回放完成 → HOLD')
                else:
                    pos_d, vel_d = self.traj.get_state(t_traj)
                    for mid in ALL_IDS:
                        tau_g = self._gravity_torque(mid, theta1, theta2)
                        self._send_cmd(mid, tau_g, self.kp, self.kd,
                                       pos_d[mid], vel_d[mid] * self.speed)

            # ── HOLD: 击球后短暂保持 ──
            elif self.state == self.HOLD:
                elapsed = time.time() - self._phase_start

                for mid in ALL_IDS:
                    tau_g = self._gravity_torque(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp_hold, self.kd_hold,
                                   self.end_pos[mid])

                if elapsed >= self.hold_time:
                    self.state = self.RETURN
                    self._phase_start = time.time()
                    self._initial_pos = dict(self.positions)
                    self.get_logger().info('保持结束 → RETURN')

            # ── RETURN: 回零 ──
            elif self.state == self.RETURN:
                elapsed = time.time() - self._phase_start
                frac = self._smoothstep(elapsed / self.return_time) if self.return_time > 0 else 1.0

                for mid in ALL_IDS:
                    p0 = self._initial_pos.get(mid, 0.0)
                    p_des = p0 * (1.0 - frac)  # → 0.0
                    tau_g = self._gravity_torque(mid, theta1, theta2)
                    self._send_cmd(mid, tau_g, self.kp, self.kd, p_des)

                if elapsed >= self.return_time:
                    self.state = self.IDLE
                    self.get_logger().info('回零完成 → IDLE，等待下次触发')

            # 状态汇报
            loop_count += 1
            if loop_count % (self.rate_hz * 2) == 0:
                parts = [f'M{mid}:{math.degrees(self.positions.get(mid, 0)):+5.1f}°'
                         for mid in ALL_IDS]
                self.get_logger().info(
                    f'[{self.state:12s}] {" ".join(parts)}')

            sleep_time = dt - (time.time() - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)


def main():
    # 校准使用独立 rclpy 上下文，所以节点启动前先校准
    import logging
    logger = logging.getLogger('teach_playback_control')
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        logger.addHandler(logging.StreamHandler())

    logger.info('='*50)
    logger.info('  示教回放控制节点启动')
    logger.info('='*50)

    # 零位校准 (独立 rclpy context)
    logger.info('Phase 0: 零位校准...')
    calibrate_zero(logger)

    # 创建节点并运行
    rclpy.init()
    node = TeachPlaybackControlNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 退出')
    finally:
        node._brake_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
