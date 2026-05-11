# 排球击打系统使用说明

> 更新时间：2026-04-01  
> 适用节点：`volleyball_strike_manager`

---

## 1. 硬件概述

| 参数 | 值 |
|------|-----|
| 电机型号 | Unitree GO-M8010-6 × 4 |
| 减速比 | **i = 6.33** |
| 控制模式 | MIT 阻抗（FOC） |
| 传动 | 14-200-14-200 精确平行四边形 |
| 臂间距 D | 350.6 mm |
| 大臂 L1 | 200 mm |
| 小臂 L2 | 224 mm |

### 电机映射

| 逻辑名 | CAN 总线 | ID | direction | 作用 |
|--------|----------|-----|-----------|------|
| L1 | /dev/ttyUSB0 | 0 | +1 | 左大臂（直驱） |
| L2 | /dev/ttyUSB0 | 1 | +1 | 左小臂（平行四边形） |
| R1 | /dev/ttyUSB1 | 2 | **-1** | 右大臂（镜像安装） |
| R2 | /dev/ttyUSB1 | 3 | **-1** | 右小臂（镜像安装） |

---

## 2. 协议缩放因子（关键！）

GO-M8010-6 减速比 i=6.33，MIT 模式下 **配置值 ≠ 输出轴物理值**：

```
cmd.kp  = 期望输出轴刚度(Nm/rad)   / (6.33 × 6.33) = / 40.07
cmd.kd  = 期望输出轴阻尼(Nm·s/rad) / (6.33 × 6.33) = / 40.07
cmd.tau = 期望输出轴力矩(Nm)       / 6.33
```

### 速查表

| 期望物理值 | 类型 | 配置值 |
|-----------|------|--------|
| Kp = 10 Nm/rad | ÷40.07 | **0.250** |
| Kp = 30 Nm/rad | ÷40.07 | **0.749** |
| Kd = 5 Nm·s/rad | ÷40.07 | **0.125** |
| τ = 2 Nm | ÷6.33 | **0.316** |
| τ = 8 Nm | ÷6.33 | **1.264** |
| τ = 10 Nm | ÷6.33 | **1.580** |

> ⚠️ 如果你看到 Kp > 2.0（物理>80Nm/rad）或 τ_ff > 2.5（物理>16Nm），大概率参数偏大，请谨慎。

---

## 3. 状态机

```
 HOMING (小臂→大臂两阶段合一) ──→ READY
                                          │
                    ┌─────────────────────┘
                    ↓ (收到击球命令)
                 STRIKE ──→ IMPACT_MOMENT ──→ FOLLOW_THROUGH
                                                      │
                                                      ↓
                                                  SETTLING → READY
                    
 任意状态 ──(速度突变/超时)──→ E_STOP
```

| 状态 | 作用 | Kp 策略 | τ_ff 策略 |
|------|------|---------|----------|
| HOMING | 先小臂(L2/R2)→再大臂(L1/R1)撞限位设零 | Kp=0, 纯 Kd+τ_ff | homing_torque |
| READY | 保持当前位置 | 高 Kp + 恒定前馈 | idle_hold_torque |
| STRIKE | 从 Home 直冲虚拟穿透目标 | 高 Kp + 低 Kd | 惯性前馈 + 重力补偿 |
| IMPACT_MOMENT | 纯力矩爆破（纯空间穿透退出） | 极低 Kp/Kd | 大 τ_ff 开环 |
| FOLLOW_THROUGH | Kd ramp 刹车 | 中 Kp | Kd: 0→8 Nm·s/rad |
| SETTLING | 缓冲回位 | 高 Kp | idle_hold_torque |
| E_STOP | 紧急制动 | Kp=0 | Kd ramp 到 5 Nm·s/rad |

---

## 4. 无视觉系统测试

### 4.1 启动节点

```bash
# 终端1: 启动电机驱动（确保 ttyUSB0/1 已连接）
source install/setup.bash
ros2 launch motor_control_ros2 <your_motor_launch>.py

# 终端2: 启动排球控制节点
source install/setup.bash
ros2 run motor_control_ros2 volleyball_strike_manager
```

### 4.2 测试命令话题

节点订阅 `/volleyball/test_command`（std_msgs/String），支持以下命令：

```bash
# 重新归零（从任意状态重新开始 homing）
ros2 topic pub --once /volleyball/test_command std_msgs/msg/String "{data: 'home'}"

# 强制切 IDLE（跳过当前状态）
ros2 topic pub --once /volleyball/test_command std_msgs/msg/String "{data: 'idle'}"

# 紧急停止
ros2 topic pub --once /volleyball/test_command std_msgs/msg/String "{data: 'estop'}"

# 默认测试击球（拍面中心 x=0.25m, y=0, θ=0°, 1.5秒后击球）
ros2 topic pub --once /volleyball/test_command std_msgs/msg/String "{data: 'strike'}"
```

### 4.3 监控状态

```bash
# 查看当前状态机状态
ros2 topic echo /volleyball/strike_status

# 查看电机原始反馈
ros2 topic echo unitree_go8010_states
```

### 4.4 自定义击球命令

如果需要指定精确参数（不使用默认测试值）：

```bash
ros2 topic pub --once /volleyball/strike_command motor_control_ros2/msg/StrikeCommand \
  "{x_target: 0.25, y_target: 0.0, theta_target: 0.0, ball_velocity: 0.0, time_to_impact: 1.5}"
```

参数说明：
- `x_target`：拍面中心 X 坐标（m），基座向前为正
- `y_target`：拍面中心 Y 坐标（m），基座向左为正
- `theta_target`：拍面仰角（rad），0=水平
- `ball_velocity`：球速估计（m/s），当前未使用
- `time_to_impact`：从收到命令到击球的时间（s）

---

## 5. 参数调试流程（推荐顺序）

### Step 1: 验证归零

1. 启动节点，观察日志 `HOMING_FOREARM 启动`
2. 小臂应平缓运动到机械限位并停止
3. 日志显示 `小臂归零完成: L2 raw=x.xxx, R2 raw=x.xxx`
4. 接着大臂归零，最后进入 IDLE

**如果归零方向错误**：调整 `homing.torque_to_min` 正负号  
**如果归零太快/太慢**：调整 `homing.torque_to_min` 绝对值（÷6.33换算）  
**如果误判到位**：增大 `homing.min_travel_rad` 或 `homing.stable_duration_s`

### Step 2: 验证 IDLE 悬停

1. 归零完成后手动推动手臂，感受刚度
2. 松手后应能保持位置不下坠（重力补偿生效）
3. 如果下坠：增大 `dynamics.upper_arm_mass` / `forearm_mass`
4. 如果过刚：减小 `safety.idle_kp`

### Step 3: 测试完整击球

```bash
ros2 topic pub --once /volleyball/test_command std_msgs/msg/String "{data: 'strike'}"
```

观察日志中的状态转换序列，逐一检查每个阶段行为是否符合预期。

---

## 6. 当前参数速查（配置值→物理值）

### 归零阶段
| 参数 | 配置值 | 物理值 | 说明 |
|------|--------|--------|------|
| kd | 0.100 | 4.0 Nm·s/rad | 归零阻尼 |
| hold_kp | 0.525 | 21.0 Nm/rad | 非活动轴保持 |
| hold_kd | 0.202 | 8.1 Nm·s/rad | 非活动轴阻尼 |
| torque_to_min L1/R1 | ±1.2 | ±7.6 Nm | 大臂归零力矩 |
| torque_to_min L2/R2 | ±1.2 | ±7.6 Nm | 小臂归零力矩 |

### STRIKE（从 Home 直冲虚拟穿透目标）
| 参数 | 配置值 | 物理值 |
|------|--------|--------|
| kp_shoulder | 0.998 | 40 Nm/rad |
| kp_elbow | 0.499 | 20 Nm/rad |
| kd_shoulder | 0.050 | 2.0 Nm·s/rad |
| kd_elbow | 0.025 | 1.0 Nm·s/rad |

### IMPACT（纯空间穿透退出）
| 参数 | 配置值 | 物理值 |
|------|--------|--------|
| kp_shoulder | 0.050 | 2.0 Nm/rad |
| kp_elbow | 0.025 | 1.0 Nm/rad |
| tau_ff_shoulder | 1.264 | 8.0 Nm |
| tau_ff_elbow | 0.632 | 4.0 Nm |

### FOLLOW_THROUGH（刹车）
| 参数 | 配置值 | 物理值 |
|------|--------|--------|
| kp_shoulder | 0.080 | 3.2 Nm/rad |
| kp_elbow | 0.040 | 1.6 Nm/rad |
| kd_start | 0.012 | 0.5 Nm·s/rad |
| kd_end_shoulder | 0.250 | 10.0 Nm·s/rad |
| kd_end_elbow | 0.150 | 6.0 Nm·s/rad |

### 安全
| 参数 | 配置值 | 物理值 |
|------|--------|--------|
| torque_limit | 1.58 | 10 Nm |
| e_stop_kd | 0.125 | 5.0 Nm·s/rad |
