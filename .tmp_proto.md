# ODOM 协议设计讨论

## 1. 当前固件时序底盘

| 参数 | 值 |
|------|-----|
| SYSCLK | 168 MHz |
| APB2 timer clock | 168 MHz |
| TIM11 ISR 频率 | 20 kHz (50μs) |
| 编码器+IMU采样 | 每 ISR tick = 50μs |
| VOFA 输出节拍 | add>=50 → 2.5ms → 理论400Hz |
| UART1 波特率 | 115200 |
| ASCII 帧大小 | ~80 bytes |
| ASCII 实际可用频率 | ~100-144 Hz (带宽上限) |
| Python 端实测 | ~20 Hz (监控脚本瓶颈) |

## 2. 你的需求 vs 我能给的

### 必须有 (A级)

| 字段 | 能否立即提供 | 备注 |
|------|-------------|------|
| `t_sample_us` (uint64) | ✅ | `HAL_GetTick()*1000` 给 1ms 精度；若需真 μs 可用 ISR tick 计数器 × 50 |
| `x` (float, m) | ✅ | 已有 `REAL_X` = `X_tt` |
| `y` (float, m) | ✅ | 已有 `REAL_Y` = `Y_tt` |
| `yaw` (float, rad) | ⚠️ 需加 unwrap | 当前 `YAW_ANGLE` 是 [-180°,+180°]，需转换为连续弧度 |
| `status_bits` (uint16) | ✅ 可编码 | `cali`, 编码器有效, IMU有效 等 |
| `quality` (uint8) | ✅ 可简单给 | 0-3 四级 |

### 强烈建议有 (B级)

| 字段 | 能否提供 | 方案 |
|------|---------|------|
| `vx` (float, m/s, body) | ⚠️ 需新增计算 | `(W1+W2)/2 / dt` 但这是差速模型；你的底盘是麦轮？ |
| `vy` (float, m/s, body) | ⚠️ 需讨论 | 麦轮有横向速度；差速无 vy |
| `wz` (float, rad/s) | ✅ 可立即给 | IMU 陀螺仪 Z 轴已有 `mpu_data[0].gyro[2]`，或差分 yaw/dt |

## 3. 需要你确认的设计决策

### Q1: 底盘运动学类型

IM_TEST 模型里的 45°/135° 偏移暗示**这不是纯差速底盘**，而是类似全向/麦轮的分解。

```c
// IM_TEST 中的分解：
Y增量 = sin(θ + 45°) * W1 + sin(θ + 135°) * W2
X增量 = cos(θ + 45°) * W1 + cos(θ + 135°) * W2
```

- 如果是 2 轮差速：`vy_body ≈ 0`，`vx_body = (v_left + v_right)/2`
- 如果是 4 麦轮（只用了 2 个编码器）：需要更复杂的逆运动学

**你的底盘实际是什么类型？2 轮差速？4 麦轮用 2 编码器？**

### Q2: 时间戳精度

| 方案 | 精度 | 复杂度 |
|------|------|--------|
| `HAL_GetTick() * 1000` | 1 ms | 零改动 |
| ISR tick 计数器 × 50 | 50 μs | 加一个 uint64 自增变量 |
| `DWT->CYCCNT` / 168 | 1 μs | 需初始化 DWT cycle counter |

推荐：**方案 B (50μs精度)**，用已有 ISR 心跳，零额外硬件开销。

### Q3: 二进制帧格式

```
┌─────────┬───────┬──────┬─────┬──────┬─────────┬─────┐
│ Header  │ Ver   │ Type │ Seq │ Len  │ Payload │ CRC │
│ 2 bytes │ 1     │ 1    │ 1   │ 2    │ N bytes │ 2   │
└─────────┴───────┴──────┴─────┴──────┴─────────┴─────┘
```

提议：
- **Header**: `0xAA 0x55` (常见嵌入式帧头)
- **Version**: `0x01`
- **Type**: `0x01`=ODOM_POSE, `0x02`=ODOM_STATE, `0x10`=STATUS
- **Seq**: 0-255 递增，用于丢帧检测
- **Len**: payload 长度 (little-endian uint16)
- **CRC**: CRC-16/CCITT 对 Ver~Payload 全部计算

ODOM_POSE payload (24 bytes):
```
uint64  t_sample_us     8 bytes
float32 x               4 bytes
float32 y               4 bytes
float32 yaw             4 bytes
uint16  status_bits     2 bytes
uint8   quality         1 byte
uint8   reserved        1 byte
```
Total frame = 2+1+1+1+2+24+2 = **33 bytes**

ODOM_STATE payload (36 bytes):
```
uint64  t_sample_us     8 bytes
float32 x               4 bytes
float32 y               4 bytes
float32 yaw             4 bytes
float32 vx              4 bytes
float32 vy              4 bytes
float32 wz              4 bytes
uint16  status_bits     2 bytes
uint8   quality         1 byte
uint8   reserved        1 byte
```
Total frame = 2+1+1+1+2+36+2 = **45 bytes**

### Q4: 输出频率与波特率

| 帧类型 | 帧大小 | @115200 最大Hz | @460800 最大Hz |
|--------|--------|----------------|----------------|
| ODOM_POSE (33B) | 33 | ~349 | ~1396 |
| ODOM_STATE (45B) | 45 | ~256 | ~1024 |

**第一版建议：**
- ODOM_STATE @ 100 Hz + STATUS @ 5 Hz → 100×45 + 5×33 = 4665 B/s → 115200 轻松承载
- 后续如需 200 Hz：升级到 460800 即可

### Q5: VOFA 调试与二进制协议共存

方案选择：
| 方案 | 优点 | 缺点 |
|------|------|------|
| A: 编译时 `#define` 切换 | 简单 | 需重新烧录 |
| B: 上电检测引脚/命令切换 | 灵活 | 复杂度增加 |
| C: 用 UART1 二进制 + UART2/SWO 调试 | 完美隔离 | 需要第二路物理接线 |

**推荐 A + C 组合**: 编译宏切换主协议；如果 PCB 有第二路 UART 引出则用于 VOFA 调试。

## 4. 我的实施计划（待你确认后执行）

### Phase 1: 最小可用 ODOM_POSE (当前就能做)

1. 添加 yaw unwrap 逻辑 → 连续弧度
2. 添加 ISR tick 计数器 → `t_sample_us`
3. 编码 `status_bits` 和 `quality`
4. 实现二进制帧封装 + CRC16
5. 输出 ODOM_POSE @ 100 Hz
6. 保留编译宏切换 VOFA / 二进制

### Phase 2: 完整 ODOM_STATE (需确认运动学)

7. 确认底盘运动学 → 计算 `vx_body`, `vy_body`
8. 使用 IMU gyro Z 或差分 yaw → `wz`
9. 输出 ODOM_STATE @ 100 Hz

### Phase 3: STATUS + 升级路径

10. STATUS/HEARTBEAT 帧 @ 5 Hz
11. 波特率可配置升级到 460800
12. 上位机命令解析（重定位 ACK 等）

## 5. 一句话

当前固件已有 x/y/yaw 和 20kHz 高采样率的数据源，缺的是：
1. yaw unwrap
2. 速度计算
3. 二进制封装 + CRC
4. 频率/带宽适配

这些都是纯软件改动，不需要硬件变更。只要你确认 Q1-Q5 的选择，我就可以开干。
