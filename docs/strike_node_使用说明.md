# strike_node 使用说明

## 概述

`strike_node` 是一个基于 C++ 的 ROS2 击球控制节点，控制 4 个 GO-M8010-6 电机执行完整的击球动作序列。

**决策（选方案）与执行（触发状态机）严格分离**：

| 输入来源 | 决策（选方案） | 执行（触发状态机） |
|---------|-------------|----------------|
| 手柄    | A/B/X/Y 四键 | LB 键 |
| 键盘    | 1/2/3/4 键   | 空格 / Enter |
| 视觉    | 落点坐标自动选择 | — |
| 红外    | — | `/ir_trigger` 话题 |

---

## 快速启动

### 步骤 1：启动电机驱动（终端 A）

```bash
cd ~/USB2CAN_motor
source install/setup.bash
ros2 run motor_control_ros2 motor_control_node
```

### 步骤 2：启动击球节点（终端 B，前台运行）

```bash
cd ~/USB2CAN_motor
source install/setup.bash
ros2 run motor_control_ros2 strike_node
```

启动后应看到如下日志（键盘可用时）：

```
[INFO] strike_node 启动 | 200 Hz | 4 套击球方案 | 默认方案: center
[INFO] 键盘控制已启用: 1=center 2=left 3=right 4=strong  空格/Enter=触发  q=退出
[INFO] [键盘] 线程已启动
        1=center  2=left  3=right  4=strong
        空格/Enter=触发击打   q=退出
```

---

## 键盘控制

> ⚠️ 必须在**前台交互式终端**运行节点，不能用 `nohup`、`&` 后台或 `ros2 launch` 启动，否则 stdin 不是终端，键盘控制自动禁用。

| 按键 | 功能 |
|-----|------|
| `1` | 选择 **center** 方案（正面击球） |
| `2` | 选择 **left** 方案（偏左落点） |
| `3` | 选择 **right** 方案（偏右落点） |
| `4` | 选择 **strong** 方案（强力击球） |
| `空格` 或 `Enter` | **触发**：IDLE → 蓄力，READY → 击打 |
| `q` / `Q` | 安全退出节点 |

**操作流程示例：**

```
按 2         → [键盘] 方案: [left]
按 空格      → [键盘] 触发 (当前方案: left)
             → [INFO] 触发 → 蓄力 [方案: left]
             → [INFO] 蓄力位到达
再按 空格    → [INFO] 触发 → 击打! [方案: left]
             → [INFO] 到达击球位: 大臂=... → 刹停完成 → 回蓄力完成 → IDLE
```

---

## 手柄控制（Xbox 标准映射）

| 按键 | 功能 |
|-----|------|
| A（索引 0） | 选择 **center** 方案 |
| B（索引 1） | 选择 **left** 方案 |
| X（索引 2） | 选择 **right** 方案 |
| Y（索引 3） | 选择 **strong** 方案 |
| **LB（索引 4）** | **触发**：IDLE → 蓄力，READY → 击打 |

手柄需先启动 `joy_node`：

```bash
ros2 run joy joy_node
```

---

## 红外触发（自动模式）

红外传感器通过 MCU → 串口 → `positioning_bridge_node` → `/ir_trigger` 话题链路透传至本节点。

**触发链路：**

```
MCU 红外中断 (PD14 EXTI)
    → 发送 BINARY_V1 帧 (FrameKind::IR_TRIGGER = 0x07)
    → positioning_bridge_node 解析
    → 发布 std_msgs/Bool(true) 到 /ir_trigger
    → strike_node 收到 → 触发状态机
```

**启用条件：** YAML 中 `ir_trigger_enabled: true`（默认开启）。

---

## 视觉决策（落点预测）

订阅 `/strike/landing_point`（`geometry_msgs/PointStamped`），根据落点 x 坐标**自动选择方案**，不触发状态机。

| 落点 x 值 | 选择方案 |
|----------|---------|
| `x >= 0.15 m` | `right` |
| `x <= -0.15 m` | `left` |
| 其他 | `center` |
| 超过 2.0s 无信号 | 降级回 `center` |

阈值和超时均可在 YAML 中调整：

```yaml
vision_threshold_right: 0.15
vision_threshold_left: -0.15
vision_timeout_sec: 2.0
```

---

## 击球方案说明

| 方案 | 触发按键 | 大臂蓄力位 | 击打速度（大/小臂） | 特点 |
|------|--------|----------|-----------------|------|
| `center` | A / 键盘1 | 30° | 9.0 / 7.5 rad/s | 标准正面击球 |
| `left`   | B / 键盘2 | 25° | 9.5 / 8.0 rad/s | 偏左落点，速度稍快 |
| `right`  | X / 键盘3 | 35° | 8.5 / 7.0 rad/s | 偏右落点，速度适中 |
| `strong` | Y / 键盘4 | 30° | 10.5 / 9.0 rad/s | 强力击球，最大速度 |

---

## 状态机流程

```
IDLE
 │  触发（LB / 空格 / IR）
 ▼
MOVING_TO_READY  ── smoothstep 插值运动到蓄力位 ──▶ READY
                                                        │
                                                  触发（LB / 空格 / IR）
                                                        │
                                                   STRIKE_ACCEL  ── 速度模式击打
                                                        │  大臂到达 trigger_rad
                                                        ▼
                                                    BRAKING  ── 阻尼刹车
                                                        │  brake_duration 后
                                                        ▼
                                                    RETURNING  ── smoothstep 回蓄力位
                                                        │  return_time 后
                                                        ▼
                                                      IDLE
```

---

## 配置文件

配置文件路径：`config/strike_node_params.yaml`

方案参数可在配置文件的 `profiles` 部分修改，无需重新编译，重启节点即可生效。

---

## 常见问题

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| `stdin 不是终端` 警告，键盘无效 | 节点用后台/launch方式启动 | 改用 `ros2 run` 在前台交互式终端运行 |
| 按 Enter 无反应（无日志） | 键盘线程未启动（见上） | 见上 |
| 有触发日志但电机不动 | `motor_control_node` 未运行 | 先启动 `motor_control_node` |
| 电机立即停止 / 刹车 | 电机未上线，状态超时 | 检查串口连接和电机供电 |
| 击打中途切换方案无效 | 正常保护机制 | 等本次击打完成（回到 IDLE）后切换生效 |
