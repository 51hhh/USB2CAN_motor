# positioning_bridge 整体使用说明

> 适用仓库：`USB2CAN_motor`  
> 主要节点：`positioning_bridge_ros2/positioning_bridge_node`  
> 当前推荐协议：`binary_v1` ODOM 二进制协议  
> 默认串口：`/dev/ttyUSB0 @ 115200`

## 1. 系统作用

`positioning_bridge_node` 负责把 STM32F407VE 码盘定位板的数据桥接到 ROS2：

- 从串口读取 STM32 输出的定位数据；
- 解析 ODOM 二进制帧；
- 发布 ROS2 `/odom`；
- 可选广播 `odom -> base_link` TF；
- 提供启动自动归零与手动归零服务；
- 发布 `/diagnostics` 便于检查串口、协议、融合状态。

当前 MCU 侧默认使用二进制协议：

```text
AA 55 | Ver | Type | Seq | PayloadLen(LE16) | Payload | CRC16(LE)
```

常用帧：

| 方向 | Type | 含义 |
|---|---:|---|
| MCU -> Host | `0x02` | `ODOM_STATE`，45 字节整帧，包含 x/y/yaw/vx/vy/wz |
| Host -> MCU | `0x30` | `SET_LOCAL_ORIGIN`，设置局部原点 |
| MCU -> Host | `0x31` | `SET_LOCAL_ORIGIN_ACK`，原点设置应答 |
| Host -> MCU | `0x20` | 时间同步请求 |
| MCU -> Host | `0x21` | 时间同步响应 |

## 2. 启动前检查

### 2.1 检查串口

确认设备存在：

```bash
ls -l /dev/ttyUSB0
```

如果权限不足，临时授权：

```bash
sudo chmod 666 /dev/ttyUSB0
```

### 2.2 清理残留节点

`/odom` 跳变最常见原因是**重复启动了多个 `positioning_bridge_node`**。多个同名节点同时发布 `/odom` 时，`ros2 topic echo /odom` 会交替显示不同发布者的数据，看起来就是大跳变。

检查节点数量：

```bash
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 node list | sort | uniq -c
ros2 topic info /odom -v
```

正常情况：

```text
Publisher count: 1
Node name: positioning_bridge_node
```

如果看到多个 `/positioning_bridge_node` 或 `/odom` 有多个 publisher，先清理：

```bash
pkill -f '/positioning_bridge_node$'
```

再次确认：

```bash
ros2 node list | grep positioning_bridge_node
ros2 topic info /odom -v
```

## 3. 节点关系和推荐启动顺序

### 3.1 节点关系

底盘相关节点关系如下：

```text
手柄/遥控/自动控制
	│
	▼
  /cmd_vel_joy 或 /cmd_vel_remote
	│
	▼
cmd_vel_mux_node
	│ 发布 /cmd_vel
	▼
omni_chassis_control_node
	│ 订阅 /cmd_vel
	│ 订阅电机反馈
	│ 发布电机命令
	│ 发布 /odom_wheels  (轮速里程计，主要给融合用)
	▼
motor_control_node
	│ CAN/串口驱动电机

STM32 码盘定位板
	│ /dev/ttyUSB0, ODOM binary_v1
	▼
positioning_bridge_node
	│ 订阅 /odom_wheels（可选，用于互补滤波）
	│ 发布 /odom
	│ 发布 odom -> base_link TF
	▼
视觉/自动控制/闭环控制节点
```

关键点：

- `positioning_bridge_node` 是最终 `/odom` 发布者。
- `omni_chassis_control_node` 发布 `/odom_wheels`，主要提供轮速速度 `vx/vy/wz` 给 `positioning_bridge_node` 融合。
- `motor_control_node` 是电机底层驱动，负责和 USB2CAN/串口电机通信。
- `cmd_vel_mux_node` 负责选择手柄或自动控制的速度源，输出统一 `/cmd_vel`。
- 上层视觉/自动控制节点订阅 `/odom`，应在 `/odom` 稳定后再启动闭环动作。

### 3.1.1 `/cmd_vel`、`/odom`、`/odom_wheels` 的区别

| Topic | 消息类型 | 是位置吗 | 含义 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 不是 | 给底盘的**速度命令**，例如前进速度、横移速度、旋转角速度 |
| `/odom` | `nav_msgs/Odometry` | 是 | 最终里程计，包含位置 `pose` 和速度 `twist` |
| `/odom_wheels` | `nav_msgs/Odometry` | 辅助 | 电机轮速积分/速度估计，主要给 `positioning_bridge_node` 做融合预测 |

`/cmd_vel` 常用字段：

```text
linear.x   前后方向速度，单位 m/s
linear.y   左右方向速度，单位 m/s
angular.z  原地旋转角速度，单位 rad/s
```

因此 `/cmd_vel` 表示“让车怎么动”，不是“车现在在哪里”。车当前在哪里看 `/odom`。

### 3.2 最小启动：只看定位 `/odom`

只想看 STM32 码盘定位，不控制底盘时，只需要：

```bash
cd /home/toe/111/USB2CAN_motor
source install/setup.zsh
pkill -f '/positioning_bridge_node$'
ros2 run positioning_bridge_ros2 positioning_bridge_node
```

另一个终端：

```bash
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 topic info /odom -v
ros2 topic echo /odom --field pose.pose.position
```

### 3.3 推荐启动：底盘定位 + 融合 + 控制

推荐顺序：

1. **清理残留节点**，避免 `/odom` 多发布者混发。
2. **启动电机底层驱动**：`motor_control_node`。
3. **启动底盘运动学控制**：`omni_chassis_control_node`，输出 `/odom_wheels`。
4. **启动定位桥**：`positioning_bridge_node`，输出最终 `/odom`。
5. **确认 `/odom` 单发布者且稳定**。
6. **启动速度源选择/手柄/自动控制**：`cmd_vel_mux_node`、`joystick_control_node`、视觉或自动控制节点。

示例命令（建议分终端启动，便于看日志）：

```bash
# 0. 每次调试前先清理重复节点
pkill -f '/positioning_bridge_node$'

# 1. 电机底层驱动
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 run motor_control_ros2 motor_control_node
```

```bash
# 2. 底盘控制与 /odom_wheels
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 run motor_control_ros2 omni_chassis_control_node
```

```bash
# 3. STM32 定位桥与最终 /odom
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 run positioning_bridge_ros2 positioning_bridge_node
```

```bash
# 4. 检查 /odom 是否只有一个发布者
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 topic info /odom -v
ros2 topic echo /diagnostics --once
ros2 topic echo /odom --field pose.pose.position
```

```bash
# 5. 速度源选择与手柄控制（需要时）
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 run motor_control_ros2 cmd_vel_mux_node
```

```bash
# 6. 手柄节点（需要时）
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 run motor_control_ros2 joystick_control_node
```

### 3.4 启动顺序原则

| 场景 | 推荐顺序 | 原因 |
|---|---|---|
| 只看 `/odom` | `positioning_bridge_node` | 不需要电机节点 |
| 看融合后的 `/odom` | `motor_control_node` → `omni_chassis_control_node` → `positioning_bridge_node` | 先有 `/odom_wheels`，融合状态更稳定 |
| 实车闭环控制 | 电机底层 → 底盘控制 → 定位桥 → 确认 `/odom` → 上层控制 | 避免上层拿到未归零或跳变中的 `/odom` |
| 比赛/现场 | 先清理重复节点，再按顺序启动 | 防止多个 `/odom` 发布者混发 |

### 3.5 启动完成后的必须检查

```bash
ros2 topic info /odom -v
```

必须满足：

```text
Publisher count: 1
Node name: positioning_bridge_node
```

再检查诊断：

```bash
ros2 topic echo /diagnostics --once
```

重点看：

```text
serial_open: true
frames_ok: 持续增加
frames_crc_error: 不快速增加
fusion_state: NORMAL / ALERT / DISTURBED
```

## 4. 启动 ROS2 定位桥

在工作区根目录执行：

```bash
cd /home/toe/111/USB2CAN_motor
source install/setup.zsh
ros2 run positioning_bridge_ros2 positioning_bridge_node
```

正常日志应包含：

```text
正在加载 positioning_bridge 参数: .../positioning_bridge_params.yaml
启用互补滤波融合, 订阅: /odom_wheels
positioning_bridge_node 启动，协议模式: binary_v1
已连接定位串口: /dev/ttyUSB0 @ 115200
首次解析到 ... 帧
首次发布 odom: x=... y=... yaw=...
```

如果看到：

```text
启动自动归零完成 (sync_locked=false/true, elapsed=...s)
```

说明节点已经向 MCU 发送 `SET_LOCAL_ORIGIN(0,0,0)`。

## 5. 读取 `/odom`

另开一个终端：

```bash
cd /home/toe/111/USB2CAN_motor
source install/setup.zsh
ros2 topic echo /odom
```

只看位置：

```bash
ros2 topic echo /odom --field pose.pose.position
```

只看姿态四元数：

```bash
ros2 topic echo /odom --field pose.pose.orientation
```

只看速度：

```bash
ros2 topic echo /odom --field twist.twist
```

## 6. 回零方式

### 6.1 启动自动归零

配置文件：

```text
src/positioning_bridge_ros2/config/positioning_bridge_params.yaml
install/positioning_bridge_ros2/share/positioning_bridge_ros2/config/positioning_bridge_params.yaml
```

当前默认：

```yaml
auto_reset_origin_on_start: true
auto_reset_origin_timeout_sec: 3.0
```

含义：节点启动后等待时间同步锁定；若超过 3 秒仍未锁定，也会强制发送一次 `SET_LOCAL_ORIGIN(0,0,0)`。

注意：自动归零会让 `/odom` 从 MCU 当前累计值跳到接近 0，这是**正常的一次性跳变**。

### 6.2 手动 ROS2 服务归零

节点运行后执行：

```bash
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 service call /positioning/reset_local_origin std_srvs/srv/Trigger
```

返回成功只表示请求已写入串口；是否收到 MCU ACK 可看节点日志：

```text
收到重定位 ACK: seq=... result=0 event=...
```

### 6.3 Python 串口工具归零

脚本：

```text
python/src/chassis_uart_test.py
```

监听 ODOM 二进制数据：

```bash
python3 /home/toe/111/USB2CAN_motor/python/src/chassis_uart_test.py --listen --baud 115200
```

发送 ODOM 协议回零：

```bash
python3 /home/toe/111/USB2CAN_motor/python/src/chassis_uart_test.py --listen --raw --baud 115200 --reset-zero
```

重复发送 3 次：

```bash
python3 /home/toe/111/USB2CAN_motor/python/src/chassis_uart_test.py --listen --raw --baud 115200 --reset-zero --reset-repeat 3 --reset-interval 0.1
```

设置指定局部原点：

```bash
python3 /home/toe/111/USB2CAN_motor/python/src/chassis_uart_test.py --listen --baud 115200 --set-origin 0.0 0.0 0.0
```

旧协议兼容命令（不推荐作为当前主流程）：

```bash
python3 /home/toe/111/USB2CAN_motor/python/src/chassis_uart_test.py --listen --baud 115200 --reset
```

## 6. `/odom` 跳变排查

## 7. 融合方法说明

### 7.1 在哪个节点融合

融合发生在 ROS2 节点：

```text
positioning_bridge_ros2/positioning_bridge_node
```

具体实现位置：

```text
src/positioning_bridge_ros2/src/positioning_bridge_node.cpp
```

核心函数：

```text
PositioningBridgeNode::fusePose()
```

电机控制节点 `omni_chassis_control_node` 只发布 `/odom_wheels`，提供轮速里程计速度作为融合预测输入；真正发布最终 `/odom` 的节点是 `positioning_bridge_node`。

### 7.2 融合的数据来源

| 数据 | Topic/来源 | 作用 |
|---|---|---|
| STM32 码盘定位 | 串口 ODOM_STATE | 主测量值：`x/y/yaw` |
| 电机轮速里程 | `/odom_wheels` | 预测值：`vx/vy/wz` |
| 输出结果 | `/odom` | 融合后的 ROS 里程计 |

### 7.3 使用的融合方法

当前使用的是**自适应互补滤波**：

```text
预测：上一帧融合位姿 + /odom_wheels 速度积分
测量：STM32 码盘定位板输出的 x/y/yaw
输出：alpha * 预测 + (1 - alpha) * 测量
```

位置融合：

```text
fused_x = alpha * pred_x + (1 - alpha) * stm32_x
fused_y = alpha * pred_y + (1 - alpha) * stm32_y
```

角度融合使用 unwrap 后的 yaw 差值，避免 ±π 跳变：

```text
fused_yaw = stm32_yaw + alpha * unwrap(pred_yaw - stm32_yaw)
```

### 7.4 alpha 如何变化

`alpha` 是电机预测权重，越大越相信 `/odom_wheels`，越小越相信 STM32 码盘定位。

配置文件：

```text
src/positioning_bridge_ros2/config/positioning_bridge_params.yaml
```

当前默认：

```yaml
fusion_mode: "complementary"
fusion_alpha_normal:    0.30
fusion_alpha_alert:     0.15
fusion_alpha_disturbed: 0.02
```

判断逻辑：

| 状态 | 条件 | alpha | 含义 |
|---|---|---:|---|
| `NORMAL` | 预测与码盘残差小 | `0.30` | 30% 电机预测 + 70% 码盘 |
| `ALERT` | 残差中等 | `0.15` | 降低电机预测权重 |
| `DISTURBED` | 残差大或 `/odom_wheels` 超时 | `0.02` 或 `0.0` | 基本退回纯码盘 |

如果 `/odom_wheels` 超时，代码会直接令 `alpha=0.0`，输出基本等于 STM32 码盘定位。

### 7.5 如何关闭融合

如果只想看 STM32 码盘原始定位，把配置改成：

```yaml
fusion_mode: "none"
```

此时 `/odom` 由 STM32 串口数据直接发布，不再叠加 `/odom_wheels` 预测。

## 8. `/odom` 跳变排查

### 8.1 先查是否多发布者

```bash
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 topic info /odom -v
```

如果 `Publisher count > 1`，这就是第一嫌疑。处理：

```bash
pkill -f '/positioning_bridge_node$'
```

然后只启动一个 `positioning_bridge_node`。

### 8.2 判断是否自动归零造成的一次性跳变

查看节点日志：

```text
启动自动归零完成 (...)
```

如果 `/odom` 在这条日志附近从较大值跳到接近 0，这是设计行为。

如果不想启动时自动归零，把配置改成：

```yaml
auto_reset_origin_on_start: false
```

修改 `src/.../config` 后需要重新构建或同步到 `install/.../config`。

### 8.3 判断是否融合导致抖动

看 `/diagnostics`：

```bash
ros2 topic echo /diagnostics --once
```

重点字段：

| 字段 | 含义 |
|---|---|
| `fusion_mode` | 当前融合模式，`complementary` 或 `none` |
| `fusion_state` | `NORMAL` / `ALERT` / `DISTURBED` |
| `fusion_alpha` | 电机预测权重 |
| `fusion_residual_xy_m` | 电机预测与码盘测量的位置残差 |
| `motor_twist_age_sec` | `/odom_wheels` 新鲜度 |

如果只是想验证 STM32 码盘原始输出，可以临时关闭融合：

```yaml
fusion_mode: "none"
```

### 8.4 判断是否串口/协议问题

看 `/diagnostics`：

```bash
ros2 topic echo /diagnostics --once
```

重点字段：

| 字段 | 正常期望 |
|---|---|
| `serial_open` | `true` |
| `frames_ok` | 持续增加 |
| `frames_crc_error` | 不应快速增加 |
| `frames_parse_error` | 不应快速增加 |
| `bytes_rx` | 持续增加 |

少量 CRC 错误可能来自启动时半帧/残留字节；如果持续快速增长，检查串口线、波特率、是否多个程序同时打开 `/dev/ttyUSB0`。

### 8.5 判断是否时间同步未锁定

`time_sync_not_locked` 不一定会导致位置跳变，但会影响 header stamp 使用 MCU 时间还是 ROS 当前时间。

检查：

```bash
ros2 topic echo /diagnostics --once
```

字段：

```text
time_sync_locked
time_sync_samples
time_sync_offset_us
time_sync_rtt_us
```

若长期 `time_sync_samples=0`，说明 MCU 未回应 `TIME_SYNC_REQ`，需检查 MCU 是否烧录了包含 `ODOM_MSG_TIME_SYNC_REQ/RESP` 的固件版本。

## 9. 推荐调试顺序

每次测试建议按下面顺序：

```bash
# 1. 清理重复节点
pkill -f '/positioning_bridge_node$'

# 2. 启动节点
cd /home/toe/111/USB2CAN_motor
source install/setup.zsh
ros2 run positioning_bridge_ros2 positioning_bridge_node
```

另一个终端：

```bash
# 3. 确认只有一个 /odom 发布者
source /home/toe/111/USB2CAN_motor/install/setup.zsh
ros2 topic info /odom -v

# 4. 看 /odom
ros2 topic echo /odom --field pose.pose.position

# 5. 看诊断
ros2 topic echo /diagnostics --once
```

如果要手动归零：

```bash
ros2 service call /positioning/reset_local_origin std_srvs/srv/Trigger
```

## 10. 常见现象速查

| 现象 | 最可能原因 | 处理 |
|---|---|---|
| `/odom` 大幅来回跳 | 多个 `/odom` 发布者混发 | `ros2 topic info /odom -v`，清理重复节点 |
| 启动几秒后从大值跳到 0 | 自动归零 | 正常；或关闭 `auto_reset_origin_on_start` |
| `/odom` 稳定但不是 0 | 没触发回零或回零前已有偏置 | 调 service 或 Python `--reset-zero` |
| `time_sync_not_locked` | 时间同步 ACK 未收到 | 检查 MCU 固件是否支持 time sync |
| `frames_crc_error` 持续增加 | 串口噪声/协议不匹配/多程序抢串口 | 检查波特率、线缆、只保留一个串口读者 |
| `fusion_state=DISTURBED` | `/odom_wheels` 超时或残差大 | 调融合参数，或临时 `fusion_mode: none` |

## 11. 安全注意

- 回零会改变 ROS 坐标系中的位置，不会移动机器人本体。
- 回零瞬间，下游闭环控制会看到 `/odom` 跳变；比赛或实车调试时应先停止运动命令。
- 正式运行时只保留一个 `positioning_bridge_node`，避免 `/odom` 多源混发。
- 若同时用 Python 串口工具和 ROS2 节点读取 `/dev/ttyUSB0`，会抢占串口或导致数据被分走；一般不要同时运行。
