# positioning_monitor_node 说明

`positioning_monitor_node` 是类似 `motor_monitor_node` 的终端监控节点，用来更方便地查看码盘定位、`/odom`、`/odom_wheels` 和 `positioning_bridge` 诊断状态。

## 作用

该节点用于替代反复执行：

```bash
ros2 topic echo /odom
ros2 topic echo /diagnostics --once
ros2 topic info /odom -v
```

它会在终端动态刷新显示：

- `/odom` 是否在线；
- `/odom` 发布频率；
- 当前 `x / y / yaw`；
- 当前 `vx / vy / wz`；
- 相邻帧位置跳变量 `step_xy`；
- 最大跳变量 `max_step_xy`；
- `/odom_wheels` 是否在线；
- `positioning_bridge` 诊断信息；
- `time_sync_samples=0` 时提示 Host->STM32 上行可能不通。

显示界面采用表格布局，便于实车调试时快速扫一眼判断状态。

## 运行命令

```bash
cd /home/toe/111/USB2CAN_motor
source install/setup.zsh
ros2 run positioning_bridge_ros2 positioning_monitor_node
```

远端 `sunrise` 机器是 bash 时使用：

```bash
cd /home/sunrise/USB2CAN_motor
source install/setup.bash
ros2 run positioning_bridge_ros2 positioning_monitor_node
```

退出：按 `Ctrl+C`。

## 监控内容

### Topic 状态

```text
/odom        在线/超时/无数据  Hz
/odom_wheels 在线/超时/无数据  Hz
/diagnostics 在线/超时/无数据  Hz
```

### `/odom` 位姿

```text
x       当前 X，单位 m
y       当前 Y，单位 m
yaw     当前航向角，单位 rad / deg
vx      当前前后速度，单位 m/s
vy      当前左右速度，单位 m/s
wz      当前角速度，单位 rad/s
```

### 跳变监控

```text
step_xy       当前帧与上一帧的平面位移差
max_step_xy   启动监控以来最大平面跳变
step_yaw      当前帧与上一帧的 yaw 差
max_step_yaw  启动监控以来最大 yaw 跳变
jumps         超过阈值的跳变次数
```

默认阈值：

```text
jump_warn_m       0.05 m
yaw_jump_warn_rad 0.10 rad
```

也就是：

- 相邻帧位置超过 5 cm 记为一次跳变；
- 相邻帧 yaw 超过 0.1 rad 记为一次跳变。

### 诊断状态

显示 `positioning_bridge` 中的关键字段：

```text
serial_open
protocol_mode
last_serial_error
frames_ok
frames_crc_error
frames_parse_error
time_sync_locked
time_sync_samples
time_sync_rtt_us
fusion_state
fusion_alpha
fusion_residual_xy_m
fusion_residual_yaw_rad
motor_twist_age_sec
fusion_state_changes
```

## 参数

可以通过 ROS 参数修改 topic 和阈值：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `odom_topic` | `/odom` | 最终定位 topic |
| `wheel_odom_topic` | `/odom_wheels` | 轮速里程计 topic |
| `diagnostics_topic` | `/diagnostics` | 诊断 topic |
| `display_rate_hz` | `5.0` | 终端刷新频率 |
| `stale_timeout_sec` | `0.5` | topic 超时判定 |
| `jump_warn_m` | `0.05` | 位置跳变报警阈值 |
| `yaw_jump_warn_rad` | `0.10` | yaw 跳变报警阈值 |

示例：

```bash
ros2 run positioning_bridge_ros2 positioning_monitor_node --ros-args \
  -p jump_warn_m:=0.02 \
  -p yaw_jump_warn_rad:=0.05 \
  -p display_rate_hz:=10.0
```

## 典型判断

### `/odom` 正常

```text
/odom 在线
Hz 稳定
step_xy 很小
frames_ok 增加
frames_parse_error 不增加
fusion_state NORMAL
```

### 多发布者问题

该节点本身不直接统计 publisher 数量。若看到 `/odom` 大跳，仍需执行：

```bash
ros2 topic info /odom -v
```

如果 `Publisher count > 1`，先清理重复节点。

### 上行链路可能不通

如果看到：

```text
frames_ok 增加
time_sync_samples = 0
```

说明 STM32 -> Host 上报通，但 Host -> STM32 上行可能不通。此时自动回零、手动回零、时间同步都可能失败，需要检查 USB-TTL TX/RX/GND 和 MCU 固件上行解析。
