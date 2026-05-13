# ROS2 电机控制包

支持 DJI、达妙、宇树多种电机的 ROS2 控制包。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        应用层                                │
│  motor_control_node (控制)    motor_monitor_node (监控)      │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                       控制层                                 │
│  CascadeController (串级控制)                                │
│    └── PIDController (位置环/速度环)                         │
│        - 单位: 度(0-360) / RPM (与 Python 一致)              │
│        - 无 dt 参数 (假设固定 200Hz 控制频率)                 │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                       驱动层                                 │
│  DJIMotor      DamiaoMotor      UnitreeMotor                │
│  (GM6020/3508) (DM4340/4310)    (A1/GO8010)                 │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                       硬件层                                 │
│  CANInterface (USB-CAN)       SerialInterface (RS485)       │
└─────────────────────────────────────────────────────────────┘
```

## 文件结构

```
src/motor_control_ros2/
├── config/
│   ├── motors.yaml          # 电机配置 (设备、ID)
│   ├── control_params.yaml  # 控制参数 (频率)
│   └── pid_params.yaml      # PID 参数 (与 Python 一致)
├── include/motor_control_ros2/
│   ├── pid_controller.hpp   # PID 控制器
│   ├── cascade_controller.hpp # 串级控制器
│   ├── dji_motor.hpp        # DJI 电机驱动
│   ├── damiao_motor.hpp     # 达妙电机驱动
│   ├── unitree_motor.hpp    # 宇树电机驱动
│   └── hardware/            # 硬件接口
├── src/
│   ├── motor_control_node.cpp  # 主控制节点
│   ├── motor_monitor_node.cpp  # 监控节点
│   ├── dji_motor.cpp
│   ├── damiao_motor.cpp
│   ├── unitree_motor.cpp
│   └── hardware/
└── msg/                     # ROS2 消息定义
```

## 支持的电机

| 电机型号 | 通信接口 | 控制模式 |
|---------|---------|---------|
| DJI GM6020 | CAN | 电压控制 (-30000~30000) |
| DJI GM3508 | CAN | 电流控制 (-16384~16384) |
| 达妙 DM4340/4310 | CAN | MIT 模式 |
| 宇树 A1/GO8010 | RS485 | 力位混合控制 |

## 快速开始

### 1. 编译

```bash
cd <工程根目录>
colcon build --packages-select motor_control_ros2
source install/setup.bash
```

### 2. 配置

编辑 `config/motors.yaml` 配置电机：

```yaml
can_interfaces:
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: DJI6020_1
        type: GM6020
        id: 1
```

### 3. 运行

```bash
# 终端 1: 控制节点
ros2 run motor_control_ros2 motor_control_node

# 终端 2: 监控节点 (可选)
ros2 run motor_control_ros2 motor_monitor_node
```

### 4. 发送命令

```bash
# 直接输出 (mode=0)
ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  '{joint_name: "DJI6020_1", output: 1000}'

# 位置控制 (mode=2, 单位: 度)
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 2, position_target: 90.0}'
```

## PID 控制说明

### 与 Python 实现的一致性

PID 控制器与 `python/src/pid.py` 完全一致：

| 特性 | 说明 |
|------|------|
| **角度单位** | 度 (0-360) |
| **速度单位** | RPM |
| **控制频率** | 200Hz |
| **I 项计算** | `i_out += ki * error` (无 dt) |
| **D 项计算** | `d_out = kd * (err[0] - err[1])` (无 dt) |

### 默认 PID 参数 (GM6020)

```yaml
# 角度环 (外环)
position_pid:
  kp: 10.0    ki: 1.0    kd: 0.0
  i_max: 10.0    out_max: 200.0    dead_zone: 0.5

# 速度环 (内环)
velocity_pid:
  kp: 30.0    ki: 1.0    kd: 0.0
  i_max: 300.0    out_max: 10000.0    dead_zone: 5.0
```

## ROS2 话题

### 发布

| 话题 | 类型 | 频率 |
|------|------|------|
| `/dji_motor_states` | DJIMotorState | 100Hz |
| `/damiao_motor_states` | DamiaoMotorState | 100Hz |
| `/unitree_motor_states` | UnitreeMotorState | 100Hz |
| `/control_frequency` | ControlFrequency | 100Hz |

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/dji_motor_command` | DJIMotorCommand | 直接输出命令 |
| `/dji_motor_command_advanced` | DJIMotorCommandAdvanced | 位置/速度/直接控制 |
| `/damiao_motor_command` | DamiaoMotorCommand | MIT 模式命令 |
| `/unitree_motor_command` | UnitreeMotorCommand | 力位混合命令 |

## `cmd_vel` 多源切换（手动/自动）

为避免手柄与遥控节点同时写入同一 `/cmd_vel` 造成互相覆盖，当前默认链路已拆分为：

- 手柄节点输出：`/cmd_vel_joy`
- 遥控节点输出：`/cmd_vel_remote`
- 复用节点：`cmd_vel_mux_node`
- 底盘最终输入：`/cmd_vel`

`cmd_vel_mux_node` 支持参数：

| 参数名 | 默认值 | 说明 |
|------|------|------|
| `joy_input_topic` | `/cmd_vel_joy` | 手动源输入 |
| `remote_input_topic` | `/cmd_vel_remote` | 自动源输入 |
| `output_topic` | `/cmd_vel` | 转发输出 |
| `active_source` | `remote` | 当前生效源（`joy` 或 `remote`） |
| `source_timeout_sec` | `0.5` | 活动源超时阈值（秒）；`<=0` 表示关闭超时保护 |
| `timeout_mode` | `brake` | 超时策略：`brake`（刹停）或 `fallback`（回退到备用源） |
| `fallback_source` | `joy` | `timeout_mode=fallback` 时的回退目标源 |
| `lock_active_source` | `false` | `true` 时禁止运行时切换控制源（自动模式锁定） |

### 运行时切换

`active_source` 支持运行时动态修改：

- `active_source = remote`：使用自动控制输入
- `active_source = joy`：切换到手动控制输入

非法值会被拒绝（仅允许 `joy` / `remote`）。切源时若目标源已有新鲜命令，会立即转发缓存命令。

当 `lock_active_source=true` 时，运行时切源请求会被拒绝（用于自动控制锁定）。

### 源超时保护

当当前活动源在 `source_timeout_sec` 内无新命令时：

- `timeout_mode=brake`：发布零速度（刹停保护）
- `timeout_mode=fallback`：若 `fallback_source` 有新鲜命令，则自动切源并继续控制；否则刹停

自动控制锁定建议配置：`config/cmd_vel_mux_auto_params.yaml`。

### 联调检查项

1. 确认 `/cmd_vel_joy` 与 `/cmd_vel_remote` 均有数据；
2. 确认 `cmd_vel_mux_node` 的 `active_source` 与期望一致；
3. 观察 `/cmd_vel` 仅跟随当前活动源变化；
4. 人为暂停活动源输入，确认超时后按策略执行（刹停或回退）；
5. 切换源或回退时，`cmd_vel_mux_node` 日志应出现提示。
