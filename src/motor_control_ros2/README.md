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
cd /home/rick/desktop/ros/usb2can
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
