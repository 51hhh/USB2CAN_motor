# ROS2 多功能电机控制包 - 用户操作手册

## 1. 项目简介

本项目是一个基于 ROS2 (Humble/Foxy) 的通用电机控制包，旨在为机器人应用提供统一、高性能的电机驱动接口。

### 🌟 核心特性
- **全平台支持**：同时支持 DJI、达妙 (Damiao)、宇树 (Unitree) 三大主流品牌电机。
- **混合通信架构**：集成 CAN 总线和 RS485 串口通信，统一调度。
- **500Hz 实时控制**：基于 C++ 的底层控制循环，确保低延迟和高带宽。
- **统一接口**：通过统一的 ROS2 Topic 和 Service 管理不同类型的电机。
- **复杂模式支持**：支持电流/电压控制、MIT 模式、力位混合控制。

### 🛠 支持硬件列表

| 品牌 | 型号 | 控制模式 | 接口 | 备注 |
|------|------|----------|------|------|
| **DJI** | GM6020 | 电压控制 | CAN | 适用于云台/低速大扭矩 |
| **DJI** | GM3508 | 电流控制 | CAN | 适用于底盘轮组 |
| **达妙** | DM4340 | MIT 模式 | CAN | 高性能关节电机 |
| **宇树** | A1 | 力位混合 | 串口 | 四足机器人关节 |
| **宇树** | GO-8010 | 力位混合 | 串口 | 高性能关节 |

---

## 2. 安装与编译

### 2.1 环境要求
- Ubuntu 20.04 / 22.04
- ROS2 Foxy / Humble / Galactic
- C++17 编译器

### 2.2 依赖安装
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-rclcpp ros-$ROS_DISTRO-rclpy \
                 ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-sensor-msgs \
                 ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-serial-driver
```

### 2.3 编译项目
```bash
# 进入工作空间 (假设在 ~/ros2_ws)
cd ~/ros2_ws

# 编译包
colcon build --packages-select motor_control_ros2

# 刷新环境
source install/setup.bash
```

---

## 3. 配置指南

所有配置均集中在 `config/motors.yaml` 文件中。您无需修改代码即可添加或删除电机。

### 3.1 配置文件结构
路径: `src/motor_control_ros2/config/motors.yaml`

```yaml
motors:
  # 电机名称 (ROS话题会自动包含此名称)
  yaw_motor:
    type: GM6020              # 电机类型: GM6020, GM3508, DM4340, UNITREE_A1...
    can_bus: can0             # 对应的 CAN 设备 (仅 CAN 电机)
    serial_port: /dev/ttyUSB0 # 对应的串口设备 (仅宇树电机)
    motor_id: 1               # 硬件 ID
    encoder_on_output: true   # true: 编码器在输出轴 (无需减速比转换); false: 在电机侧
    gear_ratio: 1.0           # 减速比 (若 encoder_on_output=true 则设为 1.0)
    
  joint1:
    type: DM4340
    can_bus: can0
    motor_id: 1
    # 达妙电机内部已处理减速比，通常 viewed as output shaft
    encoder_on_output: true
    gear_ratio: 1.0
```

### 3.2 控制参数
路径: `src/motor_control_ros2/config/control_params.yaml`

```yaml
/**:
  ros__parameters:
    control_frequency: 500.0  # 控制循环频率 (Hz)
    publish_frequency: 100.0  # 状态发布频率 (Hz)
```

---

## 4. 运行与使用

### 4.1 硬件连接检查
在运行节点前，请确保：
1. CAN 分析仪 (如 USB2CAN) 已连接并挂载 (例如 `ip link set can0 up type can bitrate 1000000`)。
2. 串口设备 (如 `/dev/ttyUSB0`) 已连接并拥有权限 (`sudo chmod 777 /dev/ttyUSB0`)。

### 4.2 启动节点
使用 launch 文件一键启动所有节点：

```bash
ros2 launch motor_control_ros2 motor_control.launch.py
```

### 4.3 快速硬件测试
我们提供了一个交互式脚本来快速验证电机是否工作：

```bash
# 在项目根目录下
./test_hardware.sh
```
按提示选择电机类型即可发送测试指令。

---

## 5. 接口说明 (ROS2 API)

### 5.1 话题 (Topics)

#### DJI 电机
- **订阅 (Input)**: `/dji_motor_command` (`motor_control_ros2/msg/DJIMotorCommand`)
  - `joint_name`: 关节名称
  - `output`: 控制输出 (电流/电压值)
- **发布 (Output)**: `/dji_motor_states` (`motor_control_ros2/msg/DJIMotorState`)
  - `angle`: 角度 (度)
  - `rpm`: 转速
  - `current`: 电流

#### 达妙电机
- **订阅 (Input)**: `/damiao_motor_command` (`motor_control_ros2/msg/DamiaoMotorCommand`)
  - MIT 模式参数: `pos_des`, `vel_des`, `kp`, `kd`, `torque_ff`
- **发布 (Output)**: `/damiao_motor_states` (`motor_control_ros2/msg/DamiaoMotorState`)
  - 包含位置、速度、力矩、MOS温度、线圈温度等

#### 宇树电机
- **订阅 (Input)**: 
  - A1: `/unitree_motor_command` (`motor_control_ros2/msg/UnitreeMotorCommand`)
  - GO-8010: `/unitree_go8010_command` (`motor_control_ros2/msg/UnitreeGO8010Command`)
  - 参数: `mode` (10=闭环, 0=空闲), `pos_des`, `vel_des`, `kp`, `kd`, `torque_ff`
- **发布 (Output)**: 
  - `/unitree_motor_states`
  - `/unitree_go8010_states`

### 5.2 服务 (Services)

- **使能/失能**: `/motor_enable` (类型: `MotorEnable`)
  - Request: `motor_names[]`, `enable (bool)`
- **设置零点**: `/motor_set_zero` (类型: `MotorSetZero`)
  - Request: `motor_names[]`

---

## 6. 常见问题 (FAQ)

**Q: 启动时报错 `Permission denied: /dev/ttyACM0`?**
A: 当前用户没有串口访问权限。
解决方法：`sudo chmod 777 /dev/ttyACM0` 或将用户加入 dialout 组：`sudo usermod -aG dialout $USER` (需重启生效)。

**Q: 电机没有反应?**
A: 
1. 检查电源电压是否符合电机要求 (DJI: 24V, 达妙: 24-48V, 宇树: 24V+)。
2. 确认 `can_bus` 名称是否正确 (使用 `ip link` 查看)。
3. 对于宇树电机，确认串口波特率是否匹配 (默认 4.8Mbps，可能需要专用 USB 转接板)。
4. 确认电机 ID 是否与配置文件一致。

**Q: 如何修改控制频率?**
A: 修改 `config/control_params.yaml` 中的 `control_frequency`。注意过高的频率可能导致 CPU 占用率飙升。

---

## 7. 高级开发

### 添加新电机类型
1. 在 `include/motor_control_ros2` 中继承 `MotorBase` 类。
2. 实现 `updateFeedback` 和 `getControlFrame` (CAN) 或自定义通信逻辑 (串口)。
3. 在 `motor_control_node.cpp` 中注册新电机。

### 源码结构
- `src/motor_control_node.cpp`: 系统核心，主循环。
- `src/can_driver.cpp`: CAN 通信底层。
- `src/serial_port.cpp`: 串口通信底层。
- `src/*_motor.cpp`: 各品牌电机驱动实现。
