# 项目结构说明

## 📁 当前项目结构（重构后）

```
motor_control_ros2/
├── CMakeLists.txt              # 构建配置（纯 C++）
├── package.xml                 # ROS2 包配置
├── README.md                   # 项目说明
│
├── config/                     # 配置文件
│   ├── motors.yaml            # 电机配置
│   └── control_params.yaml    # 控制参数
│
├── launch/                     # 启动文件
│   └── motor_control.launch.py
│
├── msg/                        # 消息定义
│   ├── DJIMotorState.msg
│   ├── DJIMotorCommand.msg
│   ├── DamiaoMotorState.msg
│   ├── DamiaoMotorCommand.msg
│   ├── UnitreeMotorState.msg
│   ├── UnitreeMotorCommand.msg
│   ├── UnitreeGO8010State.msg
│   ├── UnitreeGO8010Command.msg
│   └── MotorStateGeneric.msg
│
├── srv/                        # 服务定义
│   ├── MotorEnable.srv
│   ├── MotorSetZero.srv
│   └── MotorSwitchMode.srv
│
├── include/motor_control_ros2/ # 头文件
│   ├── hardware/              # 硬件抽象层
│   │   ├── can_interface.hpp      # CAN 接口
│   │   ├── serial_interface.hpp   # 串口接口
│   │   └── hardware_manager.hpp   # 硬件管理器
│   │
│   ├── motor_base.hpp         # 电机基类
│   ├── dji_motor.hpp          # DJI 电机驱动
│   ├── damiao_motor.hpp       # 达妙电机驱动
│   └── unitree_motor.hpp      # 宇树电机驱动
│
├── src/                        # 源文件
│   ├── hardware/              # 硬件层实现
│   │   ├── can_interface.cpp
│   │   ├── serial_interface.cpp
│   │   └── hardware_manager.cpp
│   │
│   ├── dji_motor.cpp          # DJI 电机实现
│   ├── damiao_motor.cpp       # 达妙电机实现
│   ├── unitree_motor.cpp      # 宇树电机实现
│   └── motor_control_node.cpp # 主控制节点
│
├── resource/                   # 资源文件
└── test/                       # 测试文件（待添加）
```

## 🏗️ 架构说明

### 三层架构

1. **硬件层** (`hardware/`)
   - `CANInterface`: CAN 总线通信（USB-CAN 适配器）
   - `SerialInterface`: 串口通信（宇树电机 RS485）
   - `HardwareManager`: 统一管理硬件接口

2. **驱动层** (电机驱动)
   - `MotorBase`: 电机基类，定义统一接口
   - `DJIMotor`: DJI GM6020/GM3508 驱动
   - `DamiaoMotor`: 达妙 DM4340 驱动
   - `UnitreeMotor`: 宇树 A1/GO-8010 驱动

3. **应用层**
   - `MotorControlNode`: ROS2 主控制节点
     - 500Hz 控制循环
     - 电机状态发布
     - 命令订阅

## 🔧 技术栈

- **语言**: 纯 C++ (移除了 Python 依赖)
- **ROS2**: Humble
- **通信**:
  - CAN: USB-CAN 适配器 (`/dev/ttyACM0`)
  - 串口: USB-485 适配器 (`/dev/ttyUSB0`)
- **依赖**:
  - rclcpp
  - std_msgs, sensor_msgs, geometry_msgs
  - yaml-cpp (未来用于配置解析)

## 📊 支持的电机

| 电机型号 | 数量 | 通信接口 | 控制模式 |
|---------|------|---------|---------|
| DJI GM6020 | 4 | CAN | 电压控制 |
| DJI GM3508 | 4 | CAN | 电流控制 |
| 达妙 DM4340 | 4 | CAN | MIT 模式 |
| 宇树 GO-8010 | 4 | RS485 | 力位混合 |

## 🚀 编译与运行

```bash
# 编译
cd /home/rick/desktop/ros/usb2can
colcon build --packages-select motor_control_ros2

# 运行
source install/setup.bash
ros2 run motor_control_ros2 motor_control_node
```

## 📝 下一步计划

1. ✅ 硬件层重构完成
2. 🔧 硬件测试（当前任务）
3. ⏳ YAML 配置解析
4. ⏳ 电机驱动优化
5. ⏳ 性能测试
