# ROS2 电机控制包

支持 DJI、达妙、宇树多种电机的 ROS2 控制包，采用 C++/Python 混合架构实现 500Hz 实时控制。

## 支持的电机

- **DJI**：GM3508（电流控制）、GM6020（电压控制）
- **达妙**：DM4340（MIT 模式）
- **宇树**：A1、GO-8010（力位混合控制）

## 快速开始

### 1. 编译

```bash
cd /home/rick/desktop/ros/usb2can
colcon build --packages-select motor_control_ros2
source install/setup.bash
```

### 2. 配置

编辑 `config/motors.yaml` 添加您的电机配置：

```yaml
motors:
  yaw_motor:
    type: GM6020
    can_bus: can0
    motor_id: 1
    encoder_on_output: true
    gear_ratio: 1.0
```

### 3. 启动

```bash
ros2 launch motor_control_ros2 motor_control.launch.py
```

### 4. 发送命令

```bash
# DJI 电机
ros2 topic pub /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  "{joint_name: 'yaw_motor', output: 1000}"

# 达妙电机
ros2 topic pub /damiao_motor_command motor_control_ros2/msg/DamiaoMotorCommand \
  "{joint_name: 'joint1', pos_des: 0.0, vel_des: 0.0, kp: 10.0, kd: 1.0, torque_ff: 0.0}"
```

## 架构

- **C++ 实时控制层**：硬件通信、500Hz 控制循环
- **Python 上层控制**：配置管理、生命周期管理

## 文档

- **用户手册**：详细操作指南请见 [USER_MANUAL.md](USER_MANUAL.md)
- **开发日志**：开发过程记录请见 `walkthrough.md`

## 快速参考

查看 `USER_MANUAL.md` 获取完整的配置和故障排除指南。
