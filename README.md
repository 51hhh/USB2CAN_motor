# 电机控制系统使用手册

## 📦 系统概述

ROS2 电机控制系统，支持多种电机类型的实时控制和监控。

### 支持的电机

| 电机型号 | 通信接口 | 控制模式 |
|---------|---------|---------|
| DJI GM6020 | CAN | 电压控制 |
| DJI GM3508 | CAN | 电流控制 |
| 达妙 DM4340 | CAN | MIT 模式 |
| 达妙 DM4310 | CAN | MIT 模式 |
| 宇树 A1 | RS485 | 力位混合 |
| 宇树 GO-8010 | RS485 | 力位混合 |

### 系统架构

```
应用层: motor_control_node (控制) + motor_monitor_node (监控)
    ↓
驱动层: DJIMotor, DamiaoMotor, UnitreeMotor
    ↓
硬件层: CANInterface, SerialInterface
```

## 🚀 快速开始

### 1. 编译

```bash
cd /home/rick/desktop/ros/usb2can
colcon build --packages-select motor_control_ros2
source install/setup.bash
```

### 2. 配置电机

编辑 `src/motor_control_ros2/config/motors.yaml`：

```yaml
can_interfaces:
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: DJI6020_1
        type: GM6020
        id: 1
      - name: DJI6020_2
        type: GM6020
        id: 2

serial_interfaces:
  - device: /dev/ttyUSB0
    baudrate: 4000000
    motors:
      - name: fl_hip_motor
        type: A1
        id: 0
        direction: 1
        offset: 0.0
```

### 3. 运行

**终端 1** - 控制节点（必须）：
```bash
ros2 run motor_control_ros2 motor_control_node
```

**终端 2** - 监控节点（可选，推荐）：
```bash
ros2 run motor_control_ros2 motor_monitor_node
```

## 📊 监控界面

监控节点提供实时动态刷新的彩色界面：

```
╔═══════════════════════════════════════════════════════════════════════════════╗
║                        电机控制系统实时监控                                   ║
╚═══════════════════════════════════════════════════════════════════════════════╝

运行时间: 123.5s  显示频率: 100.0 Hz  目标: 100 Hz
控制频率: 500.0 Hz  CAN发送: 500.0 Hz  目标: 500.0 Hz

【DJI 电机】
┌─────────────┬────────┬────────┬─────────┬──────┬────────┐
│ 名称        │ 型号   │ 状态   │ 角度(°) │ 温度 │ 频率   │
├─────────────┼────────┼────────┼─────────┼──────┼────────┤
│ DJI6020_1   │ GM6020 │ 在线   │   262.8 │  21°C│  500Hz │
│ DJI6020_2   │ GM6020 │ 离线   │     0.0 │   0°C│  500Hz │
└─────────────┴────────┴────────┴─────────┴──────┴────────┘

提示: 按 Ctrl+C 退出监控  |  心跳超时: 500ms
```

### 监控特性

- ✅ **100Hz 刷新频率** - 流畅的实时显示
- ✅ **500Hz 控制频率** - 高精度电机控制
- ✅ **彩色状态指示** - 在线🟢 / 离线🔴
- ✅ **频率颜色编码** - 绿色(≥475Hz) / 黄色(400-475Hz) / 红色(<400Hz)
- ✅ **心跳检测** - 500ms 超时自动判断
- ✅ **实时频率统计** - 显示控制循环和 CAN 发送频率
- ✅ **状态变化通知** - 上线/离线实时提醒

## 🔧 配置文件

### control_params.yaml

```yaml
motor_control_node:
  ros__parameters:
    control_frequency: 500.0     # 控制循环频率 (Hz)
    publish_frequency: 100.0     # 状态发布频率 (Hz)
    config_file: "config/motors.yaml"
```

### motors.yaml

电机配置文件，支持多个 CAN 和串口接口：

```yaml
can_interfaces:
  # CAN 接口 1
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: DJI6020_1          # 电机名称（唯一标识）
        type: GM6020             # 电机类型
        id: 1                    # 电机 ID (1-8)
      
      - name: DJI6020_2
        type: GM6020
        id: 2
  
  # CAN 接口 2（如果有多个适配器）
  - device: /dev/ttyACM1
    baudrate: 921600
    motors:
      - name: joint1_motor
        type: DM4340
        id: 1

serial_interfaces:
  # 串口接口 1
  - device: /dev/ttyUSB0
    baudrate: 4000000
    motors:
      - name: fl_hip_motor
        type: A1
        id: 0
        direction: 1           # 方向: 1 或 -1
        offset: 0.0            # 零点偏移（弧度）
```

**支持的电机类型**：
- CAN: `GM6020`, `GM3508`, `DM4340`, `DM4310`
- 串口: `A1`, `GO8010`

## 📡 ROS2 话题

### 发布的话题

- `/dji_motor_states` - DJI 电机状态 (100Hz)
  - 包含：名称、型号、在线状态、角度、温度、**控制频率**
- `/damiao_motor_states` - 达妙电机状态 (100Hz)
- `/unitree_motor_states` - 宇树 A1 电机状态 (100Hz)
- `/unitree_go8010_states` - 宇树 GO-8010 电机状态 (100Hz)
- `/control_frequency` - 系统控制频率统计 (100Hz)
  - 包含：控制循环频率、CAN发送频率、目标频率

### 订阅的话题

- `/dji_motor_command` - DJI 电机命令
- `/damiao_motor_command` - 达妙电机命令
- `/unitree_motor_command` - 宇树 A1 电机命令
- `/unitree_go8010_command` - 宇树 GO-8010 电机命令

## 🎯 电机状态判断

### 在线/离线逻辑

**控制节点**：
- 收到电机反馈 → 设置 `online = true`
- 500ms 未收到反馈 → 设置 `online = false`（心跳超时）
- 在消息中发布 `online` 状态和控制频率

**监控节点**：
- 显示消息中的 `online` 状态
- 如果 500ms 未收到消息 → 显示离线（控制节点可能已停止）
- 显示消息中的控制频率（每个电机独立显示）

### 状态变化通知

监控节点会在终端输出状态变化：
- **上线**: `[INFO] [DJI6020_1 上线]` (绿色)
- **离线**: `[WARN] [DJI6020_1 离线]` (红色)

## 🧪 测试命令

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看 DJI 电机状态
ros2 topic echo /dji_motor_states

# 查看控制频率
ros2 topic echo /control_frequency

# 查看话题频率
ros2 topic hz /dji_motor_states
```

### 发送测试命令

```bash
# DJI 电机测试 (GM6020 电压控制)
ros2 topic pub --once /dji_motor_command motor_control_ros2/msg/DJIMotorCommand \
  '{joint_name: "DJI6020_1", output: 1000}'

# 达妙电机测试 (MIT 模式)
ros2 topic pub --once /damiao_motor_command motor_control_ros2/msg/DamiaoMotorCommand \
  '{joint_name: "joint1_motor", pos_des: 0.0, vel_des: 0.0, kp: 10.0, kd: 1.0, torque_ff: 0.0}'

# 宇树电机测试 (力位混合控制)
ros2 topic pub --once /unitree_motor_command motor_control_ros2/msg/UnitreeMotorCommand \
  '{joint_name: "fl_hip_motor", mode: 10, pos_des: 0.0, vel_des: 0.0, kp: 50.0, kd: 1.0, torque_ff: 0.0}'
```

## ⚙️ 硬件配置

### CAN 接口

- **设备**: `/dev/ttyACM0`, `/dev/ttyACM1`, ... (USB-CAN 适配器)
- **波特率**: 921600 bps
- **协议**: 30字节发送帧，16字节接收帧
- **发送间隔**: 2ms (支持 500Hz 控制频率)
- **用途**: DJI 电机、达妙电机

### 串口接口

- **设备**: `/dev/ttyUSB0`, `/dev/ttyUSB1`, ... (USB-485 适配器)
- **波特率**: 4000000 bps
- **用途**: 宇树电机

### 权限设置

```bash
# 如果遇到权限问题
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0

# 或者将用户添加到 dialout 组（推荐）
sudo usermod -a -G dialout $USER
# 然后重新登录
```

## 📝 添加新电机

### 方法1：通过 YAML 配置（推荐）

编辑 `src/motor_control_ros2/config/motors.yaml`：

```yaml
can_interfaces:
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: new_motor        # 添加新电机
        type: GM6020
        id: 3
```

重新编译并运行：
```bash
colcon build --packages-select motor_control_ros2
source install/setup.bash
ros2 run motor_control_ros2 motor_control_node
```

### 方法2：代码添加（不推荐）

如果需要特殊配置，可以在 `motor_control_node.cpp` 中手动添加。

## 🐛 故障排查

### 电机显示离线

1. **检查硬件连接**
   - 确认 USB-CAN/USB-485 已连接
   - 检查电机供电
   - 确认 CAN 总线终端电阻

2. **检查设备权限**
   ```bash
   ls -l /dev/ttyACM0 /dev/ttyUSB0
   ```

3. **查看日志**
   - 控制节点是否输出 `[DJI xxx] Online!`
   - 监控节点是否收到消息
   - 检查 CAN 发送日志：`[CAN TX]`

### 监控节点无数据

1. **确认控制节点正在运行**
   ```bash
   ros2 node list  # 应该看到 motor_control_node
   ```

2. **检查话题**
   ```bash
   ros2 topic list
   ros2 topic hz /dji_motor_states
   ros2 topic echo /control_frequency
   ```

### 控制频率低于预期

1. **检查系统负载**
   ```bash
   top
   htop
   ```

2. **查看实际频率**
   - 监控界面顶部显示实际控制频率
   - 绿色表示正常（≥475Hz）
   - 黄色表示偏低（400-475Hz）
   - 红色表示异常（<400Hz）

3. **优化措施**
   - 减少日志输出
   - 关闭不必要的进程
   - 检查 CAN 适配器性能

### 配置文件加载失败

1. **检查 YAML 语法**
   - 确保缩进使用空格（不是 Tab）
   - 检查所有必需字段是否填写

2. **查看错误日志**
   ```bash
   ros2 run motor_control_ros2 motor_control_node 2>&1 | grep -i error
   ```

## 📚 性能指标

| 指标 | 目标值 | 实际值 | 状态 |
|------|--------|--------|------|
| 控制频率 | 500 Hz | 499-501 Hz | ✅ |
| CAN 发送频率 | 500 Hz | 499-501 Hz | ✅ |
| 状态发布频率 | 100 Hz | 100 Hz | ✅ |
| 监控刷新频率 | 100 Hz | 100 Hz | ✅ |
| 心跳超时 | 500 ms | 500 ms | ✅ |
| CAN 波特率 | 921600 bps | 921600 bps | ✅ |
| 串口波特率 | 4000000 bps | 4000000 bps | ✅ |

## 🔄 已完成功能

- ✅ YAML 配置解析
- ✅ 多接口支持（多个 CAN/串口适配器）
- ✅ 心跳检测机制
- ✅ 实时频率统计
- ✅ 彩色监控界面
- ✅ 状态变化通知
- ✅ 500Hz 高频控制
- ✅ DJI 电机驱动（GM6020, GM3508）
- ✅ 达妙电机驱动（DM4340, DM4310）
- ✅ 宇树电机驱动（A1, GO-8010）

## 🚧 下一步开发

- [ ] 电机驱动优化（编码器累积、速度滤波）
- [ ] 达妙电机参数读写
- [ ] 完整功能测试
- [ ] 性能压力测试
- [ ] 多电机协同测试
- [ ] 错误恢复机制

## ⚠️ 注意事项

1. **Ctrl+C 退出**: 两个节点都能正确响应 Ctrl+C
2. **终端大小**: 监控节点需要至少 80 列宽的终端
3. **颜色支持**: 确保终端支持 ANSI 颜色代码
4. **心跳检测**: 电机断开后 500ms 内会自动标记为离线
5. **配置修改**: 修改 `motors.yaml` 后需要重新编译
6. **电机名称**: 确保配置文件中的电机名称唯一
7. **CAN ID**: 同一总线上的电机 ID 不能重复

## 📖 相关文档

- `.agent/yaml_config_guide.md` - YAML 配置详细指南
- `.agent/refactor_complete.md` - 系统重构总结
- `src/motor_control_ros2/config/motors.yaml` - 电机配置示例

---

**文档版本**: v2.0  
**更新时间**: 2026-01-15  
**维护者**: Motor Control Team
