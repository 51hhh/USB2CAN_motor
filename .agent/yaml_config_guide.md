# 🎉 YAML 配置功能完成

## ✅ 新功能

现在支持通过 YAML 配置文件轻松添加电机，无需修改代码！

### 配置文件位置

```
src/motor_control_ros2/config/motors.yaml
```

### 配置示例

```yaml
can_interfaces:
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: yaw_motor
        type: GM6020
        id: 1
      
      - name: pitch_motor
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

## 🚀 使用方法

### 1. 编辑配置文件

```bash
cd /home/rick/desktop/ros/usb2can
nano src/motor_control_ros2/config/motors.yaml
```

### 2. 添加您的电机

取消注释需要的部分，修改参数：

```yaml
can_interfaces:
  - device: /dev/ttyACM0    # USB-CAN 设备路径
    baudrate: 921600         # 波特率
    motors:
      - name: yaw_motor      # 电机名称（唯一）
        type: GM6020         # 电机类型
        id: 1                # 电机 ID
```

### 3. 编译并运行

```bash
colcon build --packages-select motor_control_ros2
source install/setup.bash
ros2 run motor_control_ros2 motor_control_node
```

## 📋 支持的电机类型

### CAN 电机

| 类型 | 说明 |
|------|------|
| `GM6020` | DJI GM6020 云台电机 |
| `GM3508` | DJI GM3508 动力电机 |
| `DM4340` | 达妙 DM4340 |
| `DM4310` | 达妙 DM4310 |

### 串口电机

| 类型 | 说明 |
|------|------|
| `A1` | 宇树 A1 电机 |
| `GO8010` | 宇树 GO-8010 电机 |

## 🔧 配置参数说明

### CAN 接口

```yaml
- device: /dev/ttyACM0      # 设备路径
  baudrate: 921600          # 波特率
  motors:                   # 电机列表
    - name: motor_name      # 电机名称（必须唯一）
      type: GM6020          # 电机类型
      id: 1                 # 电机 ID (1-8)
```

### 串口接口

```yaml
- device: /dev/ttyUSB0      # 设备路径
  baudrate: 4000000         # 波特率
  motors:                   # 电机列表
    - name: motor_name      # 电机名称（必须唯一）
      type: A1              # 电机类型
      id: 0                 # 电机 ID
      direction: 1          # 方向 (1 或 -1)
      offset: 0.0           # 零点偏移（弧度）
```

## 💡 使用技巧

### 多个 CAN 适配器

```yaml
can_interfaces:
  - device: /dev/ttyACM0
    baudrate: 921600
    motors:
      - name: yaw_motor
        type: GM6020
        id: 1
  
  - device: /dev/ttyACM1    # 第二个适配器
    baudrate: 921600
    motors:
      - name: joint1_motor
        type: DM4340
        id: 1
```

### 多个串口适配器

```yaml
serial_interfaces:
  - device: /dev/ttyUSB0
    baudrate: 4000000
    motors:
      - name: fl_hip_motor
        type: A1
        id: 0
  
  - device: /dev/ttyUSB1    # 第二个适配器
    baudrate: 4000000
    motors:
      - name: fr_hip_motor
        type: GO8010
        id: 0
```

## ⚠️ 注意事项

1. **电机名称必须唯一** - 不能有重复的名称
2. **设备路径必须正确** - 确保设备已连接
3. **ID 不能冲突** - 同一总线上的电机 ID 不能重复
4. **修改后需重新编译** - 配置文件会被安装到 share 目录

## 🐛 故障排查

### 配置加载失败

如果看到错误：`配置加载失败: ...`

1. 检查 YAML 语法是否正确
2. 确保缩进使用空格（不是 Tab）
3. 检查所有必需字段是否填写

### 电机未初始化

如果电机没有出现：

1. 检查配置文件中是否取消了注释
2. 确认电机类型拼写正确
3. 查看日志中的错误信息

## 📊 启动日志示例

成功加载配置后，您会看到：

```
[ConfigParser] 成功加载配置文件: .../motors.yaml
[ConfigParser] CAN 接口数: 1
[ConfigParser] 串口接口数: 0
[ConfigParser] CAN 接口: /dev/ttyACM0 @ 921600 bps, 1 个电机
[motor_control_node] 添加 DJI 电机: yaw_motor (GM6020, ID=1)
[motor_control_node] 配置加载完成 - DJI 电机: 1, 宇树电机: 0
```

---

**功能完成时间**: 2026-01-15 13:20  
**简化程度**: ⭐⭐⭐⭐⭐  
**易用性**: ⭐⭐⭐⭐⭐
