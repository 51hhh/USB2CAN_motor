# Xbox 手柄控制全向轮底盘使用指南

## 🎮 系统架构

```
Xbox 手柄 → joy_node (ROS2 joy包) → /joy
                                      ↓
                          joystick_control_node → /cmd_vel
                                                     ↓
                             omni_chassis_control_node → /dji_motor_command_advanced
                                                             ↓
                                        motor_control_node → CAN总线 → GM3508电机
```

## 📦 功能特性

### 🎯 手柄控制节点 (joystick_control_node)
- **左摇杆**：控制底盘前后/左右平移
  - 上/下：前进/后退 (Y轴)
  - 左/右：左移/右移 (X轴)
- **右摇杆**：控制底盘旋转 (X轴)
- **RB按钮**：Turbo模式，速度翻倍加速
- **死区处理**：防止摇杆漂移（默认±0.1）
- **速度平滑**：指数滤波，防止速度突变
- **安全保护**：手柄断连自动停止

### 🚗 底盘控制节点 (omni_chassis_control_node)
- **X形全向轮运动学**：45°斜向安装的4轮全向底盘
- **速度控制模式**：发送目标角速度到电机
- **里程计发布**：根据电机反馈计算底盘位置 `/odom`
- **速度限制**：可配置最大线速度和角速度
- **命令超时保护**：0.5秒无命令自动停止

## 🔧 配置文件

### 1. 手柄参数 (`config/joystick_params.yaml`)

```yaml
joystick_control_node:
  ros__parameters:
    # 摇杆映射
    axis_linear_x: 1      # 左摇杆上下 (前后)
    axis_linear_y: 0      # 左摇杆左右 (左右)
    axis_angular: 2       # 右摇杆左右 (旋转)
    
    # Turbo按钮
    button_turbo: 5       # RB按钮 (索引从0开始)
    
    # 速度限制
    max_linear_velocity: 1.0    # m/s
    max_angular_velocity: 1.57  # rad/s (约90度/秒)
    turbo_multiplier: 2.0       # Turbo倍数
    
    # 死区
    deadzone: 0.1               # 摇杆死区 [0.0-1.0]
    
    # 平滑滤波
    filter_alpha: 0.3           # 低通滤波系数，越小越平滑
    
    # 超时保护
    joy_timeout: 0.5            # 手柄断连超时 (秒)
```

### 2. 底盘参数 (`config/omni_chassis_params.yaml`)

```yaml
omni_chassis_control_node:
  ros__parameters:
    # 机械参数
    wheel_base_x: 0.88      # 前后轮距 (m)
    wheel_base_y: 0.88      # 左右轮距 (m)
    wheel_radius: 0.075     # 轮子半径 (m)
    install_angle: 45.0     # 全向轮安装角度
    
    # 控制参数
    control_frequency: 100.0           # 控制循环频率 (Hz)
    velocity_filter_alpha: 0.3         # 速度滤波系数
    
    # 速度限制
    max_linear_velocity: 2.0    # m/s
    max_angular_velocity: 3.14  # rad/s
    
    # 电机映射
    fl_motor: "DJI3508_1"      # 左前
    fr_motor: "DJI3508_2"      # 右前
    rl_motor: "DJI3508_3"      # 左后
    rr_motor: "DJI3508_4"      # 右后
```

### 3. 电机配置 (`config/motors.yaml`)

确保配置了4个GM3508电机：
```yaml
motors:
  dji_motors:
    - name: "DJI3508_1"
      can_interface: "can0"
      can_id: 1
      model: "GM3508"
      
    - name: "DJI3508_2"
      can_interface: "can0"
      can_id: 2
      model: "GM3508"
      
    - name: "DJI3508_3"
      can_interface: "can0"
      can_id: 3
      model: "GM3508"
      
    - name: "DJI3508_4"
      can_interface: "can0"
      can_id: 4
      model: "GM3508"
```

## 🚀 启动流程

### 方法1：一键启动（推荐）

```bash
cd /home/rosemaryrabbit/USB2CAN_motor
./start_joystick_control.sh
```
 
### 方法2：分步启动

#### 步骤1：插入Xbox手柄接收器并确认设备
```bash
# 查看joy设备
ls -l /dev/input/js*
# 应该看到 /dev/input/js0 或类似设备

# 测试手柄输入
ros2 run joy joy_node
# 另一个终端查看数据
ros2 topic echo /joy
```

#### 步骤2：启动底层电机控制节点
```bash
cd /home/rosemaryrabbit/USB2CAN_motor
source install/setup.bash

# 启动电机控制节点
ros2 run motor_control_ros2 motor_control_node \
  --ros-args --params-file src/motor_control_ros2/config/motors.yaml \
             --params-file src/motor_control_ros2/config/pid_params.yaml \
             --params-file src/motor_control_ros2/config/control_params.yaml
```

#### 步骤3：启动全向轮底盘控制节点
```bash
source install/setup.bash

ros2 run motor_control_ros2 omni_chassis_control_node \
  --ros-args --params-file src/motor_control_ros2/config/omni_chassis_params.yaml
```

#### 步骤4：启动手柄节点
```bash
source install/setup.bash

# 启动ROS2 joy节点
ros2 run joy joy_node

# 另一个终端启动手柄控制节点
ros2 run motor_control_ros2 joystick_control_node \
  --ros-args --params-file src/motor_control_ros2/config/joystick_params.yaml
```

## 🎮 Xbox手柄按键映射

```
          [LB]  [RB (Turbo)]
            \    /
             \  /
    [Left]   \/   [Right]
     摇杆         摇杆
      |            |
      └─ 平移      └─ 旋转
     (X/Y)         (X)
```

### 标准Xbox手柄轴映射
- **Axis 0**: 左摇杆 X (左 -1.0 → 右 +1.0)
- **Axis 1**: 左摇杆 Y (上 -1.0 → 下 +1.0)
- **Axis 2**: 右摇杆 X (左 -1.0 → 右 +1.0)
- **Axis 3**: 右摇杆 Y (上 -1.0 → 下 +1.0)
- **Axis 4**: LT (松开 0.0 → 按下 +1.0)
- **Axis 5**: RT (松开 0.0 → 按下 +1.0)

### 按钮映射
- **Button 0**: A
- **Button 1**: B
- **Button 2**: X
- **Button 3**: Y
- **Button 4**: LB
- **Button 5**: RB (Turbo)
- **Button 6**: Back
- **Button 7**: Start

## 🔍 调试与监控

### 查看话题数据
```bash
# 手柄原始数据
ros2 topic echo /joy

# 速度命令
ros2 topic echo /cmd_vel

# 电机命令
ros2 topic echo /dji_motor_command_advanced

# 电机状态
ros2 topic echo /dji_motor_states

# 里程计
ros2 topic echo /odom
```

### 查看节点图
```bash
# 安装图形工具
sudo apt install ros-humble-rqt-graph

# 查看节点连接
rqt_graph
```

### 监控电机状态
```bash
source install/setup.bash
ros2 run motor_control_ros2 motor_monitor_node
```

## ⚙️ 参数调优

### 1. 改善控制响应性
```yaml
# joystick_params.yaml
filter_alpha: 0.5  # 增大 → 响应更快，但更抖动

# omni_chassis_params.yaml
velocity_filter_alpha: 0.5  # 增大 → 响应更快
```

### 2. 增加平滑度
```yaml
filter_alpha: 0.1  # 减小 → 更平滑，但响应慢
velocity_filter_alpha: 0.1
```

### 3. 调整速度限制
```yaml
# joystick_params.yaml
max_linear_velocity: 2.0    # 增大 → 更快
max_angular_velocity: 3.14  # 增大 → 转得更快
turbo_multiplier: 3.0       # Turbo模式更猛
```

### 4. 调整摇杆死区
```yaml
deadzone: 0.15  # 增大 → 摇杆需要推得更远才响应
deadzone: 0.05  # 减小 → 更灵敏，但可能漂移
```

## ❗ 常见问题

### Q1: 手柄无响应
```bash
# 检查设备
ls -l /dev/input/js*

# 测试joy节点
ros2 run joy joy_node
ros2 topic echo /joy  # 移动摇杆应该看到数据变化

# 检查权限
sudo chmod 666 /dev/input/js0
```

### Q2: 电机不转
```bash
# 检查话题连接
ros2 topic list | grep motor

# 检查电机状态
ros2 topic echo /dji_motor_states

# 检查CAN接口
ip link show can0
# 应该看到 UP 状态
```

### Q3: 底盘运动方向错误
- 检查 `omni_chassis_params.yaml` 中的电机映射
- 可能需要调整 fl_motor, fr_motor, rl_motor, rr_motor 的顺序
- 或者在运动学中调整符号

### Q4: 速度太快/太慢
- 调整 `joystick_params.yaml` 中的 `max_linear_velocity`
- 调整 `omni_chassis_params.yaml` 中的 `wheel_radius`（如果实际半径不对）

### Q5: 里程计不准
- 检查 `wheel_radius` 是否准确测量
- 检查 `wheel_base_x` 和 `wheel_base_y` 是否正确
- 全向轮在侧滑时里程计会有误差（这是正常的）

## 🛡️ 安全注意事项

1. **首次测试时**：将底盘架空，确保电机转向正确
2. **急停方案**：松开摇杆，0.5秒后自动停止
3. **断连保护**：手柄断连后0.5秒自动停止
4. **速度限制**：初次使用时降低 `max_linear_velocity`
5. **测试区域**：在空旷区域测试，注意周围障碍物

## 📊 性能指标

- **控制频率**: 100 Hz
- **手柄更新**: ~30 Hz
- **里程计更新**: 100 Hz
- **命令延迟**: <20ms
- **CAN总线负载**: ~40%

## 🔧 高级配置

### 使用不同的手柄
如果使用非Xbox手柄，需要修改 `joystick_params.yaml` 中的轴和按钮映射：

```bash
# 测试手柄获取轴/按钮索引
ros2 run joy joy_node
ros2 topic echo /joy

# 移动摇杆/按按钮，记录对应的索引
# 然后更新配置文件
```

### Launch文件（高级用户）
可以创建一个launch文件同时启动所有节点：

```python
# src/motor_control_ros2/launch/joystick_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('motor_control_ros2'),
        'config'
    )
    
    return LaunchDescription([
        # Joy节点
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        
        # 手柄控制节点
        Node(
            package='motor_control_ros2',
            executable='joystick_control_node',
            name='joystick_control_node',
            parameters=[os.path.join(config_dir, 'joystick_params.yaml')]
        ),
        
        # 底盘控制节点
        Node(
            package='motor_control_ros2',
            executable='omni_chassis_control_node',
            name='omni_chassis_control_node',
            parameters=[os.path.join(config_dir, 'omni_chassis_params.yaml')]
        ),
    ])
```

使用launch文件：
```bash
ros2 launch motor_control_ros2 joystick_control.launch.py
```

## 📚 相关文档

- [ROS2 Joy包文档](https://index.ros.org/p/joy/)
- [Twist消息定义](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [全向轮运动学原理](./src/motor_control_ros2/include/motor_control_ros2/omni_wheel_kinematics.hpp)

---

**版本**: 1.0  
**最后更新**: 2026-01-24  
**维护者**: rosemaryrabbit
