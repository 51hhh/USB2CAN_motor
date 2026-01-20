# 舵轮底盘电机零位和方向配置指南

## 📋 概述

由于机械安装的物理限制，四轮独立转向底盘的电机存在以下差异：

1. **GM6020 转向电机零位差异**：每个转向电机安装时的初始位置不同，导致编码器零点不一致
2. **GM3508 驱动电机方向差异**：由于对称安装，左右两侧电机的正转方向相反

本配置系统通过 `chassis_params.yaml` 解决这些问题，无需修改代码。

---

## ⚙️ 配置参数说明

### 1. 转向电机零位偏移 (`steer_offset`)

**作用**：补偿机械安装导致的编码器零点差异

**单位**：度（°）

**含义**：当舵轮朝向正前方时，GM6020 编码器显示的角度值

**示例**：
```yaml
fl_steer_offset: 45.0   # 左前转向电机朝向正前方时，编码器显示 45°
fr_steer_offset: -30.0  # 右前转向电机朝向正前方时，编码器显示 -30°
```

### 2. 驱动电机方向 (`drive_direction`)

**作用**：修正对称安装导致的电机旋转方向差异

**取值**：
- `1`：正转时轮子向前滚动
- `-1`：正转时轮子向后滚动（需要反向）

**默认配置**（对称安装）：
```yaml
fl_drive_direction: 1    # 左前：正转向前
fr_drive_direction: -1   # 右前：正转向后（对称安装）
rl_drive_direction: 1    # 左后：正转向前
rr_drive_direction: -1   # 右后:正转向后（对称安装）
```

---

## 🔧 零位标定步骤

### 步骤 1：手动调整舵轮朝向

1. 启动电机控制节点：
   ```bash
   ros2 run motor_control_ros2 motor_control_node
   ```

2. 在另一个终端启动监控节点：
   ```bash
   ros2 run motor_control_ros2 motor_monitor_node
   ```

3. **手动将每个舵轮调整到正前方位置**（与底盘纵向轴线平行）

### 步骤 2：记录编码器角度

在监控界面中，记录每个转向电机的当前角度：

```
【DJI 电机】
┌─────────────┬────────┬────────┬─────────┬──────┬────────┐
│ 名称        │ 型号   │ 状态   │ 角度(°) │ 温度 │ 频率   │
├─────────────┼────────┼────────┼─────────┼──────┼────────┤
│ DJI6020_1   │ GM6020 │ 在线   │   45.2  │  21°C│  500Hz │  ← 左前零位
│ DJI6020_2   │ GM6020 │ 在线   │  -30.8  │  22°C│  500Hz │  ← 右前零位
│ DJI6020_3   │ GM6020 │ 在线   │   12.5  │  21°C│  500Hz │  ← 左后零位
│ DJI6020_4   │ GM6020 │ 在线   │  -15.3  │  22°C│  500Hz │  ← 右后零位
└─────────────┴────────┴────────┴─────────┴──────┴────────┘
```

### 步骤 3：更新配置文件

编辑 `src/motor_control_ros2/config/chassis_params.yaml`：

```yaml
chassis_control_node:
  ros__parameters:
    # ... 其他参数 ...
    
    # 左前舵轮单元
    fl_steer_motor: "DJI6020_1"
    fl_drive_motor: "DJI3508_1"
    fl_steer_offset: 45.2        # ← 填入记录的角度
    fl_drive_direction: 1
    
    # 右前舵轮单元
    fr_steer_motor: "DJI6020_2"
    fr_drive_motor: "DJI3508_2"
    fr_steer_offset: -30.8       # ← 填入记录的角度
    fr_drive_direction: -1
    
    # 左后舵轮单元
    rl_steer_motor: "DJI6020_3"
    rl_drive_motor: "DJI3508_3"
    rl_steer_offset: 12.5        # ← 填入记录的角度
    rl_drive_direction: 1
    
    # 右后舵轮单元
    rr_steer_motor: "DJI6020_4"
    rr_drive_motor: "DJI3508_4"
    rr_steer_offset: -15.3       # ← 填入记录的角度
    rr_drive_direction: -1
```

### 步骤 4：验证配置

1. 重启底盘控制节点：
   ```bash
   ros2 run motor_control_ros2 chassis_control_node
   ```

2. 发送前进命令测试：
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

3. **验证点**：
   - ✅ 所有舵轮应该朝向正前方（0°）
   - ✅ 所有驱动轮应该向前滚动
   - ✅ 底盘应该直线前进

---

## 🧪 方向测试步骤

### 测试 1：直线前进

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**预期结果**：
- 所有舵轮朝向 0° (正前方)
- 所有驱动轮向前滚动
- 底盘直线前进

### 测试 2：横向平移

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**预期结果**：
- 所有舵轮朝向 90° (正左方)
- 底盘向左平移

### 测试 3：原地旋转

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

**预期结果**：
- 四个舵轮形成切向排列
- 底盘逆时针旋转

---

## 🔍 故障排查

### 问题 1：底盘不能直线前进，向一侧偏移

**可能原因**：驱动电机方向配置错误

**解决方法**：
1. 观察哪个轮子转向相反
2. 将对应的 `drive_direction` 取反（1 ↔ -1）

### 问题 2：舵轮转向角度不正确

**可能原因**：转向零位偏移配置错误

**解决方法**：
1. 重新标定零位（参考步骤 1-3）
2. 确保手动调整时舵轮真正朝向正前方

### 问题 3：舵轮转向时选择远路径（转 270° 而不是 90°）

**可能原因**：零位偏移符号错误

**解决方法**：
1. 将对应的 `steer_offset` 取反（正 ↔ 负）
2. 重新测试

---

## 📐 工作原理

### 转向控制

```
目标角度(发送给电机) = 运动学计算角度 + 零位偏移

例如：
- 运动学计算：舵轮应朝向 0° (正前方)
- 零位偏移：fl_steer_offset = 45.0°
- 实际发送：0° + 45° = 45° (编码器值)
```

### 驱动控制

```
目标速度(发送给电机) = 运动学计算速度 × 方向系数

例如：
- 运动学计算：轮速 = 100 RPM
- 方向系数：fr_drive_direction = -1
- 实际发送：100 × (-1) = -100 RPM (反转)
```

### 角度优化

在舵角优化时，使用**修正后的当前角度**：

```cpp
double current_angle = motor_states_[motor_name].angle - steer_offset;
```

这确保优化算法基于**机械零点**而非**编码器零点**进行计算。

---

## 📝 配置文件模板

完整的 `chassis_params.yaml` 配置示例：

```yaml
chassis_control_node:
  ros__parameters:
    # 控制频率
    control_frequency: 100.0  # Hz
    
    # 底盘几何参数
    wheel_base_x: 0.65    # 前后轮距 (m)
    wheel_base_y: 0.62    # 左右轴距 (m)
    wheel_radius: 0.055   # 轮子半径 (m)
    
    # 速度限制
    max_linear_velocity: 2.0   # 最大线速度 (m/s)
    max_angular_velocity: 3.14 # 最大角速度 (rad/s)
    
    # 电机映射和配置
    # 左前舵轮单元
    fl_steer_motor: "DJI6020_1"
    fl_drive_motor: "DJI3508_1"
    fl_steer_offset: 0.0         # ← 需要标定
    fl_drive_direction: 1
    
    # 右前舵轮单元
    fr_steer_motor: "DJI6020_2"
    fr_drive_motor: "DJI3508_2"
    fr_steer_offset: 0.0         # ← 需要标定
    fr_drive_direction: -1
    
    # 左后舵轮单元
    rl_steer_motor: "DJI6020_3"
    rl_drive_motor: "DJI3508_3"
    rl_steer_offset: 0.0         # ← 需要标定
    rl_drive_direction: 1
    
    # 右后舵轮单元
    rr_steer_motor: "DJI6020_4"
    rr_drive_motor: "DJI3508_4"
    rr_steer_offset: 0.0         # ← 需要标定
    rr_drive_direction: -1
```

---

## ✅ 配置检查清单

- [ ] 已手动将所有舵轮调整到正前方位置
- [ ] 已记录所有转向电机的编码器角度
- [ ] 已更新 `chassis_params.yaml` 中的 `steer_offset` 值
- [ ] 已根据机械安装配置 `drive_direction` 值
- [ ] 已重新编译 ROS2 包（如果修改了代码）
- [ ] 已测试直线前进功能
- [ ] 已测试横向平移功能
- [ ] 已测试原地旋转功能
- [ ] 底盘运动符合预期，无异常

---

**文档版本**: v1.0  
**更新时间**: 2026-01-20  
**适用系统**: motor_control_ros2 (四轮独立转向底盘)
