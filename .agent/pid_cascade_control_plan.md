# DJI 电机 PID 多环串级控制实现方案

## 📋 需求分析

### 当前状态
- ✅ DJI GM6020: 仅支持电压控制（直接输出）
- ✅ DJI GM3508: 仅支持电流控制（直接输出）
- ❌ 缺少位置控制和速度控制

### 目标功能
1. **通用 PID 控制器** - 支持 P/I/D 参数配置、积分限幅、输出限幅、死区
2. **多环串级控制** - 位置环 → 速度环 → 电流/电压环
3. **每个电机独立配置** - 每种电机、每个环路的 PID 参数可独立配置
4. **控制模式切换** - 支持位置控制、速度控制、直接输出（原始模式）

## 🏗️ 架构设计

### 控制链路

```
位置控制模式:
  位置目标 → [位置PID] → 速度目标 → [速度PID] → 电流/电压目标 → 电机

速度控制模式:
  速度目标 → [速度PID] → 电流/电压目标 → 电机

直接输出模式:
  电流/电压目标 → 电机
```

### 控制环配置

| 电机型号 | 输出类型 | 位置环 | 速度环 | 电流环 |
|---------|---------|--------|--------|--------|
| GM6020  | 电压控制 | ✅ | ✅ | ❌ (直接输出电压) |
| GM3508  | 电流控制 | ✅ | ✅ | ❌ (直接输出电流) |

## 📁 文件结构

### 新增文件

1. **include/motor_control_ros2/pid_controller.hpp**
   - 通用 PID 控制器类
   - 支持参数配置、状态重置、死区处理

2. **include/motor_control_ros2/cascade_controller.hpp**
   - 多环串级控制器类
   - 管理位置环、速度环
   - 控制模式切换

3. **msg/DJIMotorCommandAdvanced.msg**
   - 新的高级控制命令消息
   - 支持位置/速度/直接输出三种模式

4. **config/pid_params.yaml**
   - PID 参数配置文件
   - 每个电机每个环路的独立参数

### 修改文件

1. **include/motor_control_ros2/dji_motor.hpp**
   - 添加 CascadeController 成员
   - 添加控制模式枚举
   - 添加位置/速度控制接口

2. **src/dji_motor.cpp**
   - 实现串级控制逻辑
   - 在 500Hz 控制循环中调用 PID 计算

3. **src/motor_control_node.cpp**
   - 订阅新的高级控制命令话题
   - 加载 PID 参数配置

## 🔧 实现细节

### 1. PID 控制器

```cpp
class PIDController {
  // 参数
  double kp_, ki_, kd_;
  double i_max_;      // 积分限幅
  double out_max_;    // 输出限幅
  double dead_zone_;  // 死区
  
  // 状态
  double error_[2];   // 当前和上次误差
  double i_out_;      // 积分累积
  
  // 方法
  double calculate(double target, double feedback, double dt);
  void reset();
};
```

### 2. 串级控制器

```cpp
class CascadeController {
  PIDController position_pid_;
  PIDController velocity_pid_;
  
  ControlMode mode_;  // POSITION / VELOCITY / DIRECT
  
  double update(double pos_target, double vel_target, double direct_output,
                double pos_feedback, double vel_feedback, double dt);
};
```

### 3. 控制模式

```cpp
enum class ControlMode {
  DIRECT,      // 直接输出（原始模式）
  VELOCITY,    // 速度控制
  POSITION     // 位置控制
};
```

### 4. 消息定义

```msg
# DJIMotorCommandAdvanced.msg
std_msgs/Header header
string joint_name

# 控制模式: 0=直接输出, 1=速度控制, 2=位置控制
uint8 MODE_DIRECT = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_POSITION = 2
uint8 mode

# 目标值（根据模式使用）
float64 position_target    # 位置目标（弧度）
float64 velocity_target    # 速度目标（弧度/秒）
int16 direct_output        # 直接输出（电流/电压）
```

### 5. PID 参数配置

```yaml
# pid_params.yaml
dji_motors:
  GM6020:
    position_pid:
      kp: 50.0
      ki: 0.0
      kd: 5.0
      i_max: 10.0
      out_max: 30.0      # 速度限制（rad/s）
      dead_zone: 0.01    # 位置死区（rad）
    
    velocity_pid:
      kp: 500.0
      ki: 10.0
      kd: 1.0
      i_max: 5000.0
      out_max: 30000.0   # 电压限制
      dead_zone: 0.1     # 速度死区（rad/s）
  
  GM3508:
    position_pid:
      kp: 30.0
      ki: 0.0
      kd: 3.0
      i_max: 5.0
      out_max: 20.0
      dead_zone: 0.01
    
    velocity_pid:
      kp: 300.0
      ki: 5.0
      kd: 0.5
      i_max: 3000.0
      out_max: 16384.0   # 电流限制
      dead_zone: 0.1

# 每个电机可以覆盖默认参数
motor_overrides:
  DJI6020_1:
    position_pid:
      kp: 60.0  # 覆盖默认值
```

## 🚀 实现步骤

### Step 1: 创建 PID 控制器
- [x] 创建 `pid_controller.hpp`
- [x] 实现基本 PID 算法
- [x] 添加死区、限幅功能

### Step 2: 创建串级控制器
- [x] 创建 `cascade_controller.hpp`
- [x] 实现多环串级逻辑
- [x] 实现模式切换

### Step 3: 集成到 DJI 电机
- [x] 修改 `dji_motor.hpp/cpp`
- [x] 添加控制模式
- [x] 在控制循环中调用串级控制器

### Step 4: 创建新消息类型
- [x] 创建 `DJIMotorCommandAdvanced.msg`
- [x] 更新 CMakeLists.txt

### Step 5: 创建配置文件
- [x] 创建 `pid_params.yaml`
- [x] 添加默认参数

### Step 6: 修改控制节点
- [x] 加载 PID 参数
- [x] 订阅新的控制命令话题
- [x] 保持向后兼容（旧的 output 命令仍可用）

### Step 7: 测试
- [x] 测试直接输出模式
- [x] 测试速度控制模式
- [x] 测试位置控制模式
- [x] 调整 PID 参数

## 📊 测试命令

```bash
# 1. 直接输出模式（兼容旧命令）
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 0, direct_output: 1000}'

# 2. 速度控制模式
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 1, velocity_target: 3.14}'

# 3. 位置控制模式
ros2 topic pub --once /dji_motor_command_advanced motor_control_ros2/msg/DJIMotorCommandAdvanced \
  '{joint_name: "DJI6020_1", mode: 2, position_target: 1.57}'
```

## ⚠️ 注意事项

1. **控制频率**: PID 计算在 500Hz 控制循环中执行，dt = 0.002s
2. **参数调试**: 初始参数需要根据实际电机响应调整
3. **安全限制**: 所有环路都有输出限幅，防止过大输出
4. **向后兼容**: 保留原有的 `/dji_motor_command` 话题（直接输出模式）
5. **死区处理**: 避免在目标附近震荡
6. **积分饱和**: 使用积分限幅防止积分饱和

## 🎯 预期效果

- ✅ 位置控制精度: ±0.01 rad
- ✅ 速度控制精度: ±0.1 rad/s
- ✅ 响应时间: \u003c 100ms (位置控制)
- ✅ 无超调或小超调 (\u003c 10%)
- ✅ 稳态误差: \u003c 死区阈值
