# ROS2 电机控制系统重构计划 - 纯 C++ 高频控制架构

## 📋 重构目标

将当前的 **C++ + Python 混合架构** 重构为 **纯 C++ 架构**，实现高频率电机控制（目标 500Hz+）。

### 当前架构问题分析
1. **Python 节点开销**：`lifecycle_node.py` 作为配置管理层，增加了通信延迟
2. **双语言维护成本**：C++ 和 Python 代码分离，增加了调试难度
3. **性能瓶颈**：Python GIL 和解释器开销影响实时性
4. **参考成功案例**：`fdilink_ahrs_ROS1` 使用纯 C++ 实现，达到了理想的控制频率

---

## 🎯 重构策略

### 参考架构：fdilink_ahrs_ROS1
- **dm_hw**: 硬件抽象层，直接管理串口和 CAN 通信
- **dm_controllers**: 控制器层，实现电机控制逻辑
- **纯 C++ 实现**：无 Python 依赖，所有配置通过 YAML 加载到 C++ 节点

### 新架构设计

```
motor_control_ros2/
├── include/motor_control_ros2/
│   ├── hardware/                    # 硬件抽象层
│   │   ├── can_interface.hpp        # CAN 总线接口（重构）
│   │   ├── serial_interface.hpp     # 串口接口（重构）
│   │   └── hardware_manager.hpp     # 硬件管理器（新增）
│   ├── motors/                      # 电机驱动层
│   │   ├── motor_base.hpp           # 电机基类（保留）
│   │   ├── dji_motor.hpp            # DJI 电机（优化）
│   │   ├── damiao_motor.hpp         # 达妙电机（优化）
│   │   └── unitree_motor.hpp        # 宇树电机（优化）
│   ├── controllers/                 # 控制器层（新增）
│   │   ├── motor_controller.hpp     # 电机控制器基类
│   │   └── multi_motor_controller.hpp # 多电机协调控制器
│   └── utils/                       # 工具类
│       ├── yaml_parser.hpp          # YAML 配置解析器（新增）
│       └── realtime_tools.hpp       # 实时工具（新增）
├── src/
│   ├── hardware/
│   │   ├── can_interface.cpp
│   │   ├── serial_interface.cpp
│   │   └── hardware_manager.cpp
│   ├── motors/
│   │   ├── dji_motor.cpp
│   │   ├── damiao_motor.cpp
│   │   └── unitree_motor.cpp
│   ├── controllers/
│   │   ├── motor_controller.cpp
│   │   └── multi_motor_controller.cpp
│   ├── utils/
│   │   └── yaml_parser.cpp
│   └── motor_control_node.cpp       # 主节点（重构）
├── config/
│   └── motors.yaml                  # 配置文件（保留）
└── launch/
    └── motor_control.launch.py      # 启动文件（简化）
```

---

## 📝 详细实施步骤

### 阶段 1：硬件层重构（基于 fdilink_ahrs_ROS1/dm_hw）

#### 1.1 CAN 接口优化
**目标**：提升 CAN 通信效率，减少延迟

**参考**：`fdilink_ahrs_ROS1/src/dm_hw/src/damiao.cpp` 的串口 CAN 实现

**改进点**：
- ✅ 当前已实现 30 字节发送帧格式
- ✅ 当前已实现 16 字节接收帧解析
- 🔧 需要优化：
  - 添加发送队列，批量发送减少系统调用
  - 优化接收缓冲区管理（当前使用 `rx_accumulator_`，可以改进）
  - 添加 CAN 错误检测和重传机制

**文件**：
- `include/motor_control_ros2/hardware/can_interface.hpp`（重命名自 `can_driver.hpp`）
- `src/hardware/can_interface.cpp`（重命名自 `can_driver.cpp`）

#### 1.2 串口接口优化
**目标**：支持宇树电机的高速串口通信（4Mbps+）

**参考**：`fdilink_ahrs_ROS1/src/unitree_a1/src/Motor_node.cpp`

**改进点**：
- 添加高波特率支持（4000000 bps）
- 实现宇树协议的帧封装和解析
- 添加 CRC 校验

**文件**：
- `include/motor_control_ros2/hardware/serial_interface.hpp`（重命名自 `serial_port.hpp`）
- `src/hardware/serial_interface.cpp`（重命名自 `serial_port.cpp`）

#### 1.3 硬件管理器（新增）
**目标**：统一管理所有硬件接口，提供统一的初始化和资源管理

**功能**：
- 从 YAML 配置加载硬件设备
- 管理 CAN 总线和串口的生命周期
- 提供硬件状态监控

**文件**：
- `include/motor_control_ros2/hardware/hardware_manager.hpp`
- `src/hardware/hardware_manager.cpp`

---

### 阶段 2：电机驱动层优化

#### 2.1 DJI 电机驱动
**参考当前实现**：`src/motor_control_ros2/src/dji_motor.cpp`

**优化点**：
- ✅ 保留当前的 GM6020/GM3508 实现
- 🔧 添加编码器位置累积（处理 8192 溢出）
- 🔧 添加速度滤波器（减少噪声）

#### 2.2 达妙电机驱动
**参考**：`fdilink_ahrs_ROS1/src/dm_hw/src/damiao.cpp`

**优化点**：
- 实现完整的 MIT 模式控制
- 添加参数读写功能（读取/修改电机内部寄存器）
- 实现控制模式切换（MIT/位置速度/速度/位置力矩）
- 添加电机使能/失能/清零位置命令

**关键协议**：
- 使能：`0xFC` 命令
- 失能：`0xFD` 命令
- 清零：`0xFE` 命令
- MIT 控制：`0x01` 命令 + 5 个参数（位置、速度、Kp、Kd、力矩）

#### 2.3 宇树电机驱动
**参考**：`fdilink_ahrs_ROS1/src/unitree_a1/src/Motor_node.cpp`

**支持型号**：
- Unitree A1（减速比 9.1:1）
- Unitree GO-8010（减速比 6.33:1）

**优化点**：
- 实现完整的串口通信协议
- 添加 CRC 校验
- 实现力位混合控制模式

---

### 阶段 3：配置管理（纯 C++）

#### 3.1 YAML 解析器（新增）
**目标**：在 C++ 中直接解析 YAML 配置，移除 Python 依赖

**依赖库**：`yaml-cpp`

**功能**：
- 解析 `config/motors.yaml`
- 解析 `config/control_params.yaml`
- 提供类型安全的配置访问接口

**文件**：
- `include/motor_control_ros2/utils/yaml_parser.hpp`
- `src/utils/yaml_parser.cpp`

**示例代码结构**：
```cpp
class MotorConfig {
public:
    std::string name;
    std::string type;
    std::string can_bus;
    int motor_id;
    double gear_ratio;
    bool encoder_on_output;
    // ...
};

class ConfigParser {
public:
    bool loadFromFile(const std::string& filepath);
    std::vector<MotorConfig> getMotorConfigs();
    // ...
};
```

---

### 阶段 4：主控制节点重构

#### 4.1 motor_control_node.cpp 重构
**当前问题**：
- 代码耦合度高（414 行单文件）
- 缺少模块化设计

**重构方案**：
```cpp
class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode();
    ~MotorControlNode();

private:
    // 初始化
    void loadConfiguration();
    void initializeHardware();
    void initializeMotors();
    void createRosInterfaces();
    
    // 实时控制循环（500Hz+）
    void controlLoop();
    
    // 硬件管理
    std::unique_ptr<HardwareManager> hw_manager_;
    
    // 电机管理
    std::map<std::string, std::shared_ptr<MotorBase>> motors_;
    
    // ROS 接口
    std::map<std::string, rclcpp::Publisher<...>::SharedPtr> state_publishers_;
    std::map<std::string, rclcpp::Subscription<...>::SharedPtr> command_subscribers_;
    
    // 实时控制
    std::thread control_thread_;
    std::atomic<bool> running_;
    double control_frequency_;  // 从配置读取
};
```

#### 4.2 实时性优化
**关键技术**：
- 使用独立线程运行控制循环（避免 ROS spin 干扰）
- 使用 `std::chrono::high_resolution_clock` 精确计时
- 减少动态内存分配（预分配缓冲区）
- 使用无锁数据结构（`std::atomic`）

**参考代码**：
```cpp
void MotorControlNode::controlLoop() {
    using namespace std::chrono;
    auto period = microseconds(static_cast<int>(1e6 / control_frequency_));
    auto next_time = high_resolution_clock::now() + period;
    
    while (running_) {
        // 1. 读取所有电机反馈（CAN + 串口）
        hw_manager_->readAll();
        
        // 2. 更新电机状态
        for (auto& [name, motor] : motors_) {
            // 电机状态已在 CAN 回调中更新
        }
        
        // 3. 发送控制命令
        for (auto& [name, motor] : motors_) {
            uint32_t can_id;
            uint8_t data[8];
            size_t len;
            motor->getControlFrame(can_id, data, len);
            hw_manager_->sendCAN(motor->getBusId(), can_id, data, len);
        }
        
        // 4. 精确睡眠到下一个周期
        std::this_thread::sleep_until(next_time);
        next_time += period;
    }
}
```

---

### 阶段 5：移除 Python 依赖

#### 5.1 删除文件
- `motor_control_ros2/lifecycle_node.py`
- `motor_control_ros2/__init__.py`
- `setup.py`

#### 5.2 更新 CMakeLists.txt
**移除**：
```cmake
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  motor_control_ros2/lifecycle_node.py
  ...
)
```

**保留**：
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ... C++ 依赖
```

**新增**：
```cmake
# yaml-cpp 依赖
find_package(yaml-cpp REQUIRED)
target_link_libraries(motor_control_lib yaml-cpp)
```

#### 5.3 更新 package.xml
**移除**：
```xml
<buildtool_depend>ament_cmake_python</buildtool_depend>
<depend>rclpy</depend>
```

**新增**：
```xml
<depend>yaml-cpp</depend>
```

---

### 阶段 6：测试与验证

#### 6.1 单元测试
- 测试 YAML 解析器
- 测试 CAN 帧封装/解析
- 测试串口通信

#### 6.2 硬件测试
- DJI GM6020 电压控制测试
- DJI GM3508 电流控制测试
- 达妙 DM4340 MIT 模式测试
- 宇树 A1 力位混合控制测试
- 宇树 GO-8010 力位混合控制测试

#### 6.3 性能测试
- 控制循环频率测试（目标 500Hz）
- CPU 占用率测试
- 延迟测试（命令到执行）

---

## 🔧 关键技术点

### 1. 电机类型与控制模式映射

| 电机型号 | 编码器位置 | 减速比 | 通信方式 | 控制模式 |
|---------|-----------|--------|---------|---------|
| DJI GM6020 | 输出轴 | 1:1 | CAN | 电压控制 |
| DJI GM3508 | 输出轴 | 19:1 | CAN | 电流控制 |
| 达妙 DM4340 | 输出轴（双编码器） | 40:1 | CAN | MIT 模式 |
| 宇树 A1 | 电机侧 | 9.1:1 | 串口 | 力位混合 |
| 宇树 GO-8010 | 电机侧 | 6.33:1 | 串口 | 力位混合 |

### 2. CAN 协议细节

#### 发送帧（30 字节）
```
[0-1]   帧头: 0x55 0xAA
[2]     帧长: 0x1E (30)
[3]     命令: 0x01 (转发 CAN)
[4-7]   发送次数 (小端): 0x01 0x00 0x00 0x00
[8-11]  时间间隔 (小端): 0x0A 0x00 0x00 0x00 (10ms)
[12]    ID 类型: 0x00 (标准帧)
[13-16] CAN ID (小端)
[17]    帧类型: 0x00 (数据帧)
[18]    数据长度
[19-20] 保留
[21-28] 数据 (8 字节)
[29]    CRC
```

#### 接收帧（16 字节）
```
[0-1]   帧头: 0x55 0xAA
[2-3]   未知
[4-7]   CAN ID (小端)
[8-15]  数据 (8 字节)
```

### 3. 达妙电机 MIT 模式控制

**命令格式**（8 字节）：
```cpp
// 参数范围（DM4340）
float p_min = -12.5, p_max = 12.5;      // 位置 (rad)
float v_min = -30.0, v_max = 30.0;      // 速度 (rad/s)
float t_min = -10.0, t_max = 10.0;      // 力矩 (Nm)
float kp_max = 500.0, kd_max = 5.0;     // 增益

// 编码为 16 位整数
uint16_t p_int = float_to_uint(pos, p_min, p_max, 16);
uint16_t v_int = float_to_uint(vel, v_min, v_max, 12);
uint16_t kp_int = float_to_uint(kp, 0, kp_max, 12);
uint16_t kd_int = float_to_uint(kd, 0, kd_max, 12);
uint16_t t_int = float_to_uint(torque, t_min, t_max, 12);

// 封装
data[0] = p_int >> 8;
data[1] = p_int & 0xFF;
data[2] = v_int >> 4;
data[3] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
data[4] = kp_int & 0xFF;
data[5] = kd_int >> 4;
data[6] = ((kd_int & 0x0F) << 4) | (t_int >> 8);
data[7] = t_int & 0xFF;
```

---

## 📅 实施时间表

| 阶段 | 任务 | 预计时间 |
|-----|------|---------|
| 1 | 硬件层重构 | 2-3 天 |
| 2 | 电机驱动优化 | 2-3 天 |
| 3 | 配置管理（C++） | 1 天 |
| 4 | 主节点重构 | 2 天 |
| 5 | 移除 Python | 0.5 天 |
| 6 | 测试与验证 | 2-3 天 |
| **总计** | | **10-12 天** |

---

## ✅ 项目需求规格（已确认）

### 控制性能
- **控制频率**：500Hz（满足需求）
- **实时内核**：暂不需要 PREEMPT_RT

### 电机配置
| 电机类型 | 数量 | 通信接口 | 备注 |
|---------|------|---------|------|
| DJI GM6020 | 4 | CAN (USB转CAN) | 电压控制 |
| DJI GM3508 | 4 | CAN (USB转CAN) | 电流控制 |
| 达妙 DM4340 | 4 | CAN (USB转CAN) | MIT 模式 |
| 宇树 GO-8010 | 4 | RS485 (USB转485) | 力位混合，4Mbps |
| **总计** | **16** | | **需支持扩展** |

**不支持**：宇树 A1（需要 4.8Mbps，需 RT 系统）

### 硬件设备
- **CAN 总线**：`/dev/ttyACM0`（USB 转 CAN）
- **RS485 串口**：`/dev/ttyUSB0`（USB 转 485，支持 4Mbps）
- **ROS2 版本**：Humble

### 功能要求
- ✅ 支持 4 种电机类型（GM6020、GM3508、DM4340、GO-8010）
- ✅ 可扩展架构（未来可添加更多电机）
- ✅ 达妙电机参数读写功能
- ✅ 统一的配置管理（YAML）

---

## 📌 实施计划（已开始）

### 阶段执行顺序

1. ✅ **需求确认**（已完成）
2. 🔧 **阶段 1**：硬件层重构 → 开始实施
3. 🔧 **阶段 3**：配置管理（C++ YAML 解析）
4. 🔧 **阶段 4**：主节点重构
5. 🔧 **阶段 2**：电机驱动优化（按优先级：DJI → 达妙 → 宇树 GO-8010）
6. 🧹 **阶段 5**：移除 Python 依赖
7. ✅ **阶段 6**：测试验证

### 预计完成时间
- **总工期**：10-12 天
- **第一个可测试版本**：5-6 天（完成阶段 1-4）
