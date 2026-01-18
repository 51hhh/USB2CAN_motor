# 健壮的实时电机控制架构设计

## 📊 从 fdilink_ahrs_ROS1 学到的关键设计

### 1. **生产者-消费者模式（线程安全队列）**

#### ✅ 优秀设计：Unitree A1 实现
```cpp
// motor.hpp: 318-346
class Leg {
private:
    std::unique_ptr<Queue<MotorCmd>> qMotorCmd;  // 线程安全队列
    std::unique_ptr<std::thread> _thread;
    bool startControl = false;
    
    // 专用线程处理电机命令
    void task() {
        while (startControl) {
            MotorCmd cmd = qMotorCmd->Get();  // 阻塞等待
            _motors[cmd.id]->setMotorProp(cmd);
        }
    }
};

// 主循环只负责放入命令
void UpdateMotor(int motorID, ...) {
    MotorCmd cmd;
    // ... 填充命令
    if (qMotorCmd->Full()) {
        std::cout << "Queue FULL!! Motor May Be Disconnected" << std::endl;
    } else {
        qMotorCmd->Put(cmd);  // 非阻塞放入
    }
}
```

**关键优势**：
- ✅ **解耦控制循环和通信**：主循环不会被串口 I/O 阻塞
- ✅ **队列满检测**：及时发现电机断连或通信堵塞
- ✅ **条件变量同步**：高效的线程间通信，避免忙等待

---

### 2. **同步发送-接收模式（SendRecv）**

#### ✅ 优秀设计：UART 实现
```cpp
// uart.hpp: 66-113
int SendRecv(const MotorCmd &cmd) {
    // 1. 发送命令
    int wsize = write(_fd, &_cmd.motorRawData, 34);
    if (wsize <= 0) return -1;
    
    // 2. 短暂延迟等待电机处理
    usleep(50);  // 50us
    
    // 3. 立即读取反馈（带超时）
    int rsize = Read(900);  // 900us 超时
    if (rsize <= 0) return -1;
    
    // 4. CRC 校验
    if (_buffer[0] == 0xFE && _buffer[1] == 0xEE &&
        crc32_core((uint32_t *)_buffer, 18) == *(uint32_t *)(_buffer + 78 - 4)) {
        memcpy(&(_rdata.motor_recv_data), _buffer, 78);
        return 0;
    }
    
    return -1;  // CRC 错误
}
```

**关键优势**：
- ✅ **确保每个命令都有反馈**：发送后立即读取，不会错位
- ✅ **超时保护**：900us 超时避免死锁
- ✅ **CRC 校验**：保证数据完整性

---

### 3. **Python 实现的缓冲区管理**

#### ✅ 优秀设计：滑动窗口解析
```python
# can_driver.py: 79-114
def _rx_thread(self):
    buffer = bytearray()
    while self.running and self.ser:
        waiting = self.ser.inWaiting()
        if waiting > 0:
            buffer.extend(self.ser.read(waiting))
            
            # 循环解析缓冲区
            while len(buffer) >= 16:
                # 严格校验头尾
                if buffer[0] == 0xAA and buffer[15] == 0x55:
                    frame = buffer[:16]
                    buffer = buffer[16:]  # ✅ 移出已处理数据
                    
                    # 提取并回调
                    can_id = int.from_bytes(frame[3:7], byteorder='little')
                    payload = frame[7:15]
                    if self.rx_callback:
                        self.rx_callback(can_id, payload)
                else:
                    # ✅ 帧头不对，滑动窗口丢弃 1 字节
                    buffer.pop(0)
        else:
            time.sleep(0.002)  # 2ms 轮询
```

**关键优势**：
- ✅ **滑动窗口**：逐字节滑动寻找帧头，不会因单个错误丢失整个缓冲区
- ✅ **严格校验**：同时检查帧头和帧尾
- ✅ **及时清理**：解析后立即移除，防止堆积

---

## 🔧 当前 C++ 实现的问题

### ❌ 问题 1: 异步接收 + 同步发送 = 数据错位

**当前实现**：
```cpp
// can_interface.cpp: 297-311
void receiveLoop() {
    while (rx_running_) {
        CANFrame frame;
        if (receive(frame)) {  // 异步接收
            if (rx_callback_) {
                rx_callback_(frame.can_id, frame.data, frame.len);
            }
        }
        std::this_thread::sleep_for(microseconds(500));
    }
}

// motor_control_node.cpp: 419-470
void writeDJIMotors() {
    // 发送命令，但不等待对应反馈
    can_network_->send(interface_name, control_id, data, 8);
}
```

**问题**：
- ❌ 发送命令后不知道对应的反馈是哪个
- ❌ 多个电机的反馈可能乱序
- ❌ 无法检测单个电机是否响应

---

### ❌ 问题 2: 缓冲区无限增长

**当前实现**：
```cpp
// can_interface.cpp: 254-271
bool CANInterface::receive(CANFrame& frame) {
    ssize_t n = read(fd_, rx_buffer_, sizeof(rx_buffer_));
    if (n > 0) {
        // ❌ 无限增长
        rx_accumulator_.insert(rx_accumulator_.end(), rx_buffer_, rx_buffer_ + n);
    }
    return parseFrame(frame);
}
```

**问题**：
- ❌ 4 个电机 200Hz = 800 帧/秒 × 16 字节 = 12.8 KB/秒
- ❌ 如果解析速度跟不上，缓冲区会无限增长
- ❌ 旧数据堆积导致 PID 响应延迟

---

### ❌ 问题 3: 控制循环被阻塞

**当前实现**：
```cpp
// motor_control_node.cpp: 376-405
void controlLoop() {
    readUnitreeMotors();      // 可能阻塞
    for (auto& motor : dji_motors_) {
        motor->updateController();  // PID 计算
    }
    writeDJIMotors();         // 串口写入，可能阻塞
    publishStates();          // ROS 发布
}
```

**问题**：
- ❌ 串口 I/O 阻塞导致控制循环延迟
- ❌ 无法保证 100Hz 精确定时
- ❌ 一个电机卡住会影响所有电机

---

## ✅ 健壮的解决方案

### 方案 A: **SendRecv 同步模式（推荐用于 DJI 电机）**

#### 架构设计
```
主控制循环 (100Hz)
    ↓
发送命令 → 等待反馈 (超时 5ms)
    ↓
解析反馈 → 更新电机状态
    ↓
PID 计算 → 下一个周期
```

#### 实现代码

**1. 修改 `CANInterface` 添加同步接口**

```cpp
// can_interface.hpp
class CANInterface {
public:
    /**
     * @brief 同步发送并接收（SendRecv 模式）
     * @param can_id 发送的 CAN ID
     * @param tx_data 发送数据
     * @param tx_len 发送长度
     * @param rx_frame 接收帧（输出）
     * @param timeout_ms 超时时间（毫秒）
     * @return 成功返回 true
     */
    bool sendRecv(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                  CANFrame& rx_frame, int timeout_ms = 5);
    
    /**
     * @brief 批量 SendRecv（用于 DJI 拼包）
     * @param can_id 发送的 CAN ID
     * @param tx_data 发送数据
     * @param tx_len 发送长度
     * @param expected_ids 期望接收的 CAN ID 列表
     * @param rx_frames 接收帧列表（输出）
     * @param timeout_ms 超时时间
     * @return 成功接收的帧数
     */
    size_t sendRecvBatch(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                         const std::vector<uint32_t>& expected_ids,
                         std::vector<CANFrame>& rx_frames,
                         int timeout_ms = 10);

private:
    std::mutex sendrecv_mutex_;  // SendRecv 互斥锁
};
```

**2. 实现 SendRecv**

```cpp
// can_interface.cpp
bool CANInterface::sendRecv(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                            CANFrame& rx_frame, int timeout_ms) {
    std::lock_guard<std::mutex> lock(sendrecv_mutex_);
    
    // 1. 清空接收缓冲区（避免旧数据干扰）
    tcflush(fd_, TCIFLUSH);
    rx_accumulator_.clear();
    
    // 2. 发送命令
    if (!send(can_id, tx_data, tx_len)) {
        return false;
    }
    
    // 3. 等待反馈（带超时）
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);
    
    while (true) {
        // 检查超时
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            std::lock_guard<std::mutex> stats_lock(stats_mutex_);
            stats_.rx_errors++;
            return false;  // 超时
        }
        
        // 尝试接收
        if (receive(rx_frame)) {
            return true;  // 成功接收
        }
        
        // 短暂休眠避免 CPU 占用
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

size_t CANInterface::sendRecvBatch(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                                   const std::vector<uint32_t>& expected_ids,
                                   std::vector<CANFrame>& rx_frames,
                                   int timeout_ms) {
    std::lock_guard<std::mutex> lock(sendrecv_mutex_);
    
    // 1. 清空缓冲区
    tcflush(fd_, TCIFLUSH);
    rx_accumulator_.clear();
    rx_frames.clear();
    
    // 2. 发送命令
    if (!send(can_id, tx_data, tx_len)) {
        return 0;
    }
    
    // 3. 收集所有期望的反馈
    std::set<uint32_t> remaining_ids(expected_ids.begin(), expected_ids.end());
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);
    
    while (!remaining_ids.empty()) {
        // 检查超时
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            break;  // 超时，返回已收到的
        }
        
        // 尝试接收
        CANFrame frame;
        if (receive(frame)) {
            // 检查是否是期望的 ID
            if (remaining_ids.count(frame.can_id)) {
                rx_frames.push_back(frame);
                remaining_ids.erase(frame.can_id);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    return rx_frames.size();
}
```

**3. 修改控制循环使用 SendRecv**

```cpp
// motor_control_node.cpp
void writeDJIMotors() {
    if (dji_motors_.empty()) return;
    
    // 按接口和控制ID分组
    std::map<std::string, std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>>> interface_groups;
    
    for (auto& motor : dji_motors_) {
        std::string interface_name = motor->getInterfaceName();
        uint32_t control_id = motor->getControlId();
        interface_groups[interface_name][control_id].push_back(motor);
    }
    
    // 对每个接口的每个控制ID发送
    for (auto& [interface_name, control_id_groups] : interface_groups) {
        auto interface = can_network_->getInterface(interface_name);
        if (!interface) continue;
        
        for (auto& [control_id, motors] : control_id_groups) {
            // 1. 准备发送数据
            uint8_t data[8] = {0};
            std::vector<uint32_t> expected_ids;
            
            for (auto& motor : motors) {
                uint8_t motor_id = motor->getMotorId();
                uint8_t bytes[2];
                motor->getControlBytes(bytes);
                
                int offset = ((motor_id - 1) % 4) * 2;
                data[offset] = bytes[0];
                data[offset + 1] = bytes[1];
                
                // 记录期望的反馈 ID
                expected_ids.push_back(motor->getFeedbackId());
            }
            
            // 2. SendRecv 批量发送并接收
            std::vector<CANFrame> rx_frames;
            size_t received = interface->sendRecvBatch(
                control_id, data, 8, expected_ids, rx_frames, 10  // 10ms 超时
            );
            
            // 3. 更新电机状态
            for (const auto& frame : rx_frames) {
                for (auto& motor : motors) {
                    motor->updateFeedback(frame.can_id, frame.data, frame.len);
                }
            }
            
            // 4. 检测丢失的反馈
            if (received < expected_ids.size()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[CAN] 控制ID 0x%03X: 期望 %zu 个反馈，实际收到 %zu 个",
                    control_id, expected_ids.size(), received);
            }
        }
    }
}
```

---

### 方案 B: **线程安全队列模式（推荐用于 Unitree 电机）**

#### 架构设计
```
主控制循环 (100Hz)
    ↓
放入命令到队列（非阻塞）
    ↓
继续 PID 计算

专用通信线程
    ↓
从队列取命令（阻塞）
    ↓
SendRecv 同步通信
    ↓
更新电机状态
```

#### 实现代码

**1. 创建线程安全队列**

```cpp
// thread_safe_queue.hpp
template <typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 100) 
        : max_size_(max_size), count_(0) {}
    
    // 放入元素（阻塞直到有空间）
    void put(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this] { return count_ < max_size_; });
        
        queue_.push_back(item);
        ++count_;
        
        lock.unlock();
        not_empty_.notify_one();
    }
    
    // 尝试放入（非阻塞，队列满时返回 false）
    bool tryPut(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ >= max_size_) {
            return false;  // 队列满
        }
        
        queue_.push_back(item);
        ++count_;
        not_empty_.notify_one();
        return true;
    }
    
    // 取出元素（阻塞直到有数据）
    T get() {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] { return count_ > 0; });
        
        T item = queue_.front();
        queue_.pop_front();
        --count_;
        
        lock.unlock();
        not_full_.notify_one();
        return item;
    }
    
    // 检查是否为空
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    // 检查是否已满
    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ >= max_size_;
    }
    
    // 清空队列
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        count_ = 0;
        not_full_.notify_all();
    }

private:
    size_t max_size_;
    size_t count_;
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable not_full_;
    std::condition_variable not_empty_;
};
```

**2. 修改 DJIMotor 添加命令队列**

```cpp
// dji_motor.hpp
class DJIMotor : public MotorBase {
public:
    // 异步设置输出（放入队列）
    bool setOutputAsync(int16_t value) {
        target_output_ = value;
        return true;  // 立即返回
    }
    
    // 获取待发送的命令
    bool hasPendingCommand() const {
        return has_pending_command_;
    }
    
    void markCommandSent() {
        has_pending_command_ = false;
    }
    
    void markCommandPending() {
        has_pending_command_ = true;
    }

private:
    bool has_pending_command_ = false;
};
```

**3. 创建专用通信线程**

```cpp
// motor_control_node.cpp
class MotorControlNode : public rclcpp::Node {
private:
    // CAN 通信线程
    std::thread can_comm_thread_;
    std::atomic<bool> can_comm_running_;
    
    void startCANCommThread() {
        can_comm_running_ = true;
        can_comm_thread_ = std::thread(&MotorControlNode::canCommLoop, this);
    }
    
    void stopCANCommThread() {
        can_comm_running_ = false;
        if (can_comm_thread_.joinable()) {
            can_comm_thread_.join();
        }
    }
    
    void canCommLoop() {
        using namespace std::chrono;
        auto next_time = steady_clock::now();
        auto period = microseconds(10000);  // 100Hz
        
        while (can_comm_running_) {
            // 1. 收集所有待发送的电机命令
            std::map<std::string, std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>>> pending_motors;
            
            for (auto& motor : dji_motors_) {
                if (motor->hasPendingCommand()) {
                    std::string interface_name = motor->getInterfaceName();
                    uint32_t control_id = motor->getControlId();
                    pending_motors[interface_name][control_id].push_back(motor);
                }
            }
            
            // 2. 批量 SendRecv
            for (auto& [interface_name, control_id_groups] : pending_motors) {
                auto interface = can_network_->getInterface(interface_name);
                if (!interface) continue;
                
                for (auto& [control_id, motors] : control_id_groups) {
                    // 准备数据
                    uint8_t data[8] = {0};
                    std::vector<uint32_t> expected_ids;
                    
                    for (auto& motor : motors) {
                        uint8_t motor_id = motor->getMotorId();
                        uint8_t bytes[2];
                        motor->getControlBytes(bytes);
                        
                        int offset = ((motor_id - 1) % 4) * 2;
                        data[offset] = bytes[0];
                        data[offset + 1] = bytes[1];
                        
                        expected_ids.push_back(motor->getFeedbackId());
                    }
                    
                    // SendRecv
                    std::vector<CANFrame> rx_frames;
                    interface->sendRecvBatch(control_id, data, 8, expected_ids, rx_frames, 10);
                    
                    // 更新状态
                    for (const auto& frame : rx_frames) {
                        for (auto& motor : motors) {
                            motor->updateFeedback(frame.can_id, frame.data, frame.len);
                            motor->markCommandSent();
                        }
                    }
                }
            }
            
            // 3. 精确定时
            next_time += period;
            std::this_thread::sleep_until(next_time);
        }
    }
};
```

---

## 📊 性能对比

| 方案 | 控制频率 | 通信延迟 | CPU 占用 | 数据丢失风险 | 实时性 |
|-----|---------|---------|---------|------------|-------|
| **当前异步方案** | 100-200Hz | 高（堆积） | 中 | 高 | ❌ 差 |
| **SendRecv 同步** | 100Hz | 低（5-10ms） | 低 | 极低 | ✅ 优秀 |
| **队列 + 专用线程** | 100Hz | 低（10ms） | 中 | 极低 | ✅ 优秀 |

---

## 🎯 推荐实施方案

### **组合方案：SendRecv + 缓冲区优化**

#### 步骤 1: 实现 SendRecv 接口
```bash
# 修改 can_interface.hpp 和 can_interface.cpp
# 添加 sendRecv() 和 sendRecvBatch() 方法
```

#### 步骤 2: 修改控制循环
```bash
# 修改 motor_control_node.cpp
# writeDJIMotors() 使用 sendRecvBatch()
```

#### 步骤 3: 保留缓冲区优化
```bash
# 保留之前的缓冲区大小检查
# 作为异常情况的兜底保护
```

#### 步骤 4: 降低控制频率
```bash
# control_params.yaml
control_frequency: 100.0  # 100Hz 足够舵轮控制
```

---

## 🔍 关键指标监控

### 1. 添加通信质量统计

```cpp
struct CANCommStats {
    uint64_t total_sends;
    uint64_t successful_recvs;
    uint64_t timeouts;
    uint64_t crc_errors;
    double success_rate;
    double avg_latency_ms;
};
```

### 2. 发布诊断信息

```cpp
// 每秒发布一次通信质量
auto diag_msg = motor_control_ros2::msg::CANDiagnostics();
diag_msg.success_rate = stats.success_rate;
diag_msg.avg_latency_ms = stats.avg_latency_ms;
diag_msg.timeout_count = stats.timeouts;
can_diag_pub_->publish(diag_msg);
```

---

## 📝 总结

### 核心改进
1. ✅ **SendRecv 同步模式**：确保每个命令都有对应反馈
2. ✅ **批量收集反馈**：一次发送，收集所有电机反馈
3. ✅ **超时保护**：10ms 超时避免死锁
4. ✅ **缓冲区清理**：发送前清空，避免旧数据干扰
5. ✅ **丢失检测**：统计实际收到的反馈数

### 预期效果
- ✅ **零数据堆积**：SendRecv 模式不会累积
- ✅ **低延迟**：5-10ms 往返延迟
- ✅ **高可靠性**：CRC 校验 + 超时检测
- ✅ **实时响应**：PID 基于最新反馈计算

---

**作者**: Antigravity  
**日期**: 2026-01-18  
**版本**: 2.0
