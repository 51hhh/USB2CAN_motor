#ifndef MOTOR_CONTROL_ROS2__HARDWARE__CAN_INTERFACE_HPP_
#define MOTOR_CONTROL_ROS2__HARDWARE__CAN_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <cstring>
#include <map>
#include <cstdint>
#include <chrono>

namespace motor_control {
namespace hardware {

/**
 * @brief CAN 帧结构
 */
struct CANFrame {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t len;
  std::chrono::steady_clock::time_point timestamp;
  
  CANFrame() : can_id(0), len(0), timestamp(std::chrono::steady_clock::now()) {
    memset(data, 0, sizeof(data));
  }
};

/**
 * @brief CAN 接收回调函数类型
 * @param interface_name 接口名称
 * @param can_id CAN 帧 ID
 * @param data 数据指针
 * @param len 数据长度
 */
using CANRxCallback = std::function<void(const std::string& interface_name, uint32_t can_id, const uint8_t* data, size_t len)>;

/**
 * @brief 线程安全的 CAN 帧队列
 */
class ThreadSafeQueue {
public:
  ThreadSafeQueue(size_t max_size = 1000) : max_size_(max_size), shutdown_(false) {}
  
  ~ThreadSafeQueue() {
    shutdown();
  }
  
  // 推送帧（非阻塞，队列满时丢弃最旧的帧）
  bool push(const CANFrame& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (shutdown_) {
      return false;
    }
    
    // 队列满时，移除最旧的帧（避免阻塞）
    if (queue_.size() >= max_size_) {
      queue_.pop();
      dropped_frames_++;
    }
    
    queue_.push(frame);
    cv_.notify_one();
    return true;
  }
  
  // 弹出帧（阻塞，超时返回 false）
  bool pop(CANFrame& frame, int timeout_ms = 100) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (!cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                      [this] { return !queue_.empty() || shutdown_; })) {
      return false;  // 超时
    }
    
    if (shutdown_ && queue_.empty()) {
      return false;
    }
    
    frame = queue_.front();
    queue_.pop();
    return true;
  }
  
  // 尝试弹出（非阻塞）
  bool tryPop(CANFrame& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (queue_.empty() || shutdown_) {
      return false;
    }
    
    frame = queue_.front();
    queue_.pop();
    return true;
  }
  
  // 获取队列大小
  size_t size() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }
  
  // 清空队列
  void clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  }
  
  // 关闭队列
  void shutdown() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    cv_.notify_all();
  }
  
  // 获取丢弃的帧数
  uint64_t getDroppedFrames() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return dropped_frames_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<CANFrame> queue_;
  size_t max_size_;
  std::atomic<bool> shutdown_;
  uint64_t dropped_frames_ = 0;
};

/**
 * @brief CAN 接口类（USB-CAN 适配器）
 * 
 * 实时控制优化版本：
 * - SendRecv 同步模式：发送后立即等待反馈（50us + 900us 超时）
 * - 生产者-消费者模式：解耦控制循环和通信
 * - 专用接收线程：高优先级实时接收
 * - 线程安全队列：避免数据竞争和段错误
 */
class CANInterface {
public:
  /**
   * @brief 构造函数
   * @param port 串口设备路径（如 /dev/ttyACM0）
   * @param baudrate 波特率（默认 921600）
   */
  CANInterface(const std::string& port, int baudrate = 921600);
  
  /**
   * @brief 析构函数（RAII 自动清理资源）
   */
  ~CANInterface();
  
  // 禁止拷贝和移动（避免资源管理问题）
  CANInterface(const CANInterface&) = delete;
  CANInterface& operator=(const CANInterface&) = delete;
  CANInterface(CANInterface&&) = delete;
  CANInterface& operator=(CANInterface&&) = delete;
  
  /**
   * @brief 打开串口
   * @return 成功返回 true
   */
  bool open(bool silent = false);
  
  /**
   * @brief 关闭串口
   */
  void close();
  
  /**
   * @brief 是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }
  
  /**
   * @brief SendRecv 同步模式：发送 CAN 帧并等待反馈
   * 
   * 优化实时性能：
   * - 发送后立即等待反馈（50us 延迟 + 900us 超时）
   * - 避免数据堆积
   * - 确保 PID 及时响应
   * 
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度（最大 8）
   * @param response 输出：接收到的反馈帧
   * @param timeout_us 超时时间（微秒，默认 900us）
   * @return 成功返回 true
   */
  bool sendRecv(uint32_t can_id, const uint8_t* data, size_t len, 
                CANFrame& response, int timeout_us = 900);
  
  /**
   * @brief 批量 SendRecv：发送多个帧并收集反馈
   * 
   * @param frames 发送帧列表
   * @param responses 输出：接收到的反馈帧列表
   * @param timeout_us 每帧超时时间（微秒）
   * @return 成功接收的反馈数
   */
  size_t sendRecvBatch(const std::vector<CANFrame>& frames, 
                       std::vector<CANFrame>& responses,
                       int timeout_us = 900);
  
  /**
   * @brief 传统发送模式（仅发送，不等待反馈）
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度（最大 8）
   * @return 成功返回 true
   */
  bool send(uint32_t can_id, const uint8_t* data, size_t len);
  
  /**
   * @brief 设置接收回调（用于异步接收模式）
   * @param callback 回调函数
   */
  void setRxCallback(CANRxCallback callback);
  
  /**
   * @brief 启动接收线程（异步模式）
   */
  void startRxThread();
  
  /**
   * @brief 停止接收线程
   */
  void stopRxThread();
  
  /**
   * @brief 设置接口名称（用于回调中标识来源）
   * @param name 接口名称
   */
  void setInterfaceName(const std::string& name) { interface_name_ = name; }
  
  /**
   * @brief 获取接口名称
   */
  std::string getInterfaceName() const { return interface_name_; }
  
  /**
   * @brief 获取统计信息
   */
  struct Statistics {
    uint64_t tx_frames;      // 发送帧数
    uint64_t rx_frames;      // 接收帧数
    uint64_t tx_errors;      // 发送错误
    uint64_t rx_errors;      // 接收错误
    uint64_t frame_errors;   // 帧格式错误
    uint64_t timeouts;       // 超时次数
    uint64_t queue_drops;    // 队列丢帧数
  };
  
  Statistics getStatistics() const;
  void resetStatistics();

private:
  // 串口配置
  std::string port_;
  std::string interface_name_;  // 接口名称（用于回调）
  int baudrate_;
  int fd_;  // 文件描述符
  
  // 发送/接收缓冲区
  uint8_t tx_buffer_[30];
  uint8_t rx_buffer_[256];
  std::vector<uint8_t> rx_accumulator_;  // 累积缓冲区
  mutable std::mutex rx_accumulator_mutex_;  // 保护累积缓冲区
  
  // 接收队列（生产者-消费者模式）
  std::shared_ptr<ThreadSafeQueue> rx_queue_;
  
  // 接收回调
  CANRxCallback rx_callback_;
  
  // 接收线程
  std::thread rx_thread_;
  std::atomic<bool> rx_running_;
  
  // 统计信息
  Statistics stats_;
  mutable std::mutex stats_mutex_;
  
  // 内部方法
  void receiveLoop();
  bool receiveRaw(int timeout_us = 0);  // 底层接收（微秒级超时）
  bool parseFrame(CANFrame& frame);
  void buildTxFrame(uint32_t can_id, const uint8_t* data, size_t len);
  bool sendRaw(uint32_t can_id, const uint8_t* data, size_t len);
};

/**
 * @brief CAN 网络管理类
 * 
 * 管理多条 CAN 总线，支持设备热插拔重连。
 */
class CANNetwork {
public:
  CANNetwork();
  ~CANNetwork();
  
  // 禁止拷贝和移动
  CANNetwork(const CANNetwork&) = delete;
  CANNetwork& operator=(const CANNetwork&) = delete;
  
  /**
   * @brief 待连接接口信息
   */
  struct PendingInterface {
    std::string name;
    std::string port;
    int baudrate;
  };
  
  /**
   * @brief 添加 CAN 接口
   * @param name 接口名称（如 "can0"）
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 成功返回 true
   */
  bool addInterface(const std::string& name, const std::string& port, int baudrate = 921600);
  
  /**
   * @brief 获取接口
   * @param name 接口名称
   * @return 接口指针，不存在返回 nullptr
   */
  std::shared_ptr<CANInterface> getInterface(const std::string& name);
  
  /**
   * @brief SendRecv 模式发送
   * @param interface_name 接口名称
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @param response 输出：反馈帧
   * @param timeout_us 超时时间（微秒）
   * @return 成功返回 true
   */
  bool sendRecv(const std::string& interface_name, uint32_t can_id, 
                const uint8_t* data, size_t len, 
                CANFrame& response, int timeout_us = 900);
  
  /**
   * @brief 传统发送模式
   * @param interface_name 接口名称
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功返回 true
   */
  bool send(const std::string& interface_name, uint32_t can_id, 
            const uint8_t* data, size_t len);
  
  /**
   * @brief 设置全局接收回调
   * @param callback 回调函数
   */
  void setGlobalRxCallback(CANRxCallback callback);
  
  /**
   * @brief 启动所有接收线程
   */
  void startAll();
  
  /**
   * @brief 停止所有接收线程
   */
  void stopAll();
  
  /**
   * @brief 关闭所有接口
   */
  void closeAll();
  
  /**
   * @brief 重试连接失败的设备
   * @return 成功连接的设备数量
   */
  int retryPendingInterfaces();
  
  /**
   * @brief 获取待连接设备数量
   */
  size_t getPendingCount() const;
  
  /**
   * @brief 获取待连接设备列表（用于日志）
   */
  std::vector<std::string> getPendingDevices() const;

private:
  std::map<std::string, std::shared_ptr<CANInterface>> interfaces_;
  std::vector<PendingInterface> pending_interfaces_;  // 待连接的接口
  CANRxCallback global_rx_callback_;
  mutable std::mutex mutex_;
};

} // namespace hardware
} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__HARDWARE__CAN_INTERFACE_HPP_
