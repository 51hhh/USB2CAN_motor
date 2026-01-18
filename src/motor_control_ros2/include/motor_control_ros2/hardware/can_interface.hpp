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
#include <cstring>
#include <map>
#include <cstdint>

namespace motor_control {
namespace hardware {

/**
 * @brief CAN 帧结构
 */
struct CANFrame {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t len;
  
  CANFrame() : can_id(0), len(0) {
    memset(data, 0, sizeof(data));
  }
};

/**
 * @brief CAN 接收回调函数类型
 * @param can_id CAN 帧 ID
 * @param data 数据指针
 * @param len 数据长度
 */
using CANRxCallback = std::function<void(uint32_t can_id, const uint8_t* data, size_t len)>;

/**
 * @brief CAN 接口类（USB-CAN 适配器）
 * 
 * 管理单个 USB-CAN 设备的串口通信。
 * 协议：发送 30 字节，接收 16 字节。
 * 
 * 优化特性：
 * - 发送队列批量处理
 * - 接收缓冲区累积解析
 * - 错误检测和统计
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
   * @brief 析构函数
   */
  ~CANInterface();
  
  /**
   * @brief 打开串口
   * @return 成功返回 true
   */
  bool open();
  
  /**
   * @brief 关闭串口
   */
  void close();
  
  /**
   * @brief 是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }
  
  /**
   * @brief 发送 CAN 帧
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度（最大 8）
   * @return 成功返回 true
   */
  bool send(uint32_t can_id, const uint8_t* data, size_t len);
  
  /**
   * @brief 批量发送 CAN 帧（优化性能）
   * @param frames 帧列表
   * @return 成功发送的帧数
   */
  size_t sendBatch(const std::vector<CANFrame>& frames);
  
  /**
   * @brief 接收 CAN 帧（非阻塞）
   * @param frame 输出帧
   * @return 成功返回 true
   */
  bool receive(CANFrame& frame);
  
  /**
   * @brief 同步发送并接收（SendRecv 模式）
   * 
   * 发送命令后立即等待反馈，确保命令-反馈对应关系。
   * 适用于需要实时反馈的场景（如 DJI 电机控制）。
   * 
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
   * 
   * 发送一帧命令，收集多个电机的反馈。
   * 适用于 DJI 电机拼包发送场景（4 个电机共用一个控制 ID）。
   * 
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
  
  /**
   * @brief 设置接收回调
   * @param callback 回调函数
   */
  void setRxCallback(CANRxCallback callback);
  
  /**
   * @brief 启动接收线程
   */
  void startRxThread();
  
  /**
   * @brief 停止接收线程
   */
  void stopRxThread();
  
  /**
   * @brief 获取统计信息
   */
  struct Statistics {
    uint64_t tx_frames;      // 发送帧数
    uint64_t rx_frames;      // 接收帧数
    uint64_t tx_errors;      // 发送错误
    uint64_t rx_errors;      // 接收错误
    uint64_t frame_errors;   // 帧格式错误
    uint64_t sendrecv_timeouts;  // SendRecv 超时次数
  };
  
  Statistics getStatistics() const { return stats_; }
  void resetStatistics();

private:
  // 串口配置
  std::string port_;
  int baudrate_;
  int fd_;  // 文件描述符
  
  // 发送/接收缓冲区
  uint8_t tx_buffer_[30];
  uint8_t rx_buffer_[256];
  std::vector<uint8_t> rx_accumulator_;  // 累积缓冲区
  
  // 接收回调
  CANRxCallback rx_callback_;
  
  // 接收线程
  std::thread rx_thread_;
  std::atomic<bool> rx_running_;
  
  // 统计信息
  Statistics stats_;
  mutable std::mutex stats_mutex_;
  
  // SendRecv 同步锁
  std::mutex sendrecv_mutex_;
  
  // 内部方法
  void receiveLoop();
  bool parseFrame(CANFrame& frame);
  void buildTxFrame(uint32_t can_id, const uint8_t* data, size_t len);
};

/**
 * @brief CAN 网络管理类
 * 
 * 管理多条 CAN 总线（未来扩展用）。
 * 当前项目只使用一条 CAN 总线。
 */
class CANNetwork {
public:
  CANNetwork();
  ~CANNetwork();
  
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
   * @brief 发送 CAN 帧
   * @param interface_name 接口名称
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功返回 true
   */
  bool send(const std::string& interface_name, uint32_t can_id, 
            const uint8_t* data, size_t len);
  
  /**
   * @brief 广播 CAN 帧到所有接口
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功发送的接口数
   */
  size_t broadcast(uint32_t can_id, const uint8_t* data, size_t len);
  
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

private:
  std::map<std::string, std::shared_ptr<CANInterface>> interfaces_;
  CANRxCallback global_rx_callback_;
  std::mutex mutex_;
};

} // namespace hardware
} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__HARDWARE__CAN_INTERFACE_HPP_
