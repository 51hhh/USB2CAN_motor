#ifndef MOTOR_CONTROL_ROS2__CAN_DRIVER_HPP_
#define MOTOR_CONTROL_ROS2__CAN_DRIVER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

namespace motor_control {

/**
 * @brief CAN 帧结构
 */
struct CANFrame {
  uint32_t can_id;
  uint8_t data[8];
  uint8_t len;
};

/**
 * @brief CAN 接收回调函数类型
 */
using CANRxCallback = std::function<void(uint8_t bus_id, uint32_t can_id, const uint8_t* data, size_t len)>;

/**
 * @brief CAN 总线类
 * 
 * 管理单个 USB-CAN 设备的串口通信。
 * 协议：发送30字节，接收16字节。
 */
class CANBus {
public:
  /**
   * @brief 构造函数
   * @param bus_id 总线 ID
   * @param port 串口设备路径
   * @param baudrate 波特率
   */
  CANBus(uint8_t bus_id, const std::string& port, int baudrate = 921600);
  ~CANBus();
  
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
   * @brief 发送 CAN 帧
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功返回 true
   */
  bool send(uint32_t can_id, const uint8_t* data, size_t len);
  
  /**
   * @brief 接收 CAN 帧（非阻塞）
   * @param frame 输出帧
   * @return 成功返回 true
   */
  bool receive(CANFrame& frame);
  
  /**
   * @brief 获取总线 ID
   */
  uint8_t getBusId() const { return bus_id_; }
  
  /**
   * @brief 是否已打开
   */
  bool isOpen() const { return fd_ >= 0; }

private:
  uint8_t bus_id_;
  std::string port_;
  int baudrate_;
  int fd_;  // 文件描述符
  
  // 发送/接收缓冲区
  uint8_t tx_buffer_[30];
  uint8_t rx_buffer_[256];  // 增大接收缓冲区
  std::vector<uint8_t> rx_accumulator_;  // 累积缓冲区
};

/**
 * @brief CAN 网络管理类
 * 
 * 管理多条 CAN 总线，提供统一的接收线程。
 */
class CANNetwork {
public:
  CANNetwork();
  ~CANNetwork();
  
  /**
   * @brief 添加 CAN 总线
   * @param bus_id 总线 ID
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 成功返回 true
   */
  bool addBus(uint8_t bus_id, const std::string& port, int baudrate = 921600);
  
  /**
   * @brief 发送 CAN 帧
   * @param bus_id 总线 ID
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功返回 true
   */
  bool send(uint8_t bus_id, uint32_t can_id, const uint8_t* data, size_t len);
  
  /**
   * @brief 设置接收回调
   * @param callback 回调函数
   */
  void setCallback(CANRxCallback callback);
  
  /**
   * @brief 启动接收线程
   */
  void start();
  
  /**
   * @brief 停止接收线程
   */
  void stop();
  
  /**
   * @brief 关闭所有总线
   */
  void closeAll();

private:
  std::vector<std::shared_ptr<CANBus>> buses_;
  CANRxCallback rx_callback_;
  std::thread rx_thread_;
  std::atomic<bool> running_;
  std::mutex mutex_;
  
  void receiveLoop();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__CAN_DRIVER_HPP_
