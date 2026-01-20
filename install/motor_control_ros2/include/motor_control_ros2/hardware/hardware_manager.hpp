#ifndef MOTOR_CONTROL_ROS2__HARDWARE__HARDWARE_MANAGER_HPP_
#define MOTOR_CONTROL_ROS2__HARDWARE__HARDWARE_MANAGER_HPP_

#include "motor_control_ros2/hardware/can_interface.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include <memory>
#include <string>
#include <map>

namespace motor_control {
namespace hardware {

/**
 * @brief 硬件管理器
 * 
 * 统一管理所有硬件接口（CAN 和串口）。
 * 提供统一的初始化、发送、接收接口。
 */
class HardwareManager {
public:
  HardwareManager();
  ~HardwareManager();
  
  /**
   * @brief 添加 CAN 接口
   * @param name 接口名称（如 "can0"）
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 成功返回 true
   */
  bool addCANInterface(const std::string& name, const std::string& port, int baudrate = 921600);
  
  /**
   * @brief 添加串口接口
   * @param name 接口名称（如 "unitree_port"）
   * @param port 串口设备路径
   * @param baudrate 波特率
   * @return 成功返回 true
   */
  bool addSerialInterface(const std::string& name, const std::string& port, int baudrate = 4000000);
  
  /**
   * @brief 获取 CAN 接口
   * @param name 接口名称
   * @return 接口指针，不存在返回 nullptr
   */
  std::shared_ptr<CANInterface> getCANInterface(const std::string& name);
  
  /**
   * @brief 获取串口接口
   * @param name 接口名称
   * @return 接口指针，不存在返回 nullptr
   */
  std::shared_ptr<SerialInterface> getSerialInterface(const std::string& name);
  
  /**
   * @brief 发送 CAN 帧
   * @param interface_name 接口名称
   * @param can_id CAN ID
   * @param data 数据指针
   * @param len 数据长度
   * @return 成功返回 true
   */
  bool sendCAN(const std::string& interface_name, uint32_t can_id, 
               const uint8_t* data, size_t len);
  
  /**
   * @brief 发送串口数据
   * @param interface_name 接口名称
   * @param data 数据指针
   * @param len 数据长度
   * @return 实际发送的字节数
   */
  ssize_t sendSerial(const std::string& interface_name, const uint8_t* data, size_t len);
  
  /**
   * @brief 设置 CAN 接收回调
   * @param callback 回调函数
   */
  void setCANRxCallback(CANRxCallback callback);
  
  /**
   * @brief 设置串口接收回调（针对特定接口）
   * @param interface_name 接口名称
   * @param callback 回调函数
   */
  void setSerialRxCallback(const std::string& interface_name, SerialRxCallback callback);
  
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
   * @brief 打印统计信息
   */
  void printStatistics() const;

private:
  CANNetwork can_network_;
  SerialNetwork serial_network_;
};

} // namespace hardware
} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__HARDWARE__HARDWARE_MANAGER_HPP_
