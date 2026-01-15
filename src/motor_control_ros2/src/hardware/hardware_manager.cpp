#include "motor_control_ros2/hardware/hardware_manager.hpp"
#include <iostream>

namespace motor_control {
namespace hardware {

HardwareManager::HardwareManager() {}

HardwareManager::~HardwareManager() {
  closeAll();
}

bool HardwareManager::addCANInterface(const std::string& name, const std::string& port, int baudrate) {
  return can_network_.addInterface(name, port, baudrate);
}

bool HardwareManager::addSerialInterface(const std::string& name, const std::string& port, int baudrate) {
  return serial_network_.addInterface(name, port, baudrate);
}

std::shared_ptr<CANInterface> HardwareManager::getCANInterface(const std::string& name) {
  return can_network_.getInterface(name);
}

std::shared_ptr<SerialInterface> HardwareManager::getSerialInterface(const std::string& name) {
  return serial_network_.getInterface(name);
}

bool HardwareManager::sendCAN(const std::string& interface_name, uint32_t can_id, 
                              const uint8_t* data, size_t len) {
  return can_network_.send(interface_name, can_id, data, len);
}

ssize_t HardwareManager::sendSerial(const std::string& interface_name, const uint8_t* data, size_t len) {
  return serial_network_.send(interface_name, data, len);
}

void HardwareManager::setCANRxCallback(CANRxCallback callback) {
  can_network_.setGlobalRxCallback(callback);
}

void HardwareManager::setSerialRxCallback(const std::string& interface_name, SerialRxCallback callback) {
  auto interface = serial_network_.getInterface(interface_name);
  if (interface) {
    interface->setRxCallback(callback);
  }
}

void HardwareManager::startAll() {
  can_network_.startAll();
  serial_network_.startAll();
  std::cout << "[HardwareManager] 所有硬件接口已启动" << std::endl;
}

void HardwareManager::stopAll() {
  can_network_.stopAll();
  serial_network_.stopAll();
  std::cout << "[HardwareManager] 所有硬件接口已停止" << std::endl;
}

void HardwareManager::closeAll() {
  can_network_.closeAll();
  serial_network_.closeAll();
  std::cout << "[HardwareManager] 所有硬件接口已关闭" << std::endl;
}

void HardwareManager::printStatistics() const {
  std::cout << "\n========== 硬件统计信息 ==========" << std::endl;
  
  // TODO: 实现统计信息打印
  // 需要遍历所有接口并获取统计信息
  
  std::cout << "================================\n" << std::endl;
}

} // namespace hardware
} // namespace motor_control
