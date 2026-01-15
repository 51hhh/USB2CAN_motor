#include "motor_control_ros2/hardware/serial_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

namespace motor_control {
namespace hardware {

// ========== SerialInterface 实现 ==========

SerialInterface::SerialInterface(const std::string& port, int baudrate)
  : port_(port)
  , baudrate_(baudrate)
  , fd_(-1)
  , rx_running_(false)
{
  memset(&stats_, 0, sizeof(stats_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

SerialInterface::~SerialInterface() {
  stopRxThread();
  close();
}

speed_t SerialInterface::getBaudrateConstant(int baudrate) {
  switch (baudrate) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default:
      std::cerr << "[SerialInterface] 不支持的波特率: " << baudrate 
                << ", 使用默认 4000000" << std::endl;
      return B4000000;
  }
}

bool SerialInterface::open() {
  // 打开串口
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[SerialInterface] 无法打开串口: " << port_ 
              << " (错误: " << strerror(errno) << ")" << std::endl;
    return false;
  }
  
  // 配置串口
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[SerialInterface] tcgetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // 设置波特率
  speed_t speed = getBaudrateConstant(baudrate_);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  
  // 8N1, 无流控
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ISIG;
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;
  
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << "[SerialInterface] tcsetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // 清空缓冲区
  tcflush(fd_, TCIOFLUSH);
  
  std::cout << "[SerialInterface] 成功打开串口: " << port_ 
            << " @ " << baudrate_ << " bps" << std::endl;
  return true;
}

void SerialInterface::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialInterface::send(const uint8_t* data, size_t len) {
  if (fd_ < 0) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_errors++;
    return -1;
  }
  
  ssize_t n = write(fd_, data, len);
  
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (n > 0) {
    stats_.tx_bytes += n;
  } else {
    stats_.tx_errors++;
  }
  
  return n;
}

ssize_t SerialInterface::receive(uint8_t* buffer, size_t max_len) {
  if (fd_ < 0) {
    return -1;
  }
  
  ssize_t n = read(fd_, buffer, max_len);
  
  if (n > 0) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_bytes += n;
  } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_errors++;
  }
  
  return n;
}

void SerialInterface::setRxCallback(SerialRxCallback callback) {
  rx_callback_ = callback;
}

void SerialInterface::startRxThread() {
  if (rx_running_) {
    return;
  }
  
  rx_running_ = true;
  rx_thread_ = std::thread(&SerialInterface::receiveLoop, this);
}

void SerialInterface::stopRxThread() {
  if (!rx_running_) {
    return;
  }
  
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void SerialInterface::receiveLoop() {
  using namespace std::chrono;
  
  while (rx_running_) {
    ssize_t n = receive(rx_buffer_, RX_BUFFER_SIZE);
    
    if (n > 0 && rx_callback_) {
      rx_callback_(rx_buffer_, n);
    }
    
    // 短暂休眠避免 CPU 占用过高（500us = 2kHz 轮询频率）
    std::this_thread::sleep_for(microseconds(500));
  }
}

void SerialInterface::resetStatistics() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  memset(&stats_, 0, sizeof(stats_));
}

// ========== SerialNetwork 实现 ==========

SerialNetwork::SerialNetwork() {}

SerialNetwork::~SerialNetwork() {
  closeAll();
}

bool SerialNetwork::addInterface(const std::string& name, const std::string& port, int baudrate) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 检查是否已存在
  if (interfaces_.find(name) != interfaces_.end()) {
    std::cerr << "[SerialNetwork] 接口已存在: " << name << std::endl;
    return false;
  }
  
  auto interface = std::make_shared<SerialInterface>(port, baudrate);
  if (!interface->open()) {
    return false;
  }
  
  interfaces_[name] = interface;
  std::cout << "[SerialNetwork] 添加接口: " << name << " (" << port << ")" << std::endl;
  
  return true;
}

std::shared_ptr<SerialInterface> SerialNetwork::getInterface(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = interfaces_.find(name);
  if (it != interfaces_.end()) {
    return it->second;
  }
  
  return nullptr;
}

ssize_t SerialNetwork::send(const std::string& interface_name, const uint8_t* data, size_t len) {
  auto interface = getInterface(interface_name);
  if (!interface) {
    return -1;
  }
  
  return interface->send(data, len);
}

void SerialNetwork::startAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->startRxThread();
  }
  
  std::cout << "[SerialNetwork] 启动了 " << interfaces_.size() << " 个接收线程" << std::endl;
}

void SerialNetwork::stopAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->stopRxThread();
  }
}

void SerialNetwork::closeAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->stopRxThread();
    interface->close();
  }
  
  interfaces_.clear();
}

} // namespace hardware
} // namespace motor_control
