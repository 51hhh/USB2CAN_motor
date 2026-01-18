#include "motor_control_ros2/hardware/can_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <set>

namespace motor_control {
namespace hardware {

// ========== CANInterface 实现 ==========

CANInterface::CANInterface(const std::string& port, int baudrate)
  : port_(port)
  , baudrate_(baudrate)
  , fd_(-1)
  , rx_running_(false)
{
  memset(&stats_, 0, sizeof(stats_));
  memset(tx_buffer_, 0, sizeof(tx_buffer_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

CANInterface::~CANInterface() {
  stopRxThread();
  close();
}

bool CANInterface::open() {
  // 打开串口
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[CANInterface] 无法打开串口: " << port_ 
              << " (错误: " << strerror(errno) << ")" << std::endl;
    return false;
  }
  
  // 配置串口
  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << "[CANInterface] tcgetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // 设置波特率
  speed_t speed = B921600;  // 默认 921600
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
    std::cerr << "[CANInterface] tcsetattr 失败" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // 清空缓冲区
  tcflush(fd_, TCIOFLUSH);
  
  std::cout << "[CANInterface] 成功打开串口: " << port_ << std::endl;
  return true;
}

void CANInterface::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void CANInterface::buildTxFrame(uint32_t can_id, const uint8_t* data, size_t len) {
  // 构造 30 字节发送帧（根据官方协议文档）
  memset(tx_buffer_, 0, 30);
  
  // 帧头
  tx_buffer_[0] = 0x55;
  tx_buffer_[1] = 0xAA;
  
  // 帧长
  tx_buffer_[2] = 0x1E;  // 30 字节
  
  // 命令：0x01 = 转发 CAN 数据帧
  tx_buffer_[3] = 0x01;
  
  // 发送次数 (32bit 小端) = 1
  tx_buffer_[4] = 0x01;
  tx_buffer_[5] = 0x00;
  tx_buffer_[6] = 0x00;
  tx_buffer_[7] = 0x00;
  
  // 时间间隔 (32bit 小端) = 2ms
  tx_buffer_[8] = 0x02;
  tx_buffer_[9] = 0x00;
  tx_buffer_[10] = 0x00;
  tx_buffer_[11] = 0x00;
  
  // ID 类型：00 = 标准帧
  tx_buffer_[12] = 0x00;
  
  // CAN ID (32bit 小端)
  tx_buffer_[13] = can_id & 0xFF;
  tx_buffer_[14] = (can_id >> 8) & 0xFF;
  tx_buffer_[15] = (can_id >> 16) & 0xFF;
  tx_buffer_[16] = (can_id >> 24) & 0xFF;
  
  // 帧类型：00 = 数据帧
  tx_buffer_[17] = 0x00;
  
  // 数据长度
  tx_buffer_[18] = len;
  
  // idAcc, dataAcc (保留字段)
  tx_buffer_[19] = 0x00;
  tx_buffer_[20] = 0x00;
  
  // 数据 (8 字节)
  memcpy(&tx_buffer_[21], data, len);
  
  // CRC (可以任意填充)
  tx_buffer_[29] = 0x00;
}

bool CANInterface::send(uint32_t can_id, const uint8_t* data, size_t len) {
  if (fd_ < 0 || len > 8) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.tx_errors++;
    return false;
  }
  
  buildTxFrame(can_id, data, len);
  
  // 发送
  ssize_t n = write(fd_, tx_buffer_, 30);
  
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (n == 30) {
    stats_.tx_frames++;
    return true;
  } else {
    stats_.tx_errors++;
    return false;
  }
}

size_t CANInterface::sendBatch(const std::vector<CANFrame>& frames) {
  if (fd_ < 0) {
    return 0;
  }
  
  size_t sent_count = 0;
  for (const auto& frame : frames) {
    if (send(frame.can_id, frame.data, frame.len)) {
      sent_count++;
    }
  }
  
  return sent_count;
}

bool CANInterface::parseFrame(CANFrame& frame) {
  // ✅ 滑动窗口解析：严格校验帧头和帧尾
  // 参考 Python 实现：逐字节滑动寻找有效帧
  // 帧格式：AA [CMD] [格式] [CANID 4字节] [数据 8字节] 55 (共16字节)
  
  while (rx_accumulator_.size() >= 16) {
    // ✅ 严格校验：同时检查帧头 (0xAA) 和帧尾 (0x55)
    if (rx_accumulator_[0] == 0xAA && rx_accumulator_[15] == 0x55) {
      // 找到有效帧
      frame.can_id = rx_accumulator_[3] | (rx_accumulator_[4] << 8) | 
                     (rx_accumulator_[5] << 16) | (rx_accumulator_[6] << 24);
      
      // 数据 (Byte 7-14, 固定8字节)
      frame.len = 8;
      memcpy(frame.data, &rx_accumulator_[7], 8);
      
      // ✅ 移除已处理的帧
      rx_accumulator_.erase(rx_accumulator_.begin(), rx_accumulator_.begin() + 16);
      
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.rx_frames++;
      
      return true;
    } else {
      // ✅ 帧头或帧尾不对，滑动窗口丢弃 1 字节
      rx_accumulator_.erase(rx_accumulator_.begin(), rx_accumulator_.begin() + 1);
      
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.frame_errors++;
    }
  }
  
  return false;
}

bool CANInterface::receive(CANFrame& frame) {
  if (fd_ < 0) {
    return false;
  }
  
  // 非阻塞读取可用数据
  ssize_t n = read(fd_, rx_buffer_, sizeof(rx_buffer_));
  if (n > 0) {
    // ✅ 防止缓冲区堆积：检查累积缓冲区大小
    if (rx_accumulator_.size() > 384) {
      // 丢弃旧数据，只保留最后 256 字节
      size_t keep_size = 256;
      if (rx_accumulator_.size() > keep_size) {
        rx_accumulator_.erase(
          rx_accumulator_.begin(), 
          rx_accumulator_.end() - keep_size
        );
        
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frame_errors++;  // 统计丢弃次数
      }
    }
    
    // 将新数据添加到累积缓冲区
    rx_accumulator_.insert(rx_accumulator_.end(), rx_buffer_, rx_buffer_ + n);
  } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_errors++;
  }
  
  // 尝试解析帧
  return parseFrame(frame);
}

void CANInterface::setRxCallback(CANRxCallback callback) {
  rx_callback_ = callback;
}

void CANInterface::startRxThread() {
  if (rx_running_) {
    return;
  }
  
  rx_running_ = true;
  rx_thread_ = std::thread(&CANInterface::receiveLoop, this);
}

void CANInterface::stopRxThread() {
  if (!rx_running_) {
    return;
  }
  
  rx_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void CANInterface::receiveLoop() {
  using namespace std::chrono;
  
  while (rx_running_) {
    CANFrame frame;
    
    // ✅ 批量处理多帧，减少空转
    int frames_processed = 0;
    while (receive(frame) && frames_processed < 10) {
      if (rx_callback_) {
        rx_callback_(frame.can_id, frame.data, frame.len);
      }
      frames_processed++;
    }
    
    // ✅ 动态调整休眠时间
    if (frames_processed > 0) {
      // 有数据时快速轮询
      std::this_thread::sleep_for(microseconds(100));
    } else {
      // 无数据时降低频率
      std::this_thread::sleep_for(microseconds(500));
    }
  }
}

bool CANInterface::sendRecv(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                            CANFrame& rx_frame, int timeout_ms) {
  // ✅ 先检查 fd，避免对无效文件描述符操作
  if (fd_ < 0) {
    return false;
  }
  
  std::lock_guard<std::mutex> lock(sendrecv_mutex_);
  
  // 1. ✅ 清空接收缓冲区（避免旧数据干扰）
  tcflush(fd_, TCIFLUSH);
  rx_accumulator_.clear();
  
  // 2. 发送命令
  if (!send(can_id, tx_data, tx_len)) {
    return false;
  }
  
  // 3. ✅ 等待反馈（带超时）
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::milliseconds(timeout_ms);
  
  while (true) {
    // 检查超时
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      stats_.sendrecv_timeouts++;
      return false;  // 超时
    }
    
    // 尝试接收
    if (receive(rx_frame)) {
      return true;  // 成功接收
    }
    
    // 短暂休眠避免 CPU 占用（100us，参考 Unitree 实现）
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

size_t CANInterface::sendRecvBatch(uint32_t can_id, const uint8_t* tx_data, size_t tx_len,
                                   const std::vector<uint32_t>& expected_ids,
                                   std::vector<CANFrame>& rx_frames,
                                   int timeout_ms) {
  // ✅ 先检查 fd，避免对无效文件描述符操作
  if (fd_ < 0) {
    return 0;
  }
  
  std::lock_guard<std::mutex> lock(sendrecv_mutex_);
  
  // 1. ✅ 清空缓冲区
  tcflush(fd_, TCIFLUSH);
  rx_accumulator_.clear();
  rx_frames.clear();
  
  // 2. 发送命令
  if (!send(can_id, tx_data, tx_len)) {
    return 0;
  }
  
  // 3. ✅ 收集所有期望的反馈
  std::set<uint32_t> remaining_ids(expected_ids.begin(), expected_ids.end());
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::milliseconds(timeout_ms);
  
  while (!remaining_ids.empty()) {
    // 检查超时
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      // 超时，记录统计
      if (rx_frames.size() < expected_ids.size()) {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.sendrecv_timeouts++;
      }
      break;  // 返回已收到的
    }
    
    // 尝试接收
    CANFrame frame;
    if (receive(frame)) {
      // ✅ 检查是否是期望的 ID
      if (remaining_ids.count(frame.can_id)) {
        rx_frames.push_back(frame);
        remaining_ids.erase(frame.can_id);
      }
    }
    
    // 短暂休眠（100us）
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  
  return rx_frames.size();
}

void CANInterface::resetStatistics() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  memset(&stats_, 0, sizeof(stats_));
}

// ========== CANNetwork 实现 ==========

CANNetwork::CANNetwork() {}

CANNetwork::~CANNetwork() {
  closeAll();
}

bool CANNetwork::addInterface(const std::string& name, const std::string& port, int baudrate) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  // 检查是否已存在
  if (interfaces_.find(name) != interfaces_.end()) {
    std::cerr << "[CANNetwork] 接口已存在: " << name << std::endl;
    return false;
  }
  
  auto interface = std::make_shared<CANInterface>(port, baudrate);
  if (!interface->open()) {
    return false;
  }
  
  // 设置回调（转发到全局回调）
  if (global_rx_callback_) {
    interface->setRxCallback(global_rx_callback_);
  }
  
  interfaces_[name] = interface;
  std::cout << "[CANNetwork] 添加接口: " << name << " (" << port << ")" << std::endl;
  
  return true;
}

std::shared_ptr<CANInterface> CANNetwork::getInterface(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  auto it = interfaces_.find(name);
  if (it != interfaces_.end()) {
    return it->second;
  }
  
  return nullptr;
}

bool CANNetwork::send(const std::string& interface_name, uint32_t can_id, 
                      const uint8_t* data, size_t len) {
  auto interface = getInterface(interface_name);
  if (!interface) {
    return false;
  }
  
  return interface->send(can_id, data, len);
}

size_t CANNetwork::broadcast(uint32_t can_id, const uint8_t* data, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  size_t sent_count = 0;
  for (auto& [name, interface] : interfaces_) {
    if (interface->send(can_id, data, len)) {
      sent_count++;
    }
  }
  
  return sent_count;
}

void CANNetwork::setGlobalRxCallback(CANRxCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  global_rx_callback_ = callback;
  
  // 更新所有已存在接口的回调
  for (auto& [name, interface] : interfaces_) {
    interface->setRxCallback(callback);
  }
}

void CANNetwork::startAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->startRxThread();
  }
  
  std::cout << "[CANNetwork] 启动了 " << interfaces_.size() << " 个接收线程" << std::endl;
}

void CANNetwork::stopAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->stopRxThread();
  }
}

void CANNetwork::closeAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (auto& [name, interface] : interfaces_) {
    interface->stopRxThread();
    interface->close();
  }
  
  interfaces_.clear();
}

} // namespace hardware
} // namespace motor_control
