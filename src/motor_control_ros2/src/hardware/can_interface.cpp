#include "motor_control_ros2/hardware/can_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <sys/select.h>

namespace motor_control {
namespace hardware {

// ========== CANInterface 实现 ==========

CANInterface::CANInterface(const std::string& port, int baudrate)
  : port_(port)
  , baudrate_(baudrate)
  , fd_(-1)
  , rx_queue_(std::make_shared<ThreadSafeQueue>(1000))  // 1000 帧缓冲
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
  
  // 时间间隔 (32bit 小端) = 0ms（立即发送）
  tx_buffer_[8] = 0x00;
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

bool CANInterface::sendRaw(uint32_t can_id, const uint8_t* data, size_t len) {
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

bool CANInterface::send(uint32_t can_id, const uint8_t* data, size_t len) {
  return sendRaw(can_id, data, len);
}

bool CANInterface::receiveRaw(int timeout_us) {
  if (fd_ < 0) {
    return false;
  }
  
  // 使用 select 实现微秒级超时
  if (timeout_us > 0) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    
    struct timeval tv;
    tv.tv_sec = timeout_us / 1000000;
    tv.tv_usec = timeout_us % 1000000;
    
    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      return false;  // 超时或错误
    }
  }
  
  // 非阻塞读取可用数据
  ssize_t n = read(fd_, rx_buffer_, sizeof(rx_buffer_));
  if (n > 0) {
    // 将新数据添加到累积缓冲区（线程安全）
    std::lock_guard<std::mutex> lock(rx_accumulator_mutex_);
    rx_accumulator_.insert(rx_accumulator_.end(), rx_buffer_, rx_buffer_ + n);
    return true;
  } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.rx_errors++;
  }
  
  return false;
}

bool CANInterface::parseFrame(CANFrame& frame) {
  // 尝试从累积缓冲区中解析完整帧（线程安全）
  std::lock_guard<std::mutex> lock(rx_accumulator_mutex_);
  
  // 根据文档：AA [CMD] [格式] [CANID 4字节] [数据 8字节] 55 (共16字节)
  while (rx_accumulator_.size() >= 16) {
    // 寻找帧头 0xAA
    bool found_header = false;
    size_t header_pos = 0;
    
    for (size_t i = 0; i < rx_accumulator_.size(); ++i) {
      if (rx_accumulator_[i] == 0xAA) {
        found_header = true;
        header_pos = i;
        break;
      }
    }
    
    if (!found_header) {
      // 没有找到帧头，清空缓冲区
      rx_accumulator_.clear();
      return false;
    }
    
    // 丢弃帧头之前的数据
    if (header_pos > 0) {
      rx_accumulator_.erase(rx_accumulator_.begin(), rx_accumulator_.begin() + header_pos);
      std::lock_guard<std::mutex> lock2(stats_mutex_);
      stats_.frame_errors++;
    }
    
    // 检查是否有完整的 16 字节帧
    if (rx_accumulator_.size() < 16) {
      return false;
    }
    
    // 检查帧尾
    if (rx_accumulator_[15] != 0x55) {
      // 帧尾不正确，丢弃这个帧头，继续寻找
      rx_accumulator_.erase(rx_accumulator_.begin(), rx_accumulator_.begin() + 1);
      std::lock_guard<std::mutex> lock2(stats_mutex_);
      stats_.frame_errors++;
      continue;
    }
    
    // 解析帧
    // Byte 0: 0xAA (帧头)
    // Byte 1: CMD
    // Byte 2: 格式 (包含数据长度、帧类型等)
    // Byte 3-6: CAN ID (小端)
    // Byte 7-14: 数据 (8字节)
    // Byte 15: 0x55 (帧尾)
    
    frame.can_id = rx_accumulator_[3] | (rx_accumulator_[4] << 8) | 
                   (rx_accumulator_[5] << 16) | (rx_accumulator_[6] << 24);
    
    // 数据 (Byte 7-14, 固定8字节)
    frame.len = 8;
    memcpy(frame.data, &rx_accumulator_[7], 8);
    frame.timestamp = std::chrono::steady_clock::now();
    
    // 移除已处理的帧
    rx_accumulator_.erase(rx_accumulator_.begin(), rx_accumulator_.begin() + 16);
    
    std::lock_guard<std::mutex> lock2(stats_mutex_);
    stats_.rx_frames++;
    
    return true;
  }
  
  return false;
}

bool CANInterface::sendRecv(uint32_t can_id, const uint8_t* data, size_t len, 
                            CANFrame& response, int timeout_us) {
  if (!sendRaw(can_id, data, len)) {
    return false;
  }
  
  // 延迟 50us 等待设备处理
  std::this_thread::sleep_for(std::chrono::microseconds(50));
  
  // 等待反馈（超时时间）
  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::microseconds(timeout_us);
  
  while (std::chrono::steady_clock::now() < deadline) {
    // 尝试接收数据
    receiveRaw(100);  // 100us 轮询间隔
    
    // 尝试解析帧
    if (parseFrame(response)) {
      // 检查是否是期望的反馈（根据 CAN ID 匹配）
      // DJI 电机：发送 ID 0x200/0x1FF/0x2FF，反馈 ID 0x201-0x208/0x205-0x20B
      // 这里简化处理：接收到任何帧都认为是反馈
      return true;
    }
  }
  
  // 超时
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.timeouts++;
  return false;
}

size_t CANInterface::sendRecvBatch(const std::vector<CANFrame>& frames, 
                                   std::vector<CANFrame>& responses,
                                   int timeout_us) {
  responses.clear();
  responses.reserve(frames.size());
  
  size_t success_count = 0;
  for (const auto& frame : frames) {
    CANFrame response;
    if (sendRecv(frame.can_id, frame.data, frame.len, response, timeout_us)) {
      responses.push_back(response);
      success_count++;
    }
  }
  
  return success_count;
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
  
  // 关闭队列，唤醒接收线程
  if (rx_queue_) {
    rx_queue_->shutdown();
  }
  
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

void CANInterface::receiveLoop() {
  using namespace std::chrono;
  
  while (rx_running_) {
    // 高频接收（100us 轮询间隔，10kHz）
    receiveRaw(100);
    
    // 解析所有可用帧
    CANFrame frame;
    while (parseFrame(frame)) {
      // 推送到队列
      if (rx_queue_) {
        if (!rx_queue_->push(frame)) {
          std::lock_guard<std::mutex> lock(stats_mutex_);
          stats_.queue_drops++;
        }
      }
      
      // 调用回调
      if (rx_callback_) {
        rx_callback_(frame.can_id, frame.data, frame.len);
      }
    }
    
    // 短暂休眠避免 CPU 占用过高（100us）
    std::this_thread::sleep_for(microseconds(100));
  }
}

CANInterface::Statistics CANInterface::getStatistics() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  Statistics stats = stats_;
  
  // 添加队列丢帧数
  if (rx_queue_) {
    stats.queue_drops = rx_queue_->getDroppedFrames();
  }
  
  return stats;
}

void CANInterface::resetStatistics() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  memset(&stats_, 0, sizeof(stats_));
  
  if (rx_queue_) {
    rx_queue_->clear();
  }
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

bool CANNetwork::sendRecv(const std::string& interface_name, uint32_t can_id, 
                          const uint8_t* data, size_t len, 
                          CANFrame& response, int timeout_us) {
  auto interface = getInterface(interface_name);
  if (!interface) {
    return false;
  }
  
  return interface->sendRecv(can_id, data, len, response, timeout_us);
}

bool CANNetwork::send(const std::string& interface_name, uint32_t can_id, 
                      const uint8_t* data, size_t len) {
  auto interface = getInterface(interface_name);
  if (!interface) {
    return false;
  }
  
  return interface->send(can_id, data, len);
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
