#include "motor_control_ros2/rc_usb_control_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>

namespace motor_control {

namespace {
constexpr std::size_t kFrameLen = 24;
constexpr uint16_t kMagic = 0xA55A;
constexpr uint8_t kVersion = 0x01;
constexpr uint8_t kPayloadLen = 24;
constexpr uint8_t kSwitchUp = 1;
constexpr uint8_t kSwitchDown = 2;
constexpr uint8_t kSwitchMid = 3;
}  // namespace

RCUsbControlNode::RCUsbControlNode()
: Node("rc_usb_control_node")
{
  configureParameters();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic_, 10);
  if (enable_vision_passthrough_) {
    vision_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      vision_cmd_vel_topic_, 10,
      std::bind(&RCUsbControlNode::visionCmdCallback, this, std::placeholders::_1));
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RCUsbControlNode::publishCommand, this));

  const auto now = this->now();
  last_frame_time_ = now;
  last_vision_cmd_time_ = now;
  last_diagnostics_time_ = now;
  rx_buffer_.reserve(4096);

  running_ = true;
  startSerialLoop();

  RCLCPP_INFO(
    get_logger(),
    "rc_usb_control_node 启动: serial=%s baud=%d output=%s estop=%s mode_field=%s mode_sw(up/mid/down)=%d/%d/%d",
    serial_port_.c_str(), baud_rate_, cmd_vel_topic_.c_str(), estop_topic_.c_str(),
    mode_switch_field_.c_str(), manual_switch_, estop_switch_, vision_switch_);
  RCLCPP_INFO(
    get_logger(),
    "遥控映射: lx->linear.x(右/左) ly->linear.y(前/后) rx->angular.z, 视觉输入=%s",
    enable_vision_passthrough_ ? vision_cmd_vel_topic_.c_str() : "disabled");
}

RCUsbControlNode::~RCUsbControlNode()
{
  stopSerialLoop();
}

void RCUsbControlNode::configureParameters()
{
  declare_parameter("serial_port", serial_port_);
  declare_parameter("port", serial_port_);
  declare_parameter("baud_rate", baud_rate_);
  declare_parameter("baudrate", baud_rate_);
  declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter("estop_topic", estop_topic_);
  declare_parameter("vision_cmd_vel_topic", vision_cmd_vel_topic_);
  declare_parameter("mode_switch_field", mode_switch_field_);

  declare_parameter("max_linear_velocity", max_linear_velocity_);
  declare_parameter("max_angular_velocity", max_angular_velocity_);
  declare_parameter("axis_max", axis_max_);
  declare_parameter("rc_range", axis_max_);
  declare_parameter("deadzone", deadzone_);
  declare_parameter("publish_rate", publish_rate_);
  declare_parameter("command_timeout", command_timeout_);
  declare_parameter("frame_timeout_sec", command_timeout_);
  declare_parameter("diagnostics_interval_sec", diagnostics_interval_sec_);
  declare_parameter("vision_timeout", vision_timeout_);
  declare_parameter("serial_stale_reconnect_sec", serial_stale_reconnect_sec_);

  declare_parameter("manual_switch", manual_switch_);
  declare_parameter("estop_switch", estop_switch_);
  declare_parameter("vision_switch", vision_switch_);
  declare_parameter("invert_linear_x", invert_linear_x_);
  declare_parameter("invert_linear_y", invert_linear_y_);
  declare_parameter("invert_angular_z", invert_angular_z_);
  declare_parameter("publish_zero_before_first_frame", publish_zero_before_first_frame_);
  declare_parameter("enable_vision_passthrough", enable_vision_passthrough_);

  declare_parameter("serial_read_chunk", serial_read_chunk_);
  declare_parameter("serial_reconnect_interval_ms", serial_reconnect_interval_ms_);

  get_parameter("serial_port", serial_port_);
  {
    std::string port_alias;
    get_parameter("port", port_alias);
    if (serial_port_ == "/dev/ttyACM1" && port_alias != serial_port_) {
      serial_port_ = port_alias;
    }
  }
  get_parameter("baud_rate", baud_rate_);
  {
    int baudrate_alias = baud_rate_;
    get_parameter("baudrate", baudrate_alias);
    if (baud_rate_ == 115200 && baudrate_alias != baud_rate_) {
      baud_rate_ = baudrate_alias;
    }
  }
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("estop_topic", estop_topic_);
  get_parameter("vision_cmd_vel_topic", vision_cmd_vel_topic_);
  get_parameter("mode_switch_field", mode_switch_field_);

  get_parameter("max_linear_velocity", max_linear_velocity_);
  get_parameter("max_angular_velocity", max_angular_velocity_);
  get_parameter("axis_max", axis_max_);
  {
    double rc_range_alias = axis_max_;
    get_parameter("rc_range", rc_range_alias);
    if (axis_max_ == 660.0 && rc_range_alias != axis_max_) {
      axis_max_ = rc_range_alias;
    }
  }
  get_parameter("deadzone", deadzone_);
  get_parameter("publish_rate", publish_rate_);
  get_parameter("command_timeout", command_timeout_);
  {
    double frame_timeout_alias = command_timeout_;
    get_parameter("frame_timeout_sec", frame_timeout_alias);
    if (command_timeout_ == 0.3 && frame_timeout_alias != command_timeout_) {
      command_timeout_ = frame_timeout_alias;
    }
  }
  get_parameter("diagnostics_interval_sec", diagnostics_interval_sec_);
  get_parameter("vision_timeout", vision_timeout_);
  get_parameter("serial_stale_reconnect_sec", serial_stale_reconnect_sec_);

  get_parameter("manual_switch", manual_switch_);
  get_parameter("estop_switch", estop_switch_);
  get_parameter("vision_switch", vision_switch_);
  get_parameter("invert_linear_x", invert_linear_x_);
  get_parameter("invert_linear_y", invert_linear_y_);
  get_parameter("invert_angular_z", invert_angular_z_);
  get_parameter("publish_zero_before_first_frame", publish_zero_before_first_frame_);
  get_parameter("enable_vision_passthrough", enable_vision_passthrough_);

  get_parameter("serial_read_chunk", serial_read_chunk_);
  get_parameter("serial_reconnect_interval_ms", serial_reconnect_interval_ms_);

  publish_rate_ = clamp(publish_rate_, 10.0, 200.0);
  diagnostics_interval_sec_ = std::max(0.2, diagnostics_interval_sec_);
  command_timeout_ = std::max(0.05, command_timeout_);
  vision_timeout_ = std::max(0.05, vision_timeout_);
  serial_stale_reconnect_sec_ = std::max(command_timeout_ + 0.1, serial_stale_reconnect_sec_);
  deadzone_ = clamp(deadzone_, 0.0, 0.95);
  axis_max_ = std::max(1.0, axis_max_);
  max_linear_velocity_ = std::max(0.0, max_linear_velocity_);
  max_angular_velocity_ = std::max(0.0, max_angular_velocity_);
  serial_read_chunk_ = std::max(16, serial_read_chunk_);
  serial_reconnect_interval_ms_ = std::max(200, serial_reconnect_interval_ms_);

  auto sanitizeSwitch = [](int value, int fallback) {
    return value >= 1 && value <= 3 ? value : fallback;
  };
  manual_switch_ = sanitizeSwitch(manual_switch_, kSwitchUp);
  estop_switch_ = sanitizeSwitch(estop_switch_, kSwitchMid);
  vision_switch_ = sanitizeSwitch(vision_switch_, kSwitchDown);
  if (mode_switch_field_ != "left" && mode_switch_field_ != "right") {
    mode_switch_field_ = "left";
  }
}

void RCUsbControlNode::startSerialLoop()
{
  serial_thread_ = std::thread(&RCUsbControlNode::serialReadLoop, this);
}

void RCUsbControlNode::stopSerialLoop()
{
  running_ = false;
  closeSerialPort();
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
}

void RCUsbControlNode::serialReadLoop()
{
  auto next_reconnect = std::chrono::steady_clock::now();
  std::vector<uint8_t> chunk(static_cast<std::size_t>(serial_read_chunk_));

  while (running_) {
    if (serial_fd_.load() < 0) {
      if (std::chrono::steady_clock::now() < next_reconnect) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      const int fd = openSerialPort();
      if (fd >= 0) {
        serial_fd_ = fd;
        RCLCPP_INFO(get_logger(), "已连接遥控虚拟串口: %s", serial_port_.c_str());
      } else {
        next_reconnect = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(serial_reconnect_interval_ms_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      continue;
    }

    const int fd = serial_fd_.load();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const auto node_now = now();
      const double stale_sec = has_valid_frame_
        ? (node_now - last_frame_time_).seconds()
        : 0.0;
      if (has_valid_frame_ && stale_sec > serial_stale_reconnect_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "遥控有效帧停止 %.2fs，重连串口: %s",
          stale_sec, serial_port_.c_str());
        has_valid_frame_ = false;
        control_mode_ = ControlMode::ESTOP;
        last_manual_cmd_ = zeroTwist();
        last_frame_time_ = node_now;
        closeSerialPort();
        next_reconnect = std::chrono::steady_clock::now() +
          std::chrono::milliseconds(serial_reconnect_interval_ms_);
        continue;
      }
    }

    const int n = read(fd, chunk.data(), chunk.size());
    if (n > 0) {
      onSerialData(chunk.data(), static_cast<std::size_t>(n));
      continue;
    }

    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_WARN(get_logger(), "遥控虚拟串口异常，准备重连: fd=%d errno=%d", fd, errno);
      closeSerialPort();
      next_reconnect = std::chrono::steady_clock::now();
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

int RCUsbControlNode::openSerialPort()
{
  const int fd = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    RCLCPP_DEBUG(get_logger(), "打开遥控虚拟串口失败 %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    return -1;
  }

  if (!setupSerialPort(fd)) {
    close(fd);
    return -1;
  }

  return fd;
}

void RCUsbControlNode::closeSerialPort()
{
  const int fd = serial_fd_.exchange(-1);
  if (fd >= 0) {
    close(fd);
  }

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  rx_buffer_.clear();
}

bool RCUsbControlNode::setupSerialPort(int fd)
{
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr 失败: %s", std::strerror(errno));
    return false;
  }

  cfsetospeed(&tty, baudRateToTermios(baud_rate_));
  cfsetispeed(&tty, baudRateToTermios(baud_rate_));

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr 失败: %s", std::strerror(errno));
    return false;
  }

  tcflush(fd, TCIOFLUSH);
  return true;
}

void RCUsbControlNode::onSerialData(const uint8_t* data, std::size_t len)
{
  if (data == nullptr || len == 0) {
    return;
  }

  std::vector<RcForwardFrame> frames;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    rx_bytes_ += len;
    rx_buffer_.insert(rx_buffer_.end(), data, data + len);
    frames = parseBufferedFrames();
  }

  for (const auto& frame : frames) {
    updateCommand(frame);
  }
}

std::vector<RCUsbControlNode::RcForwardFrame> RCUsbControlNode::parseBufferedFrames()
{
  std::vector<RcForwardFrame> frames;

  while (rx_buffer_.size() >= kFrameLen) {
    std::size_t header_pos = rx_buffer_.size();
    for (std::size_t i = 0; i + 1 < rx_buffer_.size(); ++i) {
      if (rx_buffer_[i] == 0x5A && rx_buffer_[i + 1] == 0xA5) {
        header_pos = i;
        break;
      }
    }

    if (header_pos == rx_buffer_.size()) {
      const bool keep_tail = rx_buffer_.back() == 0x5A;
      const uint8_t tail = rx_buffer_.back();
      dropped_bytes_ += rx_buffer_.size() - (keep_tail ? 1 : 0);
      rx_buffer_.clear();
      if (keep_tail) {
        rx_buffer_.push_back(tail);
      }
      return frames;
    }

    if (header_pos > 0) {
      dropped_bytes_ += header_pos;
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(header_pos));
    }

    if (rx_buffer_.size() < kFrameLen) {
      return frames;
    }

    const RcForwardFrame frame = parseFrame(rx_buffer_.data());
    if (validateFrame(rx_buffer_.data(), frame)) {
      frames.push_back(frame);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(kFrameLen));
    } else {
      ++invalid_frames_;
      ++dropped_bytes_;
      rx_buffer_.erase(rx_buffer_.begin());
    }
  }

  if (rx_buffer_.size() > 4096) {
    dropped_bytes_ += rx_buffer_.size();
    rx_buffer_.clear();
  }

  return frames;
}

bool RCUsbControlNode::validateFrame(const uint8_t* data, const RcForwardFrame& frame)
{
  if (frame.magic != kMagic || frame.version != kVersion || frame.payload_len != kPayloadLen) {
    return false;
  }

  const uint16_t computed_crc = crc16Modbus(data, kFrameLen - sizeof(uint16_t));
  if (computed_crc != frame.crc16) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RC 帧 CRC 错误: recv=0x%04X calc=0x%04X", frame.crc16, computed_crc);
    return false;
  }

  return true;
}

void RCUsbControlNode::updateCommand(const RcForwardFrame& frame)
{
  const uint8_t mode_switch = modeSwitchValue(frame);
  const ControlMode mode = modeFromSwitch(mode_switch);
  const geometry_msgs::msg::Twist manual_cmd = commandFromFrame(frame);

  bool log_mode_change = false;
  ControlMode previous_mode = ControlMode::ESTOP;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    log_mode_change = !has_valid_frame_ || mode != control_mode_;
    previous_mode = control_mode_;
    control_mode_ = mode;
    last_manual_cmd_ = manual_cmd;
    last_frame_time_ = now();
    last_seq_ = frame.seq;
    last_sw_left_ = frame.sw_left;
    last_sw_right_ = frame.sw_right;
    has_valid_frame_ = true;
    ++valid_frames_;
  }

  if (log_mode_change) {
    const auto modeName = [](ControlMode value) {
      switch (value) {
        case ControlMode::MANUAL: return "manual";
        case ControlMode::VISION: return "vision";
        case ControlMode::ESTOP: return "estop";
      }
      return "unknown";
    };
    RCLCPP_INFO(
      get_logger(), "遥控模式切换: %s -> %s seq=%u mode_sw=%u frame_sw_left=%u frame_sw_right=%u",
      modeName(previous_mode), modeName(mode), frame.seq, mode_switch, frame.sw_left, frame.sw_right);
  }
}

void RCUsbControlNode::publishCommand()
{
  geometry_msgs::msg::Twist cmd;
  bool should_publish = true;
  bool timed_out = false;
  bool vision_stale = false;
  bool estop_active = true;

  const auto now_time = now();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const bool have_fresh_frame =
      has_valid_frame_ && ((now_time - last_frame_time_).seconds() <= command_timeout_);

    if (!has_valid_frame_) {
      cmd = zeroTwist();
      should_publish = publish_zero_before_first_frame_;
    } else if (!have_fresh_frame) {
      cmd = zeroTwist();
      timed_out = true;
    } else if (control_mode_ == ControlMode::MANUAL) {
      cmd = last_manual_cmd_;
      estop_active = false;
    } else if (control_mode_ == ControlMode::VISION) {
      estop_active = false;
      const bool have_fresh_vision =
        enable_vision_passthrough_ && has_vision_cmd_ &&
        ((now_time - last_vision_cmd_time_).seconds() <= vision_timeout_);
      if (have_fresh_vision) {
        cmd = last_vision_cmd_;
      } else {
        cmd = zeroTwist();
        vision_stale = true;
      }
    } else {
      cmd = zeroTwist();
    }
  }

  logDiagnostics(now_time);

  if (timed_out) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "RC 帧超时，底盘零速");
  }
  if (vision_stale) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "视觉档未收到新视觉速度，底盘零速: topic=%s", vision_cmd_vel_topic_.c_str());
  }

  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = estop_active;
  estop_pub_->publish(estop_msg);

  if (should_publish) {
    cmd_vel_pub_->publish(cmd);
  }
}

void RCUsbControlNode::visionCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  last_vision_cmd_ = *msg;
  last_vision_cmd_time_ = now();
  has_vision_cmd_ = true;
}

void RCUsbControlNode::logDiagnostics(const rclcpp::Time& now_time)
{
  uint64_t rx_bytes = 0;
  uint64_t valid_frames = 0;
  uint64_t invalid_frames = 0;
  uint64_t dropped_bytes = 0;
  uint16_t last_seq = 0;
  uint8_t sw_left = 0;
  uint8_t sw_right = 0;
  ControlMode mode = ControlMode::ESTOP;
  double frame_age = -1.0;
  bool has_valid_frame = false;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if ((now_time - last_diagnostics_time_).seconds() < diagnostics_interval_sec_) {
      return;
    }
    last_diagnostics_time_ = now_time;
    valid_frames = valid_frames_;
    last_seq = last_seq_;
    sw_left = last_sw_left_;
    sw_right = last_sw_right_;
    mode = control_mode_;
    has_valid_frame = has_valid_frame_;
    frame_age = has_valid_frame_ ? (now_time - last_frame_time_).seconds() : -1.0;
  }

  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    rx_bytes = rx_bytes_;
    invalid_frames = invalid_frames_;
    dropped_bytes = dropped_bytes_;
  }

  const char* mode_name = "estop";
  if (mode == ControlMode::MANUAL) {
    mode_name = "manual";
  } else if (mode == ControlMode::VISION) {
    mode_name = "vision";
  }

  if (!has_valid_frame) {
    RCLCPP_WARN(
      get_logger(), "RC 诊断: 未收到有效帧 rx_bytes=%lu invalid=%lu dropped=%lu",
      rx_bytes, invalid_frames, dropped_bytes);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "RC 诊断: valid=%lu invalid=%lu dropped=%lu rx_bytes=%lu seq=%u age=%.3fs sw(L/R)=%u/%u mode=%s",
    valid_frames, invalid_frames, dropped_bytes, rx_bytes, last_seq, frame_age,
    sw_left, sw_right, mode_name);
}

RCUsbControlNode::ControlMode RCUsbControlNode::modeFromSwitch(uint8_t sw) const
{
  if (static_cast<int>(sw) == manual_switch_) {
    return ControlMode::MANUAL;
  }
  if (static_cast<int>(sw) == vision_switch_) {
    return ControlMode::VISION;
  }
  return ControlMode::ESTOP;
}

uint8_t RCUsbControlNode::modeSwitchValue(const RcForwardFrame& frame) const
{
  return mode_switch_field_ == "left" ? frame.sw_left : frame.sw_right;
}

geometry_msgs::msg::Twist RCUsbControlNode::commandFromFrame(const RcForwardFrame& frame) const
{
  double linear_x = normalizeAxis(frame.lx) * max_linear_velocity_;
  double linear_y = normalizeAxis(frame.ly) * max_linear_velocity_;
  double angular_z = normalizeAxis(frame.rx) * max_angular_velocity_;

  if (invert_linear_x_) {
    linear_x = -linear_x;
  }
  if (invert_linear_y_) {
    linear_y = -linear_y;
  }
  if (invert_angular_z_) {
    angular_z = -angular_z;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_x;
  cmd.linear.y = linear_y;
  cmd.angular.z = angular_z;
  return cmd;
}

geometry_msgs::msg::Twist RCUsbControlNode::zeroTwist() const
{
  return geometry_msgs::msg::Twist();
}

double RCUsbControlNode::normalizeAxis(int16_t raw) const
{
  double value = clamp(static_cast<double>(raw) / axis_max_, -1.0, 1.0);
  if (std::abs(value) < deadzone_) {
    return 0.0;
  }

  const double sign = value >= 0.0 ? 1.0 : -1.0;
  return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
}

double RCUsbControlNode::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

RCUsbControlNode::RcForwardFrame RCUsbControlNode::parseFrame(const uint8_t* data)
{
  RcForwardFrame frame;
  frame.magic = readLe16(data + 0);
  frame.version = data[2];
  frame.payload_len = data[3];
  frame.seq = readLe16(data + 4);
  frame.tick_ms = readLe32(data + 6);
  frame.lx = readLeI16(data + 10);
  frame.ly = readLeI16(data + 12);
  frame.rx = readLeI16(data + 14);
  frame.ry = readLeI16(data + 16);
  frame.sw_left = data[18];
  frame.sw_right = data[19];
  frame.reserved = readLe16(data + 20);
  frame.crc16 = readLe16(data + 22);
  return frame;
}

uint16_t RCUsbControlNode::readLe16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t RCUsbControlNode::readLe32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24);
}

int16_t RCUsbControlNode::readLeI16(const uint8_t* data)
{
  return static_cast<int16_t>(readLe16(data));
}

uint16_t RCUsbControlNode::crc16Modbus(const uint8_t* data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

int RCUsbControlNode::baudRateToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    default: return B115200;
  }
}

}  // namespace motor_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_control::RCUsbControlNode>());
  rclcpp::shutdown();
  return 0;
}
