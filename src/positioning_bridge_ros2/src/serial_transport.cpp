#include "positioning_bridge_ros2/serial_transport.hpp"

#include <cerrno>
#include <cstring>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace positioning_bridge_ros2 {

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open(const SerialConfig & config, std::string & error_message)
{
  close();

  bool supported = false;
  const speed_t baud = toBaudConstant(config.baudrate, supported);
  if (!supported) {
    error_message = "不支持的波特率: " + std::to_string(config.baudrate);
    return false;
  }

  fd_ = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_message = std::string("打开串口失败: ") + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    error_message = std::string("读取串口属性失败: ") + std::strerror(errno);
    close();
    return false;
  }

  ::cfmakeraw(&tio);
  ::cfsetispeed(&tio, baud);
  ::cfsetospeed(&tio, baud);

  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= (config.data_bits == 7) ? CS7 : CS8;

  if (config.stop_bits == 2) {
    tio.c_cflag |= CSTOPB;
  } else {
    tio.c_cflag &= ~CSTOPB;
  }

  tio.c_cflag &= ~(PARENB | PARODD);
  if (config.parity == "even") {
    tio.c_cflag |= PARENB;
  } else if (config.parity == "odd") {
    tio.c_cflag |= PARENB | PARODD;
  }

  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error_message = std::string("设置串口属性失败: ") + std::strerror(errno);
    close();
    return false;
  }

  config_ = config;
  error_message.clear();
  return true;
}

void SerialTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialTransport::isOpen() const
{
  return fd_ >= 0;
}

std::ptrdiff_t SerialTransport::readSome(uint8_t * buffer, std::size_t size, int timeout_ms, std::string & error_message)
{
  if (!isOpen()) {
    error_message = "串口未打开";
    return -1;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int poll_ret = ::poll(&pfd, 1, timeout_ms);
  if (poll_ret < 0) {
    error_message = std::string("poll 失败: ") + std::strerror(errno);
    return -1;
  }
  if (poll_ret == 0) {
    return 0;
  }

  const ssize_t bytes_read = ::read(fd_, buffer, size);
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    error_message = std::string("read 失败: ") + std::strerror(errno);
    return -1;
  }

  return bytes_read;
}

bool SerialTransport::writeAll(const std::vector<uint8_t> & data, std::string & error_message)
{
  if (!isOpen()) {
    error_message = "串口未打开";
    return false;
  }

  std::size_t offset = 0;
  while (offset < data.size()) {
    const ssize_t written = ::write(fd_, data.data() + offset, data.size() - offset);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      error_message = std::string("write 失败: ") + std::strerror(errno);
      return false;
    }
    offset += static_cast<std::size_t>(written);
  }

  error_message.clear();
  return true;
}

speed_t SerialTransport::toBaudConstant(int baudrate, bool & supported)
{
  supported = true;
  switch (baudrate) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      supported = false;
      return B115200;
  }
}

}  // namespace positioning_bridge_ros2
