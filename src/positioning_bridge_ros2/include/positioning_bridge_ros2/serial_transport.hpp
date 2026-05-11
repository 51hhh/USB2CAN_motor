#ifndef POSITIONING_BRIDGE_ROS2__SERIAL_TRANSPORT_HPP_
#define POSITIONING_BRIDGE_ROS2__SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <termios.h>
#include <vector>

namespace positioning_bridge_ros2 {

struct SerialConfig {
  std::string device;
  int baudrate {115200};
  int data_bits {8};
  int stop_bits {1};
  std::string parity {"odd"};
  int read_timeout_ms {20};
};

class SerialTransport {
public:
  SerialTransport() = default;
  ~SerialTransport();

  bool open(const SerialConfig & config, std::string & error_message);
  void close();
  bool isOpen() const;

  std::ptrdiff_t readSome(uint8_t * buffer, std::size_t size, int timeout_ms, std::string & error_message);
  bool writeAll(const std::vector<uint8_t> & data, std::string & error_message);

  static speed_t toBaudConstant(int baudrate, bool & supported);

private:
  int fd_ {-1};
  SerialConfig config_;
};

}  // namespace positioning_bridge_ros2

#endif  // POSITIONING_BRIDGE_ROS2__SERIAL_TRANSPORT_HPP_
