#ifndef POSITIONING_BRIDGE_ROS2__PROTOCOL_DECODER_HPP_
#define POSITIONING_BRIDGE_ROS2__PROTOCOL_DECODER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "positioning_bridge_ros2/protocol_types.hpp"

namespace positioning_bridge_ros2 {

class ProtocolDecoder {
public:
  explicit ProtocolDecoder(ProtocolMode mode);

  void feed(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames);
  void reset();

  ProtocolMode mode() const { return mode_; }
  const ProtocolStats & stats() const { return stats_; }

  static uint8_t crc8(const uint8_t * data, std::size_t size);
  static uint16_t crc16Ccitt(const uint8_t * data, std::size_t size);

private:
  void feedLegacyAscii(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames);
  void feedBinaryV1(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames);
  bool tryParseLegacyLine(const std::string & line, DecodedFrame & frame);
  bool tryParseBinaryFrame(DecodedFrame & frame, std::size_t & consumed_bytes);

  ProtocolMode mode_;
  ProtocolStats stats_;
  std::string ascii_buffer_;
  std::vector<uint8_t> binary_buffer_;
};

std::vector<uint8_t> buildTimeSyncRequestFrame(uint16_t seq, uint64_t host_time_us);
std::vector<uint8_t> buildSetLocalOriginFrame(
  uint16_t seq, float x, float y, float yaw, uint8_t flags = 0x01);

}  // namespace positioning_bridge_ros2

#endif  // POSITIONING_BRIDGE_ROS2__PROTOCOL_DECODER_HPP_
