#ifndef POSITIONING_BRIDGE_ROS2__PROTOCOL_TYPES_HPP_
#define POSITIONING_BRIDGE_ROS2__PROTOCOL_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace positioning_bridge_ros2 {

enum class ProtocolMode {
  LEGACY_ASCII,
  BINARY_V1,
};

enum class FrameKind {
  NONE,
  ODOM_POSE,
  ODOM_STATE,
  STATUS,
  HEARTBEAT,
  TIME_SYNC_RESPONSE,
  SET_LOCAL_ORIGIN_ACK,
};

struct PoseSample {
  double x {0.0};
  double y {0.0};
  double yaw {0.0};
  double roll {0.0};
  double vx {0.0};
  double vy {0.0};
  double wz {0.0};
  bool has_velocity {false};
  bool source_time_valid {false};
  uint64_t source_time_us {0};
  uint16_t status_bits {0};
  uint8_t quality {0};
  std::string raw_text;
};

struct StatusSample {
  bool source_time_valid {false};
  uint64_t source_time_us {0};
  uint16_t status_bits {0};
  uint8_t quality {0};
  uint8_t link_state {0};
  uint32_t uptime_ms {0};
  uint16_t error_code {0};
};

struct TimeSyncResponse {
  uint64_t echoed_host_time_us {0};
  uint64_t mcu_time_us {0};
};

struct AckSample {
  uint16_t acked_seq {0};
  uint16_t result_code {0};
  uint32_t event_counter {0};
};

struct DecodedFrame {
  FrameKind kind {FrameKind::NONE};
  uint16_t sequence {0};
  std::optional<PoseSample> pose;
  std::optional<StatusSample> status;
  std::optional<TimeSyncResponse> time_sync;
  std::optional<AckSample> ack;
};

struct ProtocolStats {
  uint64_t frames_ok {0};
  uint64_t frames_crc_error {0};
  uint64_t frames_parse_error {0};
  uint64_t bytes_rx {0};
};

}  // namespace positioning_bridge_ros2

#endif  // POSITIONING_BRIDGE_ROS2__PROTOCOL_TYPES_HPP_
