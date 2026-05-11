#include "positioning_bridge_ros2/protocol_decoder.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <sstream>

namespace positioning_bridge_ros2 {
namespace {
constexpr uint8_t kSof0 = 0xAA;
constexpr uint8_t kSof1 = 0x55;
constexpr uint8_t kVersion = 0x01;
constexpr std::size_t kBinaryHeaderSize = 7;  // AA 55 ver type seq payloadLen(2)

constexpr uint8_t kMsgOdomPose = 0x01;
constexpr uint8_t kMsgOdomState = 0x02;
constexpr uint8_t kMsgStatus = 0x10;
constexpr uint8_t kMsgHeartbeat = 0x11;
constexpr uint8_t kMsgTimeSyncRequest = 0x20;
constexpr uint8_t kMsgTimeSyncResponse = 0x21;
constexpr uint8_t kMsgSetLocalOrigin = 0x30;
constexpr uint8_t kMsgSetLocalOriginAck = 0x31;

uint16_t readLe16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t readLe32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

uint64_t readLe64(const uint8_t * data)
{
  uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    value |= (static_cast<uint64_t>(data[i]) << (8U * i));
  }
  return value;
}

float readLeFloat32(const uint8_t * data)
{
  const uint32_t raw = readLe32(data);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

void appendLe16(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void appendLe64(std::vector<uint8_t> & out, uint64_t value)
{
  for (std::size_t i = 0; i < 8; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (8U * i)) & 0xFFU));
  }
}

void appendLeFloat32(std::vector<uint8_t> & out, float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(float));
  out.push_back(static_cast<uint8_t>(raw & 0xFFU));
  out.push_back(static_cast<uint8_t>((raw >> 8U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((raw >> 16U) & 0xFFU));
  out.push_back(static_cast<uint8_t>((raw >> 24U) & 0xFFU));
}
}  // namespace

ProtocolDecoder::ProtocolDecoder(ProtocolMode mode)
: mode_(mode)
{
}

void ProtocolDecoder::feed(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames)
{
  stats_.bytes_rx += size;
  if (mode_ == ProtocolMode::LEGACY_ASCII) {
    feedLegacyAscii(data, size, out_frames);
  } else {
    feedBinaryV1(data, size, out_frames);
  }
}

void ProtocolDecoder::reset()
{
  ascii_buffer_.clear();
  binary_buffer_.clear();
  stats_ = ProtocolStats{};
}

void ProtocolDecoder::feedLegacyAscii(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames)
{
  ascii_buffer_.append(reinterpret_cast<const char *>(data), size);

  std::size_t newline_pos = ascii_buffer_.find('\n');
  while (newline_pos != std::string::npos) {
    std::string line = ascii_buffer_.substr(0, newline_pos);
    ascii_buffer_.erase(0, newline_pos + 1U);

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    DecodedFrame frame;
    if (tryParseLegacyLine(line, frame)) {
      stats_.frames_ok++;
      out_frames.push_back(frame);
    }

    newline_pos = ascii_buffer_.find('\n');
  }
}

void ProtocolDecoder::feedBinaryV1(const uint8_t * data, std::size_t size, std::vector<DecodedFrame> & out_frames)
{
  binary_buffer_.insert(binary_buffer_.end(), data, data + size);

  while (true) {
    DecodedFrame frame;
    std::size_t consumed_bytes = 0;
    if (!tryParseBinaryFrame(frame, consumed_bytes)) {
      break;
    }

    if (consumed_bytes == 0) {
      break;
    }

    binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + static_cast<std::ptrdiff_t>(consumed_bytes));

    if (frame.kind != FrameKind::NONE) {
      stats_.frames_ok++;
      out_frames.push_back(frame);
    }
  }
}

bool ProtocolDecoder::tryParseLegacyLine(const std::string & line, DecodedFrame & frame)
{
  if (line.rfind("bc ", 0) != 0) {
    return false;
  }

  std::stringstream ss(line);
  std::string tag;
  PoseSample pose;
  if (!(ss >> tag >> pose.x >> pose.y >> pose.yaw >> pose.roll)) {
    stats_.frames_parse_error++;
    return false;
  }

  pose.raw_text = line;
  frame.kind = FrameKind::ODOM_POSE;
  frame.pose = pose;
  return true;
}

bool ProtocolDecoder::tryParseBinaryFrame(DecodedFrame & frame, std::size_t & consumed_bytes)
{
  consumed_bytes = 0;
  frame = DecodedFrame{};

  // 搜索帧头 0xAA 0x55
  std::size_t start_index = 0;
  while (binary_buffer_.size() >= start_index + 2U) {
    if (binary_buffer_[start_index] == kSof0 && binary_buffer_[start_index + 1U] == kSof1) {
      break;
    }
    ++start_index;
  }

  if (start_index > 0 && binary_buffer_.size() < start_index + 2U) {
    consumed_bytes = binary_buffer_.size();
    return true;
  }

  // MCU 帧头结构 (7 字节):
  // [0-1] 0xAA 0x55
  // [2]   version
  // [3]   msg_type
  // [4]   seq (uint8)
  // [5-6] payload_len (uint16 LE)
  if (binary_buffer_.size() < start_index + kBinaryHeaderSize) {
    return false;
  }

  const uint8_t * header = binary_buffer_.data() + start_index;
  const uint8_t version = header[2];
  const uint8_t msg_type = header[3];
  const uint8_t seq_u8 = header[4];
  const uint16_t payload_len = readLe16(header + 5);

  if (version != kVersion) {
    stats_.frames_parse_error++;
    consumed_bytes = start_index + 1U;
    return true;
  }

  // 总帧长 = header(7) + payload + CRC16(2)
  const std::size_t total_size = start_index + kBinaryHeaderSize + payload_len + 2U;
  if (binary_buffer_.size() < total_size) {
    return false;
  }

  // CRC16 校验：覆盖 byte[2] 到 payload 末尾（即 version 到 payload 结尾）
  const std::size_t crc_data_len = 1U + 1U + 1U + 2U + payload_len;  // ver+type+seq+len+payload
  const uint16_t expected_crc = readLe16(binary_buffer_.data() + start_index + kBinaryHeaderSize + payload_len);
  const uint16_t computed_crc = crc16Ccitt(header + 2, crc_data_len);
  if (computed_crc != expected_crc) {
    stats_.frames_crc_error++;
    consumed_bytes = start_index + 1U;
    return true;
  }

  const uint8_t * payload = binary_buffer_.data() + start_index + kBinaryHeaderSize;
  frame.sequence = static_cast<uint16_t>(seq_u8);

  switch (msg_type) {
    case kMsgOdomPose: {
      if (payload_len != 24U) {
        stats_.frames_parse_error++;
        break;
      }
      PoseSample pose;
      pose.source_time_valid = true;
      pose.source_time_us = readLe64(payload + 0);
      pose.x = readLeFloat32(payload + 8);
      pose.y = readLeFloat32(payload + 12);
      pose.yaw = readLeFloat32(payload + 16);
      pose.status_bits = readLe16(payload + 20);
      pose.quality = payload[22];
      frame.kind = FrameKind::ODOM_POSE;
      frame.pose = pose;
      break;
    }
    case kMsgOdomState: {
      if (payload_len != 36U) {
        stats_.frames_parse_error++;
        break;
      }
      PoseSample pose;
      pose.source_time_valid = true;
      pose.source_time_us = readLe64(payload + 0);
      pose.x = readLeFloat32(payload + 8);
      pose.y = readLeFloat32(payload + 12);
      pose.yaw = readLeFloat32(payload + 16);
      pose.vx = readLeFloat32(payload + 20);
      pose.vy = readLeFloat32(payload + 24);
      pose.wz = readLeFloat32(payload + 28);
      pose.has_velocity = true;
      pose.status_bits = readLe16(payload + 32);
      pose.quality = payload[34];
      frame.kind = FrameKind::ODOM_STATE;
      frame.pose = pose;
      break;
    }
    case kMsgStatus:
    case kMsgHeartbeat: {
      if (payload_len != 20U) {
        stats_.frames_parse_error++;
        break;
      }
      StatusSample status;
      status.source_time_valid = true;
      status.source_time_us = readLe64(payload + 0);
      status.status_bits = readLe16(payload + 8);
      status.quality = payload[10];
      status.link_state = payload[11];
      status.uptime_ms = readLe32(payload + 12);
      status.error_code = readLe16(payload + 16);
      frame.kind = (msg_type == kMsgStatus) ? FrameKind::STATUS : FrameKind::HEARTBEAT;
      frame.status = status;
      break;
    }
    case kMsgTimeSyncResponse: {
      if (payload_len != 16U) {
        stats_.frames_parse_error++;
        break;
      }
      TimeSyncResponse sync;
      sync.echoed_host_time_us = readLe64(payload + 0);
      sync.mcu_time_us = readLe64(payload + 8);
      frame.kind = FrameKind::TIME_SYNC_RESPONSE;
      frame.time_sync = sync;
      break;
    }
    case kMsgSetLocalOriginAck: {
      if (payload_len != 8U) {
        stats_.frames_parse_error++;
        break;
      }
      AckSample ack;
      ack.acked_seq = readLe16(payload + 0);
      ack.result_code = readLe16(payload + 2);
      ack.event_counter = readLe32(payload + 4);
      frame.kind = FrameKind::SET_LOCAL_ORIGIN_ACK;
      frame.ack = ack;
      break;
    }
    default:
      frame.kind = FrameKind::NONE;
      break;
  }

  consumed_bytes = total_size;
  return true;
}

uint8_t ProtocolDecoder::crc8(const uint8_t * data, std::size_t size)
{
  constexpr uint8_t polynomial = 0x07U;
  uint8_t crc = 0x00U;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ polynomial) : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

uint16_t ProtocolDecoder::crc16Ccitt(const uint8_t * data, std::size_t size)
{
  constexpr uint16_t polynomial = 0x1021U;
  uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ polynomial) : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> buildTimeSyncRequestFrame(uint16_t seq, uint64_t host_time_us)
{
  std::vector<uint8_t> frame;
  frame.reserve(kBinaryHeaderSize + 8U + 2U);

  // MCU header: AA 55 ver type seq payloadLen(2)
  frame.push_back(kSof0);
  frame.push_back(kSof1);
  frame.push_back(kVersion);
  frame.push_back(kMsgTimeSyncRequest);
  frame.push_back(static_cast<uint8_t>(seq & 0xFFU));  // seq as uint8
  appendLe16(frame, 8U);  // payload_len

  // payload
  appendLe64(frame, host_time_us);

  // CRC16 covers from version (byte[2]) to end of payload
  appendLe16(frame, ProtocolDecoder::crc16Ccitt(frame.data() + 2, frame.size() - 2));

  return frame;
}

std::vector<uint8_t> buildSetLocalOriginFrame(
  uint16_t seq, float x, float y, float yaw, uint8_t flags)
{
  std::vector<uint8_t> frame;
  frame.reserve(kBinaryHeaderSize + 16U + 2U);

  // MCU header: AA 55 ver type seq payloadLen(2)
  frame.push_back(kSof0);
  frame.push_back(kSof1);
  frame.push_back(kVersion);
  frame.push_back(kMsgSetLocalOrigin);
  frame.push_back(static_cast<uint8_t>(seq & 0xFFU));  // seq as uint8
  appendLe16(frame, 16U);  // payload_len

  // payload
  appendLeFloat32(frame, x);
  appendLeFloat32(frame, y);
  appendLeFloat32(frame, yaw);
  frame.push_back(flags);
  frame.push_back(0x00U);
  frame.push_back(0x00U);
  frame.push_back(0x00U);

  // CRC16 covers from version (byte[2]) to end of payload
  appendLe16(frame, ProtocolDecoder::crc16Ccitt(frame.data() + 2, frame.size() - 2));
  return frame;
}

}  // namespace positioning_bridge_ros2
