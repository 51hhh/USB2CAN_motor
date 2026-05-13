#include "motor_control_ros2/unitree_motor_native.hpp"
#include <algorithm>
#include <cstring>
#include <chrono>

namespace motor_control {

// CRC-CCITT 查表（与Python实现完全一致）
const uint16_t CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
  0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
  0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
  0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
  0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
  0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
  0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
  0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
  0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
  0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
  0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
  0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
  0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
  0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
  0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
  0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
  0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
  0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
  0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
  0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
  0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
  0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
  0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
  0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
  0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
  0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
  0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
  0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
  0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
  0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
  0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
  0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

uint16_t calcCrcCcitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ CRC_TABLE[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

UnitreeMotorNative::UnitreeMotorNative(const std::string& name,
                                       uint8_t motor_id,
                                       double gear_ratio)
  : MotorBase(name, MotorType::UNITREE_GO8010, static_cast<float>(gear_ratio), true)
  , motor_id_(motor_id)
  , gear_ratio_(gear_ratio)
{
  memset(tx_buffer_, 0, sizeof(tx_buffer_));
}

// =============================================================================
// 协议层：命令构建
// =============================================================================

void UnitreeMotorNative::buildCommandPacket() {
  std::lock_guard<std::mutex> lock(mutex_);

  // 直接使用命令值（offset/direction 由上层节点处理）
  double pos_cmd = cmd_pos_des_;
  double vel_cmd = cmd_vel_des_;
  double tau_cmd = cmd_torque_ff_;

  // 协议头
  tx_buffer_[0] = 0xFE;
  tx_buffer_[1] = 0xEE;

  // mode 字节: id(4bit) | status(3bit) | none(1bit)
  tx_buffer_[2] = static_cast<uint8_t>((motor_id_ & 0x0F) |
                                       ((static_cast<uint8_t>(mode_) & 0x07) << 4));

  // 定点数转换
  auto clamp_i16 = [](double v) -> int16_t {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v);
  };
  auto clamp_i32 = [](double v) -> int32_t {
    if (v > 2147483647.0) return 2147483647;
    if (v < -2147483648.0) return static_cast<int32_t>(-2147483647 - 1);
    return static_cast<int32_t>(v);
  };
  auto clamp_u16 = [](double v) -> uint16_t {
    if (v < 0.0) return 0;
    if (v > 65535.0) return 65535;
    return static_cast<uint16_t>(v);
  };

  int16_t tor_des_q8 = clamp_i16(tau_cmd * 256.0);
  int16_t spd_des_q7 = clamp_i16(vel_cmd * gear_ratio_ * 256.0 / (2.0 * M_PI));
  int32_t pos_des_q15 = clamp_i32(pos_cmd * gear_ratio_ * 32768.0 / (2.0 * M_PI));
  uint16_t k_pos_q15 = clamp_u16(cmd_kp_ * 1280.0);
  uint16_t k_spd_q15 = clamp_u16(cmd_kd_ * 1280.0);

  // 小端序打包到 [3..14]
  std::memcpy(&tx_buffer_[3], &tor_des_q8, sizeof(tor_des_q8));
  std::memcpy(&tx_buffer_[5], &spd_des_q7, sizeof(spd_des_q7));
  std::memcpy(&tx_buffer_[7], &pos_des_q15, sizeof(pos_des_q15));
  std::memcpy(&tx_buffer_[11], &k_pos_q15, sizeof(k_pos_q15));
  std::memcpy(&tx_buffer_[13], &k_spd_q15, sizeof(k_spd_q15));

  // CRC16（前 15 字节）
  uint16_t crc = calcCrcCcitt(tx_buffer_, 15);
  tx_buffer_[15] = static_cast<uint8_t>(crc & 0xFF);
  tx_buffer_[16] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

size_t UnitreeMotorNative::getCommandPacket(uint8_t* buffer) {
  buildCommandPacket();
  std::memcpy(buffer, tx_buffer_, 17);
  return 17;
}

// =============================================================================
// 协议层：反馈解析
// =============================================================================

bool UnitreeMotorNative::parseFeedback(const uint8_t* data, size_t len) {
  if (len < 16) {
    return false;
  }

  // 检查头部：返回帧头是 0xFD 0xEE（发送帧头是 0xFE 0xEE）
  if (data[0] != 0xFD || data[1] != 0xEE) {
    return false;
  }

  // 检查电机ID
  uint8_t fb_id = data[2] & 0x0F;
  if (fb_id != motor_id_) {
    return false;
  }

  // 验证CRC
  uint16_t recv_crc = data[14] | (data[15] << 8);
  uint16_t calc_crc = calcCrcCcitt(data, 14);
  if (recv_crc != calc_crc) {
    return false;
  }

  // 解析定点数
  int16_t torque_q8;
  int16_t speed_q7;
  int32_t pos_q15;
  int8_t temp;

  memcpy(&torque_q8, &data[3], 2);
  memcpy(&speed_q7, &data[5], 2);
  memcpy(&pos_q15, &data[7], 4);
  memcpy(&temp, &data[11], 1);

  uint8_t merror = data[12] & 0x07;

  // ========== 与 Python 完全一致的转换 ==========
  double torque = torque_q8 / 256.0;
  double speed = speed_q7 * 2.0 * M_PI / gear_ratio_ / 256.0;
  double pos_deg = pos_q15 * 360.0 / gear_ratio_ / 32768.0;
  double pos_rad = pos_deg * M_PI / 180.0;

  // 更新基类状态（应用方向和偏移，使 MotorBase::getOutputPosition() 等直接可用）
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_code_ = merror;
  }

  // 更新基类字段（原始值，offset/direction 由上层节点处理）
  position_ = pos_rad;
  velocity_ = speed;
  torque_ = torque;
  temperature_ = static_cast<float>(temp);

  // 更新基类心跳（设置 online_ = true 并记录时间）
  updateLastFeedbackTime(std::chrono::steady_clock::now().time_since_epoch().count());

  return true;
}

// =============================================================================
// 控制命令
// =============================================================================

void UnitreeMotorNative::setFOCCommand(double pos_des, double vel_des,
                                       double kp, double kd, double torque_ff) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::FOC;
  cmd_pos_des_ = pos_des;
  cmd_vel_des_ = vel_des;
  cmd_kp_ = kp;
  cmd_kd_ = kd;
  cmd_torque_ff_ = torque_ff;
}

void UnitreeMotorNative::setBrakeCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::BRAKE;
  cmd_pos_des_ = 0.0;
  cmd_vel_des_ = 0.0;
  cmd_kp_ = 0.0;
  cmd_kd_ = 0.0;
  cmd_torque_ff_ = 0.0;
}

void UnitreeMotorNative::setCalibrateCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::CALIBRATE;
}

} // namespace motor_control
