#include "motor_control_ros2/damiao_motor.hpp"
#include <cstring>
#include <algorithm>

namespace motor_control {

DamiaoMotor::DamiaoMotor(const std::string& joint_name, MotorType type,
                         uint8_t can_id, uint8_t master_id, uint8_t bus_id)
  : MotorBase(joint_name, type, 40.0f, true)  // 减速比40:1，编码器在输出轴
  , can_id_(can_id)
  , master_id_(master_id)
  , bus_id_(bus_id)
  , p_des_(0.0f)
  , v_des_(0.0f)
  , kp_(0.0f)
  , kd_(0.0f)
  , t_ff_(0.0f)
  , error_code_(0)
  , temp_mos_(0)
  , temp_rotor_(0)
{
  // DM4340 参数范围
  P_MIN_ = -12.5f;
  P_MAX_ = 12.5f;
  V_MIN_ = -30.0f;
  V_MAX_ = 30.0f;
  T_MIN_ = -10.0f;
  T_MAX_ = 10.0f;
  KP_MIN_ = 0.0f;
  KP_MAX_ = 500.0f;
  KD_MIN_ = 0.0f;
  KD_MAX_ = 5.0f;
}

void DamiaoMotor::updateFeedback(uint32_t can_id, const uint8_t* data, size_t len) {
  // 检查 Master ID（反馈帧 ID）
  if (can_id != master_id_ || len < 8) {
    return;
  }
  
  online_ = true;
  
  // 解析反馈帧
  // D[0]: ID | ERR<<4
  uint8_t id_err = data[0];
  error_code_ = (id_err >> 4) & 0x0F;
  
  // D[1-2]: POS[15:8], POS[7:0]
  uint16_t p_int = (static_cast<uint16_t>(data[1]) << 8) | data[2];
  
  // D[3-4]: VEL[11:4], VEL[3:0]|T[11:8]
  uint16_t v_int = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
  
  // D[4-5]: VEL[3:0]|T[11:8], T[7:0]
  uint16_t t_int = ((static_cast<uint16_t>(data[4]) & 0x0F) << 8) | data[5];
  
  // D[6]: T_MOS
  temp_mos_ = data[6];
  
  // D[7]: T_Rotor
  temp_rotor_ = data[7];
  
  // 转换为浮点数
  position_ = uintToFloat(p_int, P_MIN_, P_MAX_, 16);
  velocity_ = uintToFloat(v_int, V_MIN_, V_MAX_, 12);
  torque_ = uintToFloat(t_int, T_MIN_, T_MAX_, 12);
  
  // 温度取平均
  temperature_ = (temp_mos_ + temp_rotor_) / 2.0f;
}

void DamiaoMotor::getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) {
  can_id = can_id_;
  len = 8;
  
  // 打包 MIT 控制帧
  uint16_t p_int = floatToUint(p_des_, P_MIN_, P_MAX_, 16);
  uint16_t v_int = floatToUint(v_des_, V_MIN_, V_MAX_, 12);
  uint16_t kp_int = floatToUint(kp_, KP_MIN_, KP_MAX_, 12);
  uint16_t kd_int = floatToUint(kd_, KD_MIN_, KD_MAX_, 12);
  uint16_t t_int = floatToUint(t_ff_, T_MIN_, T_MAX_, 12);
  
  // D[0-1]: p_des[15:8], p_des[7:0]
  data[0] = (p_int >> 8) & 0xFF;
  data[1] = p_int & 0xFF;
  
  // D[2]: v_des[11:4]
  data[2] = (v_int >> 4) & 0xFF;
  
  // D[3]: v_des[3:0] | Kp[11:8]
  data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
  
  // D[4]: Kp[7:0]
  data[4] = kp_int & 0xFF;
  
  // D[5]: Kd[11:4]
  data[5] = (kd_int >> 4) & 0xFF;
  
  // D[6]: Kd[3:0] | t_ff[11:8]
  data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);
  
  // D[7]: t_ff[7:0]
  data[7] = t_int & 0xFF;
}

void DamiaoMotor::enable() {
  // 使能帧: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
  // 通过特殊命令发送，这里暂时不实现
}

void DamiaoMotor::disable() {
  // 失能帧: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
  setMITCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DamiaoMotor::setMITCommand(float pos, float vel, float kp, float kd, float torque) {
  // 限幅
  p_des_ = std::max(P_MIN_, std::min(pos, P_MAX_));
  v_des_ = std::max(V_MIN_, std::min(vel, V_MAX_));
  kp_ = std::max(KP_MIN_, std::min(kp, KP_MAX_));
  kd_ = std::max(KD_MIN_, std::min(kd, KD_MAX_));
  t_ff_ = std::max(T_MIN_, std::min(torque, T_MAX_));
}

uint16_t DamiaoMotor::floatToUint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  
  // 限制范围
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  
  return static_cast<uint16_t>((x - offset) * ((1 << bits) - 1) / span);
}

float DamiaoMotor::uintToFloat(uint16_t x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return static_cast<float>(x_int) * span / ((1 << bits) - 1) + offset;
}

} // namespace motor_control
