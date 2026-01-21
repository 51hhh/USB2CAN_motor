#include "motor_control_ros2/dji_motor.hpp"
#include <cstring>
#include <iostream>
#include <iomanip>
#include <chrono>

namespace motor_control {

DJIMotor::DJIMotor(const std::string& joint_name, MotorType type, 
                   uint8_t motor_id, uint8_t bus_id)
  : MotorBase(joint_name, type, 
              (type == MotorType::DJI_GM3508) ? 19.0f : 1.0f,  // 减速比
              (type == MotorType::DJI_GM6020))  // GM6020 编码器在输出轴，GM3508 编码器在电机轴
  , motor_id_(motor_id)
  , bus_id_(bus_id)
  , target_output_(0)
  , raw_angle_(0)
  , raw_rpm_(0)
  , raw_current_(0)
  , raw_temp_(0)
  , last_raw_angle_(0)
  , encoder_rounds_(0)
  , first_feedback_(true)
  , position_target_(0.0)
  , velocity_target_(0.0)
{
  if (type == MotorType::DJI_GM6020) {
    // GM6020 配置
    feedback_id_ = 0x205 + (motor_id - 1);
    control_id_ = (motor_id <= 4) ? 0x1FF : 0x2FF;
    max_output_ = 30000;  // 电压限幅
  } else if (type == MotorType::DJI_GM3508) {
    // GM3508 配置
    feedback_id_ = 0x201 + (motor_id - 1);
    control_id_ = (motor_id <= 4) ? 0x200 : 0x1FF;
    max_output_ = 16384;  // 电流限幅
  }
  
  // 默认为直接输出模式
  cascade_controller_.setMode(ControlMode::DIRECT);
}

void DJIMotor::updateFeedback(const std::string& interface_name, uint32_t can_id, const uint8_t* data, size_t len) {
  // 检查接口名称匹配（多路 CAN 支持）
  if (!interface_name_.empty() && interface_name != interface_name_) {
    return;  // 不是该电机所属的接口
  }
  
  if (can_id != feedback_id_ || len < 8) {
    return;
  }
  
  // 更新心跳时间（会自动设置 online_ = true）
  updateLastFeedbackTime(std::chrono::steady_clock::now().time_since_epoch().count());
  
  // 解析 DJI 反馈帧 (Big Endian)
  raw_angle_ = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  raw_rpm_ = (static_cast<int16_t>(data[2]) << 8) | data[3];
  raw_current_ = (static_cast<int16_t>(data[4]) << 8) | data[5];
  raw_temp_ = data[6];
  
  // 累积编码器逻辑（用于 GM3508，编码器在电机轴）
  if (!encoder_on_output_) {
    // GM3508: 编码器在电机轴，需要累积圈数
    if (!first_feedback_) {
      // 检测过零
      int32_t delta = static_cast<int32_t>(raw_angle_) - static_cast<int32_t>(last_raw_angle_);
      
      // 过零检测：如果变化超过半圈（4096），说明发生了过零
      if (delta > 4096) {
        // 从 8191 跳到 0（逆时针过零）
        encoder_rounds_--;
      } else if (delta < -4096) {
        // 从 0 跳到 8191（顺时针过零）
        encoder_rounds_++;
      }
    }
    
    last_raw_angle_ = raw_angle_;
    first_feedback_ = false;
    
    // 计算累积位置（电机轴）= 累积圈数 × 2π + 当前圈内位置
    double rounds_position = encoder_rounds_ * 2.0 * M_PI;
    double current_round_position = (raw_angle_ / 8191.0) * 2.0 * M_PI;
    position_ = rounds_position + current_round_position;
    
  } else {
    // GM6020: 编码器在输出轴，直接转换
    position_ = (raw_angle_ / 8191.0) * 2.0 * M_PI;
  }
  
  // 速度转换（两种电机都一样，RPM 是电机轴速度）
  velocity_ = (raw_rpm_ / 60.0) * 2.0 * M_PI;
  torque_ = raw_current_;
  temperature_ = static_cast<float>(raw_temp_);
  
  // 调试日志（仅在首次上线或定期输出）
  static int feedback_count = 0;
  if (feedback_count++ % 100 == 0) {  // 每100次输出一次
    std::cout << "[DJI " << joint_name_ << "] "
              << "Angle=" << std::fixed << std::setprecision(1) << getAngleDegrees() << "° "
              << "RPM=" << raw_rpm_;
    
    if (!encoder_on_output_) {
      // GM3508: 显示累积圈数
      std::cout << " Rounds=" << encoder_rounds_;
    }
    
    std::cout << " Temp=" << static_cast<int>(raw_temp_) << "°C" << std::endl;
  }
}

void DJIMotor::getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) {
  // DJI 电机使用共享帧，不单独发送
  // 返回 len = 0 表示需要拼包
  (void)data; // 消除未使用参数警告
  can_id = control_id_;
  len = 0;
}

void DJIMotor::disable() {
  setOutput(0);
}

void DJIMotor::setOutput(int16_t value) {
  // 限幅
  if (value > max_output_) value = max_output_;
  if (value < -max_output_) value = -max_output_;
  target_output_ = value;
}

void DJIMotor::getControlBytes(uint8_t bytes[2]) const {
  // Big Endian
  bytes[0] = (target_output_ >> 8) & 0xFF;
  bytes[1] = target_output_ & 0xFF;
}

// ========== 串级控制实现 ==========

void DJIMotor::setControlMode(ControlMode mode) {
  cascade_controller_.setMode(mode);
}

ControlMode DJIMotor::getControlMode() const {
  return cascade_controller_.getMode();
}

void DJIMotor::setPositionTarget(double position) {
  position_target_ = position;
}

void DJIMotor::setVelocityTarget(double velocity) {
  velocity_target_ = velocity;
}

void DJIMotor::setPositionPID(const PIDParams& params) {
  cascade_controller_.setPositionPID(params);
}

void DJIMotor::setVelocityPID(const PIDParams& params) {
  cascade_controller_.setVelocityPID(params);
}

void DJIMotor::updateController() {
  // 使用度和 RPM 作为单位，与 Python 实现一致
  double output = cascade_controller_.update(
    position_target_,                    // 位置目标 (度)
    velocity_target_,                    // 速度目标 (RPM)
    static_cast<double>(target_output_), // 直接输出
    getAngleDegrees(),                   // 位置反馈 (度, 0-360)
    static_cast<double>(raw_rpm_)        // 速度反馈 (RPM)
  );
  
  // 将输出转换为 int16 并设置
  setOutput(static_cast<int16_t>(output));
}

} // namespace motor_control
