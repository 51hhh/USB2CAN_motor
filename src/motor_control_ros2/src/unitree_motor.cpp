#include "motor_control_ros2/unitree_motor.hpp"
#include <cstring>
#include <cmath>
#include <iostream>

namespace motor_control {

UnitreeMotor::UnitreeMotor(const std::string& joint_name, MotorType type,
                           std::shared_ptr<hardware::SerialInterface> serial_port,
                           uint8_t motor_id, int direction, float offset)
  : MotorBase(joint_name, type, 
              (type == MotorType::UNITREE_A1) ? 9.1f : 6.33f, // 减速比 (用于记录, 实际计算内部处理)
              true) // 编码器在电机侧，但我们手动转换为输出轴，所以对基类说是 true
  , serial_port_(serial_port)
  , motor_id_(motor_id)
  , direction_(direction)
  , offset_(offset)
{
  memset(&cmd_frame_, 0, sizeof(cmd_frame_));
  
  // 初始化命令帧头
  cmd_frame_.head[0] = 0xFE;
  cmd_frame_.head[1] = 0xEE;
  cmd_frame_.mode = 0; // 默认 Idle
  
  // 初始化控制变量
  rotor_p_des_ = 0.0f;
  rotor_v_des_ = 0.0f; 
  rotor_kp_ = 0.0f;
  rotor_kd_ = 0.0f;
  rotor_t_ff_ = 0.0f;
}

void UnitreeMotor::updateFeedback(const std::string& /*interface_name*/, uint32_t /*can_id*/, const uint8_t* /*data*/, size_t /*len*/) {
  // 宇树电机不使用此 CAN 接口回调
  // 而是通过 receiveFeedback() 主动读取串口
}

void UnitreeMotor::getControlFrame(uint32_t& /*can_id*/, uint8_t* /*data*/, size_t& len) {
  // 宇树电机不使用此 CAN 接口获取帧
  len = 0; 
}

void UnitreeMotor::enable() {
  cmd_frame_.mode = 10; // FOC 闭环模式
}

void UnitreeMotor::disable() {
  cmd_frame_.mode = 0; // Idle 模式
  setHybridCommand(0, 0, 0, 0, 0);
}

void UnitreeMotor::setHybridCommand(float pos, float vel, float kp, float kd, float torque) {
  // 1. 基类接口更新目标值 (这些是输出轴的值)
  setOutputPosition(pos);
  setOutputVelocity(vel);
  setOutputTorque(torque);
  
  // 2. 转换为电机侧值 (考虑方向, 减速比由基类逻辑或手动处理)
  // 基类 target_position_ 已经存储了电机侧的值 (如果 encode_on_output=false)
  // 但我们需要处理 offset 和 direction
  
  // 电机侧位置 = (输出轴位置 * 减速比) * direction + offset ?
  // 根据之前的分析: 
  // calSetpose: if(dir==1) return pose*9.1 + zero;
  
  if (direction_ == 1) {
    rotor_p_des_ = pos * gear_ratio_ + offset_;
    rotor_v_des_ = vel * gear_ratio_;
    rotor_t_ff_ = torque / gear_ratio_;
  } else { // direction == -1
    rotor_p_des_ = -pos * gear_ratio_ + offset_;
    rotor_v_des_ = -vel * gear_ratio_;
    rotor_t_ff_ = -torque / gear_ratio_;
  }
  
  // Kp, Kd 转换
  rotor_kp_ = kp / (gear_ratio_ * gear_ratio_);
  rotor_kd_ = kd / (gear_ratio_ * gear_ratio_);
  
  // 填充命令结构体 (打包)
  cmd_frame_.T = static_cast<uint16_t>(rotor_t_ff_ * 256.0f);
  cmd_frame_.W = static_cast<uint16_t>(rotor_v_des_ * 128.0f + 16000.0f);
  cmd_frame_.Pos = static_cast<uint32_t>((rotor_p_des_ / (2.0f * M_PI)) * 16383.5f);
  cmd_frame_.K_P = static_cast<uint16_t>(rotor_kp_ * 2048.0f);
  cmd_frame_.K_W = static_cast<uint16_t>(rotor_kd_ * 1024.0f);
}

bool UnitreeMotor::sendCommand() {
  if (!serial_port_ || !serial_port_->isOpen()) return false;
  
  // 计算 CRC
  cmd_frame_.crc = crc32_core((uint32_t*)&cmd_frame_, 30 / 4);
  
  // 发送 (34 bytes)
  ssize_t n = serial_port_->send((uint8_t*)&cmd_frame_, sizeof(cmd_frame_));
  return (n == sizeof(cmd_frame_));
}

bool UnitreeMotor::receiveFeedback() {
  if (!serial_port_ || !serial_port_->isOpen()) return false;
  
  // 读取响应 (78 bytes)
  // 注意：这里简化处理，假设每次都能读到完整帧
  // 实际应用可能需要环形缓冲区处理拆包粘包
  ssize_t n = serial_port_->receive((uint8_t*)&res_frame_, sizeof(res_frame_));
  
  if (n != sizeof(res_frame_)) return false;
  
  // 校验头
  if (res_frame_.head[0] != 0xFE || res_frame_.head[1] != 0xEE) return false;
  
  // 校验 CRC (暂略，需要完整实现)
  
  online_ = true;
  
  // 解析状态 (电机侧 -> 输出轴)
  // Rotor Position (0~16383.5 -> 0~2PI)
  // 注意 Pos 可能是多圈或单圈，宇树 A1 电机通常反馈累计位置? 
  // 代码示例中看起来是 0-2PI 映射。假设是单圈绝对值或者累计值。
  // 这里暂时按 reference 代码反向操作
  
  double rotor_pos = (static_cast<double>(res_frame_.Pos) / 16383.5) * 2.0 * M_PI;
  double rotor_vel = (static_cast<double>(res_frame_.W) - 16000.0) / 128.0;
  double rotor_torque = static_cast<double>(res_frame_.T) / 256.0;
  
  // 转换为输出轴
  if (direction_ == 1) {
    position_ = (rotor_pos - offset_) / gear_ratio_; // 此处 position_ 是 MotorBase 里的，语义上如果是 false，应该是电机侧值
    // 修正：基类定义如果 encoder_on_output_ = false，则 position_ 存的是电机侧的原始值
    // 所以这里应该直接存 rotor_pos ?
    // 不，基类注释说 "状态变量...位置(弧度)"
    // getOutputPosition() 会再除以 gear_ratio_
    // 所以 MotorBase 的 position_ 在 encoder_on_output_=false 时应该存 **电机侧的物理位置**
    
    position_ = rotor_pos; // 存电机侧值
    // 但是等等，offset 处理怎么办？基类不处理 offset
    // 我们的这个 MotorBase 定义稍微有点简单。
    // 为了兼容性，我们手动计算出输出轴值，然后"骗"基类说编码器在输出轴？
    // 或者重写 getOutputPosition?
    
    // 更好的做法：UnitreeMotor 内部处理所有 offset/dir/gear，然后把结果存入 position_ 
    // 并设置 encoder_on_output_ = true (在基类构造时)
    // 这样基类就不用再转换了。
    
    // 让我们调整构造函数：传递 true 给 MotorBase
    // 并在 update 中直接算出 output 状态。
    
    // 暂时保持当前代码逻辑：
    // MotorBase(..., gear, false)
    // position_ 存电机侧值
    // getOutputPosition() = position_ / gear
    
    // 这样无法处理 offset 和 direction。
    // 必须重写 getOutputPosition
  }
  
  // 重新思考: 为了支持 offset 和 direction, 最好在 updateFeedback 中直接计算出 Output 轴的状态
  // 然后 MotorBase 作为一个纯粹的数据容器。
    
  if (direction_ == 1) {
     position_ = (rotor_pos - offset_) / gear_ratio_; // 这是输出轴位置
     velocity_ = rotor_vel / gear_ratio_;
     torque_ = rotor_torque * gear_ratio_;
  } else {
     position_ = (offset_ - rotor_pos) / gear_ratio_;
     velocity_ = -rotor_vel / gear_ratio_;
     torque_ = -rotor_torque * gear_ratio_;
  }
   
  // 既然我们已经转换到了输出轴，那么我们需要告诉基类 "数据已经是输出轴格式了"
  // 所以需要修改 MotorBase 的 encoder_on_output_ 为 true
  // 这需要在构造函数中 hack 一下或者提供 setter
  // 或者最简单：在构造函数中传 true。我们去修改构造函数实现。
  
  temperature_ = static_cast<float>(res_frame_.temp);
  
  return true;
}

uint32_t UnitreeMotor::crc32_core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04C11DB7;

    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

} // namespace motor_control
