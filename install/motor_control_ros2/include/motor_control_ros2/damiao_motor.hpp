#ifndef MOTOR_CONTROL_ROS2__DAMIAO_MOTOR_HPP_
#define MOTOR_CONTROL_ROS2__DAMIAO_MOTOR_HPP_

#include "motor_control_ros2/motor_base.hpp"
#include <cstdint>

namespace motor_control {

/**
 * @brief 达妙电机驱动类
 * 
 * 支持 DM4310, DM4340 等型号。
 * 使用 MIT 模式控制，编码器在输出轴（双编码器系统）。
 */
class DamiaoMotor : public MotorBase {
public:
  /**
   * @brief 构造函数
   * @param joint_name 关节名称
   * @param type 电机类型
   * @param can_id CAN ID (接收命令)
   * @param master_id Master ID (发送反馈)
   * @param bus_id CAN 总线 ID
   */
  DamiaoMotor(const std::string& joint_name, MotorType type,
              uint8_t can_id, uint8_t master_id, uint8_t bus_id = 0);
  
  // 实现基类纯虚函数
  void updateFeedback(uint32_t can_id, const uint8_t* data, size_t len) override;
  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override;
  void enable() override;
  void disable() override;
  
  /**
   * @brief 设置 MIT 模式控制命令
   * @param pos 期望位置（弧度）
   * @param vel 期望速度（弧度/秒）
   * @param kp 位置增益
   * @param kd 速度增益
   * @param torque 前馈力矩（Nm）
   */
  void setMITCommand(float pos, float vel, float kp, float kd, float torque);
  
  /**
   * @brief 获取错误码
   */
  uint8_t getErrorCode() const { return error_code_; }
  
  /**
   * @brief 获取 MOS 温度
   */
  uint8_t getTempMOS() const { return temp_mos_; }
  
  /**
   * @brief 获取转子温度
   */
  uint8_t getTempRotor() const { return temp_rotor_; }
  
  /**
   * @brief 获取 CAN ID
   */
  uint8_t getCANId() const { return can_id_; }
  
  /**
   * @brief 获取 Master ID
   */
  uint8_t getMasterId() const { return master_id_; }

private:
  uint8_t can_id_;             // CAN ID (接收命令)
  uint8_t master_id_;          // Master ID (发送反馈)
  uint8_t bus_id_;             // CAN 总线 ID
  
  // MIT 模式参数
  float p_des_;
  float v_des_;
  float kp_;
  float kd_;
  float t_ff_;
  
  // 电机参数范围（DM4340）
  float P_MIN_;
  float P_MAX_;
  float V_MIN_;
  float V_MAX_;
  float T_MIN_;
  float T_MAX_;
  float KP_MIN_;
  float KP_MAX_;
  float KD_MIN_;
  float KD_MAX_;
  
  // 状态
  uint8_t error_code_;
  uint8_t temp_mos_;
  uint8_t temp_rotor_;
  
  // 工具函数
  uint16_t floatToUint(float x, float x_min, float x_max, int bits);
  float uintToFloat(uint16_t x_int, float x_min, float x_max, int bits);
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__DAMIAO_MOTOR_HPP_
