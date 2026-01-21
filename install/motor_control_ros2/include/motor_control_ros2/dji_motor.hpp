#ifndef MOTOR_CONTROL_ROS2__DJI_MOTOR_HPP_
#define MOTOR_CONTROL_ROS2__DJI_MOTOR_HPP_

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/cascade_controller.hpp"
#include <cstdint>
#include <cmath>

namespace motor_control {

/**
 * @brief DJI 电机驱动类
 * 
 * 支持 GM3508 和 GM6020 电机。
 * - GM6020: 编码器在输出轴，8192 线/圈，减速比 1:1
 * - GM3508: 编码器在电机轴（减速前），8192 线/圈，减速比 19:1
 */
class DJIMotor : public MotorBase {
public:
  /**
   * @brief 构造函数
   * @param joint_name 关节名称
   * @param type 电机类型 (DJI_GM3508 或 DJI_GM6020)
   * @param motor_id 电机 ID (1-8)
   * @param bus_id CAN 总线 ID
   */
  DJIMotor(const std::string& joint_name, MotorType type, 
           uint8_t motor_id, uint8_t bus_id = 0);
  
  // 实现基类纯虚函数
  void updateFeedback(const std::string& interface_name, uint32_t can_id, const uint8_t* data, size_t len) override;
  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override;
  void enable() override {}   // DJI 电机无需使能命令
  void disable() override;
  
  /**
   * @brief 设置输出值
   * @param value GM3508: 电流值 (-16384~16384), GM6020: 电压值 (-30000~30000)
   */
  void setOutput(int16_t value);
  
  /**
   * @brief 获取反馈 CAN ID
   */
  uint32_t getFeedbackId() const { return feedback_id_; }
  
  /**
   * @brief 获取控制 CAN ID
   */
  uint32_t getControlId() const { return control_id_; }
  
  /**
   * @brief 获取电机 ID
   */
  uint8_t getMotorId() const { return motor_id_; }
  
  /**
   * @brief 获取 CAN 总线 ID
   */
  uint8_t getBusId() const { return bus_id_; }
  
  /**
   * @brief 获取控制字节（用于拼包）
   * @param bytes 输出 2 字节数组
   */
  void getControlBytes(uint8_t bytes[2]) const;
  
  // ========== 串级控制接口 ==========
  
  /**
   * @brief 设置控制模式
   */
  void setControlMode(ControlMode mode);
  
  /**
   * @brief 获取控制模式
   */
  ControlMode getControlMode() const;
  
  /**
   * @brief 设置位置目标（度，0-360）
   */
  void setPositionTarget(double position);
  
  /**
   * @brief 设置速度目标（RPM）
   */
  void setVelocityTarget(double velocity);
  
  /**
   * @brief 设置位置环 PID 参数
   */
  void setPositionPID(const PIDParams& params);
  
  /**
   * @brief 设置速度环 PID 参数
   */
  void setVelocityPID(const PIDParams& params);
  
  /**
   * @brief 更新控制器（在控制循环中调用）
   * 
   * 使用度和 RPM 作为内部单位，与 Python 实现一致。
   */
  void updateController();
  
  /**
   * @brief 获取位置环 PID 参数
   */
  const PIDParams& getPositionPIDParams() const {
    return cascade_controller_.getPositionPIDParams();
  }
  
  /**
   * @brief 获取速度环 PID 参数
   */
  const PIDParams& getVelocityPIDParams() const {
    return cascade_controller_.getVelocityPIDParams();
  }
  
  /**
   * @brief 获取角度（度，0-360）
   * 
   * 统一接口：GM6020 和 GM3508 都返回输出轴角度（0-360°）
   * - GM6020: 编码器直接读取输出轴角度
   * - GM3508: 累积编码器 / 减速比，归一化到 0-360°
   */
  double getAngleDegrees() const {
    // 获取输出轴位置（弧度）
    double output_rad = getOutputPosition();
    
    // 转换为度
    double degrees = output_rad * 180.0 / M_PI;
    
    // 归一化到 [0, 360) 范围
    degrees = fmod(degrees, 360.0);
    if (degrees < 0) degrees += 360.0;
    
    return degrees;
  }
  
  /**
   * @brief 获取转速（RPM）
   * 与 Python 实现一致的单位
   */
  int16_t getRPM() const {
    return raw_rpm_;
  }

private:
  uint8_t motor_id_;           // 电机 ID (1-8)
  uint8_t bus_id_;             // CAN 总线 ID
  uint32_t feedback_id_;       // 反馈 CAN ID
  uint32_t control_id_;        // 控制 CAN ID
  int16_t target_output_;      // 目标输出值
  int16_t max_output_;         // 最大输出限幅
  
  // 原始反馈数据
  uint16_t raw_angle_;         // 0-8191（单圈编码器值）
  int16_t raw_rpm_;
  int16_t raw_current_;
  uint8_t raw_temp_;
  
  // 累积编码器（用于 GM3508，编码器在电机轴）
  uint16_t last_raw_angle_;    // 上一次的编码器值
  int32_t encoder_rounds_;     // 累积圈数（电机轴）
  bool first_feedback_;        // 是否是第一次反馈
  
  // 串级控制
  CascadeController cascade_controller_;  // 串级控制器
  double position_target_;                // 位置目标（弧度）
  double velocity_target_;                // 速度目标（弧度/秒）
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__DJI_MOTOR_HPP_
