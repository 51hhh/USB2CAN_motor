#ifndef MOTOR_CONTROL_ROS2__CASCADE_CONTROLLER_HPP_
#define MOTOR_CONTROL_ROS2__CASCADE_CONTROLLER_HPP_

#include "motor_control_ros2/pid_controller.hpp"

namespace motor_control {

/**
 * @brief 控制模式枚举
 */
enum class ControlMode {
  DIRECT = 0,    // 直接输出（电流/电压）
  VELOCITY = 1,  // 速度控制
  POSITION = 2   // 位置控制
};

/**
 * @brief 多环串级控制器
 * 
 * 控制链路:
 * - 位置模式: 位置目标 → [位置PID] → 速度目标 → [速度PID] → 电流/电压输出
 * - 速度模式: 速度目标 → [速度PID] → 电流/电压输出
 * - 直接模式: 直接输出电流/电压
 */
class CascadeController {
public:
  /**
   * @brief 构造函数
   */
  CascadeController() 
    : mode_(ControlMode::DIRECT)
  {}
  
  /**
   * @brief 设置控制模式
   */
  void setMode(ControlMode mode) {
    if (mode_ != mode) {
      // 模式切换时重置所有 PID 状态
      position_pid_.reset();
      velocity_pid_.reset();
      mode_ = mode;
    }
  }
  
  /**
   * @brief 获取当前控制模式
   */
  ControlMode getMode() const {
    return mode_;
  }
  
  /**
   * @brief 设置位置环 PID 参数
   */
  void setPositionPID(const PIDParams& params) {
    position_pid_.setParams(params);
  }
  
  /**
   * @brief 设置速度环 PID 参数
   */
  void setVelocityPID(const PIDParams& params) {
    velocity_pid_.setParams(params);
  }
  
  /**
   * @brief 获取位置环 PID 参数
   */
  const PIDParams& getPositionPIDParams() const {
    return position_pid_.getParams();
  }
  
  /**
   * @brief 获取速度环 PID 参数
   */
  const PIDParams& getVelocityPIDParams() const {
    return velocity_pid_.getParams();
  }
  
  /**
   * @brief 更新控制器并计算输出
   * 
   * 与 Python 实现一致，使用度和 RPM 作为单位。
   * 
   * @param position_target 位置目标（度）
   * @param velocity_target 速度目标（RPM）
   * @param direct_output 直接输出值（电流/电压）
   * @param position_feedback 位置反馈（度）
   * @param velocity_feedback 速度反馈（RPM）
   * @return 输出值（电流/电压）
   */
  double update(double position_target, 
                double velocity_target, 
                double direct_output,
                double position_feedback, 
                double velocity_feedback) {
    switch (mode_) {
      case ControlMode::POSITION: {
        // 位置控制模式：位置环 → 速度环
        // 处理角度过零问题：选择最短路径
        double angle_error = position_target - position_feedback;
        
        // 归一化到 [-180, 180] 度，选择最短路径
        while (angle_error > 180.0) angle_error -= 360.0;
        while (angle_error < -180.0) angle_error += 360.0;
        
        // 使用归一化后的目标进行 PID 计算
        // 目标 = 当前位置 + 最短路径误差
        double normalized_target = position_feedback + angle_error;
        
        // 位置环输出作为速度环的目标 (RPM)
        double vel_target_from_pos = position_pid_.calculate(
          normalized_target, position_feedback);
        
        // 速度环输出作为最终输出 (电压)
        return velocity_pid_.calculate(
          vel_target_from_pos, velocity_feedback);
      }
      
      case ControlMode::VELOCITY: {
        // 速度控制模式：速度环
        return velocity_pid_.calculate(
          velocity_target, velocity_feedback);
      }
      
      case ControlMode::DIRECT:
      default: {
        // 直接输出模式：不经过 PID
        return direct_output;
      }
    }
  }
  
  /**
   * @brief 重置所有 PID 状态
   */
  void reset() {
    position_pid_.reset();
    velocity_pid_.reset();
  }
  
  /**
   * @brief 获取位置环误差
   */
  double getPositionError() const {
    return position_pid_.getError();
  }
  
  /**
   * @brief 获取速度环误差
   */
  double getVelocityError() const {
    return velocity_pid_.getError();
  }

private:
  ControlMode mode_;              // 当前控制模式
  PIDController position_pid_;    // 位置环 PID
  PIDController velocity_pid_;    // 速度环 PID
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__CASCADE_CONTROLLER_HPP_
