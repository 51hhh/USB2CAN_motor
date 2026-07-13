#ifndef MOTOR_CONTROL_ROS2__PID_CONTROLLER_HPP_
#define MOTOR_CONTROL_ROS2__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace motor_control {

/**
 * @brief PID 控制器参数结构
 */
struct PIDParams {
  double kp = 0.0;        // 比例系数
  double ki = 0.0;        // 积分系数
  double kd = 0.0;        // 微分系数
  double i_max = 0.0;     // 积分限幅
  double out_max = 0.0;   // 输出限幅
  double dead_zone = 0.0; // 死区阈值
};

/**
 * @brief 通用 PID 控制器
 * 
 * 特性:
 * - 支持 P/I/D 参数独立配置
 * - 积分限幅防止积分饱和
 * - 输出限幅保证安全
 * - 死区处理避免微小震荡
 * - 状态重置功能
 */
class PIDController {
public:
  /**
   * @brief 构造函数
   */
  PIDController() 
    : error_{0.0, 0.0}
    , i_out_(0.0)
  {}
  
  /**
   * @brief 设置 PID 参数
   */
  void setParams(const PIDParams& params) {
    params_ = params;
  }
  
  /**
   * @brief 获取 PID 参数
   */
  const PIDParams& getParams() const {
    return params_;
  }
  
  /**
   * @brief 计算 PID 输出
   * 
   * 与 Python 实现一致，不使用 dt 参数。
   * 假设固定控制频率，I/D 项直接累加/差分。
   * 
   * @param target 目标值
   * @param feedback 反馈值
   * @return PID 输出
   */
  double calculate(double target, double feedback) {
    // 1. 计算原始误差
    double raw_error = target - feedback;
    
    // 2. 死区处理
    if (std::abs(raw_error) < params_.dead_zone) {
      error_[0] = 0.0;
    } else {
      error_[0] = raw_error;
    }
    
    // 3. 计算 P 项
    double p_out = params_.kp * error_[0];
    
    // 4. 计算 I 项（与 Python 一致，无 dt）
    i_out_ += params_.ki * error_[0];
    
    // 积分限幅
    i_out_ = std::clamp(i_out_, -params_.i_max, params_.i_max);
    
    // 5. 计算 D 项（与 Python 一致，无 dt）
    double d_out = params_.kd * (error_[0] - error_[1]);
    
    // 6. 保存误差历史
    error_[1] = error_[0];
    
    // 7. 总输出
    double output = p_out + i_out_ + d_out;
    
    // 8. 输出限幅
    output = std::clamp(output, -params_.out_max, params_.out_max);
    
    return output;
  }
  
  /**
   * @brief 重置 PID 状态
   * 
   * 清除误差历史和积分累积，用于模式切换或目标突变时。
   */
  void reset() {
    error_[0] = 0.0;
    error_[1] = 0.0;
    i_out_ = 0.0;
  }
  
  /**
   * @brief 获取当前误差
   */
  double getError() const {
    return error_[0];
  }
  
  /**
   * @brief 获取积分项输出
   */
  double getIntegralOutput() const {
    return i_out_;
  }

private:
  PIDParams params_;    // PID 参数
  double error_[2];     // 误差历史 [当前, 上次]
  double i_out_;        // 积分累积
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__PID_CONTROLLER_HPP_
