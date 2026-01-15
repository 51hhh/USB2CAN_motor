#ifndef MOTOR_CONTROL_ROS2__MOTOR_BASE_HPP_
#define MOTOR_CONTROL_ROS2__MOTOR_BASE_HPP_

#include <string>
#include <memory>
#include <cmath>

namespace motor_control {

/**
 * @brief 电机类型枚举
 */
enum class MotorType {
  DJI_GM3508,
  DJI_GM6020,
  DAMIAO_DM4310,
  DAMIAO_DM4340,
  UNITREE_A1,
  UNITREE_GO8010
};

/**
 * @brief 电机基类
 * 
 * 定义所有电机的统一接口，子类必须实现纯虚函数。
 * 提供编码器位置转换的通用逻辑。
 */
class MotorBase {
public:
  /**
   * @brief 构造函数
   * @param joint_name 关节名称
   * @param type 电机类型
   * @param gear_ratio 减速比
   * @param encoder_on_output 编码器是否在输出轴
   */
  MotorBase(const std::string& joint_name, MotorType type, 
            float gear_ratio = 1.0f, bool encoder_on_output = true)
    : joint_name_(joint_name)
    , motor_type_(type)
    , gear_ratio_(gear_ratio)
    , encoder_on_output_(encoder_on_output)
    , online_(false)
    , position_(0.0)
    , velocity_(0.0)
    , torque_(0.0)
    , temperature_(0.0f)
  {}
  
  virtual ~MotorBase() = default;
  
  // ========== 纯虚函数 - 子类必须实现 ==========
  
  /**
   * @brief 更新电机反馈数据
   * @param can_id CAN 帧 ID
   * @param data 数据指针
   * @param len 数据长度
   */
  virtual void updateFeedback(uint32_t can_id, const uint8_t* data, size_t len) = 0;
  
  /**
   * @brief 获取控制帧数据
   * @param can_id 输出 CAN 帧 ID
   * @param data 输出数据缓冲区
   * @param len 输出数据长度
   */
  virtual void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) = 0;
  
  /**
   * @brief 使能电机
   */
  virtual void enable() = 0;
  
  /**
   * @brief 失能电机
   */
  virtual void disable() = 0;
  
  // ========== 通用接口 ==========
  
  /**
   * @brief 获取关节名称
   */
  std::string getJointName() const { return joint_name_; }
  
  /**
   * @brief 获取电机类型
   */
  MotorType getMotorType() const { return motor_type_; }
  
  /**
   * @brief 获取在线状态
   */
  bool isOnline() const { return online_; }
  
  /**
   * @brief 检查心跳超时并更新在线状态
   * @param timeout_ms 超时时间（毫秒）
   * @param current_time_ns 当前时间（纳秒）
   */
  void checkHeartbeat(double timeout_ms, int64_t current_time_ns) {
    if (last_feedback_time_ns_ == 0) {
      // 从未收到反馈
      online_ = false;
      return;
    }
    
    double dt_ms = (current_time_ns - last_feedback_time_ns_) / 1e6;
    if (dt_ms > timeout_ms) {
      online_ = false;
    }
  }
  
  /**
   * @brief 更新最后反馈时间（在 updateFeedback 中调用）
   */
  void updateLastFeedbackTime(int64_t current_time_ns) {
    last_feedback_time_ns_ = current_time_ns;
    online_ = true;
  }
  
  /**
   * @brief 获取输出轴位置（弧度）
   * 
   * 如果编码器在输出轴，直接返回；
   * 如果编码器在电机侧，除以减速比转换。
   */
  double getOutputPosition() const {
    if (encoder_on_output_) {
      return position_;
    } else {
      return position_ / gear_ratio_;
    }
  }
  
  /**
   * @brief 获取输出轴速度（弧度/秒）
   */
  double getOutputVelocity() const {
    if (encoder_on_output_) {
      return velocity_;
    } else {
      return velocity_ / gear_ratio_;
    }
  }
  
  /**
   * @brief 获取输出轴力矩（Nm）
   */
  double getOutputTorque() const {
    if (encoder_on_output_) {
      return torque_;
    } else {
      return torque_ * gear_ratio_;  // 电机侧力矩 * 减速比 = 输出轴力矩
    }
  }
  
  /**
   * @brief 获取温度
   */
  float getTemperature() const { return temperature_; }
  
  /**
   * @brief 设置输出轴目标位置（弧度）
   * 
   * 如果编码器在输出轴，直接设置；
   * 如果编码器在电机侧，乘以减速比转换。
   */
  void setOutputPosition(double output_pos) {
    if (encoder_on_output_) {
      target_position_ = output_pos;
    } else {
      target_position_ = output_pos * gear_ratio_;
    }
  }
  
  /**
   * @brief 设置输出轴目标速度（弧度/秒）
   */
  void setOutputVelocity(double output_vel) {
    if (encoder_on_output_) {
      target_velocity_ = output_vel;
    } else {
      target_velocity_ = output_vel * gear_ratio_;
    }
  }
  
  /**
   * @brief 设置输出轴目标力矩（Nm）
   */
  void setOutputTorque(double output_torque) {
    if (encoder_on_output_) {
      target_torque_ = output_torque;
    } else {
      target_torque_ = output_torque / gear_ratio_;
    }
  }

protected:
  // 基本信息
  std::string joint_name_;
  MotorType motor_type_;
  float gear_ratio_;           // 减速比
  bool encoder_on_output_;     // 编码器是否在输出轴
  bool online_;                // 在线状态
  
  // 状态变量（编码器原始值或已转换值）
  double position_;            // 位置（弧度）
  double velocity_;            // 速度（弧度/秒）
  double torque_;              // 力矩（Nm）
  float temperature_;          // 温度（摄氏度）
  
  // 目标值（编码器原始值或已转换值）
  double target_position_;
  double target_velocity_;
  double target_torque_;
  
  // 心跳检测
  int64_t last_feedback_time_ns_ = 0;  // 最后一次收到反馈的时间（纳秒）
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__MOTOR_BASE_HPP_
