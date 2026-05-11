#ifndef MOTOR_CONTROL_ROS2__SERVE_WINDMILL_MANAGER_HPP_
#define MOTOR_CONTROL_ROS2__SERVE_WINDMILL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "motor_control_ros2/msg/dji_motor_state.hpp"
#include "motor_control_ros2/msg/dji_motor_command.hpp"

#include <string>
#include <cmath>
#include <algorithm>

namespace serve_windmill {

// =============================================================
// 发球状态机
// =============================================================
enum class ServeState {
  IDLE,             // 空闲：零电流，等待命令
  GRAVITY_HOMING,   // 重力归零：释放电机，等臂自然下垂稳定，记录零位
  WIND_UP,          // 引拍：PD 控制到 -wind_up_angle°，等待扳机
  FIRE,             // 开火：最大电流推，直到越过 fire_end_angle°
  FREE_WHEEL,       // 惯性滑行：零电流，等到越过 catch_start_angle° 或速度为零
  CATCH,            // 接住：PD 制动到 360°，然后更新零位偏移，循环回 WIND_UP
  E_STOP            // 急停：全零电流
};

inline const char* serveStateToString(ServeState s) {
  switch (s) {
    case ServeState::IDLE:           return "IDLE";
    case ServeState::GRAVITY_HOMING: return "GRAVITY_HOMING";
    case ServeState::WIND_UP:        return "WIND_UP";
    case ServeState::FIRE:           return "FIRE";
    case ServeState::FREE_WHEEL:     return "FREE_WHEEL";
    case ServeState::CATCH:          return "CATCH";
    case ServeState::E_STOP:         return "E_STOP";
    default:                         return "UNKNOWN";
  }
}

// =============================================================
// ServeWindmillManager 节点
// =============================================================
class ServeWindmillManager : public rclcpp::Node {
public:
  ServeWindmillManager();

private:
  // ==================== 控制循环 ====================
  void controlLoop();

  // ==================== 状态机处理 ====================
  void handleIdle();
  void handleGravityHoming();
  void handleWindUp();
  void handleFire();
  void handleFreeWheel();
  void handleCatch();
  void handleEStop();

  // ==================== 回调 ====================
  void motorStateCallback(
      const motor_control_ros2::msg::DJIMotorState::SharedPtr msg);
  void triggerCallback(
      const std_msgs::msg::Bool::SharedPtr msg);
  void commandCallback(
      const std_msgs::msg::String::SharedPtr msg);

  // ==================== 工具函数 ====================
  void requestExecute();
  void sendCurrent(int16_t current_m2);
  void transitionTo(ServeState new_state);
  void publishStatus() const;

  /**
   * @brief 内联 PD 位置控制
   *
   * 利用 DJIMotorState 发布的连续累积角度，无需单独 PID 类。
   * error = target_deg - feedback_angle_deg_
   * output = kp * error - kd * feedback_rpm_  (rpm 为电机轴，含 19:1 减速比因子)
   *
   * @return 电流指令 (raw, 对 M2 正方向)
   */
  int16_t computePD(double target_deg, double kp, double kd, int16_t max_current) const;

  // ==================== ROS2 通信 ====================
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ==================== 电机配置 ====================
  std::string motor_name_m1_;    // 主电机名 (DJI3508_1)
  std::string motor_name_m2_;    // 从电机名/编码器源 (DJI3508_2)
  int motor_direction_;           // 输出方向乘子 (+1 或 -1)，同时翻转反馈与输出

  // ==================== 反馈状态 (来自 M2) ====================
  double feedback_angle_deg_;    // M2 输出轴连续角度 (度)
  double feedback_rpm_;          // M2 电机轴 RPM
  int16_t feedback_current_;     // M2 实际电流
  bool feedback_online_;         // M2 在线
  bool has_feedback_;            // 是否收到过反馈

  // ==================== 状态机 ====================
  ServeState state_;
  rclcpp::Time state_enter_time_;

  // ==================== 飞车安全保护 ====================
  double state_entry_angle_deg_; // 进入当前状态时的角度 (用于 runaway 检测)

  // ==================== 重力归零 ====================
  double home_offset_deg_;       // 重力下垂位置（度），作为每一轮的 0°
  double homing_last_angle_;     // 上一次采样角度
  rclcpp::Time homing_stable_since_;
  bool homing_stability_started_;

  // ==================== 飞车安全阈值 ====================
  double runaway_threshold_deg_; // 非 FIRE/FREE_WHEEL 状态下最大允许角位移 (默认 720°=2圈)

  // ==================== 循环计数 ====================
  int serve_count_;              // 已完成的发球次数

  // ==================== 参数 (ROS2 参数服务器注入) ====================

  // -- 角度阈值（度，相对于 home_offset） --
  double wind_up_angle_deg_;          // 引拍角度 (正值, 实际目标 = home - wind_up)
  double fire_end_angle_deg_;         // 开火结束角 (正值, 实际目标 = home + fire_end)
  double catch_start_angle_deg_;      // 接住开始角 (正值, 实际目标 = home + catch_start)
  double catch_target_angle_deg_;     // 接住目标角 (360°, 一整圈)

  // -- 电流 --
  int16_t max_fire_current_;          // FIRE 阶段最大电流 (raw, 0~16384)

  // -- PD 增益 --
  //    WIND_UP & CATCH 各自可有独立增益
  double wind_up_kp_;                 // 引拍 Kp (raw_current / deg)
  double wind_up_kd_;                 // 引拍 Kd (raw_current / output_rpm)
  double catch_kp_;                   // 接住 Kp
  double catch_kd_;                   // 接住 Kd
  int16_t wind_up_max_current_;       // 引拍电流限幅
  int16_t catch_max_current_;         // 接住电流限幅
  int16_t gravity_ff_current_;        // 重力前馈补偿电流 (抵消恒定负载)

  // -- 归零参数 --
  double homing_vel_threshold_rpm_;   // 电机轴 RPM 绝对值低于此判定静止
  double homing_stable_duration_s_;   // 持续静止时长 (s)

  // -- 风车参数 --
  double free_wheel_vel_threshold_rpm_;  // FREE_WHEEL 速度过低退出阈值 (电机轴 RPM)

  // -- 安全 --
  int16_t global_max_current_;        // 全局电流限幅

  // -- 控制频率 --
  double control_frequency_;          // Hz

  // -- 到位容差 --
  double position_tolerance_deg_;     // PD 到位容差 (度)
};

}  // namespace serve_windmill

#endif  // MOTOR_CONTROL_ROS2__SERVE_WINDMILL_MANAGER_HPP_
