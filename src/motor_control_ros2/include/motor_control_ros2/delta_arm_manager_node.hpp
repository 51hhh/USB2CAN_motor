#ifndef MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
#define MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "motor_control_ros2/msg/arm_target.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <array>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

/**
 * @brief Delta 机械臂管理器节点
 *
 * 控制 3 路 GO8010 电机，实现以下状态机：
 *   INIT        → 节点启动，开始软着陆流程
 *   SOFT_LANDING → 施加向下力矩，等待三电机绝对角均 < threshold 持续 stable_duration
 *   READY       → 发布 /delta_arm/ready "READY"，等待 ArmTarget 命令
 *   EXECUTE     → 梯形速度规划跟踪目标角度，到达后返回 READY
 */
class DeltaArmManager : public rclcpp::Node {
public:
  DeltaArmManager();

private:
  // ========== 状态机 ==========
  enum class State {
    INIT,
    SOFT_LANDING,
    READY,
    EXECUTE,
    DESCENDING
  };

  void controlLoop();

  // ========== 回调 ==========
  void armTargetCallback(const motor_control_ros2::msg::ArmTarget::SharedPtr msg);
  void motorStateCallback(const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);

  // ========== 辅助函数 ==========
  void loadConfig(const std::string& config_file);
  void publishCommand(size_t idx,
                      double pos_des, double vel_des, double torque_ff,
                      double kp, double kd);
  bool allMotorsLanded() const;
  bool allMotorsReached() const;
  void publishReady();

  // ========== ROS2 通信 ==========
  rclcpp::Subscription<motor_control_ros2::msg::ArmTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr state_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ========== 配置参数 ==========
  double downward_torque_;           // 软着陆力矩 (Nm)
  double landing_timeout_;           // 软着陆超时 (s)
  double landing_velocity_threshold_; // 着陆速度阈值 (rad/s)
  double landing_stable_duration_;   // 稳定持续时间 (s)
  double landing_kd_;                // 软着陆阻尼系数
  double kp_;
  double kd_;
  double max_velocity_;              // 梯形速度限幅 (rad/s)
  double control_frequency_;         // 控制频率 (Hz)
  double position_tolerance_;        // 到达判定容差 (rad)
  double max_position_error_;        // 位置误差钳位 (rad)
  double tracking_error_pause_;      // 自适应规划暂停阈值 (rad)
  std::array<std::string, 3> motor_names_;
  std::array<std::string, 3> motor_devices_;
  std::array<uint8_t, 3> motor_ids_;

  // ========== 运行时状态 ==========
  State state_;
  std::array<double, 3> current_positions_;    // rad
  std::array<double, 3> current_velocities_;   // rad/s
  std::array<bool, 3> motors_online_;
  std::array<bool, 3> has_feedback_;
  std::array<double, 3> last_positions_;       // 用于软着陆变化量判定
  std::array<double, 3> zero_positions_;       // 解耦零点（软着陆完成时锁定）

  double target_delta_rad_;                    // EXECUTE 目标增量（相对零点）
  double planned_delta_rad_;                   // 梯形规划增量（相对零点）
  double current_planned_vel_;                 // 规划器当前速度（限加速度用）
  double max_acceleration_;                    // 最大加速度 (rad/s²)
  double planner_p_gain_;                      // 减速段比例增益

  rclcpp::Time landing_stable_since_;
  rclcpp::Time init_start_time_;
  rclcpp::Time execute_start_time_;  // EXECUTE 进入时间，用于超时判定
  bool landing_stability_started_;
  bool ready_published_;

  double gravity_compensation_torque_;  // 重力补偿前馈力矩（用户自行调试）
  double top_idle_timeout_;             // EXECUTE 超时时间（s），超时强制进入 DESCENDING
};

#endif  // MOTOR_CONTROL_ROS2__DELTA_ARM_MANAGER_NODE_HPP_
