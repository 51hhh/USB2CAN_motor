#ifndef MOTOR_CONTROL_ROS2__VOLLEYBALL_STRIKE_MANAGER_HPP_
#define MOTOR_CONTROL_ROS2__VOLLEYBALL_STRIKE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "motor_control_ros2/msg/strike_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <array>
#include <string>
#include <cmath>
#include <algorithm>

// =============================================================
// 数学常量
// =============================================================
namespace strike_math {
  constexpr double PI = 3.14159265358979323846;
}

// =============================================================
// 几何参数（从 YAML 加载）
// =============================================================
struct ArmGeometry {
  double arm_spacing;          // D:  左右肩关节间距 (m)  — 350.6mm
  double upper_arm_length;     // L1: 大臂长度，肩关节→肘关节 (m)  — 200mm
  double forearm_length;       // L2: 小臂长度，肘关节→拍面铰接点 (m)  — 224mm
  double plate_width;          // W:  拍面两侧铰接点间距 (m)  — 350.6mm
  // --- 平行四边形传动参数 (14-200-14-200) ---
  // L2电机外壳固定在大臂根部，通过14mm摇臂-200mm连杆-14mm从动臂传动
  // 精确平行四边形 → q_motor = θ_forearm_relative（小臂相对大臂折叠角）
  // 大臂电机直驱 → q_motor = θ_upper_absolute（大臂绝对角）
  double crank_length;         // 摇臂/从动臂长度 (m) — 14mm（仅用于文档记录，运动学不需要）
};

// =============================================================
// IK 求解结果：4 电机关节角
// =============================================================
struct IKResult {
  double q_L1;   // 左大臂电机角 (rad) — 直驱 = 大臂绝对角 θ1
  double q_L2;   // 左小臂电机角 (rad) — 平行四边形直通 = 肘关节折叠角 q2_rel
  double q_R1;   // 右大臂电机角 (rad) — 直驱 = 大臂绝对角 θ1
  double q_R2;   // 右小臂电机角 (rad) — 平行四边形直通 = 肘关节折叠角 q2_rel
  bool   valid;  // IK 是否有解
};

// =============================================================
// 单轴关节轨迹状态
// =============================================================
struct JointTrajectoryState {
  double position;     // rad
  double velocity;     // rad/s
};

// =============================================================
// 击球状态机枚举
// =============================================================
enum class StrikeState {
  HOMING_FOREARM,    // 归零：先小臂撞限位 → 再大臂撞限位（两阶段合一）
  READY,             // 就绪：保持当前位置，等待击球命令
  STRIKE,            // 爆发：从 Home 直冲虚拟穿透目标
  IMPACT_MOMENT,     // 击球：纯力矩开环窗口
  FOLLOW_THROUGH,    // 随挥：Kd ramp 刹车
  SETTLING,           // 缓冲回位：平滑回到 home 位置
  E_STOP             // 急停：全电机零力矩 + 最大阻尼
};

inline const char* strikeStateToString(StrikeState s) {
  switch (s) {
    case StrikeState::HOMING_FOREARM: return "HOMING";
    case StrikeState::READY:          return "READY";
    case StrikeState::STRIKE:         return "STRIKE";
    case StrikeState::IMPACT_MOMENT:  return "IMPACT_MOMENT";
    case StrikeState::FOLLOW_THROUGH: return "FOLLOW_THROUGH";
    case StrikeState::SETTLING:       return "SETTLING";
    case StrikeState::E_STOP:         return "E_STOP";
    default:                          return "UNKNOWN";
  }
}

// =============================================================
//  VolleyballStrikeManager 节点
// =============================================================
class VolleyballStrikeManager : public rclcpp::Node {
public:
  VolleyballStrikeManager();

private:
  // ==================== 电机索引常量 ====================
  static constexpr size_t L1 = 0;   // 左大臂
  static constexpr size_t L2 = 1;   // 左小臂
  static constexpr size_t R1 = 2;   // 右大臂
  static constexpr size_t R2 = 3;   // 右小臂
  static constexpr size_t NUM_MOTORS = 4;

  // ==================== 核心算法 ====================

  /**
   * @brief 双臂协作逆运动学
   *
   * 数学思路：
   *   1. 输入 (X_c, Y_c, θ) — 拍面中心坐标 + 仰角
   *   2. 根据拍面宽度 W 分解左右铰接点：
   *        P_L = (X_c - W/2·cosθ,  Y_c - W/2·sinθ)
   *        P_R = (X_c + W/2·cosθ,  Y_c + W/2·sinθ)
   *   3. 左臂肩关节坐标：M_L = (-D/2, 0)
   *      右臂肩关节坐标：M_R = (+D/2, 0)
   *   4. 各侧独立求解 2R 平面逆运动学：
   *        对左臂：从 M_L 到 P_L，链长 (L1, L2)
   *        对右臂：从 M_R 到 P_R，链长 (L1, L2)
   *
   * 2R-IK 封闭解（以左臂为例）：
   *   dx = P_Lx - M_Lx,  dy = P_Ly - M_Ly
   *   d  = sqrt(dx² + dy²)
   *   cos(q2) = (d² - L1² - L2²) / (2·L1·L2)
   *   q_L2 = atan2(elbow_sign · sqrt(1 - cos²(q2)), cos(q2))
   *   q_L1 = atan2(dy, dx) - atan2(L2·sin(q_L2), L1 + L2·cos(q_L2))
   */
  IKResult solveIK(double x_c, double y_c) const;

  /**
   * @brief 单臂 2R 逆运动学
   * @param mx, my   肩关节世界坐标
   * @param tx, ty   目标末端世界坐标
   * @param elbow_sign  +1 elbow-up, -1 elbow-down
   * @param q1, q2   输出关节角
   * @return 是否有解
   */
  bool solve2RIK(double mx, double my,
                 double tx, double ty,
                 int elbow_sign,
                 double& q1, double& q2) const;

  /**
   * @brief 对称镜像强制
   *
   * D=W=350.6mm 对称构型下，强制 L1=R1, L2=R2，
   * 消除 IK 浮点误差导致的闭链内力。
   */
  void enforceSymmetricMirror(IKResult& result) const;

  /**
   * @brief 重力补偿前馈力矩计算
   *
   * 14-200-14-200 平行四边形传动下，q2 = 肘关节折叠角（相对角）。
   * 小臂绝对角 = q1 + q2，重力矩公式：
   *   τ_g1 = (m1·g·L1/2 + m2·g·L1)·cos(q1) + m2·g·L2/2·cos(q1+q2)
   *   τ_g2 = m2·g·L2/2·cos(q1+q2)
   *
   * @param q1 大臂绝对角 (rad)
   * @param q2 肘关节折叠角 (rad)，相对角
   * @param tau_g1 [out] 大臂重力补偿力矩 (Nm)
   * @param tau_g2 [out] 小臂重力补偿力矩 (Nm)
   */
  void computeGravityCompensation(double q1, double q2,
                                   double& tau_g1, double& tau_g2) const;

  /**
   * @brief 正运动学：从关节角计算拍面中心 y 坐标
   *
   * y = L1·sin(q1_math) + L2·sin(q1_math + q2_math)
   * 其中 q_math = q_physical + homing_ik_offsets_
   *
   * @return 拍面中心的 y 坐标 (m)
   */
  double computeFK_Y() const;

  /**
   * @brief 穿透式轨迹规划器
   *
   * 思路：
   *   1. 计算虚拟穿透目标：沿击球方向延伸 penetration_depth
   *   2. 对 4 个关节做同步 S-curve 规划
   *   3. 约束：经过击球点关节角时速度达到 max_joint_velocity
   *   4. 反向推导：从 T_impact 倒推加速段起始时间
   *
   * 在 controlLoop 中每 5ms 调用一次，更新轨迹状态
   */
  void updateTrajectory(double dt);

  /**
   * @brief 消除内力的阻抗分配
   *
   * 4 电机驱动 3 自由度 → 1 维冗余。冗余方向 = "拍面内部拉/压力"。
   *
   * MIT 模式策略：
   *   大臂（L1, R1）：高 Kp → 承担刚性定位，主导拍面位姿
   *   小臂（L2, R2）：低 Kp → 柔性随动，吸收闭链加工误差
   *
   * 物理直觉：
   *   若左右小臂 Kp 都很高，任何连杆长度/装配公差都会让两侧
   *   小臂在拍面处"互相推/拉"，产生不做功的内力→磨损减速器。
   *   降低小臂 Kp，等效于在闭链中插入一个"虚拟弹簧"，
   *   允许微小形变消化误差。
   */
  void getImpedanceParams(size_t motor_idx,
                          double& kp, double& kd, double& tau_ff) const;

  // ==================== 控制循环 ====================
  void controlLoop();

  // ==================== 状态机逻辑 ====================
  void handleHomingForearm();
  void handleReady();
  void handleStrike(double dt);
  void handleImpactMoment();
  void handleFollowThrough(double dt);
  void handleSettling(double dt);
  void handleEStop();

  // ==================== 回调 ====================
  void strikeCommandCallback(
      const motor_control_ros2::msg::StrikeCommand::SharedPtr msg);
  void motorStateCallback(
      const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);
  void testCommandCallback(
      const std_msgs::msg::String::SharedPtr msg);

  // ==================== 工具函数 ====================
  void loadConfig(const std::string& config_file);
  void publishMotorCommand(size_t idx,
                           double p_des, double v_des, double tau_ff,
                           double kp, double kd);
  void transitionTo(StrikeState new_state);
  bool isInWorkspace(double x, double y) const;
  void publishStatus() const;
  bool allHomingMotorsReady(const std::array<size_t, 2>& motor_indices) const;

  /**
   * @brief 计算关节同步缩放系数（一次计算，全程锁定）
   *
   * 在每次设定 traj_target_ 后立即调用。
   * 基于当前 traj_state_ 与 traj_target_ 的差值比例，
   * 让行程短的关节主动限速等待行程长的关节。
   * 计算后 traj_sync_scale_ 不再改变，直到下次目标切换。
   */
  void computeTrajSyncScale();

  // ==================== ROS2 通信 ====================
  rclcpp::Subscription<motor_control_ros2::msg::StrikeCommand>::SharedPtr strike_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr state_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ==================== 几何配置 ====================
  ArmGeometry geom_;
  int elbow_sign_left_;    // 左臂肘构型
  int elbow_sign_right_;   // 右臂肘构型

  // ==================== 电机配置 ====================
  std::array<std::string, NUM_MOTORS> motor_names_;
  std::array<std::string, NUM_MOTORS> motor_devices_;
  std::array<uint8_t, NUM_MOTORS>     motor_ids_;
  std::array<int, NUM_MOTORS>         motor_directions_;
  std::array<double, NUM_MOTORS>      zero_offsets_;

  // ==================== 反馈状态 ====================
  std::array<double, NUM_MOTORS> raw_positions_;         // rad (电机原始单圈绝对值)
  std::array<double, NUM_MOTORS> raw_velocities_;        // rad/s (电机原始速度)
  std::array<double, NUM_MOTORS> current_positions_;     // rad (解耦后逻辑角：已去 direction/zero_offset)
  std::array<double, NUM_MOTORS> current_velocities_;    // rad/s (解耦后逻辑角速度)
  std::array<bool, NUM_MOTORS>   motors_online_;
  std::array<bool, NUM_MOTORS>   has_feedback_;

  // ==================== 状态机 ====================
  StrikeState state_;
  rclcpp::Time state_enter_time_;       // 当前状态进入时刻

  // ==================== 击球参数（来自命令）====================
  double impact_x_;                     // 击球点 (m)
  double impact_y_;

  // ==================== IK 目标 ====================
  IKResult ik_impact_;                  // 击球点的 IK 解
  IKResult ik_virtual_;                 // 虚拟穿透目标的 IK 解

  // ==================== 轨迹规划器 ====================
  std::array<JointTrajectoryState, NUM_MOTORS> traj_state_;  // 当前规划位置/速度
  std::array<double, NUM_MOTORS> traj_target_;               // 当前轨迹目标角
  std::array<double, NUM_MOTORS> traj_sync_scale_;           // 关节同步缩放系数（目标切换时一次计算，全程锁定）
  double max_joint_velocity_;
  double max_joint_acceleration_;

  // ==================== 阻抗参数 ====================
  // STRIKE（从 Home 直冲到虚拟穿透目标）
  double strike_kp_shoulder_, strike_kp_elbow_;
  double strike_kd_shoulder_, strike_kd_elbow_;
  double strike_tau_ff_scale_;
  // IMPACT_MOMENT
  double impact_kp_shoulder_, impact_kp_elbow_;
  double impact_kd_shoulder_, impact_kd_elbow_;
  double impact_tau_ff_shoulder_, impact_tau_ff_elbow_;
  double impact_window_after_s_;    // 击球后超时保护 (s)
  // 空间事件触发参数
  double impact_y_threshold_;       // FK y 距离击球点的切换阈值 (m)
  double impact_overshoot_exit_;    // IMPACT 超越击球点的退出距离 (m)
  double virtual_overshoot_y_;      // 虚拟穿透目标 y 偏移 (m)
  // FOLLOW_THROUGH
  double follow_kp_shoulder_, follow_kp_elbow_;
  double follow_kd_start_;
  double follow_kd_end_shoulder_, follow_kd_end_elbow_;
  double follow_ramp_duration_s_;
  // 随挥刹车锁定位置
  std::array<double, NUM_MOTORS> follow_lock_positions_;

  // SETTLING（缓冲回位）
  double settling_max_velocity_;     // 回位最大速度 (rad/s)，比击球慢
  double settling_max_acceleration_; // 回位最大加速度 (rad/s²)，防止突然下落
  double settling_reach_tol_;        // 到位容差 (rad)

  // ==================== 动力学参数（重力补偿） ====================
  double upper_arm_mass_;           // m1: 大臂质量 (kg)
  double forearm_mass_;             // m2: 小臂+拍面等效质量 (kg)
  bool   enable_gravity_comp_;      // 是否启用重力补偿
  bool   symmetric_mode_;           // 是否强制对称镜像

  // ==================== 开机归零参数 ====================
  bool   startup_homing_enabled_;         // 启用开机撞限位归零
  std::array<double, NUM_MOTORS> homing_ik_offsets_;  // IK数学零点与物理限位零点的偏差
  double homing_kd_;                      // 归零阶段阻尼
  double homing_hold_kp_;                 // 非当前归零轴保持位置刚度
  double homing_hold_kd_;                 // 非当前归零轴保持位置阻尼
  double homing_velocity_threshold_;      // 判定撞限位的速度阈值
  double homing_stable_duration_s_;       // 撞限位后稳定判定时长
  double homing_min_travel_rad_;          // 归零阶段最小运动角
  double homing_timeout_s_;               // 单阶段超时保护
  std::array<double, NUM_MOTORS> homing_torque_to_min_;  // 向“最小角”方向归零力矩 [L1,L2,R1,R2]
  std::array<double, NUM_MOTORS> idle_hold_torque_;      // IDLE/肩关节home保持前馈 [L1,L2,R1,R2]
  int homing_sub_phase_{0};                      // 0=小臂阶段, 1=大臂阶段
  rclcpp::Time homing_phase_start_time_;
  rclcpp::Time homing_stable_since_;
  bool homing_stability_started_;
  bool homing_phase_initialized_;
  std::array<double, NUM_MOTORS> homing_start_positions_;

  // ==================== 安全参数 ====================
  double max_position_error_;
  double velocity_spike_threshold_;
  double torque_limit_;
  double e_stop_kd_;
  double e_stop_ramp_duration_s_;   // E_STOP Kd 爬升时间 (s)
  double idle_kp_, idle_kd_;
  double control_frequency_;

  // ==================== 工作空间限制 ====================
  double ws_x_min_, ws_x_max_;
  double ws_y_min_, ws_y_max_;
};

#endif  // MOTOR_CONTROL_ROS2__VOLLEYBALL_STRIKE_MANAGER_HPP_
