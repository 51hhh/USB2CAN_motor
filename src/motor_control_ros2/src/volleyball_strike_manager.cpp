#include "motor_control_ros2/volleyball_strike_manager.hpp"
#include <yaml-cpp/yaml.h>

// GO-M8010-6 MIT 模式协议缩放因子（减速比 i = 6.33）
//   cmd.tau = 期望输出轴力矩(Nm)       / 6.33
//   cmd.kp  = 期望输出轴刚度(Nm/rad)   / 6.33² = / 40.07
//   cmd.kd  = 期望输出轴阻尼(Nm·s/rad) / 6.33² = / 40.07
static constexpr double GEAR_RATIO = 6.33;
static constexpr double TAU_PROTOCOL_SCALE = GEAR_RATIO;   // τ_config = τ_物理 / 6.33
// KP_KD_PROTOCOL_SCALE = GEAR_RATIO * GEAR_RATIO = 40.07 (仅文档参考)

// =============================================================
// 构造函数
// =============================================================
VolleyballStrikeManager::VolleyballStrikeManager()
    : Node("volleyball_strike_manager"),
      // 几何默认值（实测 2026-03-31: D=W=350.6mm, L1=200mm, L2=224mm, crank=14mm）
      geom_{0.3506, 0.200, 0.224, 0.3506, 0.014},
      elbow_sign_left_(1),
      elbow_sign_right_(-1),
      // 状态机
      state_(StrikeState::READY),
      // 击球参数
      impact_x_(0.0), impact_y_(0.0),
      // 轨迹规划器
      max_joint_velocity_(15.0),
      max_joint_acceleration_(80.0),
      // 阻抗 — STRIKE（从 Home 直冲虚拟穿透目标，复用原 STRIKE_ACCEL 参数）
      strike_kp_shoulder_(0.998), strike_kp_elbow_(0.499),
      strike_kd_shoulder_(0.050), strike_kd_elbow_(0.025),
      strike_tau_ff_scale_(0.02),
      // 阻抗 — IMPACT_MOMENT
      impact_kp_shoulder_(0.050), impact_kp_elbow_(0.025),
      impact_kd_shoulder_(0.012), impact_kd_elbow_(0.006),
      impact_tau_ff_shoulder_(1.264), impact_tau_ff_elbow_(0.632),
      impact_window_after_s_(0.030),
      // 空间事件触发
      impact_y_threshold_(0.05),
      impact_overshoot_exit_(0.10),
      virtual_overshoot_y_(0.10),
      // 阻抗 — FOLLOW_THROUGH
      follow_kp_shoulder_(0.250), follow_kp_elbow_(0.125),
      follow_kd_start_(0.012),
      follow_kd_end_shoulder_(0.200), follow_kd_end_elbow_(0.125),
      follow_ramp_duration_s_(0.100),
      // SETTLING
      settling_max_velocity_(1.2),
      settling_max_acceleration_(8.0),
      settling_reach_tol_(0.03),
      // 动力学
      upper_arm_mass_(1.0),
      forearm_mass_(0.5),
      enable_gravity_comp_(false),
      symmetric_mode_(true),
      // 开机归零
      startup_homing_enabled_(true),
      homing_ik_offsets_{0.0, 0.0, 0.0, 0.0},
      homing_kd_(0.100),
      homing_hold_kp_(0.225),
      homing_hold_kd_(0.032),
      homing_velocity_threshold_(0.12),
      homing_stable_duration_s_(0.40),
      homing_min_travel_rad_(0.12),
      homing_timeout_s_(8.0),
      homing_torque_to_min_{0.474, -0.350, 0.474, -0.350},
      idle_hold_torque_{-0.900, -0.553, -0.900, -0.553},
      homing_stability_started_(false),
      homing_phase_initialized_(false),
      // 安全
      max_position_error_(0.5),
      velocity_spike_threshold_(20.0),
      torque_limit_(1.58),
      e_stop_kd_(0.125),
      e_stop_ramp_duration_s_(0.020),
      idle_kp_(0.200), idle_kd_(0.075),
      control_frequency_(200.0),
      // 工作空间
      ws_x_min_(0.10), ws_x_max_(0.40),
      ws_y_min_(-0.25), ws_y_max_(0.25)
{
  raw_positions_.fill(0.0);
  raw_velocities_.fill(0.0);
  current_positions_.fill(0.0);
  current_velocities_.fill(0.0);
  motors_online_.fill(false);
  has_feedback_.fill(false);
  traj_target_.fill(0.0);
  traj_sync_scale_.fill(1.0);
  follow_lock_positions_.fill(0.0);
  homing_start_positions_.fill(0.0);
  zero_offsets_.fill(0.0);
  homing_ik_offsets_.fill(0.0);
  motor_directions_.fill(1);
  for (auto& ts : traj_state_) { ts.position = 0.0; ts.velocity = 0.0; }

  motor_names_   = {"arm_motor_l_1", "arm_motor_l_2", "arm_motor_r_1", "arm_motor_r_2"};
  motor_devices_ = {"", "", "", ""};
  motor_ids_     = {0, 1, 2, 3};

  // 加载 YAML 配置
  try {
    std::string config_file =
        ament_index_cpp::get_package_share_directory("motor_control_ros2") +
        "/config/volleyball_strike_params.yaml";
    loadConfig(config_file);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "配置加载失败（使用默认值）: %s", e.what());
  }

  if (startup_homing_enabled_) {
    // 单圈绝对编码器：开机位置不固定，归零阶段不使用旧 offset
    zero_offsets_.fill(0.0);
    state_ = StrikeState::HOMING_FOREARM;
  } else {
    state_ = StrikeState::READY;
  }

  // 订阅击球命令（事件驱动：收到即触发完整击球序列）
  strike_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::StrikeCommand>(
      "/volleyball/strike_command", 10,
      std::bind(&VolleyballStrikeManager::strikeCommandCallback, this, std::placeholders::_1));

  // 订阅 GO8010 电机状态反馈
  state_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&VolleyballStrikeManager::motorStateCallback, this, std::placeholders::_1));

  // 发布 GO8010 电机命令
  cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10);

  // 发布状态（供上层监控）
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/volleyball/strike_status", 10);

  // 订阅测试命令（无视觉系统时手动测试）
  test_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/volleyball/test_command", 10,
      std::bind(&VolleyballStrikeManager::testCommandCallback, this, std::placeholders::_1));

  // 200Hz 控制定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VolleyballStrikeManager::controlLoop, this));

  state_enter_time_ = this->now();
    homing_phase_start_time_ = this->now();
    homing_stable_since_ = this->now();

  RCLCPP_INFO(this->get_logger(),
      "volleyball_strike_manager 启动 | %.0f Hz | D=%.3f L1=%.3f L2=%.3f W=%.3f | 平行四边形传动 crank=%.3f | startup_homing=%d",
      control_frequency_, geom_.arm_spacing, geom_.upper_arm_length,
      geom_.forearm_length, geom_.plate_width, geom_.crank_length,
      startup_homing_enabled_ ? 1 : 0);
}

// =============================================================
// 配置加载
// =============================================================
void VolleyballStrikeManager::loadConfig(const std::string& config_file)
{
  YAML::Node cfg = YAML::LoadFile(config_file);

  // 几何
  auto geo = cfg["geometry"];
  if (geo) {
    if (geo["arm_spacing"])       geom_.arm_spacing       = geo["arm_spacing"].as<double>();
    if (geo["upper_arm_length"])  geom_.upper_arm_length  = geo["upper_arm_length"].as<double>();
    if (geo["forearm_length"])    geom_.forearm_length    = geo["forearm_length"].as<double>();
    if (geo["plate_width"])       geom_.plate_width       = geo["plate_width"].as<double>();
    if (geo["crank_length"])      geom_.crank_length      = geo["crank_length"].as<double>();
  }

  // IK 肘构型 + 对称模式
  auto ik = cfg["ik_solver"];
  if (ik) {
    if (ik["elbow_sign_left"])   elbow_sign_left_  = ik["elbow_sign_left"].as<int>();
    if (ik["elbow_sign_right"])  elbow_sign_right_ = ik["elbow_sign_right"].as<int>();
    if (ik["symmetric_mode"])    symmetric_mode_   = ik["symmetric_mode"].as<bool>();
  }

  // 电机映射
  auto motors = cfg["motors"];
  if (motors && motors.IsSequence()) {
    for (size_t i = 0; i < NUM_MOTORS && i < motors.size(); ++i) {
      if (motors[i]["name"])      motor_names_[i]      = motors[i]["name"].as<std::string>();
      if (motors[i]["device"])    motor_devices_[i]    = motors[i]["device"].as<std::string>();
      if (motors[i]["id"])        motor_ids_[i]        = motors[i]["id"].as<uint8_t>();
      if (motors[i]["direction"]) motor_directions_[i] = motors[i]["direction"].as<int>();
    }
  }

  // 标定
  auto cal = cfg["calibration"];
  if (cal && cal["zero_offsets"] && cal["zero_offsets"].IsSequence()) {
    for (size_t i = 0; i < NUM_MOTORS && i < cal["zero_offsets"].size(); ++i) {
      zero_offsets_[i] = cal["zero_offsets"][i].as<double>();
    }
  }
  if (cal && cal["homing_ik_offsets"] && cal["homing_ik_offsets"].IsSequence()) {
    for (size_t i = 0; i < NUM_MOTORS && i < cal["homing_ik_offsets"].size(); ++i) {
      homing_ik_offsets_[i] = cal["homing_ik_offsets"][i].as<double>();
    }
  }

  // 控制频率
  if (cfg["control_frequency"]) control_frequency_ = cfg["control_frequency"].as<double>();

  // 阻抗参数
  auto imp = cfg["impedance"];
  if (imp) {
    auto sa = imp["strike"];
    if (sa) {
      if (sa["kp_shoulder"])  strike_kp_shoulder_ = sa["kp_shoulder"].as<double>();
      if (sa["kp_elbow"])     strike_kp_elbow_    = sa["kp_elbow"].as<double>();
      if (sa["kd_shoulder"])  strike_kd_shoulder_ = sa["kd_shoulder"].as<double>();
      if (sa["kd_elbow"])     strike_kd_elbow_    = sa["kd_elbow"].as<double>();
      if (sa["tau_ff_scale"]) strike_tau_ff_scale_ = sa["tau_ff_scale"].as<double>();
    }
    auto im = imp["impact"];
    if (im) {
      if (im["kp_shoulder"])     impact_kp_shoulder_     = im["kp_shoulder"].as<double>();
      if (im["kp_elbow"])        impact_kp_elbow_        = im["kp_elbow"].as<double>();
      if (im["kd_shoulder"])     impact_kd_shoulder_     = im["kd_shoulder"].as<double>();
      if (im["kd_elbow"])        impact_kd_elbow_        = im["kd_elbow"].as<double>();
      if (im["tau_ff_shoulder"]) impact_tau_ff_shoulder_  = im["tau_ff_shoulder"].as<double>();
      if (im["tau_ff_elbow"])    impact_tau_ff_elbow_     = im["tau_ff_elbow"].as<double>();
      if (im["window_after_ms"])
        impact_window_after_s_  = im["window_after_ms"].as<double>() * 0.001;
    }
    auto ft = imp["follow_through"];
    if (ft) {
      if (ft["kp_shoulder"])     follow_kp_shoulder_     = ft["kp_shoulder"].as<double>();
      if (ft["kp_elbow"])        follow_kp_elbow_        = ft["kp_elbow"].as<double>();
      if (ft["kd_start"])        follow_kd_start_        = ft["kd_start"].as<double>();
      if (ft["kd_end_shoulder"]) follow_kd_end_shoulder_ = ft["kd_end_shoulder"].as<double>();
      if (ft["kd_end_elbow"])    follow_kd_end_elbow_    = ft["kd_end_elbow"].as<double>();
      if (ft["ramp_duration_ms"])
        follow_ramp_duration_s_ = ft["ramp_duration_ms"].as<double>() * 0.001;
    }
    auto stl = imp["settling"];
    if (stl) {
      if (stl["max_velocity"]) settling_max_velocity_ = stl["max_velocity"].as<double>();
      if (stl["max_acceleration"]) settling_max_acceleration_ = stl["max_acceleration"].as<double>();
      if (stl["reach_tol"])    settling_reach_tol_    = stl["reach_tol"].as<double>();
    }
  }

  // 轨迹参数
  auto traj = cfg["trajectory"];
  if (traj) {
    if (traj["max_joint_velocity"])     max_joint_velocity_     = traj["max_joint_velocity"].as<double>();
    if (traj["max_joint_acceleration"]) max_joint_acceleration_ = traj["max_joint_acceleration"].as<double>();
  }

  // 空间事件触发参数
  auto sp = cfg["spatial_trigger"];
  if (sp) {
    if (sp["impact_y_threshold"])    impact_y_threshold_    = sp["impact_y_threshold"].as<double>();
    if (sp["impact_overshoot_exit"]) impact_overshoot_exit_ = sp["impact_overshoot_exit"].as<double>();
    if (sp["virtual_overshoot_y"])   virtual_overshoot_y_   = sp["virtual_overshoot_y"].as<double>();
  }

  // 安全参数
  auto sf = cfg["safety"];
  if (sf) {
    if (sf["max_position_error"])       max_position_error_       = sf["max_position_error"].as<double>();
    if (sf["velocity_spike_threshold"]) velocity_spike_threshold_ = sf["velocity_spike_threshold"].as<double>();
    if (sf["torque_limit"])             torque_limit_             = sf["torque_limit"].as<double>();
    if (sf["e_stop_kd"])                e_stop_kd_                = sf["e_stop_kd"].as<double>();
    if (sf["e_stop_ramp_duration_ms"])
      e_stop_ramp_duration_s_ = sf["e_stop_ramp_duration_ms"].as<double>() * 0.001;
    if (sf["idle_kp"])                  idle_kp_                  = sf["idle_kp"].as<double>();
    if (sf["idle_kd"])                  idle_kd_                  = sf["idle_kd"].as<double>();
  }

  // 动力学参数（重力补偿）
  auto dyn = cfg["dynamics"];
  if (dyn) {
    if (dyn["upper_arm_mass"])      upper_arm_mass_      = dyn["upper_arm_mass"].as<double>();
    if (dyn["forearm_mass"])        forearm_mass_        = dyn["forearm_mass"].as<double>();
    if (dyn["enable_gravity_comp"]) enable_gravity_comp_ = dyn["enable_gravity_comp"].as<bool>();
  }

  // 开机撞限位归零参数
  auto hm = cfg["homing"];
  if (hm) {
    if (hm["enabled"])                startup_homing_enabled_    = hm["enabled"].as<bool>();
    if (hm["kd"])                     homing_kd_                = hm["kd"].as<double>();
    if (hm["hold_kp"])                homing_hold_kp_           = hm["hold_kp"].as<double>();
    if (hm["hold_kd"])                homing_hold_kd_           = hm["hold_kd"].as<double>();
    if (hm["velocity_threshold"])     homing_velocity_threshold_ = hm["velocity_threshold"].as<double>();
    if (hm["stable_duration_s"])      homing_stable_duration_s_  = hm["stable_duration_s"].as<double>();
    if (hm["min_travel_rad"])         homing_min_travel_rad_     = hm["min_travel_rad"].as<double>();
    if (hm["timeout_s"])              homing_timeout_s_          = hm["timeout_s"].as<double>();

    if (hm["torque_to_min"] && hm["torque_to_min"].IsSequence()) {
      for (size_t i = 0; i < NUM_MOTORS && i < hm["torque_to_min"].size(); ++i) {
        homing_torque_to_min_[i] = hm["torque_to_min"][i].as<double>();
      }
    }

    if (hm["idle_hold_torque"] && hm["idle_hold_torque"].IsSequence()) {
      for (size_t i = 0; i < NUM_MOTORS && i < hm["idle_hold_torque"].size(); ++i) {
        idle_hold_torque_[i] = hm["idle_hold_torque"][i].as<double>();
      }
    }
  }

  // 工作空间
  auto ws = cfg["workspace"];
  if (ws) {
    if (ws["x_min"])     ws_x_min_     = ws["x_min"].as<double>();
    if (ws["x_max"])     ws_x_max_     = ws["x_max"].as<double>();
    if (ws["y_min"])     ws_y_min_     = ws["y_min"].as<double>();
    if (ws["y_max"])     ws_y_max_     = ws["y_max"].as<double>();
  }

    RCLCPP_INFO(this->get_logger(), "配置加载完成 | D=%.3f L1=%.3f L2=%.3f W=%.3f | gravity_comp=%d symmetric=%d homing=%d",
      geom_.arm_spacing, geom_.upper_arm_length, geom_.forearm_length,
      geom_.plate_width, enable_gravity_comp_, symmetric_mode_, startup_homing_enabled_ ? 1 : 0);
}

// =============================================================
//  双臂协作逆运动学
// =============================================================

IKResult VolleyballStrikeManager::solveIK(double x_c, double y_c) const
{
  IKResult result{};
  result.valid = false;

  /*
   * 纯 2D 单臂逆解 + 双臂复制
   *
   * 这是一个左右平行的双臂系统，末端拍面刚性锁死。
   * 在侧视图 (X-Y 平面) 看来，左右两臂必须 100% 重合！
   * 因此：
   *   - 完全抛弃宽度的 3D 干扰（不再拆分左右铰接点）
   *   - 忽略无法独立控制的 theta（拍面角度由关节角唯一确定）
   *   - 以肩膀为原点 (0,0)，对目标 (x_c, y_c) 求纯粹的单臂 2D 逆解
   *   - 将结果 1:1 复制给 4 个电机
   */
  double q1, q2;
  bool ok = solve2RIK(0.0, 0.0, x_c, y_c, elbow_sign_left_, q1, q2);

  if (!ok) {
    return result;  // 目标太远或太近，无法到达
  }

  // 单臂角度 1:1 复制给 4 个电机
  result.q_L1 = q1;
  result.q_L2 = q2;
  result.q_R1 = q1;
  result.q_R2 = q2;
  result.valid = true;

  return result;
}

bool VolleyballStrikeManager::solve2RIK(
    double mx, double my,
    double tx, double ty,
    int elbow_sign,
    double& q1, double& q2) const
{
  const double L1 = geom_.upper_arm_length;
  const double L2 = geom_.forearm_length;

  // 肩关节到目标的向量
  const double dx = tx - mx;
  const double dy = ty - my;
  const double d_sq = dx * dx + dy * dy;
  const double d = std::sqrt(d_sq);

  // 可达性检测
  if (d > L1 + L2 - 1e-6 || d < std::abs(L1 - L2) + 1e-6) {
    return false;
  }

  // 肘关节角（相对角）: cos(q2) = (d² - L1² - L2²) / (2·L1·L2)
  const double cos_q2 = (d_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  const double cos_q2_clamped = std::clamp(cos_q2, -1.0, 1.0);
  const double sin_q2 = static_cast<double>(elbow_sign) * std::sqrt(1.0 - cos_q2_clamped * cos_q2_clamped);
  q2 = std::atan2(sin_q2, cos_q2_clamped);

  // 大臂角（绝对角）
  q1 = std::atan2(dy, dx) - std::atan2(L2 * sin_q2, L1 + L2 * cos_q2_clamped);

  return true;
}

// =============================================================
//  对称镜像强制 & 重力补偿
// =============================================================

/*
 * 14-200-14-200 平行四边形传动证明：
 *
 *   硬件拓扑（单侧，以左臂为例）：
 *
 *   [L1电机] ─── 大臂(L1=200mm, frame link) ─── 肘关节(被动)
 *       ↑                                            │
 *   [L2电机]                                         │ 小臂(L2=224mm)
 *   (外壳固定在大臂根部)                              │
 *       │                                            │
 *       └── 主动拨片(14mm) ── 连杆(200mm) ── 从动摇臂(14mm) ──┘
 *
 *   平行四边形: 14mm-200mm-14mm-200mm
 *   由于 AB=CD=200mm, AC=BD=14mm：
 *   q_motor(L2) = θ_forearm_relative_to_upper_arm（精确等式）
 *
 *   L2电机每转1rad → 小臂相对大臂也精确转1rad。
 *   大臂运动不影响小臂相对角 → 完全解耦！
 */

void VolleyballStrikeManager::enforceSymmetricMirror(IKResult& result) const
{
  // D=W=350.6mm 对称构型下，左右IK应完全相同
  // 由于浮点误差可能有微小差异，强制取平均
  double q1_avg = (result.q_L1 + result.q_R1) / 2.0;
  double q2_avg = (result.q_L2 + result.q_R2) / 2.0;
  result.q_L1 = result.q_R1 = q1_avg;
  result.q_L2 = result.q_R2 = q2_avg;
}

void VolleyballStrikeManager::computeGravityCompensation(
    double q1, double q2,
    double& tau_g1, double& tau_g2) const
{
  /*
   * 重力补偿力矩（对抗重力，使手臂能在任意姿态静止悬停）
   *
   * 坐标系：q1=0 水平向右，正值逆时针（向上）
   * q2 = 肘关节折叠角（相对角），小臂绝对角 = q1 + q2
   *
   * 静态重力矩：
   *   τ_g1 = (m1·g·L1/2 + m2·g·L1)·cos(q1) + m2·g·L2/2·cos(q1+q2)
   *   τ_g2 = m2·g·L2/2·cos(q1+q2)
   *
   * 返回值为补偿力矩（与重力矩方向相反，正值=向上托举）
   */
  constexpr double g = 9.81;
  const double L1 = geom_.upper_arm_length;
  const double L2 = geom_.forearm_length;
  const double m1 = upper_arm_mass_;
  const double m2 = forearm_mass_;

  const double cos_q1    = std::cos(q1);
  const double cos_q1q2  = std::cos(q1 + q2);

  tau_g1 = (m1 * g * L1 / 2.0 + m2 * g * L1) * cos_q1
           + m2 * g * L2 / 2.0 * cos_q1q2;
  tau_g2 = m2 * g * L2 / 2.0 * cos_q1q2;
}

double VolleyballStrikeManager::computeFK_Y() const
{
  /*
   * 正运动学：计算末端执行器 y 坐标
   *   y = L1·sin(q1) + L2·sin(q1 + q2)
   *
   * homing 完成后 current_positions_ 已经处于数学坐标系：
   *   q=0 → 水平正前方, q=-π/2 → 垂直向下
   * 因此直接使用，不需要再加 homing_ik_offsets_！
   *
   * 仅使用左臂 (L1/L2)，因为 symmetric_mode 下左右相同
   */
  const double L1_len = geom_.upper_arm_length;
  const double L2_len = geom_.forearm_length;

  const double q1 = current_positions_[L1];
  const double q2 = current_positions_[L2];

  return L1_len * std::sin(q1) + L2_len * std::sin(q1 + q2);
}

// =============================================================
//  关节同步缩放系数（一次计算，全程锁定）
// =============================================================

void VolleyballStrikeManager::computeTrajSyncScale()
{
  double max_err = 1e-6;  // 防止除零
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double err = std::abs(traj_target_[i] - traj_state_[i].position);
    if (err > max_err) max_err = err;
  }
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double err = std::abs(traj_target_[i] - traj_state_[i].position);
    traj_sync_scale_[i] = std::max(err / max_err, 0.01);
  }
}

// =============================================================
//  穿透式轨迹规划器（使用锁定的同步缩放系数）
// =============================================================

void VolleyballStrikeManager::updateTrajectory(double dt)
{
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double err = traj_target_[i] - traj_state_[i].position;

    // 使用目标切换时锁定的同步缩放系数
    double local_max_v = max_joint_velocity_ * traj_sync_scale_[i];
    double local_max_a = max_joint_acceleration_ * traj_sync_scale_[i];
    local_max_v = std::max(local_max_v, 0.05);
    local_max_a = std::max(local_max_a, 0.5);

    double v_abs = std::abs(traj_state_[i].velocity);
    double decel_dist = (v_abs * v_abs) / (2.0 * local_max_a);

    double target_vel;
    if (std::abs(err) > decel_dist + 0.01) {
      target_vel = std::copysign(local_max_v, err);
    } else {
      double v_decel = std::sqrt(2.0 * local_max_a * std::max(std::abs(err), 0.0));
      target_vel = std::copysign(std::min(v_decel, local_max_v), err);
    }

    double dv = target_vel - traj_state_[i].velocity;
    double max_dv = local_max_a * dt;
    if (std::abs(dv) > max_dv) {
      dv = std::copysign(max_dv, dv);
    }
    traj_state_[i].velocity += dv;

    traj_state_[i].velocity = std::clamp(traj_state_[i].velocity,
                                          -local_max_v, local_max_v);

    traj_state_[i].position += traj_state_[i].velocity * dt;
  }
}

// =============================================================
//  阻抗参数分配（消除内力核心逻辑）
// =============================================================

void VolleyballStrikeManager::getImpedanceParams(
    size_t motor_idx, double& kp, double& kd, double& tau_ff) const
{
  /*
   * 内力消除策略（14-200-14-200 平行四边形传动构型）：
   *
   * 4 电机驱动 3 自由度 → 1 维冗余方向。
   * 冗余方向 = 沿拍面纵轴的“拉/压”方向。
   *
   * L2电机外壳固定在大臂根部，通过拉杆传动小臂。
   * 大臂电机直驱，小臂通过平行四边形传动。
   *
   * MIT 模式非对称 Kp 策略：
   *   大臂电机 (L1, R1) → 高 Kp：直驱，刚性定位拍面位姿
   *   小臂电机 (L2, R2) → 低 Kp：柔性随动，吸收闭链加工误差
   *
   * 前馈补偿：所有状态均叠加 idle_hold_torque_（除 E_STOP）
   */

  const bool is_shoulder = (motor_idx == L1 || motor_idx == R1);

  switch (state_) {
    case StrikeState::HOMING_FOREARM:
      kp     = 0.0;
      kd     = homing_kd_;
      tau_ff = 0.0;
      break;

    case StrikeState::READY:
    case StrikeState::SETTLING:
      kp     = is_shoulder ? strike_kp_shoulder_ : strike_kp_elbow_;
      kd     = is_shoulder ? strike_kd_shoulder_ : strike_kd_elbow_;
      tau_ff = 0.0;
      break;

    case StrikeState::STRIKE: {
      kp = is_shoulder ? strike_kp_shoulder_ : strike_kp_elbow_;
      kd = is_shoulder ? strike_kd_shoulder_ : strike_kd_elbow_;
      // 前馈力矩 ∝ 规划加速度（简化惯性模型）
      // τ_ff = scale × (planned_accel)  — 在 handleStrike 中计算
      tau_ff = 0.0;  // 占位，实际在 handleStrike 中覆盖
      break;
    }

    case StrikeState::IMPACT_MOMENT:
      // 纯力矩开环爆破：极低 Kp + 极大 tau_ff
      kp     = is_shoulder ? impact_kp_shoulder_ : impact_kp_elbow_;
      kd     = is_shoulder ? impact_kd_shoulder_ : impact_kd_elbow_;
      tau_ff = is_shoulder ? impact_tau_ff_shoulder_ : impact_tau_ff_elbow_;
      break;

    case StrikeState::FOLLOW_THROUGH: {
      kp = is_shoulder ? follow_kp_shoulder_ : follow_kp_elbow_;
      // Kd 线性 ramp
      double elapsed = (this->now() - state_enter_time_).seconds();
      double t = std::clamp(elapsed / follow_ramp_duration_s_, 0.0, 1.0);
      double kd_end = is_shoulder ? follow_kd_end_shoulder_ : follow_kd_end_elbow_;
      kd     = follow_kd_start_ + t * (kd_end - follow_kd_start_);
      tau_ff = 0.0;  // 刹车阶段零前馈
      break;
    }

    case StrikeState::E_STOP:
      kp     = 0.0;
      kd     = e_stop_kd_;  // 仅作为最终值，实际由 handleEStop 的 ramp 控制
      tau_ff = 0.0;
      break;
  }
}

// =============================================================
//  主控制循环 @ 200Hz
// =============================================================

void VolleyballStrikeManager::controlLoop()
{
  const double dt = 1.0 / control_frequency_;

  // 安全检查：速度突变检测
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    if (has_feedback_[i] && std::abs(current_velocities_[i]) > velocity_spike_threshold_) {
      RCLCPP_ERROR(this->get_logger(), "电机 %s 速度突变 %.2f rad/s → E_STOP",
                   motor_names_[i].c_str(), current_velocities_[i]);
      transitionTo(StrikeState::E_STOP);
      break;
    }
  }

  switch (state_) {
    case StrikeState::HOMING_FOREARM: handleHomingForearm();      break;
    case StrikeState::READY:          handleReady();               break;
    case StrikeState::STRIKE:         handleStrike(dt);            break;
    case StrikeState::IMPACT_MOMENT:  handleImpactMoment();       break;
    case StrikeState::FOLLOW_THROUGH: handleFollowThrough(dt);    break;
    case StrikeState::SETTLING:       handleSettling(dt);          break;
    case StrikeState::E_STOP:         handleEStop();              break;
  }
}

// =============================================================
//  状态处理函数
// =============================================================

bool VolleyballStrikeManager::allHomingMotorsReady(
    const std::array<size_t, 2>& motor_indices) const
{
  for (size_t idx : motor_indices) {
    if (!has_feedback_[idx] || !motors_online_[idx]) {
      return false;
    }
  }
  return true;
}

void VolleyballStrikeManager::handleHomingForearm()
{
  // ====== 两阶段归零（合并在一个状态内）======
  //   sub_phase 0 → 小臂(L2/R2)撞限位
  //   sub_phase 1 → 大臂(L1/R1)撞限位
  const bool is_forearm_phase = (homing_sub_phase_ == 0);
  const std::array<size_t, 2> active = is_forearm_phase
      ? std::array<size_t, 2>{L2, R2}
      : std::array<size_t, 2>{L1, R1};

  if (!allHomingMotorsReady(active)) {
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      publishMotorCommand(i, 0.0, 0.0, 0.0, 0.0, 0.010);
    }
    return;
  }

  if (!homing_phase_initialized_) {
    homing_start_positions_ = current_positions_;
    homing_phase_start_time_ = this->now();
    homing_stability_started_ = false;
    homing_phase_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
        "归零阶段%d：%s 向机械限位运动",
        homing_sub_phase_ + 1,
        is_forearm_phase ? "小臂(L2/R2)" : "大臂(L1/R1)");
  }

  const double elapsed = (this->now() - homing_phase_start_time_).seconds();
  if (elapsed > homing_timeout_s_) {
    RCLCPP_ERROR(this->get_logger(), "归零阶段%d 超时 %.2f s → E_STOP",
        homing_sub_phase_ + 1, homing_timeout_s_);
    transitionTo(StrikeState::E_STOP);
    return;
  }

  // 激活轴：力矩驱动撞限位；非激活轴：保持位置
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    bool is_active = (i == active[0] || i == active[1]);
    if (is_active) {
      publishMotorCommand(i, current_positions_[i], 0.0,
                          homing_torque_to_min_[i], 0.0, homing_kd_);
    } else if (!is_forearm_phase) {
      // 大臂归零时，小臂已标定，给前馈托举
      publishMotorCommand(i, homing_start_positions_[i], 0.0,
                          idle_hold_torque_[i], homing_hold_kp_, homing_hold_kd_);
    } else {
      // 小臂归零时，大臂保持当前位置
      publishMotorCommand(i, homing_start_positions_[i], 0.0,
                          0.0, homing_hold_kp_, homing_hold_kd_);
    }
  }

  bool all_landed = true;
  for (size_t idx : active) {
    const double travel = std::abs(current_positions_[idx] - homing_start_positions_[idx]);
    const bool moved_enough = (travel >= homing_min_travel_rad_);
    const bool velocity_ok = (std::abs(current_velocities_[idx]) <= homing_velocity_threshold_);
    if (!(moved_enough && velocity_ok)) {
      all_landed = false;
      break;
    }
  }

  if (all_landed) {
    if (!homing_stability_started_) {
      homing_stability_started_ = true;
      homing_stable_since_ = this->now();
    }
    const double stable_s = (this->now() - homing_stable_since_).seconds();
    if (stable_s >= homing_stable_duration_s_) {
      // 标定当前激活轴
      for (size_t idx : active) {
        const double dir = (motor_directions_[idx] == 0) ? 1.0 : static_cast<double>(motor_directions_[idx]);
        zero_offsets_[idx] = raw_positions_[idx] - (homing_ik_offsets_[idx] * dir);
        current_positions_[idx] = homing_ik_offsets_[idx];
        traj_state_[idx].position = current_positions_[idx];
        traj_state_[idx].velocity = 0.0;
      }

      if (is_forearm_phase) {
        // 小臂完成 → 进入大臂阶段
        RCLCPP_INFO(this->get_logger(),
            "小臂归零完成 L2=%.3f R2=%.3f → 进入大臂归零",
            current_positions_[L2], current_positions_[R2]);
        homing_sub_phase_ = 1;
        homing_phase_initialized_ = false;
        homing_stability_started_ = false;
      } else {
        // 大臂完成 → 全部归零完成 → READY
        for (size_t i = 0; i < NUM_MOTORS; ++i) {
          traj_state_[i].position = current_positions_[i];
          traj_state_[i].velocity = 0.0;
        }
        homing_sub_phase_ = 0;
        homing_phase_initialized_ = false;
        homing_stability_started_ = false;
        transitionTo(StrikeState::READY);
        RCLCPP_INFO(this->get_logger(),
            "归零完成 → READY | L1=%.3f L2=%.3f R1=%.3f R2=%.3f",
            current_positions_[L1], current_positions_[L2],
            current_positions_[R1], current_positions_[R2]);
      }
    }
  } else {
    homing_stability_started_ = false;
  }
}

void VolleyballStrikeManager::handleReady()
{
  // READY：保持冻结的目标位置 + 恒定前馈托举
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    if (!has_feedback_[i]) {
      publishMotorCommand(i, 0.0, 0.0, 0.0, 0.0, 0.020);
      continue;
    }
    double kp, kd, tau_ff;
    getImpedanceParams(i, kp, kd, tau_ff);
    tau_ff = idle_hold_torque_[i];
    // 【关键】使用冻结的规划位置，让 Kp 弹簧真正生效！
    publishMotorCommand(i, traj_state_[i].position, 0.0, tau_ff, kp, kd);
  }
}

void VolleyballStrikeManager::handleStrike(double dt)
{
  // STRIKE：从 Home 直冲虚拟穿透目标，全速推进
  updateTrajectory(dt);

  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double kp, kd, tau_ff;
    getImpedanceParams(i, kp, kd, tau_ff);

    // 前馈力矩：基于规划加速度的惯性补偿
    double planned_accel = 0.0;
    if (dt > 1e-6) {
      double err = traj_target_[i] - traj_state_[i].position;
      planned_accel = std::copysign(
          std::min(max_joint_acceleration_, std::abs(err) * 10.0),
          err);
    }
    tau_ff = (strike_tau_ff_scale_ * planned_accel) / TAU_PROTOCOL_SCALE;

    // 恒定前馈托举（替代重力模型）
    tau_ff += idle_hold_torque_[i];

    tau_ff = std::clamp(tau_ff, -torque_limit_, torque_limit_);

    publishMotorCommand(i, traj_state_[i].position, traj_state_[i].velocity, tau_ff, kp, kd);
  }

  // 空间事件触发：FK y 接近击球点 → 进入 IMPACT_MOMENT
  double fk_y = computeFK_Y();
  if (std::abs(fk_y - impact_y_) < impact_y_threshold_) {
    transitionTo(StrikeState::IMPACT_MOMENT);
    RCLCPP_INFO(this->get_logger(),
        "STRIKE → IMPACT_MOMENT | FK_Y=%.3f m | target_Y=%.3f m | 关节速度 [%.1f, %.1f, %.1f, %.1f] rad/s",
        fk_y, impact_y_,
        traj_state_[L1].velocity, traj_state_[L2].velocity,
        traj_state_[R1].velocity, traj_state_[R2].velocity);
    return;
  }

  // 超时保护：2s 内未到达击球区域 → 强制回位
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > 2.0) {
    for (size_t j = 0; j < NUM_MOTORS; ++j) {
      traj_state_[j].position = current_positions_[j];
      traj_state_[j].velocity = 0.0;
    }
    traj_target_ = {homing_ik_offsets_[L1], homing_ik_offsets_[L2],
                    homing_ik_offsets_[R1], homing_ik_offsets_[R2]};
    computeTrajSyncScale();
    transitionTo(StrikeState::SETTLING);
    RCLCPP_WARN(this->get_logger(),
        "STRIKE 超时 2s → SETTLING | FK_Y=%.3f m | target_Y=%.3f m",
        fk_y, impact_y_);
  }
}

void VolleyballStrikeManager::handleImpactMoment()
{
  /*
   * IMPACT_MOMENT —— 纯力矩开环爆破式击球
   *
   * 物理逻辑：
   *   τ_motor = Kp·(p_des - p_act) + Kd·(v_des - v_act) + τ_ff
   *
   *   Kp ≈ 0 → 位置项几乎为零 → 不对抗球的反冲力
   *   Kd ≈ 0 → 阻尼项几乎为零 → 不拖慢击打速度
   *   τ_ff = 最大前馈 → 全部电机输出为纯力矩 → 当"锤子"使
   *
   * 减速器保护原理：
   *   传统高 Kp 方案中，球的反冲在 <1ms 内产生巨大的位置误差，
   *   Kp 乘以大误差 = 瞬时峰值力矩 → 减速器齿面冲击载荷。
   *   纯前馈方案中，力矩恒定且可控（= tau_ff），反冲只改变位置，
   *   不激发力矩尖峰。
   *
   * 4 电机协调：
   *   大臂 (L1, R1) 承担主要击打力矩 → tau_ff_shoulder 较大
   *   小臂 (L2, R2) 辅助 → tau_ff_elbow 较小
   *   所有电机 tau_ff 方向与击球方向一致（沿规划速度方向）
   */
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double kp, kd, tau_ff;
    getImpedanceParams(i, kp, kd, tau_ff);

    // p_des / v_des 保持上一帧的轨迹状态（冻结规划器，不更新）
    // tau_ff 方向与击球前的规划速度方向一致
    double vel_sign = (traj_state_[i].velocity >= 0.0) ? 1.0 : -1.0;
    tau_ff = std::abs(tau_ff) * vel_sign;

    publishMotorCommand(i, traj_state_[i].position, traj_state_[i].velocity, tau_ff, kp, kd);
  }

  // 退出条件：纯空间穿透判定 —— FK y 已越过击球点
  double fk_y = computeFK_Y();
  double overshoot = fk_y - impact_y_;  // 正值 = 已越过击球点

  if (overshoot > impact_overshoot_exit_) {
    // 锁定当前实际位置作为刹车目标
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      follow_lock_positions_[i] = has_feedback_[i] ? current_positions_[i] : traj_state_[i].position;
    }
    transitionTo(StrikeState::FOLLOW_THROUGH);
    RCLCPP_INFO(this->get_logger(),
        "IMPACT_MOMENT → FOLLOW_THROUGH | FK_Y=%.3f m | overshoot=%.3f m",
        fk_y, overshoot);
    return;
  }

  // 硬超时保护：防止永远卡在 IMPACT
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > impact_window_after_s_) {
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      follow_lock_positions_[i] = has_feedback_[i] ? current_positions_[i] : traj_state_[i].position;
    }
    transitionTo(StrikeState::FOLLOW_THROUGH);
    RCLCPP_WARN(this->get_logger(),
        "IMPACT_MOMENT 超时 %.0f ms → FOLLOW_THROUGH | FK_Y=%.3f m",
        elapsed * 1000.0, fk_y);
  }
}

void VolleyballStrikeManager::handleFollowThrough(double dt)
{
  /*
   * FOLLOW_THROUGH —— Kd 线性 Ramp 电子刹车
   *
   * 核心力矩公式（τ_ff = 0）：
   *   τ = Kp·(p_lock - p_act) + Kd(t)·(0 - v_act)
   *
   * p_des = 击球瞬间锁定的实际位置（不是规划位置！）
   *   → Kp 项只在超程时产生温和的恢复力
   *
   * v_des = 0
   *   → Kd 项 = -Kd(t)·v_act，纯被动阻尼
   *
   * Kd(t) 线性爬升：
   *   t=0:    Kd = kd_start  (低阻尼，不产生力矩冲击)
   *   t=100ms: Kd = kd_end   (高阻尼，强力耗散残余动能)
   *
   * 为什么不直接 Kd 拉满？
   *   从 IMPACT 高速（~15 rad/s）突然 Kd=8.0：
   *   瞬态力矩 = 8.0 × 15 = 120 Nm（虚拟值）
   *   虽然电机端会裁剪，但减速器在力矩裁剪边界承受最大冲击。
   *   Ramp 方式让 Kd 从低值爬升，初始刹车力矩 = 1.0 × 15 = 15 Nm（安全）。
   */
  (void)dt;

  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double kp, kd, tau_ff;
    getImpedanceParams(i, kp, kd, tau_ff);  // Kd 内部含 ramp 逻辑
    tau_ff += idle_hold_torque_[i];  // 恒定前馈托举，防止刹车期自由落体
    publishMotorCommand(i, follow_lock_positions_[i], 0.0, tau_ff, kp, kd);
  }

  // 检查是否完全停止
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > follow_ramp_duration_s_ + 0.2) {  // ramp 完成后再等 200ms
    bool all_stopped = true;
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      if (has_feedback_[i] && std::abs(current_velocities_[i]) > 0.5) {
        all_stopped = false;
        break;
      }
    }
    if (all_stopped) {
      // 刹停后进入 SETTLING 缓冲回位，而非直接 READY
      for (size_t j = 0; j < NUM_MOTORS; ++j) {
        traj_state_[j].position = follow_lock_positions_[j];
        traj_state_[j].velocity = 0.0;
      }
      // 回位目标 = home 位（homing_ik_offsets_）
      traj_target_ = {homing_ik_offsets_[L1], homing_ik_offsets_[L2],
                      homing_ik_offsets_[R1], homing_ik_offsets_[R2]};
      computeTrajSyncScale();  // SETTLING：同步协调回位
      transitionTo(StrikeState::SETTLING);
      RCLCPP_INFO(this->get_logger(), "FOLLOW_THROUGH → SETTLING | 开始缓冲回位");
    }
  }

  // 超时保护：无论如何 1s 后也进 SETTLING
  if (elapsed > 1.0) {
    for (size_t j = 0; j < NUM_MOTORS; ++j) {
      traj_state_[j].position = follow_lock_positions_[j];
      traj_state_[j].velocity = 0.0;
    }
    traj_target_ = {homing_ik_offsets_[L1], homing_ik_offsets_[L2],
                    homing_ik_offsets_[R1], homing_ik_offsets_[R2]};
    computeTrajSyncScale();  // SETTLING：同步协调回位
    transitionTo(StrikeState::SETTLING);
    RCLCPP_WARN(this->get_logger(), "FOLLOW_THROUGH 超时 → SETTLING");
  }
}

void VolleyballStrikeManager::handleSettling(double dt)
{
  /*
   * SETTLING —— 缓冲回位
   *
   * 形声轨迹规划器从当前位置平滑回到 home 位（homing_ik_offsets_）。
   * 使用低速（settling_max_velocity_）避免坑洼，
   * 到位后自动进入 READY。
   */

  // 临时压低轨迹规划器速度/加速度上限
  double saved_max_vel = max_joint_velocity_;
  double saved_max_acc = max_joint_acceleration_;
  max_joint_velocity_ = settling_max_velocity_;
  max_joint_acceleration_ = settling_max_acceleration_;
  updateTrajectory(dt);
  max_joint_velocity_ = saved_max_vel;
  max_joint_acceleration_ = saved_max_acc;

  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double kp, kd, tau_ff;
    getImpedanceParams(i, kp, kd, tau_ff);
    tau_ff += idle_hold_torque_[i];
    publishMotorCommand(i, traj_state_[i].position, traj_state_[i].velocity, tau_ff, kp, kd);
  }

  // 检查是否到达 home 位
  bool all_reached = true;
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    if (std::abs(current_positions_[i] - traj_target_[i]) > settling_reach_tol_) {
      all_reached = false;
      break;
    }
  }

  if (all_reached) {
    for (size_t j = 0; j < NUM_MOTORS; ++j) {
      traj_state_[j].position = current_positions_[j];
      traj_state_[j].velocity = 0.0;
    }
    transitionTo(StrikeState::READY);
    RCLCPP_INFO(this->get_logger(), "SETTLING → READY | 回位完成");
  }

  // 超时保护 3s
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed > 3.0) {
    for (size_t j = 0; j < NUM_MOTORS; ++j) {
      traj_state_[j].position = current_positions_[j];
      traj_state_[j].velocity = 0.0;
    }
    transitionTo(StrikeState::READY);
    RCLCPP_WARN(this->get_logger(), "SETTLING 超时 → READY");
  }
}

void VolleyballStrikeManager::handleEStop()
{
  // E_STOP: Kd 线性 ramp，避免瞬时全 Kd 冲击减速器
  double elapsed = (this->now() - state_enter_time_).seconds();
  double t = std::clamp(elapsed / e_stop_ramp_duration_s_, 0.0, 1.0);
  double kd_now = t * e_stop_kd_;  // 从 0 爬升到 e_stop_kd_

  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double p_des = has_feedback_[i] ? current_positions_[i] : 0.0;
    publishMotorCommand(i, p_des, 0.0, 0.0, 0.0, kd_now);
  }
}

// =============================================================
//  击球命令回调（空间事件驱动，无视觉/时间依赖）
// =============================================================

void VolleyballStrikeManager::strikeCommandCallback(
    const motor_control_ros2::msg::StrikeCommand::SharedPtr msg)
{
  // 只在 READY 状态接受新的击球命令
  if (state_ != StrikeState::READY) {
    RCLCPP_WARN(this->get_logger(), "忽略击球命令：当前状态 %s ≠ READY",
                strikeStateToString(state_));
    return;
  }

  // 检查所有电机反馈就绪
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    if (!has_feedback_[i] || !motors_online_[i]) {
      RCLCPP_WARN(this->get_logger(), "拒绝击球命令：电机 %s 未就绪",
                  motor_names_[i].c_str());
      return;
    }
  }

  // 工作空间检查
  if (!isInWorkspace(msg->x_target, msg->y_target)) {
    RCLCPP_WARN(this->get_logger(),
        "拒绝击球命令：目标 (%.3f, %.3f) 超出工作空间",
        msg->x_target, msg->y_target);
    return;
  }

  impact_x_     = msg->x_target;
  impact_y_     = msg->y_target;

  // Step 1：计算击球点 IK
  ik_impact_ = solveIK(impact_x_, impact_y_);
  if (!ik_impact_.valid) {
    RCLCPP_ERROR(this->get_logger(), "击球点 IK 无解！拒绝命令");
    return;
  }

  // Step 2：计算虚拟穿透目标（沿 y 正方向延伸 virtual_overshoot_y_）
  double virt_x = impact_x_;
  double virt_y = impact_y_ + virtual_overshoot_y_;
  ik_virtual_ = solveIK(virt_x, virt_y);
  if (!ik_virtual_.valid) {
    RCLCPP_WARN(this->get_logger(), "虚拟目标 IK 无解，缩减过冲距离重试");
    virt_y = impact_y_ + virtual_overshoot_y_ * 0.5;
    ik_virtual_ = solveIK(virt_x, virt_y);
    if (!ik_virtual_.valid) {
      RCLCPP_ERROR(this->get_logger(), "虚拟目标 IK 仍无解！拒绝命令");
      return;
    }
  }

  // Step 3：初始化轨迹规划器 —— 从 Home 直冲虚拟穿透目标
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    traj_state_[i].position = current_positions_[i];
    traj_state_[i].velocity = 0.0;
  }
  traj_target_ = {ik_virtual_.q_L1, ik_virtual_.q_L2,
                   ik_virtual_.q_R1, ik_virtual_.q_R2};
  computeTrajSyncScale();

  transitionTo(StrikeState::STRIKE);
  RCLCPP_INFO(this->get_logger(),
      "收到击球命令 → STRIKE | 目标 (%.3f, %.3f)"
      " | 击球IK: [L1=%.3f L2=%.3f]"
      " | 虚拟IK: [L1=%.3f L2=%.3f]"
      " | 当前位: [L1=%.3f L2=%.3f]"
      " | FK_Y=%.3f m",
      impact_x_, impact_y_,
      ik_impact_.q_L1, ik_impact_.q_L2,
      ik_virtual_.q_L1, ik_virtual_.q_L2,
      current_positions_[L1], current_positions_[L2],
      computeFK_Y());
}

// =============================================================
//  测试命令回调（无视觉系统时用 ros2 topic pub 手动测试）
// =============================================================

void VolleyballStrikeManager::testCommandCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
  const std::string& cmd = msg->data;

  if (cmd == "home") {
    RCLCPP_INFO(this->get_logger(), "测试命令: 重新归零");
    zero_offsets_.fill(0.0);
    transitionTo(StrikeState::HOMING_FOREARM);

  } else if (cmd == "idle") {
    RCLCPP_INFO(this->get_logger(), "测试命令: 强制进入 READY");
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
      traj_state_[i].position = current_positions_[i];
      traj_state_[i].velocity = 0.0;
    }
    transitionTo(StrikeState::READY);

  } else if (cmd == "estop") {
    RCLCPP_INFO(this->get_logger(), "测试命令: 紧急停止");
    transitionTo(StrikeState::E_STOP);

  } else if (cmd == "strike") {
    // 默认测试击球：拍面中心 (0.25, 0.0), 角度 0°
    if (state_ != StrikeState::READY) {
      RCLCPP_WARN(this->get_logger(),
          "测试击球命令忽略：当前状态 %s ≠ READY",
          strikeStateToString(state_));
      return;
    }
    auto test_msg = std::make_shared<motor_control_ros2::msg::StrikeCommand>();
    test_msg->x_target = 0.25;
    test_msg->y_target = 0.0;
    strikeCommandCallback(test_msg);
    RCLCPP_INFO(this->get_logger(),
        "测试击球: (0.25, 0.0) 空间事件驱动");

  } else {
    RCLCPP_WARN(this->get_logger(),
        "未知测试命令: '%s' (可用: home/idle/estop/strike)", cmd.c_str());
  }
}

// =============================================================
//  电机状态反馈回调
// =============================================================

void VolleyballStrikeManager::motorStateCallback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
  for (size_t i = 0; i < NUM_MOTORS; ++i) {
    if (msg->joint_name == motor_names_[i]) {
      raw_positions_[i]  = static_cast<double>(msg->position);
      raw_velocities_[i] = static_cast<double>(msg->velocity);

      const int dir_raw = motor_directions_[i];
      const double dir = (dir_raw == 0) ? 1.0 : static_cast<double>(dir_raw);
      current_positions_[i]  = (raw_positions_[i] - zero_offsets_[i]) / dir;
      current_velocities_[i] = raw_velocities_[i] / dir;
      motors_online_[i]      = msg->online;
      has_feedback_[i]       = true;
      break;
    }
  }
}

// =============================================================
//  工具函数
// =============================================================

void VolleyballStrikeManager::publishMotorCommand(
    size_t idx, double p_des, double v_des, double tau_ff,
    double kp, double kd)
{
  // 位置误差钳位（逻辑坐标系，安全）
  if (has_feedback_[idx] && max_position_error_ > 0.0) {
    double error = p_des - current_positions_[idx];
    if (std::abs(error) > max_position_error_) {
      p_des = current_positions_[idx] + std::copysign(max_position_error_, error);
    }
  }

  // 方向 + 零位偏移：逻辑坐标 -> 电机原始坐标
  const int dir_raw = motor_directions_[idx];
  const double dir = (dir_raw == 0) ? 1.0 : static_cast<double>(dir_raw);
  p_des = p_des * dir + zero_offsets_[idx];
  v_des = v_des * dir;
  tau_ff *= dir;

  // 力矩限幅
  tau_ff = std::clamp(tau_ff, -torque_limit_, torque_limit_);

  auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
  cmd.header.stamp  = this->now();
  cmd.id            = motor_ids_[idx];
  cmd.device        = motor_devices_[idx];
  cmd.mode          = motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC;
  cmd.position_target = p_des;
  cmd.velocity_target = v_des;
  cmd.torque_ff     = static_cast<float>(tau_ff);
  cmd.kp            = static_cast<float>(kp);
  cmd.kd            = static_cast<float>(kd);
  cmd_pub_->publish(cmd);
}

void VolleyballStrikeManager::transitionTo(StrikeState new_state)
{
  RCLCPP_INFO(this->get_logger(), "状态切换: %s → %s",
              strikeStateToString(state_), strikeStateToString(new_state));
  state_ = new_state;
  state_enter_time_ = this->now();
  if (new_state == StrikeState::HOMING_FOREARM) {
    homing_phase_initialized_ = false;
    homing_stability_started_ = false;
    homing_sub_phase_ = 0;
    homing_phase_start_time_ = this->now();
  }
  publishStatus();
}

bool VolleyballStrikeManager::isInWorkspace(double x, double y) const
{
  return (x >= ws_x_min_ && x <= ws_x_max_ &&
          y >= ws_y_min_ && y <= ws_y_max_);
}

void VolleyballStrikeManager::publishStatus() const
{
  auto msg = std_msgs::msg::String();
  msg.data = strikeStateToString(state_);
  status_pub_->publish(msg);
}

// =============================================================
//  main
// =============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VolleyballStrikeManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
