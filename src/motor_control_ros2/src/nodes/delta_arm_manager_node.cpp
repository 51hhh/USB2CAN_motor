#include "motor_control_ros2/delta_arm_manager_node.hpp"
#include <yaml-cpp/yaml.h>

DeltaArmManager::DeltaArmManager()
    : Node("delta_arm_manager"),
      downward_torque_(0.0),
      landing_timeout_(5.0),
      landing_velocity_threshold_(0.3),
      landing_stable_duration_(0.8),
      landing_kd_(0.05),
      kp_(0.35),
      kd_(0.10),
      max_velocity_(0.8),
      control_frequency_(200.0),
      position_tolerance_(0.05),
      max_position_error_(0.3),
      tracking_error_pause_(0.2),
      state_(State::INIT),
      target_delta_rad_(0.0),
      planned_delta_rad_(0.0),
      current_planned_vel_(0.0),
      max_acceleration_(15.0),
      planner_p_gain_(8.0),
      landing_stability_started_(false),
      ready_published_(false),
      gravity_compensation_torque_(0.0),
      top_idle_timeout_(5.0)
{
  current_positions_.fill(0.0);
  current_velocities_.fill(0.0);
  motors_online_.fill(false);
  has_feedback_.fill(false);
  last_positions_.fill(0.0);
  zero_positions_.fill(0.0);
  motor_names_ = {"arm_delta_motor_1", "arm_delta_motor_2", "arm_delta_motor_3"};
  motor_devices_ = {"", "", ""};
  motor_ids_   = {0, 0, 0};

  // 加载配置文件
  try {
    std::string config_file =
        ament_index_cpp::get_package_share_directory("motor_control_ros2") +
        "/config/arm_config.yaml";
    loadConfig(config_file);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "配置文件加载失败（使用默认值）: %s", e.what());
  }

  // 订阅 ArmTarget 命令
  target_sub_ = this->create_subscription<motor_control_ros2::msg::ArmTarget>(
      "/delta_arm/target", 10,
      std::bind(&DeltaArmManager::armTargetCallback, this, std::placeholders::_1));

  // 订阅 GO8010 电机状态
  state_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&DeltaArmManager::motorStateCallback, this, std::placeholders::_1));

  // 发布 GO8010 电机命令
  cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10);

  // 发布 ready 信号
  ready_pub_ = this->create_publisher<std_msgs::msg::String>("/delta_arm/ready", 10);

  // 创建控制定时器
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DeltaArmManager::controlLoop, this));

  init_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
      "delta_arm_manager 启动，控制频率 %.1f Hz，电机: [%s, %s, %s]",
      control_frequency_,
      motor_names_[0].c_str(), motor_names_[1].c_str(), motor_names_[2].c_str());
  RCLCPP_INFO(this->get_logger(), "状态: INIT → 进入软着陆流程");
}

// ========== 配置加载 ==========

void DeltaArmManager::loadConfig(const std::string& config_file)
{
  YAML::Node cfg = YAML::LoadFile(config_file);

  if (cfg["control_frequency"]) {
    control_frequency_ = cfg["control_frequency"].as<double>();
  }

  auto init = cfg["initialization"];
  if (init) {
    if (init["downward_torque"])          downward_torque_           = init["downward_torque"].as<double>();
    if (init["landing_timeout"])          landing_timeout_           = init["landing_timeout"].as<double>();
    if (init["landing_velocity_threshold"]) landing_velocity_threshold_ = init["landing_velocity_threshold"].as<double>();
    if (init["landing_stable_duration"])  landing_stable_duration_   = init["landing_stable_duration"].as<double>();
    if (init["landing_kd"])              landing_kd_                = init["landing_kd"].as<double>();
    if (init["gravity_compensation_torque"]) gravity_compensation_torque_ = init["gravity_compensation_torque"].as<double>();
  }

  auto pd = cfg["pd"];
  if (pd) {
    if (pd["kp"]) kp_ = pd["kp"].as<double>();
    if (pd["kd"]) kd_ = pd["kd"].as<double>();
  }

  auto mp = cfg["motion_profile"];
  if (mp) {
    if (mp["max_velocity"])     max_velocity_     = mp["max_velocity"].as<double>();
    if (mp["max_acceleration"]) max_acceleration_ = mp["max_acceleration"].as<double>();
    if (mp["planner_p_gain"])   planner_p_gain_   = mp["planner_p_gain"].as<double>();
  }

  auto motors = cfg["motors"];
  if (motors && motors.IsSequence()) {
    for (size_t i = 0; i < 3 && i < motors.size(); ++i) {
      if (motors[i]["name"]) {
        motor_names_[i] = motors[i]["name"].as<std::string>();
      }
      if (motors[i]["device"]) {
        motor_devices_[i] = motors[i]["device"].as<std::string>();
      }
      if (motors[i]["id"]) {
        motor_ids_[i] = motors[i]["id"].as<uint8_t>();
      }
    }
  }

  if (cfg["position_tolerance"]) {
    position_tolerance_ = cfg["position_tolerance"].as<double>();
  }

  auto safety = cfg["safety"];
  if (safety && safety["max_position_error"]) {
    max_position_error_ = safety["max_position_error"].as<double>();
  }

  if (cfg["tracking_error_pause"]) {
    tracking_error_pause_ = cfg["tracking_error_pause"].as<double>();
  }

  if (cfg["top_idle_timeout"]) {
    top_idle_timeout_ = cfg["top_idle_timeout"].as<double>();
  }

  RCLCPP_INFO(this->get_logger(),
      "配置加载完成: kp=%.2f kd=%.2f max_vel=%.2f max_accel=%.1f p_gain=%.1f landing_torque=%.2f gravity_comp=%.3f",
      kp_, kd_, max_velocity_, max_acceleration_, planner_p_gain_, downward_torque_, gravity_compensation_torque_);
}

// ========== 回调 ==========

void DeltaArmManager::armTargetCallback(
    const motor_control_ros2::msg::ArmTarget::SharedPtr msg)
{
  if (!msg->execute) {
    return;
  }

  bool all_feedback_ready = true;
  for (size_t i = 0; i < 3; ++i) {
    if (!has_feedback_[i] || !motors_online_[i]) {
      all_feedback_ready = false;
      break;
    }
  }//判断电机是否在线且反馈就绪，未就绪则拒绝执行命令
  if (!all_feedback_ready) {
    RCLCPP_WARN(this->get_logger(), "收到目标命令但电机反馈未就绪，拒绝执行");
    return;
  }

  if (state_ != State::READY && state_ != State::SOFT_LANDING && state_ != State::INIT && state_ != State::EXECUTE && state_ != State::DESCENDING) {
    RCLCPP_WARN(this->get_logger(),
        "收到目标命令，但当前状态为 %s，拒绝执行（仅 READY/EXECUTE/DESCENDING 状态接受命令）",
        state_ == State::INIT ? "INIT" :
        state_ == State::SOFT_LANDING ? "SOFT_LANDING" : "UNKNOWN");
    return;
  }

  for (size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(msg->target_angles[i])) {
      RCLCPP_WARN(this->get_logger(), "目标角度存在非法值，拒绝执行");
      return;
    }
  }

  // 解耦要求：三电机变化量必须一致
  const double a0 = msg->target_angles[0];
  const double a1 = msg->target_angles[1];
  const double a2 = msg->target_angles[2];
  if (std::abs(a0 - a1) > 1e-6 || std::abs(a0 - a2) > 1e-6) {
    RCLCPP_WARN(this->get_logger(),
        "拒绝执行：target_angles 三路不一致 [%.6f, %.6f, %.6f]，解耦模式要求三路增量相同",
        a0, a1, a2);
    return;
  }

  // 若仍在 INIT/SOFT_LANDING，收到 execute 后立即撤去向下力并切入执行
  if (state_ == State::INIT || state_ == State::SOFT_LANDING) {
    for (size_t i = 0; i < 3; ++i) {
      zero_positions_[i] = current_positions_[i];
    }
    landing_stability_started_ = false;
    RCLCPP_WARN(this->get_logger(),
        "收到执行命令，提前结束软着陆并撤去向下力矩，直接进入 EXECUTE");
  }

  target_delta_rad_ = a0;

  if (state_ == State::EXECUTE) {
    execute_start_time_ = this->now();  // 重置超时计时
    RCLCPP_INFO(this->get_logger(),
        "EXECUTE 内更新目标增量为 %.3f rad（保持连续规划，重置超时）",
        target_delta_rad_);
  } else if (state_ == State::DESCENDING) {
    // DESCENDING 被新命令打断，切回 EXECUTE，保持当前规划位置和速度
    state_ = State::EXECUTE;
    execute_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
        "DESCENDING → EXECUTE: 新目标增量为 %.3f rad（打断下落）",
        target_delta_rad_);
  } else {
    planned_delta_rad_ = 0.0;  // READY/INIT/SOFT_LANDING 进入 EXECUTE 时从零点起步
    current_planned_vel_ = 0.0;  // 从零速开始加速
    execute_start_time_ = this->now();  // 记录 EXECUTE 开始时间
    state_ = State::EXECUTE;
    RCLCPP_INFO(this->get_logger(),
        "READY → EXECUTE: 目标相对增量 %.3f rad（3路一致）",
        target_delta_rad_);
  }
}

void DeltaArmManager::motorStateCallback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
  for (size_t i = 0; i < 3; ++i) {
    if (msg->joint_name == motor_names_[i]) {
      current_positions_[i]  = static_cast<double>(msg->position);
      current_velocities_[i] = static_cast<double>(msg->velocity);
      motors_online_[i]      = msg->online;
      has_feedback_[i]       = true;
      break;
    }
  }
}

// ========== 主控制循环 ==========

void DeltaArmManager::controlLoop()
{
  const double dt = 1.0 / control_frequency_;

  switch (state_) {

    case State::INIT: {
      // 进入软着陆阶段
      state_ = State::SOFT_LANDING;
      landing_stability_started_ = false;
      init_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "INIT → SOFT_LANDING: 施加向下力矩 %.2f Nm + 阻尼 kd=%.3f", downward_torque_, landing_kd_);
      break;
    }

    case State::SOFT_LANDING: {
      // 检查超时
      double elapsed = (this->now() - init_start_time_).seconds();
      if (elapsed > landing_timeout_) {
        RCLCPP_WARN(this->get_logger(),
            "软着陆超时 (%.1f s)，强制进入 READY 状态", landing_timeout_);
        state_ = State::READY;
        publishReady();
        break;
      }

      // 向所有电机施加向下力矩 + 阻尼限速
      // τ = τ_ff + Kd × (0 - ω)：力矩驱动下降，阻尼自动限速
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, 0.0, 0.0, downward_torque_, 0.0, landing_kd_);
      }

      // 检查着陆稳定条件（速度反馈 < 阈值）
      if (allMotorsLanded()) {
        if (!landing_stability_started_) {
          landing_stable_since_ = this->now();
          landing_stability_started_ = true;
        }
        double stable_time = (this->now() - landing_stable_since_).seconds();
        if (stable_time >= landing_stable_duration_) {
          RCLCPP_INFO(this->get_logger(),
              "软着陆完成（稳定 %.2f s），SOFT_LANDING → READY", stable_time);
          // 解耦：将当前物理角锁定为零点，后续控制坐标从 0 rad 开始
          for (size_t i = 0; i < 3; ++i) {
            zero_positions_[i] = current_positions_[i];
          }
          planned_delta_rad_ = 0.0;
          target_delta_rad_ = 0.0;
          state_ = State::READY;
          publishReady();
        }
      } else {
        landing_stability_started_ = false;
      }
      break;
    }

    case State::READY: {
      // 发布 READY 信号（每秒一次，防止订阅者错过）
      if (!ready_published_) {
        publishReady();
      }
      /*  在 READY 阶段保持解耦零点（target_delta=0）+ 重力补偿前馈
        且每次执行完后会主动回到零点    */
      for (size_t i = 0; i < 3; ++i) {
        publishCommand(i, zero_positions_[i], 0.0, 0, kp_, kd_);
      }
      break;
    }

    case State::EXECUTE: {
      // 检查超时：不管是否到达目标，超时即强制下落
      double execute_elapsed = (this->now() - execute_start_time_).seconds();
      if (execute_elapsed > top_idle_timeout_) {
        RCLCPP_INFO(this->get_logger(), "EXECUTE 超时 (%.1f s)，强制进入 DESCENDING", top_idle_timeout_);
        target_delta_rad_ = 0.0;  // 下落目标为零点
        current_planned_vel_ = 0.0;  // 清零速度，重新开始平滑规划
        state_ = State::DESCENDING;
        break;
      }

      // 自适应规划：检查电机是否跟得上规划器
      bool tracking_ok = true;
      for (size_t i = 0; i < 3; ++i) {
        if (has_feedback_[i]) {
          double planned_physical = zero_positions_[i] + planned_delta_rad_;
          double tracking_err = std::abs(planned_physical - current_positions_[i]);
          if (tracking_err > tracking_error_pause_) {
            tracking_ok = false;
            break;
          }
        }
      }

      // 平滑轨迹生成：限加速度 + P减速
      if (tracking_ok) {
        double err = target_delta_rad_ - planned_delta_rad_;
        // P 控制器：距目标越近，期望速度越小
        double target_vel = err * planner_p_gain_;
        target_vel = std::clamp(target_vel, -max_velocity_, max_velocity_);
        // 限加速度
        double max_dv = max_acceleration_ * dt;
        if (target_vel > current_planned_vel_ + max_dv) {
          current_planned_vel_ += max_dv;
        } else if (target_vel < current_planned_vel_ - max_dv) {
          current_planned_vel_ -= max_dv;
        } else {
          current_planned_vel_ = target_vel;
        }
        planned_delta_rad_ += current_planned_vel_ * dt;
      } else {
        current_planned_vel_ = 0.0;  // 暂停→安全清零
      }
      double vel_ff = current_planned_vel_;

      for (size_t i = 0; i < 3; ++i) {
        const double physical_target = zero_positions_[i] + planned_delta_rad_;
        publishCommand(i, physical_target, vel_ff, gravity_compensation_torque_, kp_, kd_);
      }

      if (allMotorsReached()) {
        RCLCPP_INFO(this->get_logger(), "目标到达，EXECUTE → DESCENDING");
        target_delta_rad_ = 0.0;  // 下落目标为零点
        current_planned_vel_ = 0.0;  // 清零速度
        state_ = State::DESCENDING;
      }
      break;
    }

    case State::DESCENDING: {
      // 自适应规划：检查电机是否跟得上规划器
      bool tracking_ok = true;
      for (size_t i = 0; i < 3; ++i) {
        if (has_feedback_[i]) {
          double planned_physical = zero_positions_[i] + planned_delta_rad_;
          double tracking_err = std::abs(planned_physical - current_positions_[i]);
          if (tracking_err > tracking_error_pause_) {
            tracking_ok = false;
            break;
          }
        }
      }

      // 平滑轨迹生成：限加速度 + P减速（目标为零点）
      if (tracking_ok) {
        double err = 0.0 - planned_delta_rad_;  // 目标为零点
        // P 控制器：距目标越近，期望速度越小
        double target_vel = err * planner_p_gain_;
        target_vel = std::clamp(target_vel, -max_velocity_, max_velocity_);
        // 限加速度
        double max_dv = max_acceleration_ * dt;
        if (target_vel > current_planned_vel_ + max_dv) {
          current_planned_vel_ += max_dv;
        } else if (target_vel < current_planned_vel_ - max_dv) {
          current_planned_vel_ -= max_dv;
        } else {
          current_planned_vel_ = target_vel;
        }
        planned_delta_rad_ += current_planned_vel_ * dt;
      } else {
        current_planned_vel_ = 0.0;  // 暂停→安全清零
      }
      double vel_ff = current_planned_vel_;

      for (size_t i = 0; i < 3; ++i) {
        const double physical_target = zero_positions_[i] + planned_delta_rad_;
        publishCommand(i, physical_target, vel_ff, gravity_compensation_torque_, kp_, kd_);
      }

      // 到达零点判定
      if (std::abs(planned_delta_rad_) < position_tolerance_ && std::abs(current_planned_vel_) < 0.01) {
        RCLCPP_INFO(this->get_logger(), "到达零点，DESCENDING → SOFT_LANDING");
        planned_delta_rad_ = 0.0;
        current_planned_vel_ = 0.0;
        target_delta_rad_ = 0.0;
        state_ = State::SOFT_LANDING;
        landing_stability_started_ = false;
        init_start_time_ = this->now();
      }
      break;
    }
  }
}

// ========== 辅助函数 ==========

bool DeltaArmManager::allMotorsLanded() const
{
  // 使用速度反馈判定着陆（速度由电机内部高频计算，不受 ROS 反馈频率影响）
  for (size_t i = 0; i < 3; ++i) {
    if (!motors_online_[i] || !has_feedback_[i]) {
      return false;
    }
    if (std::abs(current_velocities_[i]) >= landing_velocity_threshold_) {
      return false;
    }
  }
  return true;
}

bool DeltaArmManager::allMotorsReached() const
{
  for (size_t i = 0; i < 3; ++i) {
    if (!motors_online_[i] || !has_feedback_[i]) {
      return false;
    }
    const double logical_pos = current_positions_[i] - zero_positions_[i];
    if (std::abs(target_delta_rad_ - logical_pos) > position_tolerance_) {
      return false;
    }
  }
  return true;
}

void DeltaArmManager::publishReady()
{
  auto msg = std_msgs::msg::String();
  msg.data = "READY";
  ready_pub_->publish(msg);
  ready_published_ = true;
  RCLCPP_INFO(this->get_logger(), "发布 /delta_arm/ready: READY");
}

void DeltaArmManager::publishCommand(size_t idx,
    double pos_des, double vel_des, double torque_ff,
    double kp, double kd)
{
  // 位置误差钳位：防止大误差产生过大力矩导致振荡
  double clamped_pos = pos_des;
  if (has_feedback_[idx] && max_position_error_ > 0.0) {
    double error = pos_des - current_positions_[idx];
    if (std::abs(error) > max_position_error_) {
      clamped_pos = current_positions_[idx] + std::copysign(max_position_error_, error);
    }
  }

  
  double clamped_torque_ff = torque_ff;

  auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
  cmd.header.stamp = this->now();
  cmd.id = motor_ids_[idx];  // motor_control_node 通过 id 路由命令
  cmd.device = motor_devices_[idx];  // 多串口同 ID 场景必须指定 device
  cmd.mode = motor_control_ros2::msg::UnitreeGO8010Command::MODE_FOC;
  cmd.position_target = clamped_pos;
  cmd.velocity_target = vel_des;
  cmd.torque_ff       = static_cast<float>(clamped_torque_ff);
  cmd.kp              = static_cast<float>(kp);
  cmd.kd              = static_cast<float>(kd);
  cmd_pub_->publish(cmd);
}

// ========== main ==========

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
