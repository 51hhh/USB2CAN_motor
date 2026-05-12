/**
 * @file parallel_arm_control_node.cpp
 * @brief 平面 RR 串联臂控制节点（双臂版本）
 *
 * 机构等效于标准平面 RR 串联机械臂。
 * 大臂电机控制 θ1（大臂绝对角），小臂电机控制 θ2_rel（小臂相对角）。
 * 平行四边形四连杆保证 θ2_rel = 小臂电机转角（1:1 线性映射）。
 *
 * 订阅：
 *   /strike_command  (StrikeCommand)    — 任务空间(x,y)或关节空间(θ1,θ2_rel)目标
 *
 * 发布：
 *   /unitree_go8010_command (UnitreeGO8010Command) — 电机控制指令
 *
 * 订阅（状态反馈）：
 *   /unitree_go8010_states (UnitreeGO8010State)    — 电机状态
 */

#include <rclcpp/rclcpp.hpp>
#include "motor_control_ros2/parallel_arm_kinematics.hpp"
#include "motor_control_ros2/msg/strike_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include <array>
#include <cmath>
#include <string>

using motor_control::ParallelArmKinematics;
using motor_control_ros2::msg::StrikeCommand;
using motor_control_ros2::msg::UnitreeGO8010Command;
using motor_control_ros2::msg::UnitreeGO8010State;

class ParallelArmControlNode : public rclcpp::Node {
public:
  ParallelArmControlNode()
  : Node("parallel_arm_control_node")
  {
    // ── 参数声明 ──────────────────────────────────
    this->declare_parameter("default_kp", 1.0);
    this->declare_parameter("default_kd", 0.15);
    this->declare_parameter("control_rate", 200.0);
    this->declare_parameter("command_timeout", 0.5);

    // 机构参数
    this->declare_parameter("L1", 0.200);
    this->declare_parameter("L2", 0.221);
    this->declare_parameter("D", 0.3506);

    // 电机 ID 和设备映射
    this->declare_parameter("motor_L1_id", 0);
    this->declare_parameter("motor_L1_device", std::string("/dev/ttyUSB0"));
    this->declare_parameter("motor_L2_id", 1);
    this->declare_parameter("motor_L2_device", std::string("/dev/ttyUSB0"));
    this->declare_parameter("motor_R1_id", 2);
    this->declare_parameter("motor_R1_device", std::string("/dev/ttyUSB1"));
    this->declare_parameter("motor_R2_id", 3);
    this->declare_parameter("motor_R2_device", std::string("/dev/ttyUSB1"));

    // ── 初始化运动学 ─────────────────────────────
    ParallelArmKinematics::Params p;
    p.L1 = this->get_parameter("L1").as_double();
    p.L2 = this->get_parameter("L2").as_double();
    p.D  = this->get_parameter("D").as_double();
    kin_ = ParallelArmKinematics(p);

    default_kp_ = static_cast<float>(this->get_parameter("default_kp").as_double());
    default_kd_ = static_cast<float>(this->get_parameter("default_kd").as_double());
    command_timeout_ = this->get_parameter("command_timeout").as_double();

    // 电机配置
    motor_ids_  = {
      static_cast<int>(this->get_parameter("motor_L1_id").as_int()),
      static_cast<int>(this->get_parameter("motor_L2_id").as_int()),
      static_cast<int>(this->get_parameter("motor_R1_id").as_int()),
      static_cast<int>(this->get_parameter("motor_R2_id").as_int())
    };
    motor_devs_ = {
      this->get_parameter("motor_L1_device").as_string(),
      this->get_parameter("motor_L2_device").as_string(),
      this->get_parameter("motor_R1_device").as_string(),
      this->get_parameter("motor_R2_device").as_string()
    };

    // ── 话题 ──────────────────────────────────────
    cmd_pub_ = this->create_publisher<UnitreeGO8010Command>("unitree_go8010_command", 10);

    cmd_sub_ = this->create_subscription<StrikeCommand>(
      "strike_command", 10,
      std::bind(&ParallelArmControlNode::commandCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<UnitreeGO8010State>(
      "unitree_go8010_states", 10,
      std::bind(&ParallelArmControlNode::stateCallback, this, std::placeholders::_1));

    // ── 控制定时器 ───────────────────────────────
    double rate = this->get_parameter("control_rate").as_double();
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / rate)),
      std::bind(&ParallelArmControlNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "ParallelArmControlNode 已启动：L1=%.3fm L2=%.3fm D=%.3fm kp=%.2f kd=%.2f rate=%.0fHz",
      p.L1, p.L2, p.D, default_kp_, default_kd_, rate);
  }

private:
  // ── 电机索引 ── L1=θ1左, L2=θ2_rel左, R1=θ1右, R2=θ2_rel右
  static constexpr int IDX_L1 = 0;
  static constexpr int IDX_L2 = 1;
  static constexpr int IDX_R1 = 2;
  static constexpr int IDX_R2 = 3;

  void commandCallback(const StrikeCommand::SharedPtr msg) {
    last_cmd_time_ = this->now();
    float kp = (msg->kp > 0.0f) ? msg->kp : default_kp_;
    float kd = (msg->kd > 0.0f) ? msg->kd : default_kd_;

    switch (msg->mode) {
      case StrikeCommand::MODE_IDLE: {
        brakeAll();
        active_ = false;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IDLE: 全部刹车");
        return;
      }

      case StrikeCommand::MODE_TASK_SPACE: {
        // 单臂 IK：(x, y) → (θ1, θ2_rel)
        auto ik_result = kin_.singleArmIK(msg->x, msg->y);
        if (!ik_result) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IK 无解：x=%.4fm y=%.4fm", msg->x, msg->y);
          return;
        }
        if (!kin_.checkLimits(*ik_result)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IK 超出限位：θ1=%.1f° θ2_rel=%.1f°",
            ik_result->theta1 * 180.0 / M_PI, ik_result->theta2_rel * 180.0 / M_PI);
          return;
        }

        // 左右对称（同一组目标角）
        target_angles_ = {
          ik_result->theta1,  ik_result->theta2_rel,
          ik_result->theta1,  ik_result->theta2_rel
        };

        // 速度前馈
        target_vels_.fill(0.0);
        if (std::abs(msg->dx) > 1e-6 || std::abs(msg->dy) > 1e-6) {
          auto vel = kin_.singleArmInverseVelocity(
            ik_result->theta1, ik_result->theta2_rel, msg->dx, msg->dy);
          if (vel) {
            target_vels_ = {(*vel)[0], (*vel)[1], (*vel)[0], (*vel)[1]};
          }
        }

        target_kp_ = kp;
        target_kd_ = kd;
        active_ = true;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "任务空间：x=%.1fmm y=%.1fmm → θ1=%.1f° θ2_rel=%.1f°",
          msg->x * 1000.0, msg->y * 1000.0,
          ik_result->theta1 * 180.0 / M_PI, ik_result->theta2_rel * 180.0 / M_PI);
        break;
      }

      case StrikeCommand::MODE_JOINT_SPACE: {
        double theta1 = msg->theta1;
        double theta2_rel = msg->theta2_rel;

        const auto& pm = kin_.params();
        theta1 = std::clamp(theta1, pm.theta1_min, pm.theta1_max);
        theta2_rel = std::clamp(theta2_rel, pm.theta2_rel_min, pm.theta2_rel_max);

        target_angles_ = {theta1, theta2_rel, theta1, theta2_rel};
        target_vels_.fill(0.0);
        target_kp_ = kp;
        target_kd_ = kd;
        active_ = true;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "关节空间：θ1=%.1f° θ2_rel=%.1f°",
          theta1 * 180.0 / M_PI, theta2_rel * 180.0 / M_PI);
        break;
      }

      default:
        RCLCPP_WARN(this->get_logger(), "未知模式: %d", msg->mode);
        break;
    }
  }

  void stateCallback(const UnitreeGO8010State::SharedPtr msg) {
    for (int i = 0; i < 4; ++i) {
      if (msg->motor_id == motor_ids_[i]) {
        feedback_pos_[i] = msg->position;
        feedback_vel_[i] = msg->velocity;
        motor_online_[i] = msg->online;
        break;
      }
    }
  }

  void controlLoop() {
    if (active_) {
      double elapsed = (this->now() - last_cmd_time_).seconds();
      if (elapsed > command_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "命令超时 (%.2fs)，保持当前位置", elapsed);
      }
    }

    if (!active_) return;

    // 发送四个电机 FOC 命令：L1(θ1_L), L2(θ2_rel_L), R1(θ1_R), R2(θ2_rel_R)
    for (int i = 0; i < 4; ++i) {
      auto cmd = UnitreeGO8010Command();
      cmd.header.stamp = this->now();
      cmd.id = static_cast<uint8_t>(motor_ids_[i]);
      cmd.device = motor_devs_[i];
      cmd.mode = 1;  // FOC
      cmd.position_target = target_angles_[i];
      cmd.velocity_target = target_vels_[i];
      cmd.torque_ff = 0.0f;
      cmd.kp = target_kp_;
      cmd.kd = target_kd_;
      cmd_pub_->publish(cmd);
    }
  }

  void brakeAll() {
    for (int i = 0; i < 4; ++i) {
      auto cmd = UnitreeGO8010Command();
      cmd.id = static_cast<uint8_t>(motor_ids_[i]);
      cmd.device = motor_devs_[i];
      cmd.mode = 0;  // BRAKE
      cmd_pub_->publish(cmd);
    }
  }

  // ── 成员变量 ──────────────────────────────────
  ParallelArmKinematics kin_;
  float default_kp_{1.0f};
  float default_kd_{0.15f};
  double command_timeout_{0.5};

  std::array<int, 4> motor_ids_{};
  std::array<std::string, 4> motor_devs_{};

  bool active_{false};
  std::array<double, 4> target_angles_{};
  std::array<double, 4> target_vels_{};
  float target_kp_{1.0f};
  float target_kd_{0.15f};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  std::array<double, 4> feedback_pos_{};
  std::array<double, 4> feedback_vel_{};
  std::array<bool, 4> motor_online_{};

  rclcpp::Publisher<UnitreeGO8010Command>::SharedPtr cmd_pub_;
  rclcpp::Subscription<StrikeCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<UnitreeGO8010State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParallelArmControlNode>());
  rclcpp::shutdown();
  return 0;
}
