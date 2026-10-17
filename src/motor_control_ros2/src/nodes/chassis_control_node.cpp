#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/steer_wheel_kinematics.hpp"
#include "motor_control_ros2/msg/chassis_state.hpp"
#include "motor_control_ros2/msg/steer_wheel_state.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace motor_control {
namespace msg = motor_control_ros2::msg;

class ChassisControlNode : public rclcpp::Node {
public:
  ChassisControlNode() : Node("chassis_control_node") {
    this->declare_parameter<std::string>("config_file", "");
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") +
                    "/config/chassis_params.yaml";
    }
    loadChassisParams(config_file);

    const auto now = this->now();
    last_cmd_time_ = now;
    last_estop_time_ = now;
    last_control_time_ = now;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&ChassisControlNode::cmdVelCallback, this, std::placeholders::_1));
    motor_state_sub_ = this->create_subscription<msg::DJIMotorState>(
      motor_state_topic_, 32,
      std::bind(&ChassisControlNode::motorStateCallback, this, std::placeholders::_1));
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 10,
      std::bind(&ChassisControlNode::estopCallback, this, std::placeholders::_1));

    motor_cmd_pub_ = this->create_publisher<msg::DJIMotorCommandAdvanced>(motor_command_topic_, 32);
    chassis_state_pub_ = this->create_publisher<msg::ChassisState>(chassis_state_topic_, 10);
    limited_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(limited_cmd_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChassisControlNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "四舵轮控制启动: %.1f Hz, input=%s, limited=%s, estop=%s, watchdog=%.3fs",
      control_frequency_, cmd_vel_topic_.c_str(), limited_cmd_topic_.c_str(),
      estop_topic_.c_str(), command_watchdog_s_);
  }

private:
  struct ModuleConfig {
    std::string id;
    std::string steer_motor;
    std::string drive_motor;
    double steer_offset_deg {0.0};
    int steer_direction {1};
    int drive_direction {1};
  };

  struct BodyCommand {
    double vx {0.0};
    double vy {0.0};
    double wz {0.0};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr input) {
    if (!std::isfinite(input->linear.x) || !std::isfinite(input->linear.y) ||
        !std::isfinite(input->angular.z)) {
      requested_ = {};
      command_received_ = false;
      RCLCPP_ERROR(this->get_logger(), "拒绝非有限底盘命令并执行安全停车");
      return;
    }
    requested_.vx = input->linear.x;
    requested_.vy = input->linear.y;
    requested_.wz = input->angular.z;
    last_cmd_time_ = this->now();
    command_received_ = true;
  }

  void motorStateCallback(const msg::DJIMotorState::SharedPtr state) {
    motor_states_[state->joint_name] = *state;
    motor_state_times_[state->joint_name] = this->now();
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr state) {
    estop_active_ = state->data;
    estop_received_ = true;
    last_estop_time_ = this->now();
  }

  static double clampDelta(double target, double current, double maximum_delta) {
    return current + std::clamp(target - current, -maximum_delta, maximum_delta);
  }

  BodyCommand limitBodyVelocity(const BodyCommand& input) const {
    BodyCommand output = input;
    const double normalized = std::hypot(output.vx / max_vx_m_s_, output.vy / max_vy_m_s_);
    if (normalized > 1.0) {
      output.vx /= normalized;
      output.vy /= normalized;
    }
    output.wz = std::clamp(output.wz, -max_wz_rad_s_, max_wz_rad_s_);
    return output;
  }

  BodyCommand slewBodyVelocity(const BodyCommand& target, double dt) const {
    return {
      clampDelta(target.vx, applied_.vx, max_ax_m_s2_ * dt),
      clampDelta(target.vy, applied_.vy, max_ay_m_s2_ * dt),
      clampDelta(target.wz, applied_.wz, max_awz_rad_s2_ * dt)
    };
  }

  bool stateFresh(const std::string& name, const rclcpp::Time& now) const {
    const auto it = motor_state_times_.find(name);
    return it != motor_state_times_.end() &&
           (now - it->second).seconds() <= motor_state_watchdog_s_;
  }

  bool motorFault(const std::string& name, const rclcpp::Time& now) const {
    const auto it = motor_states_.find(name);
    if (it == motor_states_.end() || !stateFresh(name, now) || !it->second.online) {
      return true;
    }
    return !std::isfinite(it->second.angle) ||
           it->second.temperature >= fault_temperature_c_ ||
           std::abs(static_cast<int>(it->second.current)) >= fault_current_abs_;
  }

  bool allMotorsHealthy(const rclcpp::Time& now) const {
    for (const auto& module : modules_) {
      if (motorFault(module.steer_motor, now) || motorFault(module.drive_motor, now)) {
        return false;
      }
    }
    return true;
  }

  bool allMotorsOnline(const rclcpp::Time& now) const {
    for (const auto& module : modules_) {
      for (const auto& name : {module.steer_motor, module.drive_motor}) {
        const auto it = motor_states_.find(name);
        if (it == motor_states_.end() || !it->second.online || !stateFresh(name, now)) {
          return false;
        }
      }
    }
    return true;
  }

  std::size_t moduleIndex(const ModuleConfig& module) const {
    return static_cast<std::size_t>(&module - modules_.data());
  }

  double measuredSteerDeg(const ModuleConfig& module) const {
    const auto it = motor_states_.find(module.steer_motor);
    if (it == motor_states_.end()) {
      return last_commands_[moduleIndex(module)].angle;
    }
    return SteerWheelKinematics::normalizeAngle(
      (it->second.angle - module.steer_offset_deg) * module.steer_direction);
  }

  double measuredDriveMps(const ModuleConfig& module) const {
    const auto it = motor_states_.find(module.drive_motor);
    if (it == motor_states_.end()) {
      return 0.0;
    }
    return static_cast<double>(it->second.rpm) * (2.0 * M_PI / 60.0) *
           wheel_radius_ * module.drive_direction;
  }

  void controlLoop() {
    const auto now = this->now();
    double dt = (now - last_control_time_).seconds();
    last_control_time_ = now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / control_frequency_;
    }

    command_timed_out_ = !command_received_ ||
      (now - last_cmd_time_).seconds() > command_watchdog_s_;
    const bool estop_timeout = require_estop_heartbeat_ &&
      (!estop_received_ || (now - last_estop_time_).seconds() > estop_watchdog_s_);
    const bool motors_online = allMotorsOnline(now);
    const bool motors_healthy = allMotorsHealthy(now);
    estop_effective_ = estop_active_ || estop_timeout;
    safety_stop_ = command_timed_out_ || estop_effective_ || !motors_healthy;

    if (safety_stop_) {
      applied_ = {};
    } else {
      applied_ = limitBodyVelocity(slewBodyVelocity(limitBodyVelocity(requested_), dt));
    }

    std::array<WheelCommand, 4> commands;
    std::array<bool, 4> steer_saturated {false, false, false, false};
    std::array<bool, 4> drive_saturated {false, false, false, false};
    const bool has_motion = std::hypot(applied_.vx, applied_.vy) > velocity_deadband_ ||
                            std::abs(applied_.wz) > angular_deadband_;

    if (has_motion && !safety_stop_) {
      kinematics_->inverseKinematics(applied_.vx, applied_.vy, applied_.wz,
        commands[0], commands[1], commands[2], commands[3]);
      double peak = 0.0;
      for (const auto& command : commands) peak = std::max(peak, std::abs(command.velocity));
      if (peak > max_wheel_speed_m_s_) {
        const double scale = max_wheel_speed_m_s_ / peak;
        applied_.vx *= scale;
        applied_.vy *= scale;
        applied_.wz *= scale;
        kinematics_->inverseKinematics(applied_.vx, applied_.vy, applied_.wz,
          commands[0], commands[1], commands[2], commands[3]);
        drive_saturated.fill(true);
      }
    } else {
      for (std::size_t i = 0; i < modules_.size(); ++i) {
        commands[i].angle = measuredSteerDeg(modules_[i]);
        commands[i].velocity = 0.0;
        drive_saturated[i] = safety_stop_ &&
          (std::hypot(requested_.vx, requested_.vy) > velocity_deadband_ ||
           std::abs(requested_.wz) > angular_deadband_);
      }
    }

    if (has_motion && !safety_stop_) {
      for (std::size_t i = 0; i < modules_.size(); ++i) {
        const double current = measuredSteerDeg(modules_[i]);
        const double ideal = commands[i].angle;
        double optimized = SteerWheelKinematics::optimizeSteerAngle(
          current, ideal, commands[i].velocity);
        const double target_error = SteerWheelKinematics::angleDifference(optimized, current);
        const double max_step = max_steer_rate_deg_s_ * dt;
        if (std::abs(target_error) > max_step) {
          optimized = SteerWheelKinematics::normalizeAngle(
            current + std::clamp(target_error, -max_step, max_step));
          steer_saturated[i] = true;
        }
        const double alignment_error = std::abs(target_error);
        if (cosine_compensation_) {
          const double cosine = std::max(0.0, std::cos(alignment_error * M_PI / 180.0));
          commands[i].velocity *= cosine;
          drive_saturated[i] = drive_saturated[i] || cosine < 0.999;
        }
        if (std::abs(target_error) > steer_alignment_stop_deg_) {
          commands[i].velocity = 0.0;
          drive_saturated[i] = true;
        }
        commands[i].angle = optimized;
      }
    }

    last_commands_ = commands;
    publishMotorCommands(commands, now);
    publishLimitedCommand();
    publishChassisState(commands, steer_saturated, drive_saturated,
      motors_online, motors_healthy, now);

    if (safety_stop_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "底盘安全停车: command_timeout=%s estop=%s all_motors_healthy=%s",
        command_timed_out_ ? "true" : "false", estop_effective_ ? "true" : "false",
        motors_healthy ? "true" : "false");
    }
  }

  void publishMotorCommands(const std::array<WheelCommand, 4>& commands,
                            const rclcpp::Time& stamp) {
    for (std::size_t i = 0; i < modules_.size(); ++i) {
      const auto& module = modules_[i];
      msg::DJIMotorCommandAdvanced steer;
      steer.header.stamp = stamp;
      steer.joint_name = module.steer_motor;
      steer.mode = msg::DJIMotorCommandAdvanced::MODE_POSITION;
      const double encoder_deg = commands[i].angle * module.steer_direction +
                                 module.steer_offset_deg;
      steer.position_target = encoder_deg * M_PI / 180.0;
      motor_cmd_pub_->publish(steer);

      msg::DJIMotorCommandAdvanced drive;
      drive.header.stamp = stamp;
      drive.joint_name = module.drive_motor;
      drive.mode = msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
      drive.velocity_target = commands[i].velocity / wheel_radius_ * module.drive_direction;
      motor_cmd_pub_->publish(drive);
    }
  }

  void publishLimitedCommand() {
    geometry_msgs::msg::Twist output;
    output.linear.x = applied_.vx;
    output.linear.y = applied_.vy;
    output.angular.z = applied_.wz;
    limited_cmd_pub_->publish(output);
  }

  void fillWheelState(msg::SteerWheelState& output, const ModuleConfig& module,
                      const WheelCommand& command, bool steer_saturated,
                      bool drive_saturated, const rclcpp::Time& now) const {
    output.steer_motor_name = module.steer_motor;
    output.drive_motor_name = module.drive_motor;
    output.steer_angle = measuredSteerDeg(module);
    output.steer_angle_target = command.angle;
    output.drive_velocity = measuredDriveMps(module);
    output.drive_velocity_target = command.velocity;
    const auto steer = motor_states_.find(module.steer_motor);
    const auto drive = motor_states_.find(module.drive_motor);
    output.steer_online = steer != motor_states_.end() && steer->second.online &&
                          stateFresh(module.steer_motor, now);
    output.drive_online = drive != motor_states_.end() && drive->second.online &&
                          stateFresh(module.drive_motor, now);
    output.steer_saturated = steer_saturated;
    output.drive_saturated = drive_saturated;
    output.steer_fault = motorFault(module.steer_motor, now);
    output.drive_fault = motorFault(module.drive_motor, now);
    if (steer != motor_states_.end()) {
      output.steer_current = steer->second.current;
      output.steer_temperature = steer->second.temperature;
    }
    if (drive != motor_states_.end()) {
      output.drive_current = drive->second.current;
      output.drive_temperature = drive->second.temperature;
    }
  }

  void publishChassisState(const std::array<WheelCommand, 4>& commands,
                           const std::array<bool, 4>& steer_saturated,
                           const std::array<bool, 4>& drive_saturated,
                           bool motors_online, bool motors_healthy,
                           const rclcpp::Time& now) {
    msg::ChassisState output;
    output.header.stamp = now;
    output.header.frame_id = base_frame_;
    output.requested_linear_x = requested_.vx;
    output.requested_linear_y = requested_.vy;
    output.requested_angular_z = requested_.wz;
    output.command_linear_x = applied_.vx;
    output.command_linear_y = applied_.vy;
    output.command_angular_z = applied_.wz;
    output.all_motors_online = motors_online;
    output.all_motors_healthy = motors_healthy;
    output.command_timed_out = command_timed_out_;
    output.estop_active = estop_effective_;
    output.safety_stop = safety_stop_;

    std::array<WheelCommand, 4> actual;
    if (motors_healthy) {
      for (std::size_t i = 0; i < modules_.size(); ++i) {
        actual[i].angle = measuredSteerDeg(modules_[i]);
        actual[i].velocity = measuredDriveMps(modules_[i]);
      }
      kinematics_->forwardKinematics(actual[0], actual[1], actual[2], actual[3],
        output.linear_x, output.linear_y, output.angular_z);
    }
    fillWheelState(output.fl_wheel, modules_[0], commands[0],
      steer_saturated[0], drive_saturated[0], now);
    fillWheelState(output.fr_wheel, modules_[1], commands[1],
      steer_saturated[1], drive_saturated[1], now);
    fillWheelState(output.rl_wheel, modules_[2], commands[2],
      steer_saturated[2], drive_saturated[2], now);
    fillWheelState(output.rr_wheel, modules_[3], commands[3],
      steer_saturated[3], drive_saturated[3], now);
    chassis_state_pub_->publish(output);
  }

  void loadChassisParams(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["chassis_control_node"] || !root["chassis_control_node"]["ros__parameters"]) {
      throw std::runtime_error("缺少 chassis_control_node/ros__parameters");
    }
    const auto p = root["chassis_control_node"]["ros__parameters"];
    auto required_double = [&p](const char* key) {
      if (!p[key]) throw std::runtime_error(std::string("缺少参数: ") + key);
      const double value = p[key].as<double>();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string("参数必须为正有限数: ") + key);
      }
      return value;
    };
    auto string_or = [&p](const char* key, const std::string& fallback) {
      return p[key] ? p[key].as<std::string>() : fallback;
    };
    auto double_or = [&p](const char* key, double fallback) {
      return p[key] ? p[key].as<double>() : fallback;
    };
    auto bool_or = [&p](const char* key, bool fallback) {
      return p[key] ? p[key].as<bool>() : fallback;
    };

    control_frequency_ = required_double("control_frequency");
    wheel_base_x_ = required_double("wheel_base_x");
    wheel_base_y_ = required_double("wheel_base_y");
    wheel_radius_ = required_double("wheel_radius");
    const double legacy_linear = required_double("max_linear_velocity");
    max_vx_m_s_ = double_or("max_vx_m_s", legacy_linear);
    max_vy_m_s_ = double_or("max_vy_m_s", legacy_linear);
    max_wz_rad_s_ = p["max_wz_rad_s"] ? p["max_wz_rad_s"].as<double>() :
                        required_double("max_angular_velocity");
    max_wheel_speed_m_s_ = double_or("max_wheel_speed_m_s", legacy_linear);
    max_ax_m_s2_ = required_double("max_ax_m_s2");
    max_ay_m_s2_ = required_double("max_ay_m_s2");
    max_awz_rad_s2_ = required_double("max_awz_rad_s2");
    max_steer_rate_deg_s_ = required_double("max_steer_rate_deg_s");
    command_watchdog_s_ = required_double("command_watchdog_s");
    motor_state_watchdog_s_ = required_double("motor_state_watchdog_s");
    estop_watchdog_s_ = required_double("estop_watchdog_s");
    steer_alignment_stop_deg_ = required_double("steer_alignment_stop_deg");
    velocity_deadband_ = double_or("velocity_deadband_m_s", 0.01);
    angular_deadband_ = double_or("angular_deadband_rad_s", 0.01);
    fault_temperature_c_ = static_cast<int>(double_or("fault_temperature_c", 80.0));
    fault_current_abs_ = static_cast<int>(double_or("fault_current_abs", 15000.0));
    cosine_compensation_ = bool_or("cosine_compensation", true);
    require_estop_heartbeat_ = bool_or("require_estop_heartbeat", true);
    for (const auto value : {max_vx_m_s_, max_vy_m_s_, max_wz_rad_s_,
                             max_wheel_speed_m_s_, max_ax_m_s2_, max_ay_m_s2_,
                             max_awz_rad_s2_, max_steer_rate_deg_s_, command_watchdog_s_,
                             motor_state_watchdog_s_, estop_watchdog_s_,
                             steer_alignment_stop_deg_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error("速度、加速度、舵速和看门狗参数必须为正有限数");
      }
    }
    if (steer_alignment_stop_deg_ > 90.0 || fault_temperature_c_ <= 0 ||
        fault_current_abs_ <= 0 || !std::isfinite(velocity_deadband_) ||
        !std::isfinite(angular_deadband_) || velocity_deadband_ < 0.0 ||
        angular_deadband_ < 0.0) {
      throw std::runtime_error("对准、故障或死区参数非法");
    }

    cmd_vel_topic_ = string_or("cmd_vel_topic", "/cmd_vel");
    limited_cmd_topic_ = string_or("limited_cmd_topic", "/chassis/limited_cmd");
    estop_topic_ = string_or("estop_topic", "/emergency_stop");
    motor_state_topic_ = string_or("motor_state_topic", "/dji_motor_states");
    motor_command_topic_ = string_or("motor_command_topic", "/dji_motor_command_advanced");
    chassis_state_topic_ = string_or("chassis_state_topic", "/chassis_state");
    base_frame_ = string_or("base_frame", "base_link");

    const std::array<std::string, 4> prefixes {"fl", "fr", "rl", "rr"};
    const std::array<std::string, 4> ids {"front_left", "front_right", "rear_left", "rear_right"};
    for (std::size_t i = 0; i < modules_.size(); ++i) {
      const auto& prefix = prefixes[i];
      modules_[i].id = ids[i];
      modules_[i].steer_motor = p[prefix + "_steer_motor"].as<std::string>();
      modules_[i].drive_motor = p[prefix + "_drive_motor"].as<std::string>();
      modules_[i].steer_offset_deg = p[prefix + "_steer_offset"].as<double>();
      modules_[i].steer_direction = p[prefix + "_steer_direction"].as<int>();
      modules_[i].drive_direction = p[prefix + "_drive_direction"].as<int>();
      if (std::abs(modules_[i].steer_direction) != 1 ||
          std::abs(modules_[i].drive_direction) != 1) {
        throw std::runtime_error(prefix + " direction 必须为 +/-1");
      }
    }
    kinematics_ = std::make_unique<SteerWheelKinematics>(
      wheel_base_x_, wheel_base_y_, wheel_radius_);
  }

  std::unique_ptr<SteerWheelKinematics> kinematics_;
  std::array<ModuleConfig, 4> modules_;
  std::array<WheelCommand, 4> last_commands_;
  std::map<std::string, msg::DJIMotorState> motor_states_;
  std::map<std::string, rclcpp::Time> motor_state_times_;
  double control_frequency_ {100.0};
  double wheel_base_x_ {0.0};
  double wheel_base_y_ {0.0};
  double wheel_radius_ {0.0};
  double max_vx_m_s_ {0.0};
  double max_vy_m_s_ {0.0};
  double max_wz_rad_s_ {0.0};
  double max_wheel_speed_m_s_ {0.0};
  double max_ax_m_s2_ {0.0};
  double max_ay_m_s2_ {0.0};
  double max_awz_rad_s2_ {0.0};
  double max_steer_rate_deg_s_ {0.0};
  double command_watchdog_s_ {0.2};
  double motor_state_watchdog_s_ {0.2};
  double estop_watchdog_s_ {0.2};
  double steer_alignment_stop_deg_ {60.0};
  double velocity_deadband_ {0.01};
  double angular_deadband_ {0.01};
  int fault_temperature_c_ {80};
  int fault_current_abs_ {15000};
  bool cosine_compensation_ {true};
  bool require_estop_heartbeat_ {true};
  std::string cmd_vel_topic_;
  std::string limited_cmd_topic_;
  std::string estop_topic_;
  std::string motor_state_topic_;
  std::string motor_command_topic_;
  std::string chassis_state_topic_;
  std::string base_frame_ {"base_link"};
  BodyCommand requested_;
  BodyCommand applied_;
  bool command_received_ {false};
  bool estop_received_ {false};
  bool estop_active_ {true};
  bool estop_effective_ {true};
  bool command_timed_out_ {true};
  bool safety_stop_ {true};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_estop_time_;
  rclcpp::Time last_control_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<msg::DJIMotorState>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
  rclcpp::Publisher<msg::ChassisState>::SharedPtr chassis_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr limited_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_control::ChassisControlNode>());
  rclcpp::shutdown();
  return 0;
}
