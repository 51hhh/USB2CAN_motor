#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/omni_wheel_kinematics.hpp"
#include "motor_control_ros2/pid_controller.hpp"

namespace motor_control {

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

PIDParams loadPidParams(const YAML::Node& node, const PIDParams& defaults) {
  PIDParams params = defaults;
  if (node["kp"]) params.kp = node["kp"].as<double>();
  if (node["ki"]) params.ki = node["ki"].as<double>();
  if (node["kd"]) params.kd = node["kd"].as<double>();
  if (node["i_max"]) params.i_max = node["i_max"].as<double>();
  if (node["out_max"]) params.out_max = node["out_max"].as<double>();
  if (node["dead_zone"]) params.dead_zone = node["dead_zone"].as<double>();
  return params;
}

}  // namespace

enum class ChassisMode {
  VELOCITY = 0,
  POSITION = 1
};

enum class VelocityControlMode {
  OPEN_LOOP = 0,
  CLOSED_LOOP = 1
};

class OmniChassisControlNode : public rclcpp::Node {
public:
  OmniChassisControlNode() : Node("omni_chassis_control_node") {
    this->declare_parameter("config_file", "");

    const std::string config_file = resolveConfigFile();
    loadConfig(config_file);
    const std::string initial_mode =
      this->declare_parameter<std::string>("control_mode", modeToString(mode_));
    if (!setControlMode(initial_mode)) {
      mode_ = ChassisMode::VELOCITY;
    }

    kinematics_ = std::make_unique<OmniWheelKinematics>(
      wheel_base_x_, wheel_base_y_, wheel_radius_, install_angle_);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 20,
      std::bind(&OmniChassisControlNode::cmdVelCallback, this, std::placeholders::_1));

    cmd_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
      cmd_pose_topic_, 20,
      std::bind(&OmniChassisControlNode::cmdPoseCallback, this, std::placeholders::_1));

    auto odom_qos = rclcpp::SensorDataQoS();
    odom_qos.keep_last(50);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      std::bind(&OmniChassisControlNode::odomCallback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 20,
      std::bind(&OmniChassisControlNode::estopCallback, this, std::placeholders::_1));

    motor_cmd_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
      "/dji_motor_command_advanced", 20);

    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OmniChassisControlNode::onSetParameters, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&OmniChassisControlNode::controlLoop, this));

    last_cmd_time_ = this->now();
    last_pose_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "omni_chassis_control_node 启动: mode=%s velocity_control=%s cmd_vel=%s cmd_pose=%s odom=%s estop=%s",
      mode_ == ChassisMode::VELOCITY ? "velocity" : "position",
      velocity_control_mode_ == VelocityControlMode::OPEN_LOOP ? "open_loop" : "closed_loop",
      cmd_vel_topic_.c_str(), cmd_pose_topic_.c_str(), odom_topic_.c_str(), estop_topic_.c_str());
  }

private:
  std::string resolveConfigFile() const {
    const std::string config_file = this->get_parameter("config_file").as_string();
    if (!config_file.empty()) {
      return config_file;
    }
    return ament_index_cpp::get_package_share_directory("motor_control_ros2") +
      "/config/omni_chassis_params.yaml";
  }

  void loadConfig(const std::string& config_file) {
    const YAML::Node root = YAML::LoadFile(config_file);
    YAML::Node params = root;
    if (root["omni_chassis_control_node"] && root["omni_chassis_control_node"]["ros__parameters"]) {
      params = root["omni_chassis_control_node"]["ros__parameters"];
    }

    auto loadString = [&params](const char* key, std::string& value) {
      if (params[key]) value = params[key].as<std::string>();
    };
    auto loadDouble = [&params](const char* key, double& value) {
      if (params[key]) value = params[key].as<double>();
    };
    auto loadInt = [&params](const char* key, int& value) {
      if (params[key]) value = params[key].as<int>();
    };
    auto loadBool = [&params](const char* key, bool& value) {
      if (params[key]) value = params[key].as<bool>();
    };

    loadDouble("control_frequency", control_frequency_);
    loadDouble("wheel_base_x", wheel_base_x_);
    loadDouble("wheel_base_y", wheel_base_y_);
    loadDouble("wheel_radius", wheel_radius_);
    loadDouble("install_angle", install_angle_);
    loadDouble("max_linear_velocity", max_linear_velocity_);
    loadDouble("max_angular_velocity", max_angular_velocity_);
    loadDouble("max_wheel_linear_velocity", max_wheel_linear_velocity_);
    loadDouble("cmd_timeout", cmd_timeout_);
    loadDouble("zero_target_deadband", zero_target_deadband_);
    loadDouble("yaw_feedback_sign", yaw_feedback_sign_);
    loadBool("yaw_hold_enabled", yaw_hold_enabled_);
    loadDouble("yaw_hold_kp", yaw_hold_kp_);
    loadDouble("yaw_hold_max_angular_velocity", yaw_hold_max_angular_velocity_);
    loadDouble("yaw_hold_deadband", yaw_hold_deadband_);
    loadDouble("yaw_hold_release_angular_velocity", yaw_hold_release_angular_velocity_);
    loadDouble("position_timeout", position_timeout_);
    loadDouble("goal_tolerance_xy", goal_tolerance_xy_);
    loadDouble("goal_tolerance_yaw", goal_tolerance_yaw_);

    loadString("cmd_vel_topic", cmd_vel_topic_);
    loadString("cmd_pose_topic", cmd_pose_topic_);
    loadString("odom_topic", odom_topic_);
    loadString("estop_topic", estop_topic_);
    std::string velocity_control_mode = "open_loop";
    loadString("velocity_control_mode", velocity_control_mode);
    velocity_control_mode_ =
      velocity_control_mode == "closed_loop" ? VelocityControlMode::CLOSED_LOOP : VelocityControlMode::OPEN_LOOP;

    loadString("fl_motor", motor_names_[0]);
    loadString("fr_motor", motor_names_[1]);
    loadString("rl_motor", motor_names_[2]);
    loadString("rr_motor", motor_names_[3]);

    loadInt("fl_drive_direction", drive_directions_[0]);
    loadInt("fr_drive_direction", drive_directions_[1]);
    loadInt("rl_drive_direction", drive_directions_[2]);
    loadInt("rr_drive_direction", drive_directions_[3]);

    if (params["velocity_pid_x"]) {
      velocity_pid_x_.setParams(loadPidParams(params["velocity_pid_x"], velocity_pid_x_.getParams()));
    }
    if (params["velocity_pid_y"]) {
      velocity_pid_y_.setParams(loadPidParams(params["velocity_pid_y"], velocity_pid_y_.getParams()));
    }
    if (params["velocity_pid_yaw"]) {
      velocity_pid_yaw_.setParams(loadPidParams(params["velocity_pid_yaw"], velocity_pid_yaw_.getParams()));
    }
    if (params["position_pid_x"]) {
      position_pid_x_.setParams(loadPidParams(params["position_pid_x"], position_pid_x_.getParams()));
    }
    if (params["position_pid_y"]) {
      position_pid_y_.setParams(loadPidParams(params["position_pid_y"], position_pid_y_.getParams()));
    }
    if (params["position_pid_yaw"]) {
      position_pid_yaw_.setParams(loadPidParams(params["position_pid_yaw"], position_pid_yaw_.getParams()));
    }

    std::string default_mode = "velocity";
    loadString("default_mode", default_mode);
    mode_ = default_mode == "position" ? ChassisMode::POSITION : ChassisMode::VELOCITY;

    yaw_hold_kp_ = std::abs(yaw_hold_kp_);
    yaw_hold_max_angular_velocity_ = std::abs(yaw_hold_max_angular_velocity_);
    yaw_hold_deadband_ = std::max(0.0, yaw_hold_deadband_);
    yaw_hold_release_angular_velocity_ = std::max(0.0, yaw_hold_release_angular_velocity_);
  }

  std::string modeToString(ChassisMode mode) const {
    return mode == ChassisMode::POSITION ? "position" : "velocity";
  }

  bool setControlMode(const std::string& value) {
    if (value == "velocity") {
      mode_ = ChassisMode::VELOCITY;
      resetControllers();
      return true;
    }
    if (value == "position") {
      mode_ = ChassisMode::POSITION;
      resetControllers();
      return true;
    }
    return false;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& parameter : parameters) {
      if (parameter.get_name() == "control_mode") {
        const auto value = parameter.as_string();
        if (!setControlMode(value)) {
          result.successful = false;
          result.reason = "control_mode 只支持 velocity 或 position";
          return result;
        }
      }
    }

    return result;
  }

  void resetControllers() {
    velocity_pid_x_.reset();
    velocity_pid_y_.reset();
    velocity_pid_yaw_.reset();
    position_pid_x_.reset();
    position_pid_y_.reset();
    position_pid_yaw_.reset();
    has_yaw_hold_reference_ = false;
    last_yaw_hold_error_ = 0.0;
  }

  double applyYawHold(double requested_wz, bool fresh_odom) {
    if (!yaw_hold_enabled_ || !fresh_odom) {
      has_yaw_hold_reference_ = false;
      last_yaw_hold_error_ = 0.0;
      return requested_wz;
    }

    if (std::abs(requested_wz) > yaw_hold_release_angular_velocity_) {
      yaw_hold_reference_ = latest_yaw_;
      has_yaw_hold_reference_ = true;
      last_yaw_hold_error_ = 0.0;
      return requested_wz;
    }

    if (!has_yaw_hold_reference_) {
      yaw_hold_reference_ = latest_yaw_;
      has_yaw_hold_reference_ = true;
    }

    const double yaw_error = normalizeAngle(yaw_hold_reference_ - latest_yaw_);
    last_yaw_hold_error_ = yaw_error;
    if (std::abs(yaw_error) <= yaw_hold_deadband_) {
      return 0.0;
    }

    const double desired_odom_wz = yaw_hold_kp_ * yaw_error;
    return std::clamp(
      yaw_feedback_sign_ * desired_odom_wz,
      -yaw_hold_max_angular_velocity_,
      yaw_hold_max_angular_velocity_);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_target_.linear.x = std::clamp(
      msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    velocity_target_.linear.y = std::clamp(
      msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
    velocity_target_.angular.z = std::clamp(
      msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    last_cmd_time_ = this->now();
  }

  void cmdPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    pose_target_ = *msg;
    pose_target_.theta = normalizeAngle(pose_target_.theta);
    last_pose_cmd_time_ = this->now();
    has_pose_target_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = *msg;
    latest_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    last_odom_time_ = this->now();
    has_odom_ = true;
  }

  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) {
      return;
    }
    estop_active_ = msg->data;
    if (estop_active_) {
      resetControllers();
    }
  }

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Twist computeVelocityModeCommand() {
    geometry_msgs::msg::Twist cmd;
    const auto now = this->now();
    if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
      velocity_pid_x_.reset();
      velocity_pid_y_.reset();
      velocity_pid_yaw_.reset();
      has_yaw_hold_reference_ = false;
      last_yaw_hold_error_ = 0.0;
      return cmd;
    }

    const bool fresh_odom =
      has_odom_ && ((now - last_odom_time_).seconds() <= cmd_timeout_);

    geometry_msgs::msg::Twist target = velocity_target_;
    if (isZeroVelocityTarget(target)) {
      velocity_pid_x_.reset();
      velocity_pid_y_.reset();
      velocity_pid_yaw_.reset();
      has_yaw_hold_reference_ = false;
      last_yaw_hold_error_ = 0.0;
      return cmd;
    }

    target.angular.z = applyYawHold(target.angular.z, fresh_odom);

    if (isZeroVelocityTarget(target)) {
      velocity_pid_x_.reset();
      velocity_pid_y_.reset();
      velocity_pid_yaw_.reset();
      return cmd;
    }

    if (velocity_control_mode_ == VelocityControlMode::OPEN_LOOP) {
      cmd = target;
      clampCommand(cmd);
      velocity_pid_x_.reset();
      velocity_pid_y_.reset();
      velocity_pid_yaw_.reset();
      return cmd;
    }

    if (!fresh_odom) {
      velocity_pid_x_.reset();
      velocity_pid_y_.reset();
      velocity_pid_yaw_.reset();
      return cmd;
    }

    const auto& twist = latest_odom_.twist.twist;
    const double feedback_right = -twist.linear.y;
    const double feedback_forward = twist.linear.x;
    const double feedback_yaw = yaw_feedback_sign_ * twist.angular.z;
    cmd = target;
    cmd.linear.x += velocity_pid_x_.calculate(target.linear.x, feedback_right);
    cmd.linear.y += velocity_pid_y_.calculate(target.linear.y, feedback_forward);
    cmd.angular.z += velocity_pid_yaw_.calculate(target.angular.z, feedback_yaw);
    clampCommand(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "速度闭环: target(right=%.2f forward=%.2f yaw=%.2f) feedback(right=%.2f forward=%.2f yaw=%.2f) yaw_hold(err=%.3f ref=%.3f now=%.3f) cmd(right=%.2f forward=%.2f yaw=%.2f)",
      target.linear.x, target.linear.y, target.angular.z,
      feedback_right, feedback_forward, feedback_yaw,
      last_yaw_hold_error_, yaw_hold_reference_, latest_yaw_,
      cmd.linear.x, cmd.linear.y, cmd.angular.z);
    return cmd;
  }

  geometry_msgs::msg::Twist computePositionModeCommand() {
    geometry_msgs::msg::Twist cmd;
    const auto now = this->now();
    if (!has_pose_target_ || !has_odom_) {
      return cmd;
    }
    if ((now - last_pose_cmd_time_).seconds() > position_timeout_) {
      return cmd;
    }
    if ((now - last_odom_time_).seconds() > cmd_timeout_) {
      return cmd;
    }

    const double current_x = latest_odom_.pose.pose.position.x;
    const double current_y = latest_odom_.pose.pose.position.y;
    const double current_yaw = latest_yaw_;

    const double error_x_world = pose_target_.x - current_x;
    const double error_y_world = pose_target_.y - current_y;
    const double error_yaw = normalizeAngle(pose_target_.theta - current_yaw);

    if (std::hypot(error_x_world, error_y_world) < goal_tolerance_xy_ &&
        std::abs(error_yaw) < goal_tolerance_yaw_) {
      return cmd;
    }

    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);
    const double error_forward_body = cos_yaw * error_x_world + sin_yaw * error_y_world;
    const double error_left_body = -sin_yaw * error_x_world + cos_yaw * error_y_world;

    const double target_forward = position_pid_x_.calculate(error_forward_body, 0.0);
    const double target_right = -position_pid_y_.calculate(error_left_body, 0.0);
    const double target_wz = position_pid_yaw_.calculate(error_yaw, 0.0);

    const auto& twist = latest_odom_.twist.twist;
    const double feedback_right = -twist.linear.y;
    const double feedback_forward = twist.linear.x;
    const double feedback_yaw = yaw_feedback_sign_ * twist.angular.z;
    cmd.linear.x = velocity_pid_x_.calculate(target_right, feedback_right);
    cmd.linear.y = velocity_pid_y_.calculate(target_forward, feedback_forward);
    cmd.angular.z = velocity_pid_yaw_.calculate(target_wz, feedback_yaw);
    clampCommand(cmd);
    return cmd;
  }

  void clampCommand(geometry_msgs::msg::Twist& cmd) const {
    cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_velocity_, max_linear_velocity_);
    cmd.linear.y = std::clamp(cmd.linear.y, -max_linear_velocity_, max_linear_velocity_);
    cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_velocity_, max_angular_velocity_);
  }

  bool isZeroVelocityTarget(const geometry_msgs::msg::Twist& target) const {
    return std::abs(target.linear.x) <= zero_target_deadband_ &&
      std::abs(target.linear.y) <= zero_target_deadband_ &&
      std::abs(target.angular.z) <= zero_target_deadband_;
  }

  void controlLoop() {
    if (estop_active_) {
      resetControllers();
      publishMotorCommands({0.0, 0.0, 0.0, 0.0}, this->now());
      return;
    }

    geometry_msgs::msg::Twist chassis_cmd;
    if (mode_ == ChassisMode::POSITION) {
      chassis_cmd = computePositionModeCommand();
    } else {
      chassis_cmd = computeVelocityModeCommand();
    }

    auto wheel_velocities = kinematics_->inverseKinematics(
      chassis_cmd.linear.x, chassis_cmd.linear.y, chassis_cmd.angular.z);
    scaleWheelVelocities(wheel_velocities);
    publishMotorCommands(wheel_velocities, this->now());
  }

  void scaleWheelVelocities(std::array<double, 4>& wheel_velocities) {
    if (max_wheel_linear_velocity_ <= 0.0) {
      return;
    }

    double max_abs = 0.0;
    for (const double velocity : wheel_velocities) {
      max_abs = std::max(max_abs, std::abs(velocity));
    }
    if (max_abs <= max_wheel_linear_velocity_) {
      return;
    }

    const double scale = max_wheel_linear_velocity_ / max_abs;
    for (double& velocity : wheel_velocities) {
      velocity *= scale;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "轮速统一缩放: max=%.2f limit=%.2f scale=%.2f",
      max_abs, max_wheel_linear_velocity_, scale);
  }

  void publishMotorCommands(
    const std::array<double, 4>& wheel_velocities,
    const rclcpp::Time& stamp)
  {
    for (size_t i = 0; i < wheel_velocities.size(); ++i) {
      const double wheel_angular_vel = wheel_velocities[i] / kinematics_->getWheelRadius();
      const double motor_angular_vel =
        wheel_angular_vel * kinematics_->getGearRatio() * static_cast<double>(drive_directions_[i]);

      auto msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
      msg.header.stamp = stamp;
      msg.joint_name = motor_names_[i];
      msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
      msg.velocity_target = motor_angular_vel;
      motor_cmd_pub_->publish(msg);
    }
  }

  std::unique_ptr<OmniWheelKinematics> kinematics_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr cmd_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::array<std::string, 4> motor_names_ {{"DJI3508_1", "DJI3508_2", "DJI3508_3", "DJI3508_4"}};
  std::array<int, 4> drive_directions_ {{1, 1, 1, 1}};

  std::string cmd_vel_topic_ {"/cmd_vel"};
  std::string cmd_pose_topic_ {"/cmd_pose"};
  std::string odom_topic_ {"/odom"};
  std::string estop_topic_ {"/chassis/estop"};

  double control_frequency_ {100.0};
  double wheel_base_x_ {0.88};
  double wheel_base_y_ {0.88};
  double wheel_radius_ {0.1062};
  double install_angle_ {45.0};
  double max_linear_velocity_ {2.0};
  double max_angular_velocity_ {2.0};
  double max_wheel_linear_velocity_ {3.0};
  double cmd_timeout_ {0.5};
  double zero_target_deadband_ {1e-3};
  double yaw_feedback_sign_ {-1.0};
  bool yaw_hold_enabled_ {false};
  double yaw_hold_kp_ {1.2};
  double yaw_hold_max_angular_velocity_ {0.6};
  double yaw_hold_deadband_ {0.03};
  double yaw_hold_release_angular_velocity_ {0.05};
  double position_timeout_ {1.0};
  double goal_tolerance_xy_ {0.02};
  double goal_tolerance_yaw_ {0.05};

  ChassisMode mode_ {ChassisMode::VELOCITY};
  VelocityControlMode velocity_control_mode_ {VelocityControlMode::OPEN_LOOP};
  bool estop_active_ {true};
  geometry_msgs::msg::Twist velocity_target_;
  geometry_msgs::msg::Pose2D pose_target_;
  bool has_pose_target_ {false};
  bool has_odom_ {false};
  bool has_yaw_hold_reference_ {false};
  nav_msgs::msg::Odometry latest_odom_;
  double latest_yaw_ {0.0};
  double yaw_hold_reference_ {0.0};
  double last_yaw_hold_error_ {0.0};
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_pose_cmd_time_;
  rclcpp::Time last_odom_time_;

  PIDController velocity_pid_x_;
  PIDController velocity_pid_y_;
  PIDController velocity_pid_yaw_;
  PIDController position_pid_x_;
  PIDController position_pid_y_;
  PIDController position_pid_yaw_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::OmniChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
