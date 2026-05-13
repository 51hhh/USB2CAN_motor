#include "motor_control_ros2/serve_windmill_manager.hpp"

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>

namespace serve_windmill {

ServeWindmillManager::ServeWindmillManager()
  : Node("serve_windmill_manager")
{
  // 默认值
  motor_name_m1_ = "DJI3508_1";
  motor_name_m2_ = "DJI3508_2";
  motor_direction_ = -1;
  wind_up_angle_deg_ = 200.0;
  fire_end_angle_deg_ = 90.0;
  catch_start_angle_deg_ = 270.0;
  catch_target_angle_deg_ = 360.0;
  max_fire_current_ = 14000;
  wind_up_max_current_ = 8000;
  catch_max_current_ = 8000;
  global_max_current_ = 14000;
  gravity_ff_current_ = 0;
  wind_up_kp_ = 80.0;
  wind_up_kd_ = 3.0;
  catch_kp_ = 60.0;
  catch_kd_ = 4.0;
  homing_vel_threshold_rpm_ = 8.0;
  homing_stable_duration_s_ = 1.0;
  free_wheel_vel_threshold_rpm_ = 20.0;
  runaway_threshold_deg_ = 1080.0;
  control_frequency_ = 200.0;
  position_tolerance_deg_ = 3.0;

  // 从 YAML 加载参数（与 motor_control_node 同款写法）
  try {
    std::string config_file =
        ament_index_cpp::get_package_share_directory("motor_control_ros2") +
        "/config/serve_windmill_params.yaml";

    YAML::Node root = YAML::LoadFile(config_file);
    auto p = root["serve_windmill_manager"]["ros__parameters"];

    auto m = p["motors"];
    if (m) {
      if (m["master_name"])  motor_name_m1_  = m["master_name"].as<std::string>();
      if (m["encoder_name"]) motor_name_m2_  = m["encoder_name"].as<std::string>();
      if (m["direction"])    motor_direction_ = m["direction"].as<int>();
    }
    auto a = p["angles_deg"];
    if (a) {
      if (a["wind_up"])      wind_up_angle_deg_     = a["wind_up"].as<double>();
      if (a["fire_end"])     fire_end_angle_deg_    = a["fire_end"].as<double>();
      if (a["catch_start"])  catch_start_angle_deg_ = a["catch_start"].as<double>();
      if (a["catch_target"]) catch_target_angle_deg_= a["catch_target"].as<double>();
    }
    auto c = p["current_limits"];
    if (c) {
      auto clamp16 = [](int v){ return static_cast<int16_t>(std::clamp(v, -32767, 32767)); };
      if (c["fire"])       max_fire_current_     = clamp16(c["fire"].as<int>());
      if (c["wind_up_max"])wind_up_max_current_  = clamp16(c["wind_up_max"].as<int>());
      if (c["catch_max"])  catch_max_current_    = clamp16(c["catch_max"].as<int>());
      if (c["global_max"]) global_max_current_   = clamp16(c["global_max"].as<int>());
    }
    auto g = p["gains"];
    if (g) {
      if (g["wind_up"]) { wind_up_kp_ = g["wind_up"]["kp"].as<double>(); wind_up_kd_ = g["wind_up"]["kd"].as<double>(); }
      if (g["catch"])   { catch_kp_   = g["catch"]["kp"].as<double>();    catch_kd_   = g["catch"]["kd"].as<double>(); }
      if (g["gravity_ff_current"]) gravity_ff_current_ = static_cast<int16_t>(g["gravity_ff_current"].as<int>());
    }
    if (p["homing"]) {
      if (p["homing"]["vel_threshold_rpm"]) homing_vel_threshold_rpm_ = p["homing"]["vel_threshold_rpm"].as<double>();
      if (p["homing"]["stable_duration_s"]) homing_stable_duration_s_ = p["homing"]["stable_duration_s"].as<double>();
    }
    if (p["free_wheel"] && p["free_wheel"]["vel_threshold_rpm"])
      free_wheel_vel_threshold_rpm_ = p["free_wheel"]["vel_threshold_rpm"].as<double>();
    if (p["safety"] && p["safety"]["runaway_threshold_deg"])
      runaway_threshold_deg_ = p["safety"]["runaway_threshold_deg"].as<double>();
    if (p["control"]) {
      if (p["control"]["frequency"])              control_frequency_      = p["control"]["frequency"].as<double>();
      if (p["control"]["position_tolerance_deg"]) position_tolerance_deg_ = p["control"]["position_tolerance_deg"].as<double>();
    }

    RCLCPP_INFO(this->get_logger(), "已加载参数: %s | wind_up=%.1f fire_end=%.1f",
                config_file.c_str(), wind_up_angle_deg_, fire_end_angle_deg_);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "参数加载失败，使用默认值: %s", e.what());
  }

  // 其余成员变量初始化
  feedback_angle_deg_ = 0.0;
  feedback_rpm_ = 0.0;
  feedback_current_ = 0;
  feedback_online_ = false;
  has_feedback_ = false;
  state_ = ServeState::IDLE;
  state_entry_angle_deg_ = 0.0;
  home_offset_deg_ = 0.0;
  homing_last_angle_ = 0.0;
  homing_stability_started_ = false;
  serve_count_ = 0;

  state_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorState>(
      "dji_motor_states", 50,
      std::bind(&ServeWindmillManager::motorStateCallback, this, std::placeholders::_1));

  trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/serve/trigger", 10,
      std::bind(&ServeWindmillManager::triggerCallback, this, std::placeholders::_1));

  cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/serve/command", 10,
      std::bind(&ServeWindmillManager::commandCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorCommand>(
      "dji_motor_command", 20);

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/serve/status", 10);

  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ServeWindmillManager::controlLoop, this));

  state_enter_time_ = this->now();
  homing_stable_since_ = this->now();
  publishStatus();
  transitionTo(ServeState::GRAVITY_HOMING);

  RCLCPP_INFO(this->get_logger(),
              "serve_windmill_manager 启动 | %.0f Hz | m1=%s m2=%s | wind_up=%.1f fire_end=%.1f catch_start=%.1f catch_target=%.1f",
              control_frequency_, motor_name_m1_.c_str(), motor_name_m2_.c_str(),
              wind_up_angle_deg_, fire_end_angle_deg_, catch_start_angle_deg_, catch_target_angle_deg_);
}

void ServeWindmillManager::motorStateCallback(
    const motor_control_ros2::msg::DJIMotorState::SharedPtr msg)
{
  if (msg->joint_name != motor_name_m2_) {
    return;
  }

  feedback_angle_deg_ = msg->angle * motor_direction_;
  feedback_rpm_ = static_cast<double>(msg->rpm) * motor_direction_;
  feedback_current_ = msg->current;
  feedback_online_ = msg->online;
  has_feedback_ = true;
}

void ServeWindmillManager::triggerCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  requestExecute();
}

void ServeWindmillManager::commandCallback(
    const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "start") {
    requestExecute();
    return;
  }

  if (msg->data == "stop") {
    transitionTo(ServeState::IDLE);
    return;
  }

  if (msg->data == "home") {
    transitionTo(ServeState::GRAVITY_HOMING);
    return;
  }

  if (msg->data == "estop") {
    transitionTo(ServeState::E_STOP);
    return;
  }

  if (msg->data == "resume") {
    if (state_ == ServeState::E_STOP) {
      transitionTo(ServeState::IDLE);
    }
    return;
  }
}

void ServeWindmillManager::requestExecute()
{
  switch (state_) {
    case ServeState::IDLE:
      RCLCPP_INFO(this->get_logger(), "收到发球触发，开始归零");
      transitionTo(ServeState::GRAVITY_HOMING);
      break;
    case ServeState::WIND_UP:
      RCLCPP_INFO(this->get_logger(), "收到发球触发，直接开火");
      transitionTo(ServeState::FIRE);
      break;
    default:
      RCLCPP_DEBUG(this->get_logger(), "当前状态 %s，忽略执行触发",
                   serveStateToString(state_));
      break;
  }
}

int16_t ServeWindmillManager::computePD(
    double target_deg, double kp, double kd, int16_t max_current) const
{
  const double error_deg = target_deg - feedback_angle_deg_;
  const double output = kp * error_deg - kd * feedback_rpm_;
  const int16_t limit = std::min<int16_t>(max_current, global_max_current_);
  const double clamped = std::clamp(output, -static_cast<double>(limit), static_cast<double>(limit));
  return static_cast<int16_t>(clamped);
}

void ServeWindmillManager::sendCurrent(int16_t current_m2)
{
  current_m2 = static_cast<int16_t>(current_m2 * motor_direction_);
  current_m2 = std::clamp<int16_t>(current_m2, -global_max_current_, global_max_current_);
  const int16_t current_m1 = static_cast<int16_t>(-current_m2);

  auto msg_m1 = motor_control_ros2::msg::DJIMotorCommand();
  msg_m1.header.stamp = this->now();
  msg_m1.joint_name = motor_name_m1_;
  msg_m1.output = current_m1;
  cmd_pub_->publish(msg_m1);

  auto msg_m2 = motor_control_ros2::msg::DJIMotorCommand();
  msg_m2.header.stamp = msg_m1.header.stamp;
  msg_m2.joint_name = motor_name_m2_;
  msg_m2.output = current_m2;
  cmd_pub_->publish(msg_m2);
}

void ServeWindmillManager::transitionTo(ServeState new_state)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "状态切换: %s -> %s",
              serveStateToString(state_), serveStateToString(new_state));

  state_ = new_state;
  state_enter_time_ = this->now();
  state_entry_angle_deg_ = feedback_angle_deg_;
  publishStatus();

  if (new_state == ServeState::GRAVITY_HOMING) {
    homing_stability_started_ = false;
    homing_last_angle_ = feedback_angle_deg_;
  }
}

void ServeWindmillManager::publishStatus() const
{
  auto msg = std_msgs::msg::String();
  msg.data = serveStateToString(state_);
  status_pub_->publish(msg);
}

void ServeWindmillManager::handleIdle()
{
  sendCurrent(0);
}

void ServeWindmillManager::handleGravityHoming()
{
  sendCurrent(0);

  const double now_angle = feedback_angle_deg_;
  const double delta_angle = std::abs(now_angle - homing_last_angle_);
  homing_last_angle_ = now_angle;

  const bool low_velocity = std::abs(feedback_rpm_) <= homing_vel_threshold_rpm_;
  const bool low_motion = delta_angle <= 0.2;

  if (low_velocity && low_motion) {
    if (!homing_stability_started_) {
      homing_stability_started_ = true;
      homing_stable_since_ = this->now();
    }

    const double stable_time = (this->now() - homing_stable_since_).seconds();
    if (stable_time >= homing_stable_duration_s_) {
      home_offset_deg_ = feedback_angle_deg_;
      RCLCPP_INFO(this->get_logger(), "重力归零完成，home_offset=%.2f deg", home_offset_deg_);
      transitionTo(ServeState::WIND_UP);
    }
  } else {
    homing_stability_started_ = false;
  }
}

void ServeWindmillManager::handleWindUp()
{
  const double target_deg = home_offset_deg_ - wind_up_angle_deg_;
  const int16_t pd_output = computePD(target_deg, wind_up_kp_, wind_up_kd_, wind_up_max_current_);
  const int16_t output = static_cast<int16_t>(std::clamp(
      static_cast<int>(pd_output) + static_cast<int>(gravity_ff_current_),
      -static_cast<int>(wind_up_max_current_), static_cast<int>(wind_up_max_current_)));
  sendCurrent(output);
}

void ServeWindmillManager::handleFire()
{
  sendCurrent(max_fire_current_);

  const double relative_deg = feedback_angle_deg_ - home_offset_deg_;
  if (relative_deg >= fire_end_angle_deg_) {
    transitionTo(ServeState::FREE_WHEEL);
  }
}

void ServeWindmillManager::handleFreeWheel()
{
  sendCurrent(0);

  const double relative_deg = feedback_angle_deg_ - home_offset_deg_;
  const bool reached_catch_zone = relative_deg >= catch_start_angle_deg_;
  const bool too_slow = std::abs(feedback_rpm_) <= free_wheel_vel_threshold_rpm_;

  if (reached_catch_zone || too_slow) {
    transitionTo(ServeState::CATCH);
  }
}

void ServeWindmillManager::handleCatch()
{
  const double target_deg = home_offset_deg_ + catch_target_angle_deg_;
  const int16_t output = computePD(target_deg, catch_kp_, catch_kd_, catch_max_current_);
  sendCurrent(output);

  const bool reached = std::abs(feedback_angle_deg_ - target_deg) <= position_tolerance_deg_;
  const bool slow_enough = std::abs(feedback_rpm_) <= homing_vel_threshold_rpm_;
  const double elapsed = (this->now() - state_enter_time_).seconds();

  if ((reached && slow_enough) || elapsed > 5.0) {
    ++serve_count_;
    RCLCPP_INFO(this->get_logger(), "接住完成（%.1fs），第 %d 次循环，回归重力归零", elapsed, serve_count_);
    transitionTo(ServeState::GRAVITY_HOMING);
  }
}

void ServeWindmillManager::handleEStop()
{
  sendCurrent(0);
}

void ServeWindmillManager::controlLoop()
{
  if (!has_feedback_) {
    sendCurrent(0);
    return;
  }

  if (!feedback_online_ && state_ != ServeState::IDLE && state_ != ServeState::E_STOP) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "M2 电机离线，进入 E_STOP");
    transitionTo(ServeState::E_STOP);
  }

  // 飞车安全保护：任何状态下角位移超过阈值立即 E_STOP
  if (state_ != ServeState::E_STOP) {
    const double travel = std::abs(feedback_angle_deg_ - state_entry_angle_deg_);
    if (travel > runaway_threshold_deg_) {
      RCLCPP_ERROR(this->get_logger(),
                   "飞车保护触发！状态 %s 内角位移 %.1f° 超过 %.1f°，紧急停止",
                   serveStateToString(state_), travel, runaway_threshold_deg_);
      transitionTo(ServeState::E_STOP);
      return;
    }
  }

  switch (state_) {
    case ServeState::IDLE:
      handleIdle();
      break;
    case ServeState::GRAVITY_HOMING:
      handleGravityHoming();
      break;
    case ServeState::WIND_UP:
      handleWindUp();
      break;
    case ServeState::FIRE:
      handleFire();
      break;
    case ServeState::FREE_WHEEL:
      handleFreeWheel();
      break;
    case ServeState::CATCH:
      handleCatch();
      break;
    case ServeState::E_STOP:
      handleEStop();
      break;
    default:
      sendCurrent(0);
      break;
  }
}

}  // namespace serve_windmill

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<serve_windmill::ServeWindmillManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
