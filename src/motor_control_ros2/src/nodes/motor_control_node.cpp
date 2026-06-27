#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/config_parser.hpp"
#include "motor_control_ros2/dji_motor.hpp"
#include "motor_control_ros2/hardware/can_interface.hpp"
#include "motor_control_ros2/msg/control_frequency.hpp"
#include "motor_control_ros2/msg/dji_motor_command.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

namespace motor_control {

class MotorControlNode : public rclcpp::Node {
public:
  MotorControlNode() : Node("motor_control_node") {
    this->declare_parameter("control_frequency", 200.0);
    this->declare_parameter("command_timeout", 0.5);
    this->declare_parameter("control_config_file", "");
    this->declare_parameter("config_file", "");
    this->declare_parameter("pid_config_file", "");

    loadControlParams();
    loadMotorConfig();

    can_network_ = std::make_shared<hardware::CANNetwork>();
    can_network_->setGlobalRxCallback(
      std::bind(&MotorControlNode::canRxCallback, this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4));

    initializeCanInterfaces();
    loadPidParams();
    can_network_->startAll();

    dji_state_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorState>(
      "dji_motor_states", 10);
    control_freq_pub_ = this->create_publisher<motor_control_ros2::msg::ControlFrequency>(
      "control_frequency", 10);

    dji_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorCommand>(
      "dji_motor_command", 10,
      std::bind(&MotorControlNode::djiCommandCallback, this, std::placeholders::_1));
    dji_cmd_advanced_sub_ =
      this->create_subscription<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
        "dji_motor_command_advanced", 50,
        std::bind(&MotorControlNode::djiCommandAdvancedCallback, this, std::placeholders::_1));

    target_control_freq_ = this->get_parameter("control_frequency").as_double();
    command_timeout_ = this->get_parameter("command_timeout").as_double();
    last_freq_report_time_ = this->now();
    last_tx_report_time_ = this->now();
    const auto period = std::chrono::duration<double>(1.0 / target_control_freq_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotorControlNode::controlLoop, this));

    if (can_network_->getPendingCount() > 0) {
      reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MotorControlNode::checkReconnect, this));
    }

    RCLCPP_INFO(this->get_logger(),
      "motor_control_node 启动: DJI=%zu, 目标频率=%.1fHz",
      dji_motors_.size(), target_control_freq_);
  }

  ~MotorControlNode() override {
    if (can_network_) {
      can_network_->stopAll();
      can_network_->closeAll();
    }
  }

private:
  std::string packageShareDir() const {
    return ament_index_cpp::get_package_share_directory("motor_control_ros2");
  }

  std::string resolvePackageFile(
    const std::string& parameter_name,
    const std::string& default_relative_path) const
  {
    const std::string param = this->get_parameter(parameter_name).as_string();
    if (!param.empty()) {
      if (param.front() == '/') {
        return param;
      }
      return packageShareDir() + "/" + param;
    }
    return packageShareDir() + "/" + default_relative_path;
  }

  void loadControlParams() {
    const std::string file = resolvePackageFile("control_config_file", "config/control_params.yaml");
    const YAML::Node config = YAML::LoadFile(file);
    if (config["motor_control_node"] && config["motor_control_node"]["ros__parameters"]) {
      const auto params = config["motor_control_node"]["ros__parameters"];
      if (params["control_frequency"]) {
        this->set_parameter(
          rclcpp::Parameter("control_frequency", params["control_frequency"].as<double>()));
      }
      if (params["command_timeout"]) {
        this->set_parameter(
          rclcpp::Parameter("command_timeout", params["command_timeout"].as<double>()));
      }
      if (params["config_file"] && this->get_parameter("config_file").as_string().empty()) {
        this->set_parameter(
          rclcpp::Parameter("config_file", params["config_file"].as<std::string>()));
      }
    }
  }

  std::string resolveMotorConfigFile() const {
    return resolvePackageFile("config_file", "config/motors.yaml");
  }

  void loadMotorConfig() {
    config_ = ConfigParser::loadConfig(resolveMotorConfigFile());
  }

  void initializeCanInterfaces() {
    int interface_index = 0;
    for (const auto& can_config : config_.can_interfaces) {
      const std::string interface_name = "can_" + std::to_string(interface_index++);
      can_network_->addInterface(interface_name, can_config.device, can_config.baudrate);

      for (const auto& motor_config : can_config.motors) {
        addDjiMotor(motor_config, interface_name);
      }
    }
  }

  void addDjiMotor(const MotorConfig& config, const std::string& interface_name) {
    MotorType motor_type;
    if (config.type == "GM3508") {
      motor_type = MotorType::DJI_GM3508;
    } else if (config.type == "GM6020") {
      motor_type = MotorType::DJI_GM6020;
    } else {
      RCLCPP_WARN(this->get_logger(), "跳过非 DJI 电机配置: %s (%s)",
        config.name.c_str(), config.type.c_str());
      return;
    }

    auto motor = std::make_shared<DJIMotor>(config.name, motor_type, config.id, 0);
    motor->setInterfaceName(interface_name);
    motor->setDirection(config.direction);
    motor->setOffset(config.offset);
    motors_[config.name] = motor;
    dji_motors_.push_back(motor);

    if (!config.mirror_from.empty()) {
      dji_mirror_map_[config.name] = config.mirror_from;
    }
  }

  void loadPidParams() {
    const std::string file = resolvePackageFile("pid_config_file", "config/pid_params.yaml");
    const YAML::Node config = YAML::LoadFile(file);
    if (!config["dji_motors"]) {
      return;
    }

    std::map<std::string, std::pair<PIDParams, PIDParams>> type_params;
    for (const auto& type_node : config["dji_motors"]) {
      const std::string motor_type = type_node.first.as<std::string>();
      PIDParams pos_pid;
      PIDParams vel_pid;

      if (type_node.second["position_pid"]) {
        const auto pos = type_node.second["position_pid"];
        pos_pid.kp = pos["kp"].as<double>();
        pos_pid.ki = pos["ki"].as<double>();
        pos_pid.kd = pos["kd"].as<double>();
        pos_pid.i_max = pos["i_max"].as<double>();
        pos_pid.out_max = pos["out_max"].as<double>();
        pos_pid.dead_zone = pos["dead_zone"].as<double>();
      }
      if (type_node.second["velocity_pid"]) {
        const auto vel = type_node.second["velocity_pid"];
        vel_pid.kp = vel["kp"].as<double>();
        vel_pid.ki = vel["ki"].as<double>();
        vel_pid.kd = vel["kd"].as<double>();
        vel_pid.i_max = vel["i_max"].as<double>();
        vel_pid.out_max = vel["out_max"].as<double>();
        vel_pid.dead_zone = vel["dead_zone"].as<double>();
      }
      type_params[motor_type] = {pos_pid, vel_pid};
    }

    for (const auto& motor : dji_motors_) {
      const std::string key =
        motor->getMotorType() == MotorType::DJI_GM6020 ? "GM6020" : "GM3508";
      const auto it = type_params.find(key);
      if (it != type_params.end()) {
        motor->setPositionPID(it->second.first);
        motor->setVelocityPID(it->second.second);
      }
    }
  }

  void canRxCallback(
    const std::string& interface_name,
    uint32_t can_id,
    const uint8_t* data,
    size_t len)
  {
    for (const auto& [_, motor] : motors_) {
      motor->updateFeedback(interface_name, can_id, data, len);
    }
  }

  void checkReconnect() {
    const int connected = can_network_->retryPendingInterfaces();
    if (connected > 0) {
      RCLCPP_INFO(this->get_logger(), "成功重连 %d 个 CAN 设备", connected);
    }
    if (can_network_->getPendingCount() == 0 && reconnect_timer_) {
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    }
  }

  void controlLoop() {
    const auto now = this->now();
    control_loop_count_++;
    const double freq_dt = (now - last_freq_report_time_).seconds();
    if (freq_dt >= 1.0) {
      actual_control_freq_ = static_cast<double>(control_loop_count_) / freq_dt;
      control_loop_count_ = 0;
      last_freq_report_time_ = now;
    }

    applyCommandTimeout(now);

    for (const auto& motor : dji_motors_) {
      motor->updateController();
    }

    writeDjiMotors();
    publishStates(now);
  }

  void applyCommandTimeout(const rclcpp::Time& now) {
    for (const auto& motor : dji_motors_) {
      const auto it = last_motor_command_time_.find(motor->getJointName());
      if (it == last_motor_command_time_.end() ||
          (now - it->second).seconds() > command_timeout_) {
        motor->setControlMode(ControlMode::DIRECT);
        motor->setOutput(0);
      }
    }
  }

  void writeDjiMotors() {
    tx_frame_count_++;
    const auto now = this->now();
    const double tx_dt = (now - last_tx_report_time_).seconds();
    if (tx_dt >= 1.0) {
      actual_can_tx_freq_ = static_cast<double>(tx_frame_count_) / tx_dt;
      tx_frame_count_ = 0;
      last_tx_report_time_ = now;
    }

    std::map<std::string, std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>>> grouped;
    for (const auto& motor : dji_motors_) {
      grouped[motor->getInterfaceName()][motor->getControlId()].push_back(motor);
    }

    for (const auto& [interface_name, control_groups] : grouped) {
      for (const auto& [control_id, motors] : control_groups) {
        uint8_t data[8] = {0};
        for (const auto& motor : motors) {
          uint8_t bytes[2] = {0};
          motor->getControlBytes(bytes);
          const int offset = (static_cast<int>(motor->getMotorId()) - 1) % 4 * 2;
          data[offset] = bytes[0];
          data[offset + 1] = bytes[1];
        }
        can_network_->send(interface_name, control_id, data, 8);
      }
    }
  }

  void publishStates(const rclcpp::Time& now) {
    const int64_t current_time_ns =
      std::chrono::steady_clock::now().time_since_epoch().count();
    constexpr double heartbeat_timeout_ms = 500.0;

    for (const auto& motor : dji_motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);

      auto msg = motor_control_ros2::msg::DJIMotorState();
      msg.header.stamp = now;
      msg.joint_name = motor->getJointName();
      msg.model =
        motor->getMotorType() == MotorType::DJI_GM6020 ? "GM6020" : "GM3508";
      msg.online = motor->isOnline();

      const auto mirror_it = dji_mirror_map_.find(motor->getJointName());
      if (mirror_it != dji_mirror_map_.end()) {
        const auto src_it = motors_.find(mirror_it->second);
        if (src_it != motors_.end()) {
          auto src_dji = std::dynamic_pointer_cast<DJIMotor>(src_it->second);
          if (src_dji) {
            double degrees =
              (src_dji->getOutputPosition() - motor->getOffset()) * 180.0 / M_PI;
            degrees = std::fmod(degrees, 360.0);
            if (degrees < 0.0) {
              degrees += 360.0;
            }
            msg.angle = degrees;
          } else {
            msg.angle = motor->getAngleDegrees();
          }
        } else {
          msg.angle = motor->getAngleDegrees();
        }
      } else {
        msg.angle = motor->getAngleDegrees();
      }

      msg.rpm = motor->getRPM();
      msg.current = motor->getCurrent();
      msg.temperature = static_cast<uint8_t>(motor->getTemperature());
      msg.control_frequency = actual_control_freq_;
      dji_state_pub_->publish(msg);
    }

    auto freq_msg = motor_control_ros2::msg::ControlFrequency();
    freq_msg.header.stamp = now;
    freq_msg.control_frequency = actual_control_freq_;
    freq_msg.can_tx_frequency = actual_can_tx_freq_;
    freq_msg.target_frequency = target_control_freq_;
    control_freq_pub_->publish(freq_msg);
  }

  void djiCommandCallback(
    const motor_control_ros2::msg::DJIMotorCommand::SharedPtr msg)
  {
    const auto it = motors_.find(msg->joint_name);
    if (it == motors_.end()) {
      return;
    }
    auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
    if (dji) {
      dji->setControlMode(ControlMode::DIRECT);
      dji->setOutput(msg->output);
      last_motor_command_time_[msg->joint_name] = this->now();
    }
  }

  void djiCommandAdvancedCallback(
    const motor_control_ros2::msg::DJIMotorCommandAdvanced::SharedPtr msg)
  {
    const auto it = motors_.find(msg->joint_name);
    if (it == motors_.end()) {
      return;
    }
    auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
    if (!dji) {
      return;
    }

    const auto mode = static_cast<ControlMode>(msg->mode);
    dji->setControlMode(mode);

    switch (mode) {
      case ControlMode::POSITION:
        dji->setPositionTarget(msg->position_target * 180.0 / M_PI);
        last_motor_command_time_[msg->joint_name] = this->now();
        break;
      case ControlMode::VELOCITY:
        dji->setVelocityTarget(msg->velocity_target * 60.0 / (2.0 * M_PI));
        last_motor_command_time_[msg->joint_name] = this->now();
        break;
      case ControlMode::DIRECT:
      default:
        dji->setOutput(msg->direct_output);
        last_motor_command_time_[msg->joint_name] = this->now();
        break;
    }
  }

  SystemConfig config_;
  std::shared_ptr<hardware::CANNetwork> can_network_;
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<DJIMotor>> dji_motors_;
  std::map<std::string, std::string> dji_mirror_map_;
  std::map<std::string, rclcpp::Time> last_motor_command_time_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorState>::SharedPtr dji_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::ControlFrequency>::SharedPtr control_freq_pub_;
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommand>::SharedPtr dji_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr
    dji_cmd_advanced_sub_;

  int control_loop_count_ = 0;
  int tx_frame_count_ = 0;
  double actual_control_freq_ = 0.0;
  double actual_can_tx_freq_ = 0.0;
  double target_control_freq_ = 200.0;
  double command_timeout_ = 0.5;
  rclcpp::Time last_freq_report_time_;
  rclcpp::Time last_tx_report_time_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
