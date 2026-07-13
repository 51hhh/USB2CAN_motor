#ifndef MOTOR_CONTROL_ROS2__CONFIG_PARSER_HPP_
#define MOTOR_CONTROL_ROS2__CONFIG_PARSER_HPP_

#include <string>
#include <vector>

namespace YAML {
class Node;
}

namespace motor_control {

struct MotorConfig {
  std::string name;
  std::string type;
  int id;
  int direction = 1;
  double offset = 0.0;
  std::string mirror_from;
};

struct CANInterfaceConfig {
  std::string device;
  int baudrate;
  std::vector<MotorConfig> motors;
};

struct SystemConfig {
  std::vector<CANInterfaceConfig> can_interfaces;
};

class ConfigParser {
public:
  static SystemConfig loadConfig(const std::string& config_file);

private:
  static MotorConfig parseMotorConfig(const YAML::Node& node);
  static CANInterfaceConfig parseCANInterface(const YAML::Node& node);
};

}  // namespace motor_control

#endif
