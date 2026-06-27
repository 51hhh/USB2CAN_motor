#include "motor_control_ros2/config_parser.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace motor_control {

SystemConfig ConfigParser::loadConfig(const std::string& config_file) {
  SystemConfig config;

  try {
    const YAML::Node yaml = YAML::LoadFile(config_file);
    if (yaml["can_interfaces"]) {
      for (const auto& interface_node : yaml["can_interfaces"]) {
        config.can_interfaces.push_back(parseCANInterface(interface_node));
      }
    }
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("YAML 解析错误: " + std::string(e.what()));
  }

  return config;
}

MotorConfig ConfigParser::parseMotorConfig(const YAML::Node& node) {
  MotorConfig config;

  if (!node["name"] || !node["type"] || !node["id"]) {
    throw std::runtime_error("电机配置必须包含 name/type/id");
  }

  config.name = node["name"].as<std::string>();
  config.type = node["type"].as<std::string>();
  config.id = node["id"].as<int>();

  if (node["direction"]) {
    config.direction = node["direction"].as<int>();
  }
  if (node["offset"]) {
    config.offset = node["offset"].as<double>();
  }
  if (node["mirror_from"]) {
    config.mirror_from = node["mirror_from"].as<std::string>();
  }

  return config;
}

CANInterfaceConfig ConfigParser::parseCANInterface(const YAML::Node& node) {
  CANInterfaceConfig config;

  if (!node["device"] || !node["baudrate"]) {
    throw std::runtime_error("CAN 接口配置必须包含 device/baudrate");
  }

  config.device = node["device"].as<std::string>();
  config.baudrate = node["baudrate"].as<int>();

  if (node["motors"]) {
    for (const auto& motor_node : node["motors"]) {
      config.motors.push_back(parseMotorConfig(motor_node));
    }
  }

  return config;
}

}  // namespace motor_control
