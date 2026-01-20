#include "motor_control_ros2/config_parser.hpp"
#include <stdexcept>
#include <iostream>

namespace motor_control {

SystemConfig ConfigParser::loadConfig(const std::string& config_file) {
  SystemConfig config;
  
  try {
    YAML::Node yaml = YAML::LoadFile(config_file);
    
    // 解析 CAN 接口
    if (yaml["can_interfaces"]) {
      for (const auto& interface_node : yaml["can_interfaces"]) {
        config.can_interfaces.push_back(parseCANInterface(interface_node));
      }
    }
    
    // 解析串口接口
    if (yaml["serial_interfaces"]) {
      for (const auto& interface_node : yaml["serial_interfaces"]) {
        config.serial_interfaces.push_back(parseSerialInterface(interface_node));
      }
    }
    
    std::cout << "[ConfigParser] 成功加载配置文件: " << config_file << std::endl;
    std::cout << "[ConfigParser] CAN 接口数: " << config.can_interfaces.size() << std::endl;
    std::cout << "[ConfigParser] 串口接口数: " << config.serial_interfaces.size() << std::endl;
    
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("YAML 解析错误: " + std::string(e.what()));
  } catch (const std::exception& e) {
    throw std::runtime_error("配置加载失败: " + std::string(e.what()));
  }
  
  return config;
}

MotorConfig ConfigParser::parseMotorConfig(const YAML::Node& node) {
  MotorConfig config;
  
  if (!node["name"]) {
    throw std::runtime_error("电机配置缺少 'name' 字段");
  }
  if (!node["type"]) {
    throw std::runtime_error("电机配置缺少 'type' 字段");
  }
  if (!node["id"]) {
    throw std::runtime_error("电机配置缺少 'id' 字段");
  }
  
  config.name = node["name"].as<std::string>();
  config.type = node["type"].as<std::string>();
  config.id = node["id"].as<int>();
  
  // 可选字段
  if (node["direction"]) {
    config.direction = node["direction"].as<int>();
  }
  if (node["offset"]) {
    config.offset = node["offset"].as<float>();
  }
  
  return config;
}

CANInterfaceConfig ConfigParser::parseCANInterface(const YAML::Node& node) {
  CANInterfaceConfig config;
  
  if (!node["device"]) {
    throw std::runtime_error("CAN 接口配置缺少 'device' 字段");
  }
  if (!node["baudrate"]) {
    throw std::runtime_error("CAN 接口配置缺少 'baudrate' 字段");
  }
  
  config.device = node["device"].as<std::string>();
  config.baudrate = node["baudrate"].as<int>();
  
  // 解析电机列表
  if (node["motors"]) {
    for (const auto& motor_node : node["motors"]) {
      config.motors.push_back(parseMotorConfig(motor_node));
    }
  }
  
  std::cout << "[ConfigParser] CAN 接口: " << config.device 
            << " @ " << config.baudrate << " bps, "
            << config.motors.size() << " 个电机" << std::endl;
  
  return config;
}

SerialInterfaceConfig ConfigParser::parseSerialInterface(const YAML::Node& node) {
  SerialInterfaceConfig config;
  
  if (!node["device"]) {
    throw std::runtime_error("串口接口配置缺少 'device' 字段");
  }
  if (!node["baudrate"]) {
    throw std::runtime_error("串口接口配置缺少 'baudrate' 字段");
  }
  
  config.device = node["device"].as<std::string>();
  config.baudrate = node["baudrate"].as<int>();
  
  // 解析电机列表
  if (node["motors"]) {
    for (const auto& motor_node : node["motors"]) {
      config.motors.push_back(parseMotorConfig(motor_node));
    }
  }
  
  std::cout << "[ConfigParser] 串口接口: " << config.device 
            << " @ " << config.baudrate << " bps, "
            << config.motors.size() << " 个电机" << std::endl;
  
  return config;
}

} // namespace motor_control
