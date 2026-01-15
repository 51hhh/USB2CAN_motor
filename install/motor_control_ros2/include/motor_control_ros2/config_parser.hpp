#ifndef MOTOR_CONTROL_ROS2__CONFIG_PARSER_HPP_
#define MOTOR_CONTROL_ROS2__CONFIG_PARSER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace motor_control {

/**
 * @brief 电机配置结构
 */
struct MotorConfig {
  std::string name;           // 电机名称
  std::string type;           // 电机类型 (GM6020, GM3508, DM4340, A1, GO8010)
  int id;                     // 电机 ID
  int direction = 1;          // 方向 (仅宇树电机)
  float offset = 0.0f;        // 零点偏移 (仅宇树电机)
};

/**
 * @brief CAN 接口配置
 */
struct CANInterfaceConfig {
  std::string device;         // 设备路径 (/dev/ttyACM0)
  int baudrate;               // 波特率
  std::vector<MotorConfig> motors;  // 电机列表
};

/**
 * @brief 串口接口配置
 */
struct SerialInterfaceConfig {
  std::string device;         // 设备路径 (/dev/ttyUSB0)
  int baudrate;               // 波特率
  std::vector<MotorConfig> motors;  // 电机列表
};

/**
 * @brief 完整配置
 */
struct SystemConfig {
  std::vector<CANInterfaceConfig> can_interfaces;
  std::vector<SerialInterfaceConfig> serial_interfaces;
};

/**
 * @brief 配置文件解析器
 */
class ConfigParser {
public:
  /**
   * @brief 从文件加载配置
   * @param config_file 配置文件路径
   * @return 系统配置
   */
  static SystemConfig loadConfig(const std::string& config_file);

private:
  /**
   * @brief 解析电机配置
   */
  static MotorConfig parseMotorConfig(const YAML::Node& node);
  
  /**
   * @brief 解析 CAN 接口配置
   */
  static CANInterfaceConfig parseCANInterface(const YAML::Node& node);
  
  /**
   * @brief 解析串口接口配置
   */
  static SerialInterfaceConfig parseSerialInterface(const YAML::Node& node);
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__CONFIG_PARSER_HPP_
