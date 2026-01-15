#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <vector>
#include <map>
#include <thread>
#include <chrono>

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/dji_motor.hpp"
#include "motor_control_ros2/damiao_motor.hpp"
#include "motor_control_ros2/unitree_motor.hpp"
#include "motor_control_ros2/hardware/can_interface.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include "motor_control_ros2/config_parser.hpp"

#include "motor_control_ros2/msg/dji_motor_state.hpp"
#include "motor_control_ros2/msg/dji_motor_command.hpp"
#include "motor_control_ros2/msg/damiao_motor_state.hpp"
#include "motor_control_ros2/msg/damiao_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_motor_state.hpp"
#include "motor_control_ros2/msg/unitree_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>

// ANSI 颜色代码
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_DIM     "\033[2m"

// 清屏和光标控制
#define CLEAR_SCREEN  "\033[2J"
#define CURSOR_HOME   "\033[H"
#define CURSOR_HIDE   "\033[?25l"
#define CURSOR_SHOW   "\033[?25h"

namespace motor_control {

/**
 * @brief 电机控制节点
 * 
 * C++ 实时控制节点，负责：
 * - 管理多个电机
 * - 500Hz 控制循环
 * - CAN 通信
 * - 串口通信 (宇树电机)
 * - 发布电机状态
 */
class MotorControlNode : public rclcpp::Node {
public:
  MotorControlNode() : Node("motor_control_node") {
    // 声明参数
    this->declare_parameter("control_frequency", 500.0);
    this->declare_parameter("publish_frequency", 100.0);
    this->declare_parameter("config_file", "");
    
    // 获取配置文件路径
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      // 使用默认路径
      config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + "/config/motors.yaml";
    }
    
    // 初始化 CAN 网络
    can_network_ = std::make_shared<hardware::CANNetwork>();
    
    // 设置 CAN 接收回调
    can_network_->setGlobalRxCallback(
      std::bind(&MotorControlNode::canRxCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3)
    );
    
    // 从配置文件初始化电机
    try {
      initializeFromConfig(config_file);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "配置加载失败: %s", e.what());
      RCLCPP_WARN(this->get_logger(), "使用默认配置（仅示例电机）");
      initializeMotors();  // 回退到硬编码配置
    }
    
    // 启动 CAN 接收
    can_network_->startAll();
    
    // 创建发布者
    createPublishers();
    
    // 创建订阅者
    createSubscribers();
    
    // 启动控制循环
    double control_freq = this->get_parameter("control_frequency").as_double();
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / control_freq)),
      std::bind(&MotorControlNode::controlLoop, this)
    );
    
    // 启动发布循环
    double publish_freq = this->get_parameter("publish_frequency").as_double();
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000 / publish_freq)),
      std::bind(&MotorControlNode::publishStates, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动 - 控制频率: %.1f Hz, 发布频率: %.1f Hz",
                control_freq, publish_freq);
  }
  
  ~MotorControlNode() {
    can_network_->stopAll();
    can_network_->closeAll();
    
    for (auto& [name, port] : serial_ports_) {
       port->close();
    }
  }

private:
  void initializeMotors() {
    // 示例：添加一个 CAN 总线
    can_network_->addInterface("can0", "/dev/ttyACM0", 921600);
    
    // 示例：添加一个 DJI GM6020 电机
    auto dji_motor = std::make_shared<DJIMotor>(
      "yaw_motor", MotorType::DJI_GM6020, 1, 0
    );
    motors_["yaw_motor"] = dji_motor;
    dji_motors_.push_back(dji_motor);
    
    RCLCPP_INFO(this->get_logger(), "已初始化主要电机");
  }

  // 辅助函数：获取或创建串口
  std::shared_ptr<hardware::SerialInterface> getSerialPort(const std::string& port_name) {
    if (serial_ports_.find(port_name) == serial_ports_.end()) {
      auto port = std::make_shared<hardware::SerialInterface>(port_name, 4000000);
      if (port->open()) {
        serial_ports_[port_name] = port;
        RCLCPP_INFO(this->get_logger(), "已打开串口: %s", port_name.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", port_name.c_str());
        return nullptr;
      }
    }
    return serial_ports_[port_name];
  }

  void initializeUnitreeMotors() {
    // 示例代码已注释，请根据实际配置添加电机
    // 如需添加宇树电机，请取消注释并修改参数
    
    /*
    std::string port_name = "/dev/ttyUSB0";
    auto port = getSerialPort(port_name);
    
    if (port) {
      auto unitree_motor = std::make_shared<UnitreeMotor>(
        "fl_hip_motor", MotorType::UNITREE_A1, port, 0, 1, 0.0f
      );
      motors_["fl_hip_motor"] = unitree_motor;
      unitree_motors_.push_back(unitree_motor);
      RCLCPP_INFO(this->get_logger(), "已添加宇树 A1 电机: fl_hip_motor");
    }
    */
  }
  
  /**
   * @brief 从配置文件初始化电机
   */
  void initializeFromConfig(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());
    
    // 加载配置
    SystemConfig config = ConfigParser::loadConfig(config_file);
    
    // 初始化 CAN 接口和电机
    for (const auto& can_config : config.can_interfaces) {
      // 添加 CAN 接口
      std::string interface_name = "can_" + std::to_string(can_interfaces_count_++);
      can_network_->addInterface(interface_name, can_config.device, can_config.baudrate);
      
      // 添加电机
      for (const auto& motor_config : can_config.motors) {
        addMotorFromConfig(motor_config, interface_name);
      }
    }
    
    // 初始化串口接口和电机
    for (const auto& serial_config : config.serial_interfaces) {
      // 创建串口
      auto port = std::make_shared<hardware::SerialInterface>(
        serial_config.device, serial_config.baudrate
      );
      
      if (port->open()) {
        serial_ports_[serial_config.device] = port;
        RCLCPP_INFO(this->get_logger(), "已打开串口: %s @ %d bps", 
                    serial_config.device.c_str(), serial_config.baudrate);
        
        // 添加电机
        for (const auto& motor_config : serial_config.motors) {
          addUnitreeMotorFromConfig(motor_config, port);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", serial_config.device.c_str());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "配置加载完成 - DJI 电机: %zu, 宇树电机: %zu", 
                dji_motors_.size(), unitree_motors_.size());
  }
  
  /**
   * @brief 从配置添加 CAN 电机
   */
  void addMotorFromConfig(const MotorConfig& config, const std::string& interface_name) {
    (void)interface_name;  // 暂时未使用，保留以便将来扩展
    MotorType motor_type;
    
    // 解析电机类型
    if (config.type == "GM6020") {
      motor_type = MotorType::DJI_GM6020;
    } else if (config.type == "GM3508") {
      motor_type = MotorType::DJI_GM3508;
    } else if (config.type == "DM4340") {
      motor_type = MotorType::DAMIAO_DM4340;
    } else if (config.type == "DM4310") {
      motor_type = MotorType::DAMIAO_DM4310;
    } else {
      RCLCPP_ERROR(this->get_logger(), "未知的电机类型: %s", config.type.c_str());
      return;
    }
    
    // 创建电机
    if (motor_type == MotorType::DJI_GM6020 || motor_type == MotorType::DJI_GM3508) {
      auto motor = std::make_shared<DJIMotor>(config.name, motor_type, config.id, 0);
      motors_[config.name] = motor;
      dji_motors_.push_back(motor);
      RCLCPP_INFO(this->get_logger(), "添加 DJI 电机: %s (%s, ID=%d)", 
                  config.name.c_str(), config.type.c_str(), config.id);
    } else if (motor_type == MotorType::DAMIAO_DM4340 || motor_type == MotorType::DAMIAO_DM4310) {
      auto motor = std::make_shared<DamiaoMotor>(config.name, motor_type, config.id, 0);
      motors_[config.name] = motor;
      RCLCPP_INFO(this->get_logger(), "添加达妙电机: %s (%s, ID=%d)", 
                  config.name.c_str(), config.type.c_str(), config.id);
    }
  }
  
  /**
   * @brief 从配置添加宇树电机
   */
  void addUnitreeMotorFromConfig(const MotorConfig& config, 
                                   std::shared_ptr<hardware::SerialInterface> port) {
    MotorType motor_type;
    
    // 解析电机类型
    if (config.type == "A1") {
      motor_type = MotorType::UNITREE_A1;
    } else if (config.type == "GO8010") {
      motor_type = MotorType::UNITREE_GO8010;
    } else {
      RCLCPP_ERROR(this->get_logger(), "未知的宇树电机类型: %s", config.type.c_str());
      return;
    }
    
    // 创建电机
    auto motor = std::make_shared<UnitreeMotor>(
      config.name, motor_type, port, config.id, config.direction, config.offset
    );
    motors_[config.name] = motor;
    unitree_motors_.push_back(motor);
    
    RCLCPP_INFO(this->get_logger(), "添加宇树电机: %s (%s, ID=%d)", 
                config.name.c_str(), config.type.c_str(), config.id);
  }
  
  void createPublishers() {
    // DJI 电机状态发布者
    dji_state_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorState>(
      "dji_motor_states", 10
    );
    
    // 达妙电机状态发布者
    damiao_state_pub_ = this->create_publisher<motor_control_ros2::msg::DamiaoMotorState>(
      "damiao_motor_states", 10
    );

    // 宇树电机状态发布者
    unitree_state_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeMotorState>(
      "unitree_motor_states", 10
    );
    unitree_go_state_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010State>(
      "unitree_go8010_states", 10
    );
  }
  
  void createSubscribers() {
    // DJI 电机命令订阅者
    dji_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorCommand>(
      "dji_motor_command", 10,
      std::bind(&MotorControlNode::djiCommandCallback, this, std::placeholders::_1)
    );
    
    // 达妙电机命令订阅者
    damiao_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DamiaoMotorCommand>(
      "damiao_motor_command", 10,
      std::bind(&MotorControlNode::damiaoCommandCallback, this, std::placeholders::_1)
    );
    
    // 宇树电机命令订阅者 (A1)
    unitree_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeMotorCommand>(
      "unitree_motor_command", 10,
      std::bind(&MotorControlNode::unitreeCommandCallback, this, std::placeholders::_1)
    );

    // 宇树电机命令订阅者 (GO-8010)
    unitree_go_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010Command>(
      "unitree_go8010_command", 10,
      std::bind(&MotorControlNode::unitreeGOCommandCallback, this, std::placeholders::_1)
    );
  }
  
  void canRxCallback(uint32_t can_id, const uint8_t* data, size_t len) {
    // 调试日志：显示接收到的 CAN 帧
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[CAN RX] ID: 0x%03X, Len: %zu, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                         can_id, len, 
                         data[0], data[1], data[2], data[3], 
                         data[4], data[5], data[6], data[7]);
    
    // 将 CAN 帧分发给对应的电机
    for (auto& [name, motor] : motors_) {
      motor->updateFeedback(can_id, data, len);
    }
  }
  
  void controlLoop() {
    // 500Hz 控制循环
    
    // 1. 读取电机状态
    readUnitreeMotors();
    
    // 2. 写入电机命令
    writeDJIMotors();
    writeDamiaoMotors();
    writeUnitreeMotors();
  }

  void readUnitreeMotors() {
    for (auto& motor : unitree_motors_) {
      motor->receiveFeedback();
    }
  }

  void writeUnitreeMotors() {
    for (auto& motor : unitree_motors_) {
      motor->sendCommand();
    }
  }
  
  void writeDJIMotors() {
    if (dji_motors_.empty()) return;
    
    // DJI 电机需要拼包发送
    std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>> groups;
    
    for (auto& motor : dji_motors_) {
      groups[motor->getControlId()].push_back(motor);
    }
    
    for (auto& [control_id, motors] : groups) {
      uint8_t data[8] = {0};
      
      for (auto& motor : motors) {
        uint8_t motor_id = motor->getMotorId();
        uint8_t bytes[2];
        motor->getControlBytes(bytes);
        
        int offset = ((motor_id - 1) % 4) * 2;
        data[offset] = bytes[0];
        data[offset + 1] = bytes[1];
      }
      
      // 调试日志：显示发送的 CAN 帧
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[CAN TX] ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                           control_id,
                           data[0], data[1], data[2], data[3],
                           data[4], data[5], data[6], data[7]);
      
      can_network_->send("can0", control_id, data, 8);
    }
  }
  
  void writeDamiaoMotors() {
    for (auto& [name, motor] : motors_) {
      if (motor->getMotorType() == MotorType::DAMIAO_DM4340 ||
          motor->getMotorType() == MotorType::DAMIAO_DM4310) {
        
        auto damiao = std::dynamic_pointer_cast<DamiaoMotor>(motor);
        if (damiao) {
          uint32_t can_id;
          uint8_t data[8];
          size_t len;
          
          damiao->getControlFrame(can_id, data, len);
          if (len > 0) {
            can_network_->send("can0", can_id, data, len);
          }
        }
      }
    }
  }
  
  
  void publishStates() {
    auto now = this->now();
    int64_t current_time_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    double heartbeat_timeout_ms = 500.0;  // 500ms 超时
    
    // 检查所有电机的心跳超时
    for (auto& motor : dji_motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);
    }
    for (auto& [name, motor] : motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);
    }
    for (auto& motor : unitree_motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);
    }
    
    // 发布 DJI 电机状态
    for (auto& motor : dji_motors_) {
      auto msg = motor_control_ros2::msg::DJIMotorState();
      msg.header.stamp = now;
      msg.joint_name = motor->getJointName();
      msg.model = (motor->getMotorType() == MotorType::DJI_GM6020) ? "GM6020" : "GM3508";
      msg.online = motor->isOnline();
      msg.angle = motor->getOutputPosition() * 180.0 / M_PI;
      msg.temperature = static_cast<uint8_t>(motor->getTemperature());
      dji_state_pub_->publish(msg);
    }
    
    // 发布达妙电机状态
    for (auto& [name, motor] : motors_) {
      if (motor->getMotorType() == MotorType::DAMIAO_DM4340 ||
          motor->getMotorType() == MotorType::DAMIAO_DM4310) {
        auto damiao = std::dynamic_pointer_cast<DamiaoMotor>(motor);
        if (damiao) {
          auto msg = motor_control_ros2::msg::DamiaoMotorState();
          msg.header.stamp = now;
          msg.joint_name = damiao->getJointName();
          msg.online = damiao->isOnline();
          msg.position = damiao->getOutputPosition();
          msg.velocity = damiao->getOutputVelocity();
          msg.torque = damiao->getOutputTorque();
          msg.temp_mos = static_cast<uint8_t>(damiao->getTemperature());
          msg.temp_rotor = 0;  // 暂时设为0，需要从电机获取
          msg.error_code = 0;
          damiao_state_pub_->publish(msg);
        }
      }
    }
    
    // 发布宇树电机状态
    for (auto& motor : unitree_motors_) {
      if (motor->getMotorType() == MotorType::UNITREE_A1) {
        auto msg = motor_control_ros2::msg::UnitreeMotorState();
        msg.header.stamp = now;
        msg.joint_name = motor->getJointName();
        msg.online = motor->isOnline();
        msg.position = motor->getOutputPosition();
        msg.velocity = motor->getOutputVelocity();
        msg.torque = motor->getOutputTorque();
        msg.temperature = static_cast<int8_t>(motor->getTemperature());
        unitree_state_pub_->publish(msg);
      } else if (motor->getMotorType() == MotorType::UNITREE_GO8010) {
        auto msg = motor_control_ros2::msg::UnitreeGO8010State();
        msg.header.stamp = now;
        msg.joint_name = motor->getJointName();
        msg.online = motor->isOnline();
        msg.position = motor->getOutputPosition();
        msg.velocity = motor->getOutputVelocity();
        msg.torque = motor->getOutputTorque();
        msg.temperature = static_cast<int8_t>(motor->getTemperature());
        unitree_go_state_pub_->publish(msg);
      }
    }
  }
  
  void djiCommandCallback(const motor_control_ros2::msg::DJIMotorCommand::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), 
                "[CMD] 收到 DJI 命令: joint='%s', output=%d",
                msg->joint_name.c_str(), msg->output);
    
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
      if (dji) {
        dji->setOutput(msg->output);
        RCLCPP_INFO(this->get_logger(), 
                    "[CMD] 已设置电机 '%s' 输出为 %d",
                    msg->joint_name.c_str(), msg->output);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "[CMD] 未找到电机: %s", msg->joint_name.c_str());
    }
  }
  
  void damiaoCommandCallback(const motor_control_ros2::msg::DamiaoMotorCommand::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto damiao = std::dynamic_pointer_cast<DamiaoMotor>(it->second);
      if (damiao) {
        damiao->setMITCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
      }
    }
  }
  
  void unitreeCommandCallback(const motor_control_ros2::msg::UnitreeMotorCommand::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotor>(it->second);
      if (motor) {
        motor->setHybridCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
        if (msg->mode == 10) motor->enable();
        else if (msg->mode == 0) motor->disable();
      }
    }
  }

  void unitreeGOCommandCallback(const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it != motors_.end()) {
      auto motor = std::dynamic_pointer_cast<UnitreeMotor>(it->second);
      if (motor) {
        motor->setHybridCommand(msg->pos_des, msg->vel_des, msg->kp, msg->kd, msg->torque_ff);
         if (msg->mode == 10) motor->enable();
        else if (msg->mode == 0) motor->disable();
      }
    }
  }

private:
  std::shared_ptr<hardware::CANNetwork> can_network_;
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<DJIMotor>> dji_motors_;
  
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorState>::SharedPtr dji_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::DamiaoMotorState>::SharedPtr damiao_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeMotorState>::SharedPtr unitree_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr unitree_go_state_pub_;
  
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommand>::SharedPtr dji_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DamiaoMotorCommand>::SharedPtr damiao_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeMotorCommand>::SharedPtr unitree_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr unitree_go_cmd_sub_;

  std::map<std::string, std::shared_ptr<hardware::SerialInterface>> serial_ports_;
  std::vector<std::shared_ptr<UnitreeMotor>> unitree_motors_;
  
  // 配置相关
  int can_interfaces_count_ = 0;
};

} // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
