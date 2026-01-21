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
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/damiao_motor_state.hpp"
#include "motor_control_ros2/msg/damiao_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_motor_state.hpp"
#include "motor_control_ros2/msg/unitree_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/control_frequency.hpp"

#include <yaml-cpp/yaml.h>

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
    // 声明参数（使用 200Hz 作为默认值，与 Python 一致）
    this->declare_parameter("control_frequency", 200.0);
    this->declare_parameter("config_file", "");
    
    // 尝试从 control_params.yaml 加载控制参数
    try {
      std::string control_params_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + "/config/control_params.yaml";
      loadControlParams(control_params_file);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "控制参数加载失败: %s，使用默认参数", e.what());
    }
    
    // 获取配置文件路径
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      // 使用默认路径
      config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + "/config/motors.yaml";
    }
    
    // 初始化 CAN 网络
    can_network_ = std::make_shared<hardware::CANNetwork>();
    
    // 设置 CAN 接收回调（添加接口名称参数）
    can_network_->setGlobalRxCallback(
      std::bind(&MotorControlNode::canRxCallback, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4)
    );
    
    // 从配置文件初始化电机
    try {
      initializeFromConfig(config_file);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "配置加载失败: %s", e.what());
      RCLCPP_WARN(this->get_logger(), "使用默认配置（仅示例电机）");
      initializeMotors();  // 回退到硬编码配置
    }
    
    // 加载 PID 参数配置
    std::string pid_config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + "/config/pid_params.yaml";
    try {
      loadPIDParams(pid_config_file);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "PID 参数加载失败: %s，使用默认参数", e.what());
    }
    
    // 启动 CAN 接收
    can_network_->startAll();
    
    // 创建发布者
    createPublishers();
    
    // 创建订阅者
    createSubscribers();
    
    // 启动控制循环（同时发布电机状态）
    double control_freq = this->get_parameter("control_frequency").as_double();
    target_control_freq_ = control_freq;  // 保存目标频率用于发布
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / control_freq)),
      std::bind(&MotorControlNode::controlLoop, this)
    );
    
    // 启动设备重连定时器（每 2 秒检查一次）
    if (can_network_->getPendingCount() > 0) {
      reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MotorControlNode::checkReconnect, this)
      );
      RCLCPP_WARN(this->get_logger(), "有 %zu 个 CAN 设备待连接，将每 2 秒重试",
                  can_network_->getPendingCount());
    }
    
    // 初始化频率统计
    last_freq_report_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动 - 控制频率: %.1f Hz",
                control_freq);
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
      motor->setInterfaceName(interface_name);  // 设置接口名称
      motors_[config.name] = motor;
      dji_motors_.push_back(motor);
      RCLCPP_INFO(this->get_logger(), "添加 DJI 电机: %s (%s, ID=%d) -> %s", 
                  config.name.c_str(), config.type.c_str(), config.id, interface_name.c_str());
    } else if (motor_type == MotorType::DAMIAO_DM4340 || motor_type == MotorType::DAMIAO_DM4310) {
      auto motor = std::make_shared<DamiaoMotor>(config.name, motor_type, config.id, 0);
      motor->setInterfaceName(interface_name);  // 设置接口名称
      motors_[config.name] = motor;
      RCLCPP_INFO(this->get_logger(), "添加达妙电机: %s (%s, ID=%d) -> %s", 
                  config.name.c_str(), config.type.c_str(), config.id, interface_name.c_str());
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
    
    // 控制频率发布者
    control_freq_pub_ = this->create_publisher<motor_control_ros2::msg::ControlFrequency>(
      "control_frequency", 10
    );
  }
  
  void createSubscribers() {
    // DJI 电机命令订阅者（基础命令，向后兼容）
    dji_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorCommand>(
      "dji_motor_command", 10,
      std::bind(&MotorControlNode::djiCommandCallback, this, std::placeholders::_1)
    );
    
    // DJI 电机高级命令订阅者（支持位置/速度/直接输出）
    dji_cmd_advanced_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
      "dji_motor_command_advanced", 10,
      std::bind(&MotorControlNode::djiCommandAdvancedCallback, this, std::placeholders::_1)
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
  
  void canRxCallback(const std::string& interface_name, uint32_t can_id, const uint8_t* data, size_t len) {
    // 调试日志：显示接收到的 CAN 帧（包含接口来源）
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[CAN RX] %s ID: 0x%03X, Len: %zu, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                         interface_name.c_str(), can_id, len, 
                         data[0], data[1], data[2], data[3], 
                         data[4], data[5], data[6], data[7]);
    
    // 将 CAN 帧分发给对应的电机（传递接口名称用于过滤）
    for (auto& [name, motor] : motors_) {
      motor->updateFeedback(interface_name, can_id, data, len);
    }
  }
  
  /**
   * @brief 检查并重试连接失败的 CAN 设备
   */
  void checkReconnect() {
    int connected = can_network_->retryPendingInterfaces();
    
    if (connected > 0) {
      RCLCPP_INFO(this->get_logger(), "成功重连 %d 个 CAN 设备", connected);
    }
    
    // 如果所有设备都已连接，停止重连定时器
    if (can_network_->getPendingCount() == 0) {
      RCLCPP_INFO(this->get_logger(), "所有 CAN 设备已连接，停止重连检测");
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    } else {
      // 输出待连接设备列表
      auto pending = can_network_->getPendingDevices();
      std::string devices_str;
      for (const auto& dev : pending) {
        if (!devices_str.empty()) devices_str += ", ";
        devices_str += dev;
      }
      RCLCPP_DEBUG(this->get_logger(), "待连接设备: %s", devices_str.c_str());
    }
  }
  
  void controlLoop() {
    // 控制循环（频率由 control_frequency 参数决定）
    
    // 频率统计
    control_loop_count_++;
    auto now = this->now();
    double dt = (now - last_freq_report_time_).seconds();
    if (dt >= 1.0) {  // 每秒更新一次
      actual_control_freq_ = control_loop_count_ / dt;
      control_loop_count_ = 0;
      last_freq_report_time_ = now;
    }
    
    // 1. 读取电机状态
    readUnitreeMotors();
    
    // 2. 更新 DJI 电机控制器（PID 计算）
    // 与 Python 实现一致，不传递 dt 参数
    for (auto& motor : dji_motors_) {
      motor->updateController();
    }
    
    // 3. 写入电机命令
    writeDJIMotors();
    writeDamiaoMotors();
    writeUnitreeMotors();
    
    // 4. 发布电机状态（每次控制循环都发布）
    publishStates();
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
    
    // 发送频率统计
    static int tx_count = 0;
    static auto last_tx_report = this->now();
    tx_count++;
    
    auto now_tx = this->now();
    double dt_tx = (now_tx - last_tx_report).seconds();
    if (dt_tx >= 1.0) {  // 每秒更新一次
      actual_can_tx_freq_ = tx_count / dt_tx;
      tx_count = 0;
      last_tx_report = now_tx;
    }
    
    // DJI 电机需要拼包发送
    // 按接口和控制ID分组
    std::map<std::string, std::map<uint32_t, std::vector<std::shared_ptr<DJIMotor>>>> interface_groups;
    
    for (auto& motor : dji_motors_) {
      std::string interface_name = motor->getInterfaceName();
      uint32_t control_id = motor->getControlId();
      interface_groups[interface_name][control_id].push_back(motor);
    }
    
    // 对每个接口的每个控制ID发送（使用 SendRecv 同步模式）
    for (auto& [interface_name, control_id_groups] : interface_groups) {
      for (auto& [control_id, motors] : control_id_groups) {
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
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CAN TX] %s ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                             interface_name.c_str(), control_id,
                             data[0], data[1], data[2], data[3],
                             data[4], data[5], data[6], data[7]);
        
        // ========== SendRecv 同步模式 ==========
        // 发送命令后立即等待反馈（50us + 900us 超时）
        // 这样可以避免数据堆积，确保 PID 及时响应
        hardware::CANFrame response;
        if (can_network_->sendRecv(interface_name, control_id, data, 8, response, 900)) {
          // 收到反馈，立即更新电机状态
          // 注意：DJI 电机的反馈 ID 与控制 ID 不同
          // 反馈会通过 canRxCallback 自动分发到对应电机
          RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "[CAN RX] %s 收到反馈 ID: 0x%03X", 
                               interface_name.c_str(), response.can_id);
        } else {
          // 超时，记录警告
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "[CAN TIMEOUT] %s ID: 0x%03X 未收到反馈", 
                              interface_name.c_str(), control_id);
        }
      }
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
      msg.control_frequency = actual_control_freq_;  // 添加控制频率
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
    
    // 发布控制频率
    auto freq_msg = motor_control_ros2::msg::ControlFrequency();
    freq_msg.header.stamp = now;
    freq_msg.control_frequency = actual_control_freq_;
    freq_msg.can_tx_frequency = actual_can_tx_freq_;
    freq_msg.target_frequency = target_control_freq_;
    control_freq_pub_->publish(freq_msg);
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
  
  /**
   * @brief DJI 电机高级命令回调（支持位置/速度/直接输出）
   */
  void djiCommandAdvancedCallback(const motor_control_ros2::msg::DJIMotorCommandAdvanced::SharedPtr msg) {
    auto it = motors_.find(msg->joint_name);
    if (it == motors_.end()) {
      RCLCPP_WARN(this->get_logger(), "[CMD ADV] 未找到电机: %s", msg->joint_name.c_str());
      return;
    }
    
    auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
    if (!dji) {
      RCLCPP_WARN(this->get_logger(), "[CMD ADV] 电机 %s 不是 DJI 电机", msg->joint_name.c_str());
      return;
    }
    
    // 设置控制模式
    ControlMode mode = static_cast<ControlMode>(msg->mode);
    dji->setControlMode(mode);
    
    // 根据模式设置目标值
    switch (mode) {
      case ControlMode::POSITION:
        dji->setPositionTarget(msg->position_target);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD ADV] %s 位置控制: %.3f rad", 
                             msg->joint_name.c_str(), msg->position_target);
        break;
        
      case ControlMode::VELOCITY:
        dji->setVelocityTarget(msg->velocity_target);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD ADV] %s 速度控制: %.3f rad/s", 
                             msg->joint_name.c_str(), msg->velocity_target);
        break;
        
      case ControlMode::DIRECT:
      default:
        dji->setOutput(msg->direct_output);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD ADV] %s 直接输出: %d", 
                             msg->joint_name.c_str(), msg->direct_output);
        break;
    }
  }
  
  /**
   * @brief 加载控制参数配置
   */
  void loadControlParams(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "正在加载控制参数: %s", config_file.c_str());
    
    YAML::Node config = YAML::LoadFile(config_file);
    
    if (config["motor_control_node"] && config["motor_control_node"]["ros__parameters"]) {
      auto params = config["motor_control_node"]["ros__parameters"];
      
      if (params["control_frequency"]) {
        double freq = params["control_frequency"].as<double>();
        this->set_parameter(rclcpp::Parameter("control_frequency", freq));
        RCLCPP_INFO(this->get_logger(), "控制频率设置为: %.1f Hz", freq);
      }
    }
  }
  
  /**
   * @brief 加载 PID 参数配置
   */
  void loadPIDParams(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "正在加载 PID 参数: %s", config_file.c_str());
    
    YAML::Node config = YAML::LoadFile(config_file);
    
    if (!config["dji_motors"]) {
      RCLCPP_WARN(this->get_logger(), "配置文件中未找到 dji_motors 节点");
      return;
    }
    
    // 加载每种电机类型的默认参数
    std::map<std::string, std::pair<PIDParams, PIDParams>> type_params;
    
    for (auto type_node : config["dji_motors"]) {
      std::string motor_type = type_node.first.as<std::string>();
      
      PIDParams pos_pid, vel_pid;
      
      // 位置环参数
      if (type_node.second["position_pid"]) {
        auto pos = type_node.second["position_pid"];
        pos_pid.kp = pos["kp"].as<double>();
        pos_pid.ki = pos["ki"].as<double>();
        pos_pid.kd = pos["kd"].as<double>();
        pos_pid.i_max = pos["i_max"].as<double>();
        pos_pid.out_max = pos["out_max"].as<double>();
        pos_pid.dead_zone = pos["dead_zone"].as<double>();
      }
      
      // 速度环参数
      if (type_node.second["velocity_pid"]) {
        auto vel = type_node.second["velocity_pid"];
        vel_pid.kp = vel["kp"].as<double>();
        vel_pid.ki = vel["ki"].as<double>();
        vel_pid.kd = vel["kd"].as<double>();
        vel_pid.i_max = vel["i_max"].as<double>();
        vel_pid.out_max = vel["out_max"].as<double>();
        vel_pid.dead_zone = vel["dead_zone"].as<double>();
      }
      
      type_params[motor_type] = {pos_pid, vel_pid};
      
      RCLCPP_INFO(this->get_logger(), "加载 %s 默认 PID 参数: Pos(%.1f,%.1f,%.1f) Vel(%.1f,%.1f,%.1f)",
                  motor_type.c_str(),
                  pos_pid.kp, pos_pid.ki, pos_pid.kd,
                  vel_pid.kp, vel_pid.ki, vel_pid.kd);
    }
    
    // 应用参数到所有 DJI 电机
    for (auto& motor : dji_motors_) {
      std::string motor_type = (motor->getMotorType() == MotorType::DJI_GM6020) ? "GM6020" : "GM3508";
      
      if (type_params.find(motor_type) != type_params.end()) {
        motor->setPositionPID(type_params[motor_type].first);
        motor->setVelocityPID(type_params[motor_type].second);
        
        RCLCPP_INFO(this->get_logger(), "电机 %s (%s) 已应用 PID 参数",
                    motor->getJointName().c_str(), motor_type.c_str());
      }
    }
    
    // 加载电机特定覆盖参数
    if (config["motor_overrides"]) {
      for (auto override_node : config["motor_overrides"]) {
        std::string motor_name = override_node.first.as<std::string>();
        
        auto it = motors_.find(motor_name);
        if (it != motors_.end()) {
          auto dji = std::dynamic_pointer_cast<DJIMotor>(it->second);
          if (dji) {
            // 覆盖位置环参数
            if (override_node.second["position_pid"]) {
              auto pos = override_node.second["position_pid"];
              PIDParams pos_pid;
              pos_pid.kp = pos["kp"] ? pos["kp"].as<double>() : dji->getPositionPIDParams().kp;
              pos_pid.ki = pos["ki"] ? pos["ki"].as<double>() : dji->getPositionPIDParams().ki;
              pos_pid.kd = pos["kd"] ? pos["kd"].as<double>() : dji->getPositionPIDParams().kd;
              pos_pid.i_max = pos["i_max"] ? pos["i_max"].as<double>() : dji->getPositionPIDParams().i_max;
              pos_pid.out_max = pos["out_max"] ? pos["out_max"].as<double>() : dji->getPositionPIDParams().out_max;
              pos_pid.dead_zone = pos["dead_zone"] ? pos["dead_zone"].as<double>() : dji->getPositionPIDParams().dead_zone;
              dji->setPositionPID(pos_pid);
            }
            
            // 覆盖速度环参数
            if (override_node.second["velocity_pid"]) {
              auto vel = override_node.second["velocity_pid"];
              PIDParams vel_pid;
              vel_pid.kp = vel["kp"] ? vel["kp"].as<double>() : dji->getVelocityPIDParams().kp;
              vel_pid.ki = vel["ki"] ? vel["ki"].as<double>() : dji->getVelocityPIDParams().ki;
              vel_pid.kd = vel["kd"] ? vel["kd"].as<double>() : dji->getVelocityPIDParams().kd;
              vel_pid.i_max = vel["i_max"] ? vel["i_max"].as<double>() : dji->getVelocityPIDParams().i_max;
              vel_pid.out_max = vel["out_max"] ? vel["out_max"].as<double>() : dji->getVelocityPIDParams().out_max;
              vel_pid.dead_zone = vel["dead_zone"] ? vel["dead_zone"].as<double>() : dji->getVelocityPIDParams().dead_zone;
              dji->setVelocityPID(vel_pid);
            }
            
            RCLCPP_INFO(this->get_logger(), "电机 %s 已应用覆盖参数", motor_name.c_str());
          }
        }
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "PID 参数加载完成");
  }

private:
  std::shared_ptr<hardware::CANNetwork> can_network_;
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<DJIMotor>> dji_motors_;
  
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;  // 设备重连定时器
  
  rclcpp::Publisher<motor_control_ros2::msg::DJIMotorState>::SharedPtr dji_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::DamiaoMotorState>::SharedPtr damiao_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeMotorState>::SharedPtr unitree_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr unitree_go_state_pub_;
  rclcpp::Publisher<motor_control_ros2::msg::ControlFrequency>::SharedPtr control_freq_pub_;
  
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommand>::SharedPtr dji_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr dji_cmd_advanced_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::DamiaoMotorCommand>::SharedPtr damiao_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeMotorCommand>::SharedPtr unitree_cmd_sub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr unitree_go_cmd_sub_;

  std::map<std::string, std::shared_ptr<hardware::SerialInterface>> serial_ports_;
  std::vector<std::shared_ptr<UnitreeMotor>> unitree_motors_;
  
  // 配置相关
  int can_interfaces_count_ = 0;
  
  // 频率统计
  int control_loop_count_ = 0;
  rclcpp::Time last_freq_report_time_;
  double actual_control_freq_ = 0.0;
  double actual_can_tx_freq_ = 0.0;
  double target_control_freq_ = 200.0;  // 默认值，会被配置文件覆盖
};

} // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
