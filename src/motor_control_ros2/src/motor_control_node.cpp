#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <atomic>

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/dji_motor.hpp"
#include "motor_control_ros2/damiao_motor.hpp"
// #include "motor_control_ros2/unitree_motor.hpp"  // SDK版本已禁用
#include "motor_control_ros2/unitree_motor_native.hpp"  // 原生协议宇树电机
#include "motor_control_ros2/hardware/can_interface.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"  // 原生串口接口
#include "motor_control_ros2/config_parser.hpp"

// 注释掉 SDK 依赖，使用原生协议
// #include "unitreeMotor/unitreeMotor.h"
// #include "serialPort/SerialPort.h"

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
    
    // 初始化串口网络（原生协议）
    serial_network_ = std::make_shared<hardware::SerialNetwork>();
    
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
    last_dji_tx_report_time_ = this->now();

    // 启动串口独立通信线程（每个串口接口一个线程，并行收发，不阻塞主控制循环）
    startSerialThreads();
    
    RCLCPP_INFO(this->get_logger(), "电机控制节点已启动 - 控制频率: %.1f Hz, 宇树电机(原生): %zu, 串口线程: %zu",
                control_freq, unitree_native_motors_.size(), serial_comm_threads_.size());
  }
  
  ~MotorControlNode() {
    // 先停止串口通信线程
    stopSerialThreads();

    can_network_->stopAll();
    can_network_->closeAll();
    
    // 关闭原生串口网络
    serial_network_->closeAll();
  }

private:

  void  initializeFromConfig(const std::string& config_file) {
    
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
    
    // 初始化串口接口和电机（使用原生协议）
    for (const auto& serial_config : config.serial_interfaces) {
      std::string interface_name = "serial_" + std::to_string(serial_interfaces_count_++);
      
      // 添加串口接口（原生协议）
      if (!serial_network_->addInterface(interface_name, serial_config.device, serial_config.baudrate)) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", serial_config.device.c_str());
        continue;
      }

      // 添加电机
      for (const auto& motor_config : serial_config.motors) {
        addUnitreeNativeMotorFromConfig(motor_config, interface_name, serial_config.device);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "配置加载完成 - DJI 电机: %zu, 宇树电机(原生): %zu",
                dji_motors_.size(), unitree_native_motors_.size());
  }

  /**
   * @brief 从配置添加宇树电机（原生协议，不依赖SDK）
   */
  void addUnitreeNativeMotorFromConfig(const MotorConfig& config,
                                       const std::string& interface_name,
                                       const std::string& device_path) {
    auto motor = std::make_shared<UnitreeMotorNative>(
      config.name,
      static_cast<uint8_t>(config.id),
      config.gear_ratio
    );

    motor->setInterfaceName(interface_name);
    motor->setDevicePath(device_path);

    // 保存方向/零位偏移配置（用于状态发布与命令变换）
    int direction = (config.direction >= 0) ? 1 : -1;
    unitree_direction_[config.name] = direction;
    unitree_offset_[config.name] = config.offset;

    motors_[config.name] = motor;
    unitree_native_motors_.push_back(motor);

    RCLCPP_INFO(this->get_logger(),
                "添加宇树电机(原生): %s (ID=%d, 方向=%d, offset=%.4f rad, 齿轮比=%.2f) -> %s",
                config.name.c_str(), config.id,
                direction, config.offset, config.gear_ratio, interface_name.c_str());
  }

  int getUnitreeDirection(const std::string& joint_name) const {
    auto it = unitree_direction_.find(joint_name);
    return (it != unitree_direction_.end()) ? it->second : 1;
  }

  double getUnitreeOffset(const std::string& joint_name) const {
    auto it = unitree_offset_.find(joint_name);
    return (it != unitree_offset_.end()) ? it->second : 0.0;
  }

  double applyUnitreePosition(const std::string& joint_name, double raw_position) const {
    // 统一语义：输出位置 = 原始位置 * direction - offset
    return raw_position * static_cast<double>(getUnitreeDirection(joint_name)) - getUnitreeOffset(joint_name);
  }

  double applyUnitreeVelocity(const std::string& joint_name, double raw_velocity) const {
    return raw_velocity * static_cast<double>(getUnitreeDirection(joint_name));
  }

  double applyUnitreeTorque(const std::string& joint_name, double raw_torque) const {
    return raw_torque * static_cast<double>(getUnitreeDirection(joint_name));
  }

  double outputToRawUnitreePosition(const std::string& joint_name, double output_position) const {
    // 与 applyUnitreePosition 互逆：raw = (output + offset) / direction
    int dir = getUnitreeDirection(joint_name);
    double offset = getUnitreeOffset(joint_name);
    return (output_position + offset) / static_cast<double>(dir);
  }

  /**
   * @brief 从配置添加 CAN 电机
   */
  void  addMotorFromConfig(const MotorConfig& config, const std::string& interface_name) {
    MotorType motor_type;
    
    // 解析电机类型
    if (config.type == "GM6020") {
      motor_type = MotorType::DJI_GM6020;
    } else if (config.type == "GM3508") {
      motor_type = MotorType::DJI_GM3508;
    } else if (config.type == "DM4340") {
      motor_type = MotorType::DAMIAO_DM4340;
    } 
     else {
      RCLCPP_ERROR(this->get_logger(), "未知的电机类型: %s", config.type.c_str());
      return;
    }
    
    // 创建电机
    if (motor_type == MotorType::DJI_GM6020 || motor_type == MotorType::DJI_GM3508) {
      auto motor = std::make_shared<DJIMotor>(config.name, motor_type, config.id, 0);
      motor->setInterfaceName(interface_name);  // 设置接口名称
      motor->setDirection(config.direction);    // 设置方向（对称安装时用-1）
      motor->setOffset(config.offset);          // 设置零位偏移（弧度）
      motors_[config.name] = motor;
      dji_motors_.push_back(motor);
      if (!config.mirror_from.empty()) {
        dji_mirror_map_[config.name] = config.mirror_from;
      }
      RCLCPP_INFO(this->get_logger(), "添加 DJI 电机: %s (%s, ID=%d, dir=%d, offset=%.4f rad%s) -> %s", 
                  config.name.c_str(), config.type.c_str(), config.id, config.direction, config.offset,
                  config.mirror_from.empty() ? "" : (", mirror=" + config.mirror_from).c_str(),
                  interface_name.c_str());
    } else if (motor_type == MotorType::DAMIAO_DM4340 || motor_type == MotorType::DAMIAO_DM4310) {
      auto motor = std::make_shared<DamiaoMotor>(config.name, motor_type, config.id, 0);
      motor->setInterfaceName(interface_name);  // 设置接口名称
      motors_[config.name] = motor;
      RCLCPP_INFO(this->get_logger(), "添加达妙电机: %s (%s, ID=%d) -> %s", 
                  config.name.c_str(), config.type.c_str(), config.id, interface_name.c_str());
    }
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
    
    // 1. 更新 DJI 电机控制器（PID 计算）
    // 与 Python 实现一致，不传递 dt 参数
    for (auto& motor : dji_motors_) {
      motor->updateController();
    }
    
    // 2. 写入 CAN 电机命令
    writeDJIMotors();
    writeDamiaoMotors();
    
    // 3. 宇树电机（原生协议）— 已移至独立串口线程并行通信
    //    每个串口接口一个线程，三路并行，不再阻塞主控制循环
    //    反馈数据由串口线程实时更新到 UnitreeMotorNative 内部状态
    
    // 4. 发布电机状态（每次控制循环都发布）
    publishStates();
  }


 
  
  /**
   * @brief 串口通信线程函数（每个串口接口一个线程，并行收发）
   *
   * 架构改进：从主控制循环中解耦串口 I/O
   * - 旧架构：controlLoop → writeUnitreeNativeMotors（阻塞 15-48ms）→ 主循环降至 20-60Hz
   * - 新架构：独立线程并行轮询，主循环恢复 200Hz，串口每路 ~200Hz
   *
   * 分层不变：
   * 1. 协议层（UnitreeMotorNative）：构建命令包 / 解析反馈包（mutex 保护）
   * 2. 硬件层（SerialInterface）：发送接收
   */
  void serialInterfaceLoop(const std::string& iface_name,
                           std::vector<std::shared_ptr<UnitreeMotorNative>> motors) {
    constexpr size_t FRAME_LEN = 16;
    constexpr size_t BUF_SIZE  = 48;

    RCLCPP_INFO(this->get_logger(),
        "[Serial Thread] 启动: %s, 电机数: %zu", iface_name.c_str(), motors.size());

    // 线程局部诊断计数器（避免 static map 的线程安全问题）
    std::map<std::string, int> poll_cnt;
    std::map<std::string, int> diag_cnt;
    std::map<std::string, int> fail_cnt;

    while (serial_running_.load(std::memory_order_relaxed)) {
      for (auto& motor : motors) {
        if (!serial_running_.load(std::memory_order_relaxed)) break;

        auto serial = serial_network_->getInterface(iface_name);
        if (!serial || !serial->isOpen()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "[Serial Thread] %s: 串口未就绪 (%s)",
                              motor->getJointName().c_str(), iface_name.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }

        // 诊断：轮询计数
        auto& pcnt = poll_cnt[motor->getJointName()];
        pcnt++;
        if (pcnt % 200 == 1) {
          fprintf(stderr, "[DIAG-T] %s poll#%d iface=%s isOpen=%d\n",
                  motor->getJointName().c_str(), pcnt,
                  iface_name.c_str(), serial->isOpen() ? 1 : 0);
        }

        // 协议层：构建命令包
        uint8_t cmd[17];
        motor->getCommandPacket(cmd);

        uint8_t response[BUF_SIZE];
        ssize_t last_n = 0;

        // 发送接收 + 帧扫描解析
        // wait_ms=4: USB适配器往返需要~3-4ms，实测4ms→0%失败
        // timeout_ms=8: 缩短超时（原12ms），数据在wait后几乎立即到达
        auto try_recv_parse = [&](uint8_t (&buf)[BUF_SIZE]) -> bool {
          ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, BUF_SIZE, 4, 15);
          last_n = n;
          if (n <= 0) return false;
          for (ssize_t off = 0; off + static_cast<ssize_t>(FRAME_LEN) <= n; ++off) {
            if (buf[off] == 0xFD && buf[off + 1] == 0xEE) {
              if (motor->parseFeedback(&buf[off], FRAME_LEN)) {
                return true;
              }
            }
          }
          return false;
        };

        bool ok = try_recv_parse(response);

        // 诊断：通信结果（每秒打印一次）
        auto& dcnt = diag_cnt[motor->getJointName()];
        auto& fails = fail_cnt[motor->getJointName()];
        dcnt++;
        if (!ok) fails++;
        if (dcnt % 200 == 0) {
          fprintf(stderr, "[DIAG-T] %s: poll#%d ok=%d last_n=%zd fails=%d head=[",
                  motor->getJointName().c_str(), dcnt, ok ? 1 : 0, last_n, fails);
          for (ssize_t i = 0; i < std::min(last_n, (ssize_t)8); ++i) {
            fprintf(stderr, "%02X ", response[i]);
          }
          fprintf(stderr, "]\n");
        }

        if (!ok) {
          // 优化重试：仅重试一次（减少延迟放大），不再重试3次
          ok = try_recv_parse(response);
          if (ok) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[Serial Thread] %s: 重试成功", motor->getJointName().c_str());
          } else {
            std::string hex_dump;
            for (ssize_t i = 0; i < std::min(last_n, (ssize_t)8); ++i) {
              char hex[8];
              snprintf(hex, sizeof(hex), "%02X ", response[i]);
              hex_dump += hex;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[Serial Thread] %s: 通信失败 (recv=%zd bytes, head=[%s]) iface=%s",
                motor->getJointName().c_str(), last_n,
                hex_dump.c_str(), iface_name.c_str());
          }
        }

        if (ok) {
          const std::string joint = motor->getJointName();
          const double pos = applyUnitreePosition(joint, motor->getOutputPosition());
          const double vel = applyUnitreeVelocity(joint, motor->getOutputVelocity());
          const double tor = applyUnitreeTorque(joint, motor->getOutputTorque());

          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "[Serial Thread] %s: Pos=%.3f° Vel=%.3f rad/s Tor=%.3f Nm Tmp=%d°C Online=%d",
              motor->getJointName().c_str(),
              pos * 180.0 / M_PI,
              vel,
              tor,
              (int)motor->getTemperature(),
              motor->isOnline() ? 1 : 0);
        }
      }
      // 无需额外 sleep — sendRecvAccumulate 本身已耗时 ~4ms/电机
    }

    RCLCPP_INFO(this->get_logger(), "[Serial Thread] 退出: %s", iface_name.c_str());
  }

  /**
   * @brief 启动串口并行通信线程（每个串口接口一个线程）
   */
  void startSerialThreads() {
    if (unitree_native_motors_.empty()) return;

    // 按串口接口名称分组电机
    std::map<std::string, std::vector<std::shared_ptr<UnitreeMotorNative>>> iface_motors;
    for (auto& motor : unitree_native_motors_) {
      iface_motors[motor->getInterfaceName()].push_back(motor);
    }

    serial_running_.store(true, std::memory_order_release);

    for (auto& [name, motors] : iface_motors) {
      serial_comm_threads_.emplace_back(
          &MotorControlNode::serialInterfaceLoop, this, name, motors);
      RCLCPP_INFO(this->get_logger(),
          "[Serial] 为接口 %s 启动独立通信线程（%zu 个电机）",
          name.c_str(), motors.size());
    }

    RCLCPP_INFO(this->get_logger(),
        "[Serial] 共启动 %zu 个串口并行通信线程（三路并行，不再阻塞主循环）",
        serial_comm_threads_.size());
  }

  /**
   * @brief 停止所有串口通信线程
   */
  void stopSerialThreads() {
    serial_running_.store(false, std::memory_order_release);
    for (auto& t : serial_comm_threads_) {
      if (t.joinable()) t.join();
    }
    serial_comm_threads_.clear();
    RCLCPP_INFO(this->get_logger(), "[Serial] 所有串口通信线程已停止");
  }

  void writeDJIMotors() {
    if (dji_motors_.empty()) 
    {return;}
    
    // 发送频率统计（使用成员变量，避免 static 在多接口下不独立）
    dji_tx_count_++;
    
    auto now_tx = this->now();
    double dt_tx = (now_tx - last_dji_tx_report_time_).seconds();
    if (dt_tx >= 1.0) {  // 每秒更新一次
      actual_can_tx_freq_ = dji_tx_count_ / dt_tx;
      dji_tx_count_ = 0;
      last_dji_tx_report_time_ = now_tx;
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
        
        // 只发送控制帧，反馈由接收线程统一处理
        // 避免 sendRecv 与接收线程竞争串口数据导致编码器帧丢失
        can_network_->send(interface_name, control_id, data, 8);
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
            can_network_->send(damiao->getInterfaceName(), can_id, data, len);
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
    for (auto& motor : unitree_native_motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);
    }
    
    // 发布 DJI 电机状态
    for (auto& motor : dji_motors_) {
      auto msg = motor_control_ros2::msg::DJIMotorState();
      msg.header.stamp = now;
      msg.joint_name = motor->getJointName();
      msg.model = (motor->getMotorType() == MotorType::DJI_GM6020) ? "GM6020" : "GM3508";
      msg.online = motor->isOnline();
      // 如果配置了镜像源，使用源电机的角度（方向和偏移仍由自身配置决定）
      auto mirror_it = dji_mirror_map_.find(motor->getJointName());
      if (mirror_it != dji_mirror_map_.end()) {
        auto src_it = motors_.find(mirror_it->second);
        if (src_it != motors_.end()) {
          auto src_dji = std::dynamic_pointer_cast<DJIMotor>(src_it->second);
          if (src_dji) {
            // 镜像模式：直接使用源电机的角度，应用目标电机的 offset
            double src_output_rad = src_dji->getOutputPosition();
            double output_rad = src_output_rad - motor->getOffset();
            double degrees = output_rad * 180.0 / M_PI;
            degrees = fmod(degrees, 360.0);
            if (degrees < 0) degrees += 360.0;
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
    
    // 发布原生协议宇树电机状态
    for (auto& motor : unitree_native_motors_) {
      auto msg = motor_control_ros2::msg::UnitreeGO8010State();
      const std::string joint = motor->getJointName();
      msg.header.stamp = now;
      msg.joint_name = joint;
      msg.motor_id = motor->getMotorId();
      msg.online = motor->isOnline();
      msg.position = applyUnitreePosition(joint, motor->getOutputPosition());
      msg.velocity = applyUnitreeVelocity(joint, motor->getOutputVelocity());
      msg.torque = applyUnitreeTorque(joint, motor->getOutputTorque());
      msg.temperature = static_cast<int8_t>(motor->getTemperature());
      msg.error = motor->getErrorCode();
      unitree_go_state_pub_->publish(msg);
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
  
  void unitreeCommandCallback(const motor_control_ros2::msg::UnitreeMotorCommand::SharedPtr /*msg*/) {
    // SDK版本已禁用，使用 unitreeGOCommand 和原生协议版本
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[CMD] UnitreeMotorCommand 已禁用，请使用 UnitreeGO8010Command");
  }

  void unitreeGOCommandCallback(const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr msg) {
    // 按 ID（和可选的 device）在原生宇树电机列表中查找
    std::vector<std::shared_ptr<UnitreeMotorNative>> matched_motors;
    for (auto& m : unitree_native_motors_) {
      if (m->getMotorId() == msg->id) {
        // 如果指定了 device，只匹配该设备上的电机
        if (!msg->device.empty() && m->getDevicePath() != msg->device) {
          continue;
        }
        matched_motors.push_back(m);
      }
    }

    if (matched_motors.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[CMD GO8010] 未找到电机 ID=%d device='%s'（可能该 ID 未接线或未在 motors.yaml 配置）",
                           msg->id, msg->device.c_str());
      return;
    }

    if (matched_motors.size() > 1 && msg->device.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[CMD GO8010] ID=%d 对应 %zu 台电机（多串口同 ID，未指定 device），广播下发",
                           msg->id, matched_motors.size());
    }

    // 根据控制模式设置命令
    switch (msg->mode) {
      case 0:  // MODE_BRAKE
        for (auto& native_motor : matched_motors) {
          native_motor->setBrakeCommand();
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD GO8010 Native] ID=%d 刹车模式", msg->id);
        break;

      case 1:  // MODE_FOC
        for (auto& native_motor : matched_motors) {
          const std::string& joint = native_motor->getJointName();
          const int dir = getUnitreeDirection(joint);

          const double raw_pos = outputToRawUnitreePosition(joint, msg->position_target);
          const double raw_vel = msg->velocity_target / static_cast<double>(dir);
          const double raw_tau = msg->torque_ff / static_cast<double>(dir);

          native_motor->setFOCCommand(raw_pos, raw_vel, msg->kp, msg->kd, raw_tau);
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD GO8010 Native] ID=%d FOC: pos=%.2f°, vel=%.2f rad/s, kp=%.2f, kd=%.2f, tau=%.3f",
                             msg->id,
                             msg->position_target * 180.0 / M_PI,
                             msg->velocity_target, msg->kp, msg->kd, msg->torque_ff);
        break;

      case 2:  // MODE_CALIBRATE
        for (auto& native_motor : matched_motors) {
          native_motor->setCalibrateCommand();
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD GO8010 Native] ID=%d 校准模式", msg->id);
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "[CMD GO8010 Native] ID=%d 未知模式: %d",
                   msg->id, msg->mode);
        break;
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
        // ROS 消息使用弧度，底层 PID 使用度
        dji->setPositionTarget(msg->position_target * 180.0 / M_PI);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[CMD ADV] %s 位置控制: %.3f rad (%.1f deg)", 
                             msg->joint_name.c_str(), msg->position_target,
                             msg->position_target * 180.0 / M_PI);
        break;
        
      case ControlMode::VELOCITY:
        // ROS 消息使用弧度/秒，底层 PID 使用 RPM
        // 1 rad/s = 60 / (2*PI) RPM ≈ 9.55 RPM
        {
          double rpm = msg->velocity_target * 60.0 / (2.0 * M_PI);
          dji->setVelocityTarget(rpm);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "[CMD ADV] %s 速度控制: %.3f rad/s (%.1f RPM)", 
                               msg->joint_name.c_str(), msg->velocity_target, rpm);
        }
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
  std::shared_ptr<hardware::SerialNetwork> serial_network_;  // 原生串口网络
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<DJIMotor>> dji_motors_;
  std::map<std::string, std::string> dji_mirror_map_;  // target -> source 镜像映射
  
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

  // 原生协议电机（不依赖SDK）
  std::vector<std::shared_ptr<UnitreeMotorNative>> unitree_native_motors_;
  
  // 配置相关
  int can_interfaces_count_ = 0;
  int serial_interfaces_count_ = 0;
  
  // 频率统计
  int control_loop_count_ = 0;
  rclcpp::Time last_freq_report_time_;
  double actual_control_freq_ = 0.0;
  double actual_can_tx_freq_ = 0.0;
  double target_control_freq_ = 200.0;  // 默认值，会被配置文件覆盖
  
  // DJI 发送频率统计（成员变量，避免 static 问题）
  int dji_tx_count_ = 0;
  rclcpp::Time last_dji_tx_report_time_;

  // 串口通信线程（每个串口接口一个线程，并行收发）
  std::vector<std::thread> serial_comm_threads_;
  std::atomic<bool> serial_running_{false};

  // 宇树电机坐标映射（从 motors.yaml 读取）
  std::unordered_map<std::string, int> unitree_direction_;
  std::unordered_map<std::string, double> unitree_offset_;
};

// 补齐 namespace 闭合
}  // namespace motor_control

// 补齐可执行入口
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
