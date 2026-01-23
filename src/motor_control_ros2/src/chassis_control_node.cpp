#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/steer_wheel_kinematics.hpp"
#include "motor_control_ros2/msg/chassis_state.hpp"
#include "motor_control_ros2/msg/chassis_command.hpp"
#include "motor_control_ros2/msg/steer_wheel_state.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

#include <map>
#include <string>
#include <memory>
#include <chrono>

namespace motor_control {

/**
 * @brief 底盘控制节点
 * 
 * 功能：
 * - 订阅底盘速度命令 (/cmd_vel)
 * - 运动学逆解算 → 4 个舵轮命令
 * - 发布 DJI 电机控制命令
 * - 从电机反馈计算里程计
 * - 发布底盘状态
 */
class ChassisControlNode : public rclcpp::Node {
public:
    ChassisControlNode() : Node("chassis_control_node") {
        RCLCPP_INFO(this->get_logger(), "正在初始化底盘控制节点...");
        
        // 自动加载配置文件
        std::string config_file;
        try {
            config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + 
                         "/config/chassis_params.yaml";
            RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());
            loadChassisParams(config_file);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "底盘控制节点初始化失败: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), 
                "配置文件路径: %s", config_file.c_str());
            RCLCPP_ERROR(this->get_logger(), 
                "请确保配置文件存在且格式正确");
            throw;  // 重新抛出异常,终止节点启动
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "底盘控制节点启动成功 - 轮距: %.3fm, 轴距: %.3fm, 轮半径: %.3fm",
            wheel_base_x_, wheel_base_y_, wheel_radius_);
        RCLCPP_INFO(this->get_logger(), 
            "电机配置 - FL偏移: %.1f°, FR偏移: %.1f°, RL偏移: %.1f°, RR偏移: %.1f°",
            motor_config_.fl_steer_offset, motor_config_.fr_steer_offset,
            motor_config_.rl_steer_offset, motor_config_.rr_steer_offset);
        
        // 初始化时间戳（必须在创建订阅者之前，确保时间源一致）
        last_cmd_time_ = this->now();
        
        // 创建订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ChassisControlNode::cmdVelCallback, this, std::placeholders::_1)
        );
        
        motor_state_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorState>(
            "/dji_motor_states", 10,
            std::bind(&ChassisControlNode::motorStateCallback, this, std::placeholders::_1)
        );
        
        // 创建发布者
        motor_cmd_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
            "/dji_motor_command_advanced", 10
        );
        
        chassis_state_pub_ = this->create_publisher<motor_control_ros2::msg::ChassisState>(
            "/chassis_state", 10
        );
        
        // 创建控制循环定时器
        auto period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ChassisControlNode::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "底盘控制循环启动 - 频率: %.1f Hz", control_frequency_);
    }

private:
    // 电机名称映射
    struct MotorNames {
        std::string fl_steer, fl_drive;
        std::string fr_steer, fr_drive;
        std::string rl_steer, rl_drive;
        std::string rr_steer, rr_drive;
    };
    
    // 电机配置（零位偏移和方向）
    struct MotorConfig {
        double fl_steer_offset, fr_steer_offset, rl_steer_offset, rr_steer_offset;  // 转向零位偏移（度）
        int fl_drive_direction, fr_drive_direction, rl_drive_direction, rr_drive_direction;  // 驱动方向 (1 或 -1)
    };
    
    // 底盘速度命令回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 限制速度
        cmd_vx_ = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
        cmd_vy_ = std::clamp(msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
        cmd_wz_ = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
        
        last_cmd_time_ = this->now();
    }
    
    // 电机状态回调（用于里程计）
    void motorStateCallback(const motor_control_ros2::msg::DJIMotorState::SharedPtr msg) {
        // 存储电机状态
        motor_states_[msg->joint_name] = *msg;
    }
    
    // 控制循环
    void controlLoop() {
        // 检查命令超时（500ms）
        auto now = this->now();
        if ((now - last_cmd_time_).seconds() > 0.5) {
            cmd_vx_ = 0.0;
            cmd_vy_ = 0.0;
            cmd_wz_ = 0.0;
        }
        
        // 判断是否有实际运动命令（速度阈值：0.01 m/s 或 0.01 rad/s）
        const double velocity_threshold = 0.01;
        bool has_motion = (std::abs(cmd_vx_) > velocity_threshold) ||
                         (std::abs(cmd_vy_) > velocity_threshold) ||
                         (std::abs(cmd_wz_) > velocity_threshold);
        
        // 只在有运动命令时才执行控制
        WheelCommand fl_cmd, fr_cmd, rl_cmd, rr_cmd;
        
        if (!has_motion) {
            // 无运动命令时：所有舵轮转到机械零位（正前方，0 度）
            // 注意：这里的 angle 是机械角度，后续会加上 offset 转换为编码器角度
            fl_cmd.angle = 0.0;  // 机械零位 = 0 度（正前方）
            fl_cmd.velocity = 0.0;
            
            fr_cmd.angle = 0.0;
            fr_cmd.velocity = 0.0;
            
            rl_cmd.angle = 0.0;
            rl_cmd.velocity = 0.0;
            
            rr_cmd.angle = 0.0;
            rr_cmd.velocity = 0.0;
        } else {
            // 有运动命令：执行逆运动学解算（得到机械角度）
            kinematics_->inverseKinematics(
                cmd_vx_, cmd_vy_, cmd_wz_,
                fl_cmd, fr_cmd, rl_cmd, rr_cmd
            );
        
            // 舵角优化（最短路径）
            // 坐标转换说明：
            // - offset: 机械零位(正前方)时的编码器角度
            // - 机械角度 = 编码器角度 - offset
            // - 编码器角度 = 机械角度 + offset
            
            if (motor_states_.count(motor_names_.fl_steer)) {
                // 将编码器角度转换为机械角度
                double current_mechanical_angle = motor_states_[motor_names_.fl_steer].angle - motor_config_.fl_steer_offset;
                // 优化舵角（输入输出都是机械角度）
                fl_cmd.angle = SteerWheelKinematics::optimizeSteerAngle(
                    current_mechanical_angle, fl_cmd.angle, fl_cmd.velocity
                );
            }
            
            if (motor_states_.count(motor_names_.fr_steer)) {
                double current_mechanical_angle = motor_states_[motor_names_.fr_steer].angle - motor_config_.fr_steer_offset;
                fr_cmd.angle = SteerWheelKinematics::optimizeSteerAngle(
                    current_mechanical_angle, fr_cmd.angle, fr_cmd.velocity
                );
            }
            
            if (motor_states_.count(motor_names_.rl_steer)) {
                double current_mechanical_angle = motor_states_[motor_names_.rl_steer].angle - motor_config_.rl_steer_offset;
                rl_cmd.angle = SteerWheelKinematics::optimizeSteerAngle(
                    current_mechanical_angle, rl_cmd.angle, rl_cmd.velocity
                );
            }
            
            if (motor_states_.count(motor_names_.rr_steer)) {
                double current_mechanical_angle = motor_states_[motor_names_.rr_steer].angle - motor_config_.rr_steer_offset;
                rr_cmd.angle = SteerWheelKinematics::optimizeSteerAngle(
                    current_mechanical_angle, rr_cmd.angle, rr_cmd.velocity
                );
            }
        }
        
        // 发布电机命令
        publishMotorCommands(fl_cmd, fr_cmd, rl_cmd, rr_cmd);
        
        // 发布底盘状态
        publishChassisState(fl_cmd, fr_cmd, rl_cmd, rr_cmd);
    }
    
    // 发布电机命令
    void publishMotorCommands(
        const WheelCommand& fl, const WheelCommand& fr,
        const WheelCommand& rl, const WheelCommand& rr)
    {
        auto now = this->now();
        
        // 左前舵轮
        publishWheelCommand(motor_names_.fl_steer, motor_names_.fl_drive, fl, now,
                           motor_config_.fl_steer_offset, motor_config_.fl_drive_direction);
        
        // 右前舵轮
        publishWheelCommand(motor_names_.fr_steer, motor_names_.fr_drive, fr, now,
                           motor_config_.fr_steer_offset, motor_config_.fr_drive_direction);
        
        // 左后舵轮
        publishWheelCommand(motor_names_.rl_steer, motor_names_.rl_drive, rl, now,
                           motor_config_.rl_steer_offset, motor_config_.rl_drive_direction);
        
        // 右后舵轮
        publishWheelCommand(motor_names_.rr_steer, motor_names_.rr_drive, rr, now,
                           motor_config_.rr_steer_offset, motor_config_.rr_drive_direction);
    }
    
    // 发布单个舵轮命令
    void publishWheelCommand(
        const std::string& steer_name,
        const std::string& drive_name,
        const WheelCommand& cmd,
        const rclcpp::Time& timestamp,
        double steer_offset,
        int drive_direction)
    {
        // 转向电机命令（位置控制）
        auto steer_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
        steer_msg.header.stamp = timestamp;
        steer_msg.joint_name = steer_name;
        steer_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_POSITION;
        
        // 坐标转换: 机械角度 -> 编码器角度
        // cmd.angle 是机械角度(运动学计算结果)
        // steer_offset 是机械零位时的编码器角度
        // 编码器目标角度 = 机械角度 + offset
        double encoder_angle_deg = cmd.angle + steer_offset;
        steer_msg.position_target = encoder_angle_deg * M_PI / 180.0;  // 转换为弧度
        motor_cmd_pub_->publish(steer_msg);
        
        // 驱动电机命令（速度控制）- 应用方向修正
        // 将线速度转换为 RPM
        double rpm = (cmd.velocity / (2.0 * M_PI * wheel_radius_)) * 60.0;
        
        auto drive_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
        drive_msg.header.stamp = timestamp;
        drive_msg.joint_name = drive_name;
        drive_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
        // 应用方向修正：1=正转向前，-1=正转向后
        drive_msg.velocity_target = rpm * M_PI / 30.0 * drive_direction;  // 转换为弧度/秒并应用方向
        motor_cmd_pub_->publish(drive_msg);
    }
    
    // 发布底盘状态
    void publishChassisState(
        const WheelCommand& fl, const WheelCommand& fr,
        const WheelCommand& rl, const WheelCommand& rr)
    {
        auto msg = motor_control_ros2::msg::ChassisState();
        msg.header.stamp = this->now();
        
        // 从轮子状态计算底盘速度（正运动学）
        WheelCommand fl_actual, fr_actual, rl_actual, rr_actual;
        
        // 从电机反馈获取实际轮子状态
        if (motor_states_.count(motor_names_.fl_steer) && 
            motor_states_.count(motor_names_.fl_drive)) {
            fl_actual.angle = motor_states_[motor_names_.fl_steer].angle;
            fl_actual.velocity = motor_states_[motor_names_.fl_drive].rpm * 0.055;  // RPM to m/s
        }
        
        // ... 类似处理其他轮子 ...
        
        // 正运动学
        kinematics_->forwardKinematics(
            fl_actual, fr_actual, rl_actual, rr_actual,
            msg.linear_x, msg.linear_y, msg.angular_z
        );
        
        // 填充舵轮状态
        fillWheelState(msg.fl_wheel, motor_names_.fl_steer, motor_names_.fl_drive, fl);
        fillWheelState(msg.fr_wheel, motor_names_.fr_steer, motor_names_.fr_drive, fr);
        fillWheelState(msg.rl_wheel, motor_names_.rl_steer, motor_names_.rl_drive, rl);
        fillWheelState(msg.rr_wheel, motor_names_.rr_steer, motor_names_.rr_drive, rr);
        
        chassis_state_pub_->publish(msg);
    }
    
    // 填充舵轮状态
    void fillWheelState(
        motor_control_ros2::msg::SteerWheelState& wheel_state,
        const std::string& steer_name,
        const std::string& drive_name,
        const WheelCommand& cmd)
    {
        wheel_state.steer_motor_name = steer_name;
        wheel_state.drive_motor_name = drive_name;
        wheel_state.steer_angle_target = cmd.angle;
        wheel_state.drive_velocity_target = cmd.velocity;
        
        // 从电机状态获取当前值
        if (motor_states_.count(steer_name)) {
            wheel_state.steer_angle = motor_states_[steer_name].angle;
        }
        
        if (motor_states_.count(drive_name)) {
            wheel_state.drive_velocity = motor_states_[drive_name].rpm * wheel_radius_;  // RPM to m/s
        }
    }
    
    /**
     * @brief 从 YAML 配置文件加载底盘参数
     */
    void loadChassisParams(const std::string& config_file) {
        YAML::Node config = YAML::LoadFile(config_file);
        
        if (!config["chassis_control_node"] || !config["chassis_control_node"]["ros__parameters"]) {
            throw std::runtime_error("配置文件格式错误: 缺少 chassis_control_node/ros__parameters 节点");
        }
        
        auto params = config["chassis_control_node"]["ros__parameters"];
        
        // 读取控制参数
        if (!params["control_frequency"]) {
            throw std::runtime_error("配置文件缺少必需参数: control_frequency");
        }
        control_frequency_ = params["control_frequency"].as<double>();
        
        if (!params["wheel_base_x"]) {
            throw std::runtime_error("配置文件缺少必需参数: wheel_base_x");
        }
        wheel_base_x_ = params["wheel_base_x"].as<double>();
        
        if (!params["wheel_base_y"]) {
            throw std::runtime_error("配置文件缺少必需参数: wheel_base_y");
        }
        wheel_base_y_ = params["wheel_base_y"].as<double>();
        
        if (!params["wheel_radius"]) {
            throw std::runtime_error("配置文件缺少必需参数: wheel_radius");
        }
        wheel_radius_ = params["wheel_radius"].as<double>();
        
        if (!params["max_linear_velocity"]) {
            throw std::runtime_error("配置文件缺少必需参数: max_linear_velocity");
        }
        max_linear_velocity_ = params["max_linear_velocity"].as<double>();
        
        if (!params["max_angular_velocity"]) {
            throw std::runtime_error("配置文件缺少必需参数: max_angular_velocity");
        }
        max_angular_velocity_ = params["max_angular_velocity"].as<double>();
        
        // 读取电机映射和配置 - 左前
        if (!params["fl_steer_motor"]) throw std::runtime_error("配置文件缺少必需参数: fl_steer_motor");
        motor_names_.fl_steer = params["fl_steer_motor"].as<std::string>();
        
        if (!params["fl_drive_motor"]) throw std::runtime_error("配置文件缺少必需参数: fl_drive_motor");
        motor_names_.fl_drive = params["fl_drive_motor"].as<std::string>();
        
        if (!params["fl_steer_offset"]) throw std::runtime_error("配置文件缺少必需参数: fl_steer_offset");
        motor_config_.fl_steer_offset = params["fl_steer_offset"].as<double>();
        
        if (!params["fl_drive_direction"]) throw std::runtime_error("配置文件缺少必需参数: fl_drive_direction");
        motor_config_.fl_drive_direction = params["fl_drive_direction"].as<int>();
        
        // 读取电机映射和配置 - 右前
        if (!params["fr_steer_motor"]) throw std::runtime_error("配置文件缺少必需参数: fr_steer_motor");
        motor_names_.fr_steer = params["fr_steer_motor"].as<std::string>();
        
        if (!params["fr_drive_motor"]) throw std::runtime_error("配置文件缺少必需参数: fr_drive_motor");
        motor_names_.fr_drive = params["fr_drive_motor"].as<std::string>();
        
        if (!params["fr_steer_offset"]) throw std::runtime_error("配置文件缺少必需参数: fr_steer_offset");
        motor_config_.fr_steer_offset = params["fr_steer_offset"].as<double>();
        
        if (!params["fr_drive_direction"]) throw std::runtime_error("配置文件缺少必需参数: fr_drive_direction");
        motor_config_.fr_drive_direction = params["fr_drive_direction"].as<int>();
        
        // 读取电机映射和配置 - 左后
        if (!params["rl_steer_motor"]) throw std::runtime_error("配置文件缺少必需参数: rl_steer_motor");
        motor_names_.rl_steer = params["rl_steer_motor"].as<std::string>();
        
        if (!params["rl_drive_motor"]) throw std::runtime_error("配置文件缺少必需参数: rl_drive_motor");
        motor_names_.rl_drive = params["rl_drive_motor"].as<std::string>();
        
        if (!params["rl_steer_offset"]) throw std::runtime_error("配置文件缺少必需参数: rl_steer_offset");
        motor_config_.rl_steer_offset = params["rl_steer_offset"].as<double>();
        
        if (!params["rl_drive_direction"]) throw std::runtime_error("配置文件缺少必需参数: rl_drive_direction");
        motor_config_.rl_drive_direction = params["rl_drive_direction"].as<int>();
        
        // 读取电机映射和配置 - 右后
        if (!params["rr_steer_motor"]) throw std::runtime_error("配置文件缺少必需参数: rr_steer_motor");
        motor_names_.rr_steer = params["rr_steer_motor"].as<std::string>();
        
        if (!params["rr_drive_motor"]) throw std::runtime_error("配置文件缺少必需参数: rr_drive_motor");
        motor_names_.rr_drive = params["rr_drive_motor"].as<std::string>();
        
        if (!params["rr_steer_offset"]) throw std::runtime_error("配置文件缺少必需参数: rr_steer_offset");
        motor_config_.rr_steer_offset = params["rr_steer_offset"].as<double>();
        
        if (!params["rr_drive_direction"]) throw std::runtime_error("配置文件缺少必需参数: rr_drive_direction");
        motor_config_.rr_drive_direction = params["rr_drive_direction"].as<int>();
        
        // 初始化运动学
        kinematics_ = std::make_unique<SteerWheelKinematics>(
            wheel_base_x_, wheel_base_y_, wheel_radius_
        );
    }
    
    // 成员变量
    std::unique_ptr<SteerWheelKinematics> kinematics_;
    MotorNames motor_names_;
    MotorConfig motor_config_;
    
    // 参数
    double control_frequency_;
    double wheel_base_x_;
    double wheel_base_y_;
    double wheel_radius_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    
    // 底盘速度命令
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0;
    rclcpp::Time last_cmd_time_;
    
    // 电机状态缓存
    std::map<std::string, motor_control_ros2::msg::DJIMotorState> motor_states_;
    
    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr motor_state_sub_;
    rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<motor_control_ros2::msg::ChassisState>::SharedPtr chassis_state_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::ChassisControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
