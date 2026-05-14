#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/omni_wheel_kinematics.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

#include <array>
#include <string>
#include <map>
#include <memory>
#include <chrono>

namespace motor_control {

/**
 * @brief X 形全向轮底盘控制节点
 * 
 * 功能：
 * - 订阅底盘速度命令 (/cmd_vel)
 * - X 形全向轮运动学逆解算 → 4 个电机速度命令
 * - 发布 DJI GM3508 电机控制命令
 * - 从电机反馈计算里程计
 * - 发布里程计信息 (/odom)
 */
class OmniChassisControlNode : public rclcpp::Node {
public:
    OmniChassisControlNode() : Node("omni_chassis_control_node") {
        this->declare_parameter("config_file", "");

        const std::string config_file = resolveConfigFile();
        try {
            loadParams(config_file);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "omni_chassis_control_node 配置加载失败: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "配置文件路径: %s", config_file.c_str());
            throw;
        }

        if (control_frequency_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "control_frequency=%.3f 非法，回退到 100.0", control_frequency_);
            control_frequency_ = 100.0;
        }
        if (wheel_radius_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "wheel_radius=%.3f 非法，回退到 0.1062", wheel_radius_);
            wheel_radius_ = 0.1062;
        }
        if (cmd_timeout_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "cmd_timeout=%.3f 非法，回退到 0.5", cmd_timeout_);
            cmd_timeout_ = 0.5;
        }
        
        // 初始化运动学
        kinematics_ = std::make_unique<OmniWheelKinematics>(
            wheel_base_x_, wheel_base_y_, wheel_radius_, install_angle_
        );
        
        RCLCPP_INFO(this->get_logger(), 
            "X 形全向轮底盘控制节点启动");
        RCLCPP_INFO(this->get_logger(),
            "轮距: %.3fm x %.3fm, 轮半径: %.3fm, 安装角: %.1f°",
            wheel_base_x_, wheel_base_y_, wheel_radius_, install_angle_);
        RCLCPP_INFO(this->get_logger(),
            "电机映射: FL=%s, FR=%s, RL=%s, RR=%s",
            motor_names_[0].c_str(), motor_names_[1].c_str(),
            motor_names_[2].c_str(), motor_names_[3].c_str());
        RCLCPP_INFO(this->get_logger(),
            "驱动方向: FL=%d, FR=%d, RL=%d, RR=%d",
            drive_directions_[0], drive_directions_[1], drive_directions_[2], drive_directions_[3]);
        
        // 创建订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&OmniChassisControlNode::cmdVelCallback, this, std::placeholders::_1)
        );
        
        motor_state_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorState>(
            "/dji_motor_states", 10,
            std::bind(&OmniChassisControlNode::motorStateCallback, this, std::placeholders::_1)
        );
        
        // 创建发布者
        motor_cmd_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
            "/dji_motor_command_advanced", 10
        );
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_wheels", 10
        );
        RCLCPP_INFO(this->get_logger(), "发布轮速里程计到 /odom_wheels (供 positioning_bridge 融合)");
        
        // 创建控制循环定时器
        auto period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OmniChassisControlNode::controlLoop, this)
        );
        
        // 初始化里程计
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        last_odom_time_ = this->now();
        last_cmd_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), 
            "控制循环启动 - 频率: %.1f Hz", control_frequency_);
    }

private:
    std::string resolveConfigFile() {
        std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            return config_file;
        }

        return ament_index_cpp::get_package_share_directory("motor_control_ros2") +
            "/config/omni_chassis_params.yaml";
    }

    void loadParams(const std::string & config_file) {
        RCLCPP_INFO(this->get_logger(), "正在加载全向轮底盘参数: %s", config_file.c_str());

        YAML::Node root = YAML::LoadFile(config_file);
        YAML::Node params = root;
        if (root["omni_chassis_control_node"] && root["omni_chassis_control_node"]["ros__parameters"]) {
            params = root["omni_chassis_control_node"]["ros__parameters"];
        }

        auto loadString = [&params](const char * key, std::string & value) {
            if (params[key]) {
                value = params[key].as<std::string>();
            }
        };

        auto loadDouble = [&params](const char * key, double & value) {
            if (params[key]) {
                value = params[key].as<double>();
            }
        };

        auto loadInt = [&params](const char * key, int & value) {
            if (params[key]) {
                value = params[key].as<int>();
            }
        };

        loadDouble("control_frequency", control_frequency_);
        loadDouble("wheel_base_x", wheel_base_x_);
        loadDouble("wheel_base_y", wheel_base_y_);
        loadDouble("wheel_radius", wheel_radius_);
        loadDouble("install_angle", install_angle_);
        loadDouble("max_linear_velocity", max_linear_velocity_);
        loadDouble("max_angular_velocity", max_angular_velocity_);
        loadDouble("cmd_timeout", cmd_timeout_);
        loadString("fl_motor", motor_names_[0]);
        loadString("fr_motor", motor_names_[1]);
        loadString("rl_motor", motor_names_[2]);
        loadString("rr_motor", motor_names_[3]);

        loadInt("fl_drive_direction", drive_directions_[0]);
        loadInt("fr_drive_direction", drive_directions_[1]);
        loadInt("rl_drive_direction", drive_directions_[2]);
        loadInt("rr_drive_direction", drive_directions_[3]);

        RCLCPP_INFO(this->get_logger(), "omni_chassis_control_node 参数加载完成: %s", config_file.c_str());
    }

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
        auto now = this->now();
        
        // 检查命令超时
        if ((now - last_cmd_time_).seconds() > cmd_timeout_) {
            cmd_vx_ = 0.0;
            cmd_vy_ = 0.0;
            cmd_wz_ = 0.0;
        }
        
        // 逆运动学解算：底盘速度 → 4 个轮子速度
        // 注意：运动学代码使用的坐标系是 X向右，Y向前
        // 而ROS标准是 X向前，Y向左
        // 所以这里需要交换：ROS的vx(前后)对应运动学的vy，ROS的vy(左右)对应运动学的vx
        // vy取反以匹配左右方向
        auto wheel_velocities = kinematics_->inverseKinematics(-cmd_vy_, cmd_vx_, cmd_wz_);
        
        // 转换为电机 RPM 并发布命令
        publishMotorCommands(wheel_velocities, now);
        
        // 更新并发布里程计
        updateAndPublishOdometry(now);
    }
    
    // 发布电机命令
    void publishMotorCommands(const std::array<double, 4>& wheel_velocities, 
                              const rclcpp::Time& timestamp) {
        for (size_t i = 0; i < 4; ++i) {
            // 将轮子线速度转换为电机角速度 (rad/s)
            double wheel_angular_vel = wheel_velocities[i] / kinematics_->getWheelRadius();
            // 考虑减速比转换为电机角速度
            double motor_angular_vel = wheel_angular_vel * 19.0 * static_cast<double>(drive_directions_[i]);  // GM3508 减速比 19:1
            
            auto msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
            msg.header.stamp = timestamp;
            msg.joint_name = motor_names_[i];
            msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
            msg.velocity_target = motor_angular_vel;  // 弧度/秒（消息定义要求的单位）
            
            motor_cmd_pub_->publish(msg);
        }
    }
    
    // 更新并发布里程计
    void updateAndPublishOdometry(const rclcpp::Time& current_time) {
        // 计算时间差
        double dt = (current_time - last_odom_time_).seconds();
        if (dt <= 0.0) return;
        
        // 从电机反馈获取实际轮子速度
        std::array<double, 4> wheel_velocities = {0.0, 0.0, 0.0, 0.0};
        bool all_motors_online = true;
        
        for (size_t i = 0; i < 4; ++i) {
            if (motor_states_.count(motor_names_[i])) {
                const auto& state = motor_states_[motor_names_[i]];
                if (state.online) {
                    // RPM → 线速度
                    wheel_velocities[i] = kinematics_->rpmToVelocity(
                        static_cast<double>(state.rpm) * static_cast<double>(drive_directions_[i]));
                } else {
                    all_motors_online = false;
                }
            } else {
                all_motors_online = false;
            }
        }
        
        // 正运动学：轮子速度 → 底盘速度
        // 注意：运动学输出坐标系仍然是 X向右，Y向前；这里要转换回 ROS 标准坐标系
        // 运动学系 -> ROS 系：ros_vx = kin_vy, ros_vy = -kin_vx
        double kin_vx, kin_vy, wz;
        kinematics_->forwardKinematics(wheel_velocities, kin_vx, kin_vy, wz);
        const double vx = kin_vy;
        const double vy = -kin_vx;
        
        // 更新里程计（在全局坐标系中积分）
        double delta_x = (vx * cos(odom_theta_) - vy * sin(odom_theta_)) * dt;
        double delta_y = (vx * sin(odom_theta_) + vy * cos(odom_theta_)) * dt;
        double delta_theta = wz * dt;
        
        odom_x_ += delta_x;
        odom_y_ += delta_y;
        odom_theta_ += delta_theta;
        
        // 归一化角度到 [-π, π]
        while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
        while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
        
        // 发布里程计消息
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // 位置
        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // 姿态（四元数）
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odom_theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // 速度（在底盘坐标系中）
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = wz;
        
        // 设置协方差（简单估计，可根据实际情况调整）
        if (all_motors_online) {
            // 电机在线，协方差较小
            odom_msg.pose.covariance[0] = 0.01;   // x
            odom_msg.pose.covariance[7] = 0.01;   // y
            odom_msg.pose.covariance[35] = 0.05;  // theta
            odom_msg.twist.covariance[0] = 0.01;  // vx
            odom_msg.twist.covariance[7] = 0.01;  // vy
            odom_msg.twist.covariance[35] = 0.05; // wz
        } else {
            // 电机离线，协方差较大
            odom_msg.pose.covariance[0] = 0.1;
            odom_msg.pose.covariance[7] = 0.1;
            odom_msg.pose.covariance[35] = 0.5;
            odom_msg.twist.covariance[0] = 0.1;
            odom_msg.twist.covariance[7] = 0.1;
            odom_msg.twist.covariance[35] = 0.5;
        }
        
        odom_pub_->publish(odom_msg);
        
        last_odom_time_ = current_time;
    }
    
    // 成员变量
    std::unique_ptr<OmniWheelKinematics> kinematics_;
    std::array<std::string, 4> motor_names_ {{"DJI3508_1", "DJI3508_2", "DJI3508_3", "DJI3508_4"}};  // [FL, FR, RL, RR]
    std::array<int, 4> drive_directions_ {{1, 1, 1, 1}};  // [FL, FR, RL, RR]
    
    // 参数
    double control_frequency_ {100.0};
    double wheel_base_x_ {0.50};
    double wheel_base_y_ {0.50};
    double wheel_radius_ {0.1062};
    double install_angle_ {45.0};
    double max_linear_velocity_ {2.0};
    double max_angular_velocity_ {3.14};
    double cmd_timeout_ {0.5};
    
    // 底盘速度命令
    double cmd_vx_ = 0.0;
    double cmd_vy_ = 0.0;
    double cmd_wz_ = 0.0;
    rclcpp::Time last_cmd_time_;
    
    // 里程计
    double odom_x_;
    double odom_y_;
    double odom_theta_;
    rclcpp::Time last_odom_time_;
    
    // 电机状态缓存
    std::map<std::string, motor_control_ros2::msg::DJIMotorState> motor_states_;
    
    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr motor_state_sub_;
    rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::OmniChassisControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
