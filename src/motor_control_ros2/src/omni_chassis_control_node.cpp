#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        // 声明参数
        this->declare_parameter("control_frequency", 100.0);
        this->declare_parameter("wheel_base_x", 0.50);  // 前后轮距 (m)
        this->declare_parameter("wheel_base_y", 0.50);  // 左右轮距 (m)
        this->declare_parameter("wheel_radius", 0.076); // 轮子半径 (m)
        this->declare_parameter("install_angle", 45.0); // 安装角度 (度)
        this->declare_parameter("max_linear_velocity", 2.0);
        this->declare_parameter("max_angular_velocity", 3.14);
        this->declare_parameter("cmd_timeout", 0.5);    // 命令超时 (s)
        this->declare_parameter("velocity_filter_alpha", 0.3);  // 速度滤波系数
        
        // 电机映射参数 (4 个 GM3508)
        this->declare_parameter("fl_motor", "DJI3508_1");  // 左前
        this->declare_parameter("fr_motor", "DJI3508_2");  // 右前
        this->declare_parameter("rl_motor", "DJI3508_3");  // 左后
        this->declare_parameter("rr_motor", "DJI3508_4");  // 右后
        
        // 读取参数
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        double wheel_base_x = this->get_parameter("wheel_base_x").as_double();
        double wheel_base_y = this->get_parameter("wheel_base_y").as_double();
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double install_angle = this->get_parameter("install_angle").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
        velocity_filter_alpha_ = this->get_parameter("velocity_filter_alpha").as_double();
        
        // 读取电机映射
        motor_names_[0] = this->get_parameter("fl_motor").as_string();  // FL
        motor_names_[1] = this->get_parameter("fr_motor").as_string();  // FR
        motor_names_[2] = this->get_parameter("rl_motor").as_string();  // RL
        motor_names_[3] = this->get_parameter("rr_motor").as_string();  // RR
        
        // 初始化运动学
        kinematics_ = std::make_unique<OmniWheelKinematics>(
            wheel_base_x, wheel_base_y, wheel_radius, install_angle
        );
        
        RCLCPP_INFO(this->get_logger(), 
            "X 形全向轮底盘控制节点启动");
        RCLCPP_INFO(this->get_logger(),
            "轮距: %.3fm x %.3fm, 轮半径: %.3fm, 安装角: %.1f°",
            wheel_base_x, wheel_base_y, wheel_radius, install_angle);
        RCLCPP_INFO(this->get_logger(),
            "电机映射: FL=%s, FR=%s, RL=%s, RR=%s",
            motor_names_[0].c_str(), motor_names_[1].c_str(),
            motor_names_[2].c_str(), motor_names_[3].c_str());
        
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
            "/odom", 10
        );
        
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
    // 底盘速度命令回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 限制速度
        double raw_vx = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
        double raw_vy = std::clamp(msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
        double raw_wz = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
        
        // 应用低通滤波（平滑速度变化）
        // filtered = alpha * new + (1 - alpha) * old
        filtered_vx_ = velocity_filter_alpha_ * raw_vx + (1.0 - velocity_filter_alpha_) * filtered_vx_;
        filtered_vy_ = velocity_filter_alpha_ * raw_vy + (1.0 - velocity_filter_alpha_) * filtered_vy_;
        filtered_wz_ = velocity_filter_alpha_ * raw_wz + (1.0 - velocity_filter_alpha_) * filtered_wz_;
        
        // 使用滤波后的速度
        cmd_vx_ = filtered_vx_;
        cmd_vy_ = filtered_vy_;
        cmd_wz_ = filtered_wz_;
        
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
            // 清零滤波器状态
            filtered_vx_ = 0.0;
            filtered_vy_ = 0.0;
            filtered_wz_ = 0.0;
        }
        
        // 逆运动学解算：底盘速度 → 4 个轮子速度
        auto wheel_velocities = kinematics_->inverseKinematics(cmd_vx_, cmd_vy_, cmd_wz_);
        
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
            double motor_angular_vel = wheel_angular_vel * 19.0;  // GM3508 减速比 19:1
            
            // 转换为 RPM（电机驱动期望的单位）
            // 1 rad/s = 60/(2*π) RPM ≈ 9.5493 RPM
            double motor_rpm = motor_angular_vel * 60.0 / (2.0 * M_PI);
            
            auto msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
            msg.header.stamp = timestamp;
            msg.joint_name = motor_names_[i];
            msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
            msg.velocity_target = motor_rpm;  // RPM（正确单位）
            
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
                    wheel_velocities[i] = kinematics_->rpmToVelocity(state.rpm);
                } else {
                    all_motors_online = false;
                }
            } else {
                all_motors_online = false;
            }
        }
        
        // 正运动学：轮子速度 → 底盘速度
        double vx, vy, wz;
        kinematics_->forwardKinematics(wheel_velocities, vx, vy, wz);
        
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
    std::array<std::string, 4> motor_names_;  // [FL, FR, RL, RR]
    
    // 参数
    double control_frequency_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double cmd_timeout_;
    
    // 速度平滑滤波参数
    double velocity_filter_alpha_;  // 低通滤波系数 [0, 1]，越小越平滑
    
    // 上一次的滤波速度（用于低通滤波）
    double filtered_vx_ = 0.0;
    double filtered_vy_ = 0.0;
    double filtered_wz_ = 0.0;
    
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
