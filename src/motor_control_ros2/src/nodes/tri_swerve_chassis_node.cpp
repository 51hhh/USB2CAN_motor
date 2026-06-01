#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/steer_wheel_kinematics.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

#include <cmath>
#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <chrono>

namespace motor_control {

/**
 * @brief 等边三角形三舵轮底盘控制节点
 *
 * 机械布局（ROS 坐标系：X 向前，Y 向左）：
 *
 *              F (前轮)
 *             /  \
 *            /    \
 *           BL----BR
 *
 * 轮子位置（圆周半径 R = circumradius_）：
 *   F  : (R,       0         )   前方中心
 *   BL : (-R/2,  +R*sqrt(3)/2)  后左
 *   BR : (-R/2,  -R*sqrt(3)/2)  后右
 *
 * 每轮配置：GM6020 转向（位置控制） + GM3508 驱动（速度控制）
 *
 * 话题：
 *   订阅：/cmd_vel (geometry_msgs/Twist)
 *   订阅：/dji_motor_states (DJIMotorState)
 *   发布：/dji_motor_command_advanced (DJIMotorCommandAdvanced)
 */
class TriSwerveChassisNode : public rclcpp::Node {
public:
    TriSwerveChassisNode() : Node("tri_swerve_chassis_node") {
        RCLCPP_INFO(this->get_logger(), "正在初始化三舵轮底盘控制节点...");

        std::string config_file;
        try {
            config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2")
                        + "/config/tri_swerve_params.yaml";
            RCLCPP_INFO(this->get_logger(), "加载配置文件: %s", config_file.c_str());
            loadParams(config_file);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
            throw;
        }

        last_cmd_time_ = this->now();

        // 订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            std::bind(&TriSwerveChassisNode::cmdVelCallback, this, std::placeholders::_1));

        motor_state_sub_ = this->create_subscription<motor_control_ros2::msg::DJIMotorState>(
            "/dji_motor_states", 10,
            std::bind(&TriSwerveChassisNode::motorStateCallback, this, std::placeholders::_1));

        // 发布
        motor_cmd_pub_ = this->create_publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>(
            "/dji_motor_command_advanced", 10);

        // 控制定时器
        auto period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&TriSwerveChassisNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "三舵轮底盘节点启动 - 圆周半径: %.3fm, 轮半径: %.3fm, 频率: %.1fHz",
            circumradius_, wheel_radius_, control_frequency_);
        RCLCPP_INFO(this->get_logger(),
            "转向偏移 - F: %.1f°, BL: %.1f°, BR: %.1f°",
            steer_offset_[F], steer_offset_[BL], steer_offset_[BR]);
        RCLCPP_INFO(this->get_logger(), "速度指令话题: %s", cmd_vel_topic_.c_str());
    }

private:
    // 轮子索引
    enum WheelIdx { F = 0, BL = 1, BR = 2, NUM_WHEELS = 3 };

    // ------------------------------------------------------------------
    // 回调
    // ------------------------------------------------------------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 平移速度矢量限幅
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        const double linear_speed = std::hypot(vx, vy);
        if (linear_speed > max_linear_velocity_ && linear_speed > 1e-9) {
            const double scale = max_linear_velocity_ / linear_speed;
            vx *= scale;
            vy *= scale;
        }
        cmd_vx_ = vx;
        cmd_vy_ = vy;
        cmd_wz_ = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
        last_cmd_time_ = this->now();
    }

    void motorStateCallback(const motor_control_ros2::msg::DJIMotorState::SharedPtr msg) {
        motor_states_[msg->joint_name] = *msg;
    }

    // ------------------------------------------------------------------
    // 控制循环
    // ------------------------------------------------------------------
    void controlLoop() {
        // 命令超时（500ms）自动清零
        if ((this->now() - last_cmd_time_).seconds() > 0.5) {
            cmd_vx_ = 0.0;
            cmd_vy_ = 0.0;
            cmd_wz_ = 0.0;
        }

        const bool has_motion =
            std::abs(cmd_vx_) > 0.01 || std::abs(cmd_vy_) > 0.01 || std::abs(cmd_wz_) > 0.01;

        std::array<WheelCommand, NUM_WHEELS> cmds;

        if (!has_motion) {
            // 静止：舵轮归零，驱动停止
            for (auto& c : cmds) { c.angle = 0.0; c.velocity = 0.0; }
        } else {
            inverseKinematics(cmd_vx_, cmd_vy_, cmd_wz_, cmds);

            // 轮速统一缩放，不超过最大线速度
            double max_speed = 0.0;
            for (const auto& c : cmds) max_speed = std::max(max_speed, std::abs(c.velocity));
            if (max_speed > max_linear_velocity_ && max_speed > 1e-9) {
                const double scale = max_linear_velocity_ / max_speed;
                for (auto& c : cmds) c.velocity *= scale;
            }

            // 舵角最短路径优化
            const std::array<std::string, NUM_WHEELS> steer_names = {
                steer_motor_[F], steer_motor_[BL], steer_motor_[BR]
            };
            for (int i = 0; i < NUM_WHEELS; ++i) {
                if (motor_states_.count(steer_names[i])) {
                    double current_mech =
                        motor_states_[steer_names[i]].angle - steer_offset_[i];
                    double target = cmds[i].angle * steer_direction_[i];
                    cmds[i].angle = SteerWheelKinematics::optimizeSteerAngle(
                        current_mech, target, cmds[i].velocity);
                }
            }
        }

        publishMotorCommands(cmds);
    }

    // ------------------------------------------------------------------
    // 三舵轮逆运动学
    //
    // 轮位置（圆周半径 R）：
    //   F  : rx = R,      ry = 0
    //   BL : rx = -R/2,   ry = +R*sqrt(3)/2
    //   BR : rx = -R/2,   ry = -R*sqrt(3)/2
    //
    // 每轮速度向量：
    //   vx_w = vx - wz * ry
    //   vy_w = vy + wz * rx
    // ------------------------------------------------------------------
    void inverseKinematics(double vx, double vy, double wz,
                           std::array<WheelCommand, NUM_WHEELS>& cmds) {
        const double R     = circumradius_;
        const double R_y   = R * std::sqrt(3.0) / 2.0;
        const double R_x_b = -R / 2.0;

        // 三轮位置
        const double rx[3] = { R,     R_x_b,  R_x_b };
        const double ry[3] = { 0.0,   R_y,   -R_y   };

        for (int i = 0; i < NUM_WHEELS; ++i) {
            double vx_w = vx - wz * ry[i];
            double vy_w = vy + wz * rx[i];
            cmds[i].velocity = std::sqrt(vx_w * vx_w + vy_w * vy_w);
            cmds[i].angle = SteerWheelKinematics::normalizeAngle(
                std::atan2(vy_w, vx_w) * 180.0 / M_PI);
        }
    }

    // ------------------------------------------------------------------
    // 发布电机命令
    // ------------------------------------------------------------------
    void publishMotorCommands(const std::array<WheelCommand, NUM_WHEELS>& cmds) {
        auto now = this->now();
        const char* wheel_name[3] = { "F", "BL", "BR" };
        (void)wheel_name;

        for (int i = 0; i < NUM_WHEELS; ++i) {
            // 转向电机：位置控制（机械角度 → 编码器角度）
            auto steer_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
            steer_msg.header.stamp = now;
            steer_msg.joint_name = steer_motor_[i];
            steer_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_POSITION;
            double encoder_deg = cmds[i].angle + steer_offset_[i];
            steer_msg.position_target = encoder_deg * M_PI / 180.0;
            motor_cmd_pub_->publish(steer_msg);

            // 驱动电机：速度控制（m/s → 电机轴 RPM，匹配 PID 期望单位）
            auto drive_msg = motor_control_ros2::msg::DJIMotorCommandAdvanced();
            drive_msg.header.stamp = now;
            drive_msg.joint_name = drive_motor_[i];
            drive_msg.mode = motor_control_ros2::msg::DJIMotorCommandAdvanced::MODE_VELOCITY;
            // v(m/s) / r(m) = ω(rad/s,输出轴) → ×60/(2π)×gear_ratio → RPM(电机轴)
            double output_rad_s = cmds[i].velocity / wheel_radius_;
            double motor_rpm = output_rad_s * 60.0 / (2.0 * M_PI) * drive_gear_ratio_;
            // 硬限幅：3500 RPM 上限（防止 PID 积分饱和超速）
            motor_rpm = std::clamp(motor_rpm, -3500.0, 3500.0);
            drive_msg.velocity_target = motor_rpm * drive_direction_[i];
            motor_cmd_pub_->publish(drive_msg);
        }
    }

    // ------------------------------------------------------------------
    // 参数加载
    // ------------------------------------------------------------------
    void loadParams(const std::string& config_file) {
        YAML::Node config = YAML::LoadFile(config_file);

        if (!config["tri_swerve_chassis_node"] ||
            !config["tri_swerve_chassis_node"]["ros__parameters"]) {
            throw std::runtime_error(
                "配置文件格式错误: 缺少 tri_swerve_chassis_node/ros__parameters");
        }
        auto p = config["tri_swerve_chassis_node"]["ros__parameters"];

        auto require = [&](const char* key) {
            if (!p[key])
                throw std::runtime_error(std::string("配置缺少必需参数: ") + key);
        };

        require("control_frequency"); control_frequency_ = p["control_frequency"].as<double>();
        require("circumradius");      circumradius_       = p["circumradius"].as<double>();
        require("wheel_radius");      wheel_radius_       = p["wheel_radius"].as<double>();
        require("max_linear_velocity");  max_linear_velocity_  = p["max_linear_velocity"].as<double>();
        require("max_angular_velocity"); max_angular_velocity_ = p["max_angular_velocity"].as<double>();
        if (p["drive_gear_ratio"]) drive_gear_ratio_ = p["drive_gear_ratio"].as<double>();

        if (p["cmd_vel_topic"]) cmd_vel_topic_ = p["cmd_vel_topic"].as<std::string>();

        // 轮子参数（F / BL / BR）
        const std::array<const char*, 3> prefix = { "f", "bl", "br" };
        for (int i = 0; i < NUM_WHEELS; ++i) {
            std::string sm = std::string(prefix[i]) + "_steer_motor";
            std::string dm = std::string(prefix[i]) + "_drive_motor";
            std::string so = std::string(prefix[i]) + "_steer_offset";
            std::string sd = std::string(prefix[i]) + "_steer_direction";
            std::string dd = std::string(prefix[i]) + "_drive_direction";
            require(sm.c_str()); steer_motor_[i]     = p[sm].as<std::string>();
            require(dm.c_str()); drive_motor_[i]     = p[dm].as<std::string>();
            require(so.c_str()); steer_offset_[i]    = p[so].as<double>();
            require(sd.c_str()); steer_direction_[i] = p[sd].as<int>();
            require(dd.c_str()); drive_direction_[i] = p[dd].as<int>();
        }
    }

    // ------------------------------------------------------------------
    // 成员变量
    // ------------------------------------------------------------------
    // 参数
    double control_frequency_  { 100.0 };
    double circumradius_       { 0.25  };
    double wheel_radius_       { 0.15  };
    double drive_gear_ratio_   { 19.0 };   // GM3508 减速比
    double max_linear_velocity_  { 3.0  };
    double max_angular_velocity_ { 3.0  };
    std::string cmd_vel_topic_ { "/cmd_vel" };

    // 各轮电机名称（F / BL / BR）
    std::array<std::string, NUM_WHEELS> steer_motor_;
    std::array<std::string, NUM_WHEELS> drive_motor_;
    std::array<double, NUM_WHEELS>      steer_offset_    {};
    std::array<int,    NUM_WHEELS>      steer_direction_ { 1, 1, 1 };
    std::array<int,    NUM_WHEELS>      drive_direction_ { 1, 1, 1 };

    // 速度命令
    double cmd_vx_ { 0.0 };
    double cmd_vy_ { 0.0 };
    double cmd_wz_ { 0.0 };
    rclcpp::Time last_cmd_time_;

    // 电机状态缓存
    std::map<std::string, motor_control_ros2::msg::DJIMotorState> motor_states_;

    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          cmd_vel_sub_;
    rclcpp::Subscription<motor_control_ros2::msg::DJIMotorState>::SharedPtr motor_state_sub_;
    rclcpp::Publisher<motor_control_ros2::msg::DJIMotorCommandAdvanced>::SharedPtr motor_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::TriSwerveChassisNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
