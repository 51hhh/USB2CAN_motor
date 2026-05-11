#ifndef MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP
#define MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include "motor_control_ros2/msg/arm_target.hpp"
#include <memory>
#include <string>

namespace motor_control {

/**
 * @brief 遥控器节点
 * 
 * 功能：
 * - 订阅遥控器输入（Joy 消息）
 * - 转换为底盘速度命令（Twist 消息）
 * - 发布到可配置的速度话题供下游底盘/复用节点使用
 * 
 * 支持的运动：
 * - 前后移动（linear.x）
 * - 左右移动（linear.y）
 * - 旋转（angular.z）
 */
class RemoteControlNode : public rclcpp::Node {
public:
    RemoteControlNode();

private:
    /**
     * @brief 处理遥控器输入
     * 
     * 典型的遥控器映射（以 Xbox 风格手柄为例）：
     * - 左摇杆：前后运动、左右运动
     * - 右摇杆：旋转运动
     * - 按钮：快速/慢速模式切换
     * 
     * @param msg 遥控器输入消息
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * @brief 死区处理
     * 
     * @param value 输入值
     * @param deadzone 死区范围 [0, 1)
     * @return 处理后的值
     */
    double applyDeadzone(double value, double deadzone);

    /**
     * @brief 获取速度缩放因子（根据模式）
     * 
     * @return 缩放因子 [0, 1]
     */
    double getSpeedScaleFactor();
    double normalizeTrigger(double raw_value) const;
    void readyCallback(const std_msgs::msg::String::SharedPtr msg);
    void updateArmControl(const sensor_msgs::msg::Joy::SharedPtr msg);
    void loadRemoteControlParams(const std::string& config_file);

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_ready_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<motor_control_ros2::msg::ArmTarget>::SharedPtr arm_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr serve_trigger_pub_;

    // 配置参数
    struct Config {
        // 轴映射
        int axis_forward = 1;        // 前后运动轴（通常是左摇杆 Y 轴）
        int axis_strafe = 0;         // 左右运动轴（通常是左摇杆 X 轴）
        int axis_rotate = 3;         // 旋转轴（通常是右摇杆 X 轴）

        // 按钮映射
        int button_speed_up = 5;     // 加速按钮（RB）
        int button_speed_down = 4;   // 减速按钮（LB）
        int button_stop = 6;         // 停止按钮（Back）
        int button_serve_trigger = 0; // 发球触发按钮（A）

        // delta arm 扳机控制
        int arm_trigger_axis = 5;             // RT
        double arm_trigger_threshold = 0.15;  // 触发阈值（归一化后）
        double arm_trigger_reset_threshold = 0.05;  // 回位阈值（归一化后）
        double arm_min_angle_rad = 0.0;
        double arm_max_angle_rad = 0.5;
        int arm_protection_press_limit = 3;
        double arm_protection_window_sec = 2.0;

        // 速度限制（m/s 和 rad/s）
        double max_linear_velocity = 2.7;    // 最大线速度
        double max_angular_velocity = 6.0;   // 最大角速度

        // 默认速度缩放因子
        double default_speed_scale = 0.5;    // 50% 的最大速度
        double speed_scale_step = 0.5;       // 每次按下/释放快速键调整 50%

        // 死区
        double deadzone = 0.1;               // 摇杆死区范围

        // 发布频率
        double publish_frequency = 50.0;     // Hz

        // 话题配置
        std::string joy_topic = "/joy";
        std::string cmd_vel_topic = "/cmd_vel_remote";
    } config_;

    enum class ArmTriggerState {
        IDLE,
        COMMAND_SENT,
        WAIT_TRIGGER_RESET,
        PROTECTED
    };

    // 状态
    double current_speed_scale_ = 1.0;  // 当前速度缩放因子 [0, 10.0]，即 0
    geometry_msgs::msg::Twist last_cmd_;  // 上一条命令
    rclcpp::TimerBase::SharedPtr publish_timer_;  // 定时发布定时器
    ArmTriggerState arm_trigger_state_ = ArmTriggerState::IDLE;
    bool arm_ready_ = true;
    int arm_press_count_ = 0;
    rclcpp::Time last_arm_press_time_;

    // 按钮状态记录（用于检测按下事件）
    bool last_button_speed_up_ = false;      // 上一帧加速按钮状态
    bool last_button_speed_down_ = false;    // 上一帧减速按钮状态
    bool last_button_stop_ = false;          // 上一帧停止按钮状态
    bool last_button_serve_trigger_ = false; // 上一帧发球按钮状态

    /**
     * @brief 定时发布最后一条命令
     * 
     * 用于保持持续的速度控制
     */
    void publishTimer();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP
