#include "motor_control_ros2/joystick_control_node.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control {

JoystickControlNode::JoystickControlNode() 
    : Node("joystick_control_node")
    , enabled_(false)
    , emergency_stop_(false)
    , current_speed_gain_(1.0)
    , target_vx_(0.0), target_vy_(0.0), target_wz_(0.0)
    , current_vx_(0.0), current_vy_(0.0), current_wz_(0.0)
{
    // 声明参数 - 速度限制
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 3.14);
    
    // 摇杆参数
    this->declare_parameter("deadzone", 0.15);
    this->declare_parameter("speed_curve_exponent", 2.0);
    
    // 控制参数
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("accel_limit", 3.0);        // m/s²
    this->declare_parameter("angular_accel_limit", 5.0); // rad/s²
    this->declare_parameter("joy_timeout", 0.5);
    
    // 速度模式增益
    this->declare_parameter("speed_mode_slow", 0.3);
    this->declare_parameter("speed_mode_normal", 0.6);
    this->declare_parameter("speed_mode_fast", 1.0);
    
    // 轴映射（Xbox 手柄标准映射）
    this->declare_parameter("axis_linear_x", 1);   // 左摇杆 Y
    this->declare_parameter("axis_linear_y", 0);   // 左摇杆 X
    this->declare_parameter("axis_angular", 3);    // 右摇杆 X
    this->declare_parameter("axis_lt", 2);         // LT 触发器
    this->declare_parameter("axis_rt", 5);         // RT 触发器
    
    // 按钮映射
    this->declare_parameter("button_emergency_stop", 1);  // B
    this->declare_parameter("button_slow_mode", 2);       // X
    this->declare_parameter("button_normal_mode", 0);     // A
    this->declare_parameter("button_fast_mode", 3);       // Y
    this->declare_parameter("button_enable", 7);          // Start
    
    // 读取参数
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    speed_curve_exponent_ = this->get_parameter("speed_curve_exponent").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    accel_limit_ = this->get_parameter("accel_limit").as_double();
    angular_accel_limit_ = this->get_parameter("angular_accel_limit").as_double();
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();
    
    speed_mode_slow_ = this->get_parameter("speed_mode_slow").as_double();
    speed_mode_normal_ = this->get_parameter("speed_mode_normal").as_double();
    speed_mode_fast_ = this->get_parameter("speed_mode_fast").as_double();
    
    axis_linear_x_ = this->get_parameter("axis_linear_x").as_int();
    axis_linear_y_ = this->get_parameter("axis_linear_y").as_int();
    axis_angular_ = this->get_parameter("axis_angular").as_int();
    axis_lt_ = this->get_parameter("axis_lt").as_int();
    axis_rt_ = this->get_parameter("axis_rt").as_int();
    
    button_emergency_stop_ = this->get_parameter("button_emergency_stop").as_int();
    button_slow_mode_ = this->get_parameter("button_slow_mode").as_int();
    button_normal_mode_ = this->get_parameter("button_normal_mode").as_int();
    button_fast_mode_ = this->get_parameter("button_fast_mode").as_int();
    button_enable_ = this->get_parameter("button_enable").as_int();
    
    // 初始化为正常模式
    current_speed_gain_ = speed_mode_normal_;
    
    RCLCPP_INFO(this->get_logger(), 
        "Xbox 手柄控制节点启动");
    RCLCPP_INFO(this->get_logger(),
        "速度限制: 线速度 %.2f m/s, 角速度 %.2f rad/s",
        max_linear_velocity_, max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(),
        "速度模式: 慢速 %.0f%%, 正常 %.0f%%, 快速 %.0f%%",
        speed_mode_slow_ * 100, speed_mode_normal_ * 100, speed_mode_fast_ * 100);
    RCLCPP_INFO(this->get_logger(),
        "按 Start 键启用控制，按 B 键急停");
    
    // 创建订阅者
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&JoystickControlNode::joyCallback, this, std::placeholders::_1)
    );
    
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );
    
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/chassis_enabled", 10
    );
    
    // 创建定时发布器
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&JoystickControlNode::publishCmdVel, this)
    );
    
    last_joy_time_ = this->now();
    last_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), 
        "等待手柄输入... (订阅 /joy 话题)");
}

void JoystickControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joy_time_ = this->now();
    
    // 检查轴和按钮索引是否有效
    if (msg->axes.empty() || msg->buttons.empty()) {
        return;
    }
    
    // 处理使能按钮（Start 键切换）
    static bool last_enable_button = false;
    bool enable_button = msg->buttons.size() > (size_t)button_enable_ && 
                         msg->buttons[button_enable_];
    if (enable_button && !last_enable_button) {
        enabled_ = !enabled_;
        RCLCPP_INFO(this->get_logger(), "控制 %s", enabled_ ? "启用" : "禁用");
        
        // 发布使能状态
        auto enable_msg = std_msgs::msg::Bool();
        enable_msg.data = enabled_;
        enable_pub_->publish(enable_msg);
        
        // 禁用时清零速度
        if (!enabled_) {
            target_vx_ = target_vy_ = target_wz_ = 0.0;
            current_vx_ = current_vy_ = current_wz_ = 0.0;
        }
    }
    last_enable_button = enable_button;
    
    // 处理急停按钮（B 键）
    static bool last_estop_button = false;
    bool estop_button = msg->buttons.size() > (size_t)button_emergency_stop_ && 
                        msg->buttons[button_emergency_stop_];
    if (estop_button && !last_estop_button) {
        emergency_stop_ = !emergency_stop_;
        if (emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "急停激活！");
            target_vx_ = target_vy_ = target_wz_ = 0.0;
            current_vx_ = current_vy_ = current_wz_ = 0.0;
            enabled_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "急停解除");
        }
    }
    last_estop_button = estop_button;
    
    // 急停状态下不处理其他输入
    if (emergency_stop_ || !enabled_) {
        return;
    }
    
    // 处理速度模式按钮
    if (msg->buttons.size() > (size_t)button_slow_mode_ && 
        msg->buttons[button_slow_mode_]) {
        current_speed_gain_ = speed_mode_slow_;
        RCLCPP_INFO(this->get_logger(), "切换到慢速模式 (%.0f%%)", 
                    current_speed_gain_ * 100);
    }
    if (msg->buttons.size() > (size_t)button_normal_mode_ && 
        msg->buttons[button_normal_mode_]) {
        current_speed_gain_ = speed_mode_normal_;
        RCLCPP_INFO(this->get_logger(), "切换到正常模式 (%.0f%%)", 
                    current_speed_gain_ * 100);
    }
    if (msg->buttons.size() > (size_t)button_fast_mode_ && 
        msg->buttons[button_fast_mode_]) {
        current_speed_gain_ = speed_mode_fast_;
        RCLCPP_INFO(this->get_logger(), "切换到快速模式 (%.0f%%)", 
                    current_speed_gain_ * 100);
    }
    
    // 读取摇杆值
    double raw_vx = msg->axes.size() > (size_t)axis_linear_x_ ? 
                    msg->axes[axis_linear_x_] : 0.0;
    double raw_vy = msg->axes.size() > (size_t)axis_linear_y_ ? 
                    msg->axes[axis_linear_y_] : 0.0;
    double raw_wz = msg->axes.size() > (size_t)axis_angular_ ? 
                    msg->axes[axis_angular_] : 0.0;
    
    // 应用死区
    raw_vx = applyDeadzone(raw_vx, deadzone_);
    raw_vy = applyDeadzone(raw_vy, deadzone_);
    raw_wz = applyDeadzone(raw_wz, deadzone_);
    
    // 应用速度曲线（使控制更平滑）
    raw_vx = applySpeedCurve(raw_vx, speed_curve_exponent_);
    raw_vy = applySpeedCurve(raw_vy, speed_curve_exponent_);
    raw_wz = applySpeedCurve(raw_wz, speed_curve_exponent_);
    
    // 读取触发器（LT 减速，RT 加速）- 默认设为1.0使摇杆直接控制
    double trigger_gain = 1.0;
    if (msg->axes.size() > (size_t)axis_lt_ && msg->axes.size() > (size_t)axis_rt_) {
        double lt = (1.0 - msg->axes[axis_lt_]) / 2.0;  // 0~1
        double rt = (1.0 - msg->axes[axis_rt_]) / 2.0;  // 0~1
        // 仅当触发器被实际使用时才调整增益
        if (lt > 0.01 || rt > 0.01) {
            trigger_gain = 0.3 + 0.7 * rt - 0.6 * lt;       // 范围约 0.1~1.0
            trigger_gain = std::clamp(trigger_gain, 0.1, 1.0);
        }
    }
    
    // 计算目标速度
    double total_gain = current_speed_gain_ * trigger_gain;
    target_vx_ = raw_vx * max_linear_velocity_ * total_gain;
    target_vy_ = raw_vy * max_linear_velocity_ * total_gain;
    target_wz_ = raw_wz * max_angular_velocity_ * total_gain;
}

void JoystickControlNode::publishCmdVel() {
    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    
    // 检查手柄超时
    if ((now - last_joy_time_).seconds() > joy_timeout_) {
        target_vx_ = target_vy_ = target_wz_ = 0.0;
        if (enabled_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "手柄信号超时，速度清零");
        }
    }
    
    // 急停或禁用状态下发送零速度
    if (emergency_stop_ || !enabled_) {
        target_vx_ = target_vy_ = target_wz_ = 0.0;
    }
    
    // 应用加速度限制（平滑控制）
    if (dt > 0.0 && dt < 1.0) {
        current_vx_ = rateLimit(current_vx_, target_vx_, accel_limit_, dt);
        current_vy_ = rateLimit(current_vy_, target_vy_, accel_limit_, dt);
        current_wz_ = rateLimit(current_wz_, target_wz_, angular_accel_limit_, dt);
    } else {
        current_vx_ = target_vx_;
        current_vy_ = target_vy_;
        current_wz_ = target_wz_;
    }
    
    // 发布速度命令
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = current_vx_;
    cmd_msg.linear.y = current_vy_;
    cmd_msg.angular.z = current_wz_;
    
    cmd_vel_pub_->publish(cmd_msg);
}

double JoystickControlNode::applyDeadzone(double value, double deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    // 重新映射到 [0, 1] 范围
    double sign = value > 0 ? 1.0 : -1.0;
    return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
}

double JoystickControlNode::applySpeedCurve(double value, double exponent) {
    double sign = value > 0 ? 1.0 : -1.0;
    return sign * std::pow(std::abs(value), exponent);
}

double JoystickControlNode::rateLimit(double current, double target, 
                                      double rate, double dt) {
    double max_change = rate * dt;
    double error = target - current;
    
    if (std::abs(error) < max_change) {
        return target;
    }
    
    return current + (error > 0 ? max_change : -max_change);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::JoystickControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
