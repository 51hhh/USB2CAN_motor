#include "motor_control_ros2/joystick_strike_control_node.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control {

JoystickStrikeControlNode::JoystickStrikeControlNode()
    : Node("joystick_strike_control_node"),
      enabled_(false),
      emergency_stop_(false),
      control_mode_(0),  // 默认任务空间模式
      current_speed_gain_(1.0),
      target_x_(0.4), target_y_(0.0), target_orientation_(0.0),
      current_x_(0.4), current_y_(0.0), current_orientation_(0.0),
      has_state_feedback_(false)
{
    RCLCPP_INFO(this->get_logger(), "正在初始化手柄击球拍控制节点...");
    
    // 声明参数 - 摇杆参数
    this->declare_parameter("deadzone", 0.10);
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("joy_timeout", 0.5);
    
    // 任务空间参数
    this->declare_parameter("max_position_delta", 0.5);     // m/s
    this->declare_parameter("max_orientation_delta", 0.8);  // rad/s
    this->declare_parameter("min_x", 0.2);
    this->declare_parameter("max_x", 0.8);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("max_orientation", 0.524);      // 30度
    
    // 关节空间参数
    this->declare_parameter("max_joint_delta", 1.0);        // rad/s
    
    // PID 参数（默认值）
    this->declare_parameter("default_kp", std::vector<double>{80.0, 60.0, 80.0, 60.0});
    this->declare_parameter("default_kd", std::vector<double>{8.0, 6.0, 8.0, 6.0});
    this->declare_parameter("kp_step", 10.0);
    
    // 速度增益
    this->declare_parameter("speed_gain_slow", 0.3);
    this->declare_parameter("speed_gain_normal", 0.6);
    this->declare_parameter("speed_gain_fast", 1.0);
    
    // Xbox 手柄轴映射
    this->declare_parameter("axis_left_x", 0);
    this->declare_parameter("axis_left_y", 1);
    this->declare_parameter("axis_right_x", 3);
    this->declare_parameter("axis_right_y", 4);
    this->declare_parameter("axis_lt", 2);         // LT 触发器
    this->declare_parameter("axis_rt", 5);         // RT 触发器
    this->declare_parameter("axis_dpad_x", 6);     // D-pad 左右
    this->declare_parameter("axis_dpad_y", 7);     // D-pad 上下
    
    // 按钮映射
    this->declare_parameter("button_a", 0);
    this->declare_parameter("button_b", 1);
    this->declare_parameter("button_x", 2);
    this->declare_parameter("button_y", 3);
    this->declare_parameter("button_lb", 4);
    this->declare_parameter("button_rb", 5);
    this->declare_parameter("button_back", 6);
    this->declare_parameter("button_start", 7);
    
    // 读取参数
    deadzone_ = this->get_parameter("deadzone").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    joy_timeout_ = this->get_parameter("joy_timeout").as_double();
    
    max_position_delta_ = this->get_parameter("max_position_delta").as_double();
    max_orientation_delta_ = this->get_parameter("max_orientation_delta").as_double();
    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    max_orientation_ = this->get_parameter("max_orientation").as_double();
    
    max_joint_delta_ = this->get_parameter("max_joint_delta").as_double();
    
    auto kp_vec = this->get_parameter("default_kp").as_double_array();
    auto kd_vec = this->get_parameter("default_kd").as_double_array();
    for (size_t i = 0; i < 4; ++i) {
        kp_values_[i] = kp_vec[i];
        kd_values_[i] = kd_vec[i];
    }
    kp_step_ = this->get_parameter("kp_step").as_double();
    
    speed_gain_slow_ = this->get_parameter("speed_gain_slow").as_double();
    speed_gain_normal_ = this->get_parameter("speed_gain_normal").as_double();
    speed_gain_fast_ = this->get_parameter("speed_gain_fast").as_double();
    current_speed_gain_ = speed_gain_normal_;
    
    axis_left_x_ = this->get_parameter("axis_left_x").as_int();
    axis_left_y_ = this->get_parameter("axis_left_y").as_int();
    axis_right_x_ = this->get_parameter("axis_right_x").as_int();
    axis_right_y_ = this->get_parameter("axis_right_y").as_int();
    axis_lt_ = this->get_parameter("axis_lt").as_int();
    axis_rt_ = this->get_parameter("axis_rt").as_int();
    axis_dpad_x_ = this->get_parameter("axis_dpad_x").as_int();
    axis_dpad_y_ = this->get_parameter("axis_dpad_y").as_int();
    
    button_a_ = this->get_parameter("button_a").as_int();
    button_b_ = this->get_parameter("button_b").as_int();
    button_x_ = this->get_parameter("button_x").as_int();
    button_y_ = this->get_parameter("button_y").as_int();
    button_lb_ = this->get_parameter("button_lb").as_int();
    button_rb_ = this->get_parameter("button_rb").as_int();
    button_back_ = this->get_parameter("button_back").as_int();
    button_start_ = this->get_parameter("button_start").as_int();
    
    // 初始化关节角度目标
    target_joint_angles_.fill(0.0);
    current_joint_angles_.fill(0.0);
    
    // 初始化按钮状态数组（假设最多16个按钮）
    button_pressed_last_.resize(16, false);
    
    // 创建订阅者
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&JoystickStrikeControlNode::joyCallback, this, std::placeholders::_1)
    );
    
    state_sub_ = this->create_subscription<motor_control_ros2::msg::ArmState>(
        "/strike_state", 10,
        std::bind(&JoystickStrikeControlNode::stateCallback, this, std::placeholders::_1)
    );
    
    // 创建发布者
    cmd_pub_ = this->create_publisher<motor_control_ros2::msg::ArmJointCommand>(
        "/strike_command", 10
    );
    
    // 创建定时器（用于周期性发布命令）
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&JoystickStrikeControlNode::publishCommand, this)
    );
    
    // 初始化时间戳
    last_joy_time_ = this->now();
    last_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "手柄击球拍控制节点启动成功！");
    RCLCPP_INFO(this->get_logger(), "工作空间: X[%.2f, %.2f]m, Y[%.2f, %.2f]m",
        min_x_, max_x_, min_y_, max_y_);
    RCLCPP_INFO(this->get_logger(), "当前模式: 任务空间（控制末端位姿）");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "=== 手柄操作说明 ===");
    RCLCPP_INFO(this->get_logger(), "Start 键: 启用/禁用控制");
    RCLCPP_INFO(this->get_logger(), "A 键: 回中心位置");
    RCLCPP_INFO(this->get_logger(), "B 键: 急停");
    RCLCPP_INFO(this->get_logger(), "X 键: 切换到任务空间模式");
    RCLCPP_INFO(this->get_logger(), "Y 键: 切换到关节空间模式");
    RCLCPP_INFO(this->get_logger(), "LT/RT: 减速/加速");
    RCLCPP_INFO(this->get_logger(), "LB/RB: 降低/提高刚度");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_WARN(this->get_logger(), "⚠️ 当前控制未启用，按 Start 键激活");
}

double JoystickStrikeControlNode::applyDeadzone(double value, double deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    // 线性重映射，使死区外的值从0开始
    double sign = (value > 0) ? 1.0 : -1.0;
    return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
}

void JoystickStrikeControlNode::resetToCenter() {
    if (control_mode_ == 0) {
        // 任务空间：回到中心位置
        target_x_ = 0.4;
        target_y_ = 0.0;
        target_orientation_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "复位到中心位置: (0.4, 0.0, 0°)");
    } else {
        // 关节空间：所有电机回零
        target_joint_angles_.fill(0.0);
        RCLCPP_INFO(this->get_logger(), "所有关节回零位");
    }
}

void JoystickStrikeControlNode::emergencyStop() {
    emergency_stop_ = true;
    enabled_ = false;
    RCLCPP_WARN(this->get_logger(), "🛑 急停触发！控制已禁用");
}

void JoystickStrikeControlNode::switchToTaskSpace() {
    if (control_mode_ != 0) {
        control_mode_ = 0;
        // 使用当前状态作为新目标
        if (has_state_feedback_) {
            target_x_ = current_x_;
            target_y_ = current_y_;
            target_orientation_ = current_orientation_;
        }
        RCLCPP_INFO(this->get_logger(), 
            "✓ 切换到任务空间模式（控制末端位姿）");
    }
}

void JoystickStrikeControlNode::switchToJointSpace() {
    if (control_mode_ != 1) {
        control_mode_ = 1;
        // 使用当前关节角度作为新目标
        if (has_state_feedback_) {
            target_joint_angles_ = current_joint_angles_;
        }
        RCLCPP_INFO(this->get_logger(), 
            "✓ 切换到关节空间模式（直接控制电机）");
    }
}

void JoystickStrikeControlNode::stateCallback(
    const motor_control_ros2::msg::ArmState::SharedPtr msg) {
    
    // 更新末端状态
    current_x_ = msg->plate_position.x;
    current_y_ = msg->plate_position.y;
    current_orientation_ = msg->plate_orientation;
    
    // 更新关节角度
    for (size_t i = 0; i < 4; ++i) {
        current_joint_angles_[i] = msg->motor_positions[i];
    }
    
    has_state_feedback_ = true;
}

void JoystickStrikeControlNode::joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
    
    last_joy_time_ = this->now();
    
    // 边沿检测：检测按钮按下事件（从未按下变为按下）
    auto is_button_pressed = [&](int button_id) -> bool {
        if (button_id >= static_cast<int>(msg->buttons.size())) return false;
        bool pressed_now = msg->buttons[button_id] > 0;
        bool was_pressed = button_pressed_last_[button_id];
        button_pressed_last_[button_id] = pressed_now;
        return pressed_now && !was_pressed;  // 按下沿
    };
    
    // 处理按钮事件（仅在按下沿触发）
    if (is_button_pressed(button_start_)) {
        enabled_ = !enabled_;
        emergency_stop_ = false;
        if (enabled_) {
            RCLCPP_INFO(this->get_logger(), "✓ 控制已启用");
            // 启用时，使用当前状态作为初始目标
            if (has_state_feedback_) {
                if (control_mode_ == 0) {
                    target_x_ = current_x_;
                    target_y_ = current_y_;
                    target_orientation_ = current_orientation_;
                } else {
                    target_joint_angles_ = current_joint_angles_;
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 控制已禁用");
        }
    }
    
    if (is_button_pressed(button_a_)) {
        resetToCenter();
    }
    
    if (is_button_pressed(button_b_)) {
        emergencyStop();
    }
    
    if (is_button_pressed(button_x_)) {
        switchToTaskSpace();
    }
    
    if (is_button_pressed(button_y_)) {
        switchToJointSpace();
    }
    
    if (is_button_pressed(button_lb_)) {
        // 降低 PID 增益（更柔顺）
        for (size_t i = 0; i < 4; ++i) {
            kp_values_[i] = std::max(10.0, kp_values_[i] - kp_step_);
            kd_values_[i] = kp_values_[i] * 0.1;  // Kd = Kp / 10
        }
        RCLCPP_INFO(this->get_logger(), "刚度降低 → Kp: [%.0f, %.0f, %.0f, %.0f]",
            kp_values_[0], kp_values_[1], kp_values_[2], kp_values_[3]);
    }
    
    if (is_button_pressed(button_rb_)) {
        // 提高 PID 增益（更刚性）
        for (size_t i = 0; i < 4; ++i) {
            kp_values_[i] = std::min(150.0, kp_values_[i] + kp_step_);
            kd_values_[i] = kp_values_[i] * 0.1;
        }
        RCLCPP_INFO(this->get_logger(), "刚度提高 → Kp: [%.0f, %.0f, %.0f, %.0f]",
            kp_values_[0], kp_values_[1], kp_values_[2], kp_values_[3]);
    }
    
    if (is_button_pressed(button_back_)) {
        // 重置到初始状态
        enabled_ = false;
        emergency_stop_ = false;
        control_mode_ = 0;
        resetToCenter();
        RCLCPP_INFO(this->get_logger(), "系统重置到初始状态");
    }
    
    // 如果未启用或急停，不处理摇杆输入
    if (!enabled_ || emergency_stop_) {
        return;
    }
    
    // 读取轴数据（带死区处理）
    auto get_axis = [&](int axis_id) -> double {
        if (axis_id >= static_cast<int>(msg->axes.size())) return 0.0;
        return applyDeadzone(msg->axes[axis_id], deadzone_);
    };
    
    double left_x = get_axis(axis_left_x_);
    double left_y = get_axis(axis_left_y_);
    double right_x = get_axis(axis_right_x_);
    double right_y = get_axis(axis_right_y_);
    
    // LT/RT 触发器读取（从 1.0 到 -1.0，未按=1.0，全按=-1.0）
    double lt = (axis_lt_ < static_cast<int>(msg->axes.size())) ? msg->axes[axis_lt_] : 1.0;
    double rt = (axis_rt_ < static_cast<int>(msg->axes.size())) ? msg->axes[axis_rt_] : 1.0;
    
    // 计算速度增益（LT减速，RT加速）
    double trigger_gain = 1.0;
    if (lt < 0.9) {  // LT 被按下
        trigger_gain = speed_gain_slow_;
    } else if (rt < 0.9) {  // RT 被按下
        trigger_gain = speed_gain_fast_;
    } else {
        trigger_gain = current_speed_gain_;
    }
    
    // D-pad 读取
    double dpad_x = get_axis(axis_dpad_x_);
    double dpad_y = get_axis(axis_dpad_y_);
    
    // 计算时间增量
    auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    
    if (dt <= 0.0 || dt > 1.0) {
        dt = 1.0 / publish_rate_;
    }
    
    // 根据控制模式更新目标
    if (control_mode_ == 0) {
        // 任务空间模式：控制末端位置和姿态
        double dx = left_y * max_position_delta_ * trigger_gain * dt;
        double dy = -left_x * max_position_delta_ * trigger_gain * dt;  // 左负右正
        double dori = -right_x * max_orientation_delta_ * trigger_gain * dt;
        
        // D-pad 微调
        dx += dpad_y * 0.01;  // 每次1cm
        dy += dpad_x * 0.01;
        
        target_x_ += dx;
        target_y_ += dy;
        target_orientation_ += dori;
        
        // 限幅
        target_x_ = std::clamp(target_x_, min_x_, max_x_);
        target_y_ = std::clamp(target_y_, min_y_, max_y_);
        target_orientation_ = std::clamp(target_orientation_, -max_orientation_, max_orientation_);
        
    } else {
        // 关节空间模式：直接控制4个电机
        double d_left_main = left_y * max_joint_delta_ * trigger_gain * dt;
        double d_left_sub = -left_x * max_joint_delta_ * trigger_gain * dt;
        double d_right_main = right_y * max_joint_delta_ * trigger_gain * dt;
        double d_right_sub = -right_x * max_joint_delta_ * trigger_gain * dt;
        
        target_joint_angles_[0] += d_left_main;
        target_joint_angles_[1] += d_left_sub;
        target_joint_angles_[2] += d_right_main;
        target_joint_angles_[3] += d_right_sub;
        
        // 关节角度限幅（±π）
        for (auto& angle : target_joint_angles_) {
            angle = std::clamp(angle, -M_PI, M_PI);
        }
    }
}

void JoystickStrikeControlNode::publishCommand() {
    // 检查手柄超时
    double time_since_joy = (this->now() - last_joy_time_).seconds();
    if (time_since_joy > joy_timeout_) {
        if (enabled_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⚠️ 手柄信号超时 (%.1fs)，保持当前位置", time_since_joy);
        }
        // 超时不发布命令（保持上一次的目标）
        return;
    }
    
    // 如果未启用，不发布
    if (!enabled_) {
        return;
    }
    
    // 构造命令消息
    auto cmd = motor_control_ros2::msg::ArmJointCommand();
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "joystick";
    cmd.control_mode = control_mode_;
    
    if (control_mode_ == 0) {
        // 任务空间
        cmd.target_position.x = target_x_;
        cmd.target_position.y = target_y_;
        cmd.target_position.z = 0.0;
        cmd.target_orientation = target_orientation_;
        cmd.target_velocity.x = 0.0;
        cmd.target_velocity.y = 0.0;
        cmd.target_velocity.z = 0.0;
    } else {
        // 关节空间
        cmd.left_main_motor = target_joint_angles_[0];
        cmd.left_sub_motor = target_joint_angles_[1];
        cmd.right_main_motor = target_joint_angles_[2];
        cmd.right_sub_motor = target_joint_angles_[3];
    }
    
    // PID 参数
    for (size_t i = 0; i < 4; ++i) {
        cmd.kp[i] = kp_values_[i];
        cmd.kd[i] = kd_values_[i];
        cmd.torque_ff[i] = 0.0;
    }
    
    cmd_pub_->publish(cmd);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::JoystickStrikeControlNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "节点异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
