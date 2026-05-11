#include "motor_control_ros2/remote_control_node.hpp"
#include <algorithm>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace motor_control {

RemoteControlNode::RemoteControlNode() : Node("remote_control_node") {
    RCLCPP_INFO(this->get_logger(), "正在初始化遥控器节点...");

    std::string config_file;
    try {
        config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") +
            "/config/remote_control_params.yaml";
        RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());
        loadRemoteControlParams(config_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "遥控器节点初始化失败: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "配置文件路径: %s", config_file.c_str());
        throw;
    }

    current_speed_scale_ = config_.default_speed_scale;
    last_arm_press_time_ = this->now();

    // 创建订阅者
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        config_.joy_topic, 10,
        std::bind(&RemoteControlNode::joyCallback, this, std::placeholders::_1)
    );
    arm_ready_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/delta_arm/ready", 10,
        std::bind(&RemoteControlNode::readyCallback, this, std::placeholders::_1)
    );

    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, 10
    );
    arm_target_pub_ = this->create_publisher<motor_control_ros2::msg::ArmTarget>(
        "/delta_arm/target", 10
    );
    serve_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/serve/trigger", 10
    );
    
    // 创建定时发布器
    auto period = std::chrono::duration<double>(1.0 / config_.publish_frequency);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RemoteControlNode::publishTimer, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "遥控器节点启动成功");
    RCLCPP_INFO(this->get_logger(), 
        "轴配置 - 前后: %d, 左右: %d, 旋转: %d",
        config_.axis_forward, config_.axis_strafe, config_.axis_rotate);
    RCLCPP_INFO(this->get_logger(), 
        "按钮配置 - 加速: %d, 减速: %d, 停止: %d, 发球: %d",
        config_.button_speed_up, config_.button_speed_down, config_.button_stop,
        config_.button_serve_trigger);
    RCLCPP_INFO(this->get_logger(), 
        "速度配置 - 线速度上限: %.2f m/s, 角速度上限: %.2f rad/s",
        config_.max_linear_velocity, config_.max_angular_velocity);
    RCLCPP_INFO(this->get_logger(),
        "话题配置 - Joy: %s, CmdVel输出: %s",
        config_.joy_topic.c_str(), config_.cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
        "默认速度缩放因子: %.1f%%, 死区: %.2f",
        config_.default_speed_scale * 100.0, config_.deadzone);
    RCLCPP_INFO(this->get_logger(),
        "delta_arm 扳机控制: axis=%d range=[%.3f, %.3f] rad trigger=%.2f reset=%.2f",
        config_.arm_trigger_axis,
        config_.arm_min_angle_rad,
        config_.arm_max_angle_rad,
        config_.arm_trigger_threshold,
        config_.arm_trigger_reset_threshold);
}

void RemoteControlNode::loadRemoteControlParams(const std::string& config_file) {
    YAML::Node root = YAML::LoadFile(config_file);
    YAML::Node params = root;

    if (root["remote_control_node"] && root["remote_control_node"]["ros__parameters"]) {
        params = root["remote_control_node"]["ros__parameters"];
    }

    auto loadInt = [&params](const char* key, int& value) {
        if (params[key]) {
            value = params[key].as<int>();
        }
    };

    auto loadDouble = [&params](const char* key, double& value) {
        if (params[key]) {
            value = params[key].as<double>();
        }
    };

    auto loadString = [&params](const char* key, std::string& value) {
        if (params[key]) {
            value = params[key].as<std::string>();
        }
    };

    loadInt("axis_forward", config_.axis_forward);
    loadInt("axis_strafe", config_.axis_strafe);
    loadInt("axis_rotate", config_.axis_rotate);
    loadInt("button_speed_up", config_.button_speed_up);
    loadInt("button_speed_down", config_.button_speed_down);
    loadInt("button_stop", config_.button_stop);
    loadInt("button_serve_trigger", config_.button_serve_trigger);

    loadInt("arm_trigger_axis", config_.arm_trigger_axis);
    loadDouble("arm_trigger_threshold", config_.arm_trigger_threshold);
    loadDouble("arm_trigger_reset_threshold", config_.arm_trigger_reset_threshold);
    loadDouble("arm_min_angle_rad", config_.arm_min_angle_rad);
    loadDouble("arm_max_angle_rad", config_.arm_max_angle_rad);
    loadInt("arm_protection_press_limit", config_.arm_protection_press_limit);
    loadDouble("arm_protection_window_sec", config_.arm_protection_window_sec);

    loadDouble("max_linear_velocity", config_.max_linear_velocity);
    loadDouble("max_angular_velocity", config_.max_angular_velocity);
    loadDouble("default_speed_scale", config_.default_speed_scale);
    loadDouble("speed_scale_step", config_.speed_scale_step);
    loadDouble("deadzone", config_.deadzone);
    loadDouble("publish_frequency", config_.publish_frequency);
    loadString("joy_topic", config_.joy_topic);
    loadString("cmd_vel_topic", config_.cmd_vel_topic);

    if (config_.arm_max_angle_rad < config_.arm_min_angle_rad) {
        std::swap(config_.arm_max_angle_rad, config_.arm_min_angle_rad);
        RCLCPP_WARN(this->get_logger(), "检测到 arm_min_angle_rad > arm_max_angle_rad，已自动交换");
    }

    if (config_.publish_frequency <= 0.0) {
        throw std::runtime_error("publish_frequency 必须大于 0");
    }

    config_.deadzone = std::clamp(config_.deadzone, 0.0, 0.99);
    config_.default_speed_scale = std::max(0.0, config_.default_speed_scale);
    config_.speed_scale_step = std::max(0.0, config_.speed_scale_step);
    config_.arm_trigger_threshold = std::clamp(config_.arm_trigger_threshold, 0.0, 1.0);
    config_.arm_trigger_reset_threshold = std::clamp(config_.arm_trigger_reset_threshold, 0.0, 1.0);

    RCLCPP_INFO(this->get_logger(), "遥控器参数加载完成");
}

void RemoteControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // 检查按钮索引是否有效
    if (msg->buttons.size() <= static_cast<size_t>(std::max({
        config_.button_speed_up, 
        config_.button_speed_down, 
        config_.button_stop,
        config_.button_serve_trigger}))) {
        return;
    }
    
    // 检查轴索引是否有效
    if (msg->axes.size() <= static_cast<size_t>(std::max({
        config_.axis_forward,
        config_.axis_strafe,
        config_.axis_rotate,
        config_.arm_trigger_axis}))) {
        return;
    }
    
    // 速度缩放调整（检测按钮按下事件，而不是按住）
    // 加速按钮：每按一下增加 10%
    if (msg->buttons[config_.button_speed_up] && !last_button_speed_up_) {
        current_speed_scale_ = std::min(20.0,  // 最大 1000%
            current_speed_scale_ + config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "加速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_up_ = msg->buttons[config_.button_speed_up];
    
    // 减速按钮：每按一下减少 10%
    if (msg->buttons[config_.button_speed_down] && !last_button_speed_down_) {
        current_speed_scale_ = std::max(0.0,  // 最小 0%
            current_speed_scale_ - config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "减速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_down_ = msg->buttons[config_.button_speed_down];
    
    // 紧急停止按钮（检测按下事件）
    if (msg->buttons[config_.button_stop] && !last_button_stop_) {
        last_cmd_.linear.x = 0.0;
        last_cmd_.linear.y = 0.0;
        last_cmd_.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "紧急停止");
    }
    last_button_stop_ = msg->buttons[config_.button_stop];

    if (msg->buttons[config_.button_serve_trigger] && !last_button_serve_trigger_) {
        auto serve_msg = std_msgs::msg::Bool();
        serve_msg.data = true;
        serve_trigger_pub_->publish(serve_msg);
        RCLCPP_INFO(this->get_logger(), "发送风车发球触发");
    }
    last_button_serve_trigger_ = msg->buttons[config_.button_serve_trigger];
    
    // 读取摇杆输入并应用死区
    double forward = applyDeadzone(msg->axes[config_.axis_forward], config_.deadzone);
    double strafe = applyDeadzone(msg->axes[config_.axis_strafe], config_.deadzone);
    double rotate = applyDeadzone(msg->axes[config_.axis_rotate], config_.deadzone);

    // 对平移摇杆做圆形归一化，避免前后/左右分量各自吃满后，
    // 斜向满杆出现 sqrt(2) 倍的合速度。
    const double linear_input_norm = std::hypot(forward, strafe);
    if (linear_input_norm > 1.0) {
        forward /= linear_input_norm;
        strafe /= linear_input_norm;
    }
    
    // 应用速度缩放和最大速度限制
    double speed_scale = getSpeedScaleFactor();
    
    // 构建速度命令
    last_cmd_.linear.x = forward * config_.max_linear_velocity * speed_scale;
    last_cmd_.linear.y = strafe * config_.max_linear_velocity * speed_scale;
    last_cmd_.angular.z = rotate * config_.max_angular_velocity * speed_scale;

    updateArmControl(msg);

    // 立即发布
    cmd_vel_pub_->publish(last_cmd_);
}

double RemoteControlNode::applyDeadzone(double value, double deadzone) {
    // 限制在 [-100, 100] 范围内
    value = std::clamp(value, -100.0, 100.0);
    
    // 应用死区
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    
    // 线性调整：移除死区后进行缩放
    // 如果 value > deadzone，返回 (value - deadzone) / (1 - deadzone)
    // 如果 value < -deadzone，返回 (value + deadzone) / (1 - deadzone)
    if (value > 0) {
        return (value - deadzone) / (1.0 - deadzone);
    } else {
        return (value + deadzone) / (1.0 - deadzone);
    }
}

double RemoteControlNode::getSpeedScaleFactor() {
    return current_speed_scale_;
}

double RemoteControlNode::normalizeTrigger(double raw_value) const {
    double normalized = (1.0 - raw_value) / 2.0;
    return std::clamp(normalized, 0.0, 1.0);
}

void RemoteControlNode::readyCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "READY") {
        arm_ready_ = true;
        if (arm_trigger_state_ == ArmTriggerState::COMMAND_SENT) {
            arm_trigger_state_ = ArmTriggerState::WAIT_TRIGGER_RESET;
        }
    }
}

void RemoteControlNode::updateArmControl(const sensor_msgs::msg::Joy::SharedPtr msg) {
    const double trigger = normalizeTrigger(msg->axes[config_.arm_trigger_axis]);
    const bool trigger_active = trigger >= config_.arm_trigger_threshold;
    const bool trigger_reset = trigger <= config_.arm_trigger_reset_threshold;

    if (arm_trigger_state_ == ArmTriggerState::PROTECTED) {
        if (trigger_reset) {
            arm_trigger_state_ = ArmTriggerState::IDLE;
            arm_press_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "delta_arm 触发保护解除");
        }
        return;
    }

    if (arm_trigger_state_ == ArmTriggerState::COMMAND_SENT) {
        if (arm_ready_) {
            arm_trigger_state_ = ArmTriggerState::WAIT_TRIGGER_RESET;
        }
        return;
    }

    if (arm_trigger_state_ == ArmTriggerState::WAIT_TRIGGER_RESET) {
        if (trigger_reset) {
            arm_trigger_state_ = ArmTriggerState::IDLE;
            RCLCPP_DEBUG(this->get_logger(), "delta_arm 扳机已回位，允许再次触发");
        }
        return;
    }

    if (!trigger_active) {
        return;
    }

    auto now = this->now();
    if ((now - last_arm_press_time_).seconds() <= config_.arm_protection_window_sec) {
        arm_press_count_ += 1;
    } else {
        arm_press_count_ = 1;
    }
    last_arm_press_time_ = now;

    if (arm_press_count_ > config_.arm_protection_press_limit) {
        arm_trigger_state_ = ArmTriggerState::PROTECTED;
        RCLCPP_WARN(this->get_logger(),
            "delta_arm 触发过于频繁，进入保护（%d 次 / %.2f s）",
            arm_press_count_, config_.arm_protection_window_sec);
        return;
    }

    const double angle = config_.arm_min_angle_rad +
        (config_.arm_max_angle_rad - config_.arm_min_angle_rad) ;

    auto arm_msg = motor_control_ros2::msg::ArmTarget();
    arm_msg.header.stamp = this->now();
    arm_msg.target_angles = {angle, angle, angle};
    arm_msg.execute = true;
    arm_target_pub_->publish(arm_msg);

    arm_ready_ = false;
    arm_trigger_state_ = ArmTriggerState::COMMAND_SENT;
    RCLCPP_INFO(this->get_logger(), "发送 delta_arm 目标: %.3f rad", angle);
}

void RemoteControlNode::publishTimer() {
    // 定时发布最后一条命令
    // 这样即使遥控器输入不变，也能持续发送命令信号
    cmd_vel_pub_->publish(last_cmd_);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
