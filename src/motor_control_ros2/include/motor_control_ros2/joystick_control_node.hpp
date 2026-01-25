#ifndef MOTOR_CONTROL_ROS2__JOYSTICK_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__JOYSTICK_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

namespace motor_control {

/**
 * @brief Xbox 手柄控制节点
 * 
 * 功能：
 * - 订阅 joy 消息（来自 joy_node）
 * - 将手柄摇杆输入转换为底盘速度命令
 * - 支持速度增益调节（LT/RT 触发器）
 * - 支持急停按钮（B 键）
 * - 支持死区、加速度限制、速度平滑
 * 
 * Xbox 手柄映射（标准 ROS joy 驱动）：
 * 轴 (axes):
 *   0: 左摇杆 X 轴（左负右正）→ 底盘横向速度 vy
 *   1: 左摇杆 Y 轴（下负上正）→ 底盘前进速度 vx
 *   2: LT 触发器（未按1.0，按下-1.0）→ 减速
 *   3: 右摇杆 X 轴（左负右正）→ 底盘旋转 wz
 *   4: 右摇杆 Y 轴（下负上正）→ 未使用
 *   5: RT 触发器（未按1.0，按下-1.0）→ 加速
 * 
 * 按钮 (buttons):
 *   0: A - 正常速度模式
 *   1: B - 急停
 *   2: X - 慢速模式
 *   3: Y - 快速模式
 *   4: LB - 未使用
 *   5: RB - 未使用
 *   6: Back - 未使用
 *   7: Start - 使能/禁用控制
 */
class JoystickControlNode : public rclcpp::Node {
public:
    JoystickControlNode();

private:
    // 回调函数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publishCmdVel();
    
    // 工具函数
    double applyDeadzone(double value, double deadzone);
    double applySpeedCurve(double value, double exponent);
    double rateLimit(double current, double target, double rate, double dt);
    
    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 参数
    double max_linear_velocity_;   // 最大线速度 (m/s)
    double max_angular_velocity_;  // 最大角速度 (rad/s)
    double deadzone_;              // 摇杆死区 [0, 1]
    double speed_curve_exponent_;  // 速度曲线指数（>1 更平滑启动）
    double publish_rate_;          // 发布频率 (Hz)
    double accel_limit_;           // 加速度限制 (m/s²)
    double angular_accel_limit_;   // 角加速度限制 (rad/s²)
    
    // 速度模式增益
    double speed_mode_slow_;       // 慢速模式增益
    double speed_mode_normal_;     // 正常模式增益
    double speed_mode_fast_;       // 快速模式增益
    
    // 轴/按钮映射
    int axis_linear_x_;
    int axis_linear_y_;
    int axis_angular_;
    int axis_lt_;                  // LT 触发器（减速）
    int axis_rt_;                  // RT 触发器（加速）
    
    int button_emergency_stop_;
    int button_slow_mode_;
    int button_normal_mode_;
    int button_fast_mode_;
    int button_enable_;
    
    // 状态变量
    bool enabled_;
    bool emergency_stop_;
    double current_speed_gain_;    // 当前速度增益
    
    // 目标速度（来自手柄）
    double target_vx_;
    double target_vy_;
    double target_wz_;
    
    // 当前速度（经过限制后）
    double current_vx_;
    double current_vy_;
    double current_wz_;
    
    // 时间戳
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_update_time_;
    double joy_timeout_;           // 手柄超时 (s)
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__JOYSTICK_CONTROL_NODE_HPP_
