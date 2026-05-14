#ifndef MOTOR_CONTROL_ROS2__JOYSTICK_STRIKE_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__JOYSTICK_STRIKE_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "motor_control_ros2/msg/arm_joint_command.hpp"
#include "motor_control_ros2/msg/arm_state.hpp"

namespace motor_control {

/**
 * @brief 手柄击球拍控制节点
 * 
 * 功能：
 * - 订阅 joy 消息（来自 joy_node）
 * - 将手柄输入转换为击球拍控制命令（任务空间或关节空间）
 * - 支持两种控制模式切换：
 *   * 模式0（任务空间）：控制击球板末端位置和姿态
 *   * 模式1（关节空间）：直接控制4个电机角度
 * 
 * Xbox 手柄映射：
 * 
 * 【任务空间模式】（默认）
 * - 左摇杆 X/Y：控制击球板位置 (X, Y)
 * - 右摇杆 X：控制击球板姿态角度
 * - LT/RT：减速/加速移动速度
 * - D-pad 上/下：微调 Y 位置
 * - D-pad 左/右：微调 X 位置
 * 
 * 【关节空间模式】
 * - 左摇杆 Y：控制左主电机角度
 * - 左摇杆 X：控制左副电机角度
 * - 右摇杆 Y：控制右主电机角度
 * - 右摇杆 X：控制右副电机角度
 * 
 * 【通用按钮】
 * - A 键：回零位（中心位置）
 * - B 键：急停（保持当前位置）
 * - X 键：切换到任务空间模式
 * - Y 键：切换到关节空间模式
 * - LB：降低 PID 增益（更柔顺）
 * - RB：提高 PID 增益（更刚性）
 * - Start：启用/禁用控制
 * - Back：重置到初始状态
 */
class JoystickStrikeControlNode : public rclcpp::Node {
public:
    JoystickStrikeControlNode();

private:
    // 回调函数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void stateCallback(const motor_control_ros2::msg::ArmState::SharedPtr msg);
    void publishCommand();
    
    // 工具函数
    double applyDeadzone(double value, double deadzone);
    void resetToCenter();
    void emergencyStop();
    void switchToTaskSpace();
    void switchToJointSpace();
    
    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<motor_control_ros2::msg::ArmState>::SharedPtr state_sub_;
    rclcpp::Publisher<motor_control_ros2::msg::ArmJointCommand>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // 参数配置
    double deadzone_;              // 摇杆死区
    double publish_rate_;          // 发布频率 (Hz)
    double joy_timeout_;           // 手柄超时 (s)
    
    // 任务空间参数
    double max_position_delta_;    // 位置最大变化量 (m/s)
    double max_orientation_delta_; // 姿态最大变化量 (rad/s)
    double min_x_, max_x_;         // X 工作范围
    double min_y_, max_y_;         // Y 工作范围
    double max_orientation_;       // 最大姿态角
    
    // 关节空间参数
    double max_joint_delta_;       // 关节角度最大变化量 (rad/s)
    
    // PID 参数
    std::array<double, 4> kp_values_;
    std::array<double, 4> kd_values_;
    double kp_step_;               // PID 调节步长
    
    // 速度增益
    double speed_gain_slow_;
    double speed_gain_normal_;
    double speed_gain_fast_;
    
    // 轴映射
    int axis_left_x_, axis_left_y_;
    int axis_right_x_, axis_right_y_;
    int axis_lt_, axis_rt_;
    int axis_dpad_x_, axis_dpad_y_;
    
    // 按钮映射
    int button_a_, button_b_, button_x_, button_y_;
    int button_lb_, button_rb_;
    int button_start_, button_back_;
    
    // 状态变量
    bool enabled_;
    bool emergency_stop_;
    uint8_t control_mode_;         // 0=任务空间, 1=关节空间
    double current_speed_gain_;
    
    // 任务空间目标
    double target_x_, target_y_;
    double target_orientation_;
    
    // 关节空间目标
    std::array<double, 4> target_joint_angles_;
    
    // 当前状态（从反馈获取）
    double current_x_, current_y_, current_orientation_;
    std::array<double, 4> current_joint_angles_;
    bool has_state_feedback_;
    
    // 时间戳
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_update_time_;
    
    // 按钮状态记录（用于边沿检测）
    std::vector<bool> button_pressed_last_;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__JOYSTICK_STRIKE_CONTROL_NODE_HPP_
