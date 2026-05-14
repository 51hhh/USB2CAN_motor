#ifndef MOTOR_CONTROL_ROS2__ARM_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__ARM_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "motor_control_ros2/msg/arm_joint_command.hpp"
#include "motor_control_ros2/msg/arm_state.hpp"
#include "motor_control_ros2/msg/unitree_motor_command.hpp"
#include "motor_control_ros2/msg/unitree_motor_state.hpp"

#include <array>
#include <string>
#include <memory>
#include <cmath>

namespace motor_control {

/**
 * @brief 双臂并联五杆机构击球系统控制节点
 * 
 * 系统结构：
 * - 两条完全相同的五杆机械臂（左/右对称）
 * - 每条臂：主电机 + 副电机（串联） + 连杆驱动小臂
 * - 4个宇树电机（每臂2个）
 * - 末端刚性连接击球板 → 并联约束
 * 
 * 单臂五杆机构：
 *   主电机定子 ← 固定在躯干
 *   主电机转子 ← 连接副电机外壳
 *   副电机外壳 ← 与大臂固定一体
 *   副电机转子 ← 通过连杆驱动小臂
 *   小臂 ← 肘关节与大臂铰接，末端连击球板
 * 
 * 功能：
 * - 并联运动学求解（末端位姿 ↔ 关节角度）
 * - 双臂协调控制
 * - 五杆机构动力学补偿
 * - 订阅击球命令 (/strike_command)
 * - 发布系统状态 (/strike_state)
 */
class ArmControlNode : public rclcpp::Node {
public:
    ArmControlNode();
    
private:
    // ========== 配置加载 ==========
    void loadParallelArmParams(const std::string& config_file);
    
    // ========== 回调函数 ==========
    void strikeCommandCallback(const motor_control_ros2::msg::ArmJointCommand::SharedPtr msg);
    void motorStateCallback(const motor_control_ros2::msg::UnitreeMotorState::SharedPtr msg);
    
    // ========== 控制循环 ==========
    void controlLoop();
    
    // ========== 五杆机构运动学（单臂） ==========
    /**
     * @brief 单臂正运动学：(主电机角, 副电机角) → 末端位置
     * @param theta1 主电机角度（rad）
     * @param theta2 副电机角度（rad）
     * @param is_left 是否为左臂
     * @return 末端位置 (x, y)
     */
    std::array<double, 2> singleArmFK(double theta1, double theta2, bool is_left);
    
    /**
     * @brief 单臂逆运动学：末端位置 → (主电机角, 副电机角)
     * @param target_x 目标X坐标（m）
     * @param target_y 目标Y坐标（m）
     * @param is_left 是否为左臂
     * @param theta1 输出：主电机角度
     * @param theta2 输出：副电机角度
     * @return 是否有解
     */
    bool singleArmIK(double target_x, double target_y, bool is_left,
                     double& theta1, double& theta2);
    
    /**
     * @brief 计算肘关节角度（由连杆机构决定）
     * @param theta1 主电机角度
     * @param theta2 副电机角度
     * @return 肘关节角度（rad）
     */
    double computeElbowAngle(double theta1, double theta2);
    
    // ========== 并联运动学 ==========
    /**
     * @brief 并联正运动学：4个电机角度 → 击球板位姿
     * @param motor_angles [左主, 左副, 右主, 右副]
     * @return [x, y, orientation]
     */
    std::array<double, 3> parallelFK(const std::array<double, 4>& motor_angles);
    
    /**
     * @brief 并联逆运动学：击球板位姿 → 4个电机角度
     * @param target_x 目标X坐标（m）
     * @param target_y 目标Y坐标（m）
     * @param target_ori 目标姿态（rad）
     * @param solution 输出：4个电机角度
     * @return 是否有解
     */
    bool parallelIK(double target_x, double target_y, double target_ori,
                    std::array<double, 4>& solution);
    
    /**
     * @brief 计算双臂协调误差
     * @return 左右臂末端位置差异（米）
     */
    double computeCoordinationError();
    
    // ========== 发布命令 ==========
    void publishMotorCommands();
    void publishStrikeState();
    
    // ========== ROS2 通信 ==========
    // 订阅者
    rclcpp::Subscription<motor_control_ros2::msg::ArmJointCommand>::SharedPtr strike_cmd_sub_;
    rclcpp::Subscription<motor_control_ros2::msg::UnitreeMotorState>::SharedPtr motor_state_sub_;
    
    // 发布者
    rclcpp::Publisher<motor_control_ros2::msg::UnitreeMotorCommand>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<motor_control_ros2::msg::ArmState>::SharedPtr strike_state_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // ========== 系统参数 ==========
    struct ParallelArmParams {
        // 机械臂间距（米）
        double arm_spacing;              // 左右两臂肩部间距
        
        // 单臂五杆机构几何参数（米）
        double upper_arm_length;        // 大臂长度（L1）
        double forearm_length;          // 小臂长度（L2）
        double linkage_length;          // 连杆长度（L3）
        double linkage_attach_upper;    // 连杆在大臂上的附着点距离
        double linkage_attach_forearm;  // 连杆在小臂上的附着点距离
        
        // 击球板参数
        double plate_width;             // 击球板宽度
        double plate_height;            // 击球板高度
        
        // 运动范围
        double workspace_x_min;         // 工作空间X范围
        double workspace_x_max;
        double workspace_y_min;         // 工作空间Y范围
        double workspace_y_max;
        double max_orientation;         // 最大姿态角（rad）
        
        // 动力学参数
        double max_velocity;            // 最大速度（m/s）
        double max_acceleration;        // 最大加速度（m/s²）
        double max_motor_velocity;      // 电机最大角速度（rad/s）
        double max_motor_torque;        // 电机最大力矩（Nm）
        
        // 控制参数
        double control_frequency;       // 控制频率（Hz）
        double coordination_threshold;  // 协调误差阈值（米）
    };
    ParallelArmParams params_;
    
    // ========== 电机名称映射 ==========
    std::array<std::string, 4> motor_names_;  // [左主, 左副, 右主, 右副]
    
    // ========== 状态变量 ==========
    std::array<double, 4> current_motor_positions_;   // 当前电机角度 (rad)
    std::array<double, 4> current_motor_velocities_;  // 当前电机速度 (rad/s)
    std::array<double, 4> current_motor_torques_;     // 当前电机力矩 (Nm)
    std::array<int8_t, 4> current_temperatures_;      // 当前温度 (℃)
    std::array<bool, 4> motors_online_;               // 电机在线状态
    
    // 目标状态（任务空间）
    double target_x_, target_y_;      // 目标位置（m）
    double target_orientation_;       // 目标姿态（rad
    double target_vx_, target_vy_;    // 目标速度（m/s）
    
    // 目标状态（关节空间）
    std::array<double, 4> target_motor_positions_;    // 目标电机角度 (rad)
    std::array<double, 4> target_motor_velocities_;   // 目标电机速度 (rad/s)
    std::array<double, 4> target_motor_torques_;      // 目标力矩前馈 (Nm)
    std::array<double, 4> control_kp_;                // 位置控制增益
    std::array<double, 4> control_kd_;                // 速度控制增益
    
    uint8_t control_mode_;  // 0=任务空间，1=关节空间
    
    // ========== 安全监控 ==========
    rclcpp::Time last_command_time_;
    double command_timeout_;          // 命令超时时间 (s)
    double max_coordination_error_;   // 历史最大协调误差 (m)
    
    // ========== 统计信息 ==========
    uint64_t control_loop_count_;
    rclcpp::Time last_state_pub_time_;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__ARM_CONTROL_NODE_HPP_
