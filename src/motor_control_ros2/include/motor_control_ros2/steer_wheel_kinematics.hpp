#ifndef MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_
#define MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_

#include <cmath>

namespace motor_control {

/**
 * @brief 舵轮单元命令
 */
struct WheelCommand {
    double angle;     // 舵轮角度 (度, 0-360)
    double velocity;  // 轮速 (m/s)
    
    WheelCommand() : angle(0.0), velocity(0.0) {}
    WheelCommand(double a, double v) : angle(a), velocity(v) {}
};

/**
 * @brief 舵轮运动学类
 * 
 * 实现 4 轮舵轮底盘的运动学解算：
 * - 逆运动学：底盘速度 → 4 个舵轮的角度和速度
 * - 正运动学：4 个舵轮的角度和速度 → 底盘速度
 * - 舵角优化：最短路径旋转（避免 180° 跳变）
 */
class SteerWheelKinematics {
public:
    /**
     * @brief 构造函数
     * @param wheel_base_x 前后轮距 (m)
     * @param wheel_base_y 左右轴距 (m)
     * @param wheel_radius 轮子半径 (m)
     */
    SteerWheelKinematics(double wheel_base_x, double wheel_base_y, double wheel_radius);
    
    /**
     * @brief 逆运动学：底盘速度 → 4 个舵轮命令
     * 
     * @param vx 底盘前进速度 (m/s)
     * @param vy 底盘横向速度 (m/s)
     * @param wz 底盘旋转角速度 (rad/s)
     * @param fl 左前轮命令（输出）
     * @param fr 右前轮命令（输出）
     * @param rl 左后轮命令（输出）
     * @param rr 右后轮命令（输出）
     */
    void inverseKinematics(
        double vx, double vy, double wz,
        WheelCommand& fl, WheelCommand& fr,
        WheelCommand& rl, WheelCommand& rr
    );
    
    /**
     * @brief 正运动学：4 个舵轮状态 → 底盘速度
     * 
     * @param fl 左前轮状态
     * @param fr 右前轮状态
     * @param rl 左后轮状态
     * @param rr 右后轮状态
     * @param vx 底盘前进速度（输出）
     * @param vy 底盘横向速度（输出）
     * @param wz 底盘旋转角速度（输出）
     */
    void forwardKinematics(
        const WheelCommand& fl, const WheelCommand& fr,
        const WheelCommand& rl, const WheelCommand& rr,
        double& vx, double& vy, double& wz
    );
    
    /**
     * @brief 舵角优化：最短路径旋转
     * 
     * 如果目标角度与当前角度相差超过 90°，则反转速度并调整角度，
     * 避免舵轮大角度旋转。
     * 
     * @param current_angle 当前角度 (度, 0-360)
     * @param target_angle 目标角度 (度, 0-360)
     * @param velocity 速度 (m/s)，可能被反向
     * @return 优化后的目标角度 (度, 0-360)
     */
    static double optimizeSteerAngle(
        double current_angle,
        double target_angle,
        double& velocity
    );
    
    /**
     * @brief 归一化角度到 [0, 360) 范围
     */
    static double normalizeAngle(double angle);
    
    /**
     * @brief 计算两个角度之间的最短角度差（带符号）
     * @return 角度差 (-180, 180]
     */
    static double angleDifference(double angle1, double angle2);

private:
    double wheel_base_x_;  // 前后轮距 (m)
    double wheel_base_y_;  // 左右轴距 (m)
    double wheel_radius_;  // 轮子半径 (m)
    
    // 轮子位置（相对于底盘中心）
    double lx_;  // 半轮距
    double ly_;  // 半轴距
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__STEER_WHEEL_KINEMATICS_HPP_
