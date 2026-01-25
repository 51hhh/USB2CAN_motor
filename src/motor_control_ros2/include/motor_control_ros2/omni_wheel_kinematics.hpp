#ifndef MOTOR_CONTROL_ROS2__OMNI_WHEEL_KINEMATICS_HPP_
#define MOTOR_CONTROL_ROS2__OMNI_WHEEL_KINEMATICS_HPP_

#include <cmath>
#include <array>

namespace motor_control {

/**
 * @brief X 形分布全向轮运动学（Omni Wheel，非 Mecanum Wheel）
 * 
 * 注意：全向轮 ≠ 麦克纳姆轮！
 * - 全向轮：整个轮子以45°安装，轮上的辊子垂直于主轴
 * - 麦克纳姆轮：轮子正向安装，但辊子相对轮子主轴成45°
 * 
 * 四个 GM3508 电机驱动的全向轮呈 X 形分布（每个轮子以45°角安装）：
 * 
 *        Y (前)
 *         ↑
 *         │    ╱ Wheel1 (左前，45°)
 *         │  ╱
 *         │╱
 *    ─────┼────────→ X (右)
 *       ╱ │
 *     ╱   │ 
 *   ╱     │
 * Wheel3 (左后，225°)
 * 
 * 轮子编号、位置与驱动方向：
 * - 轮 1 (FL): 左前，驱动方向 45° (相对X轴)
 * - 轮 2 (FR): 右前，驱动方向 135° (相对X轴)
 * - 轮 3 (RL): 左后，驱动方向 225° (相对X轴)
 * - 轮 4 (RR): 右后，驱动方向 315° (相对X轴)
 */
class OmniWheelKinematics {
public:
    /**
     * @brief 构造函数
     * @param wheel_base_x 前后轮距 (m)，底盘中心到前/后轮的距离
     * @param wheel_base_y 左右轮距 (m)，底盘中心到左/右轮的距离
     * @param wheel_radius 轮子半径 (m)
     * @param install_angle 轮子安装角度 (度)，默认 45° (X 形布置)
     */
    OmniWheelKinematics(
        double wheel_base_x, 
        double wheel_base_y, 
        double wheel_radius,
        double install_angle = 45.0
    );
    
    /**
     * @brief 逆运动学：底盘速度 → 4 个轮子速度
     * 
     * @param vx 底盘前进速度 (m/s)，前进为正
     * @param vy 底盘横向速度 (m/s)，左移为正
     * @param wz 底盘旋转角速度 (rad/s)，逆时针为正
     * @return 4 个轮子的线速度 [FL, FR, RL, RR] (m/s)
     */
    std::array<double, 4> inverseKinematics(double vx, double vy, double wz) const;
    
    /**
     * @brief 正运动学：4 个轮子速度 → 底盘速度
     * 
     * @param wheel_velocities 4 个轮子的线速度 [FL, FR, RL, RR] (m/s)
     * @param vx 底盘前进速度 (m/s)（输出）
     * @param vy 底盘横向速度 (m/s)（输出）
     * @param wz 底盘旋转角速度 (rad/s)（输出）
     */
    void forwardKinematics(
        const std::array<double, 4>& wheel_velocities,
        double& vx, double& vy, double& wz
    ) const;
    
    /**
     * @brief 将轮子线速度转换为电机 RPM
     * @param linear_velocity 轮子线速度 (m/s)
     * @return 电机 RPM (考虑减速比 19:1)
     */
    double velocityToRPM(double linear_velocity) const;
    
    /**
     * @brief 将电机 RPM 转换为轮子线速度
     * @param rpm 电机 RPM
     * @return 轮子线速度 (m/s)
     */
    double rpmToVelocity(double rpm) const;
    
    /**
     * @brief 获取轮子半径
     * @return 轮子半径 (m)
     */
    double getWheelRadius() const { return wheel_radius_; }
    
    /**
     * @brief 获取减速比
     * @return 减速比
     */
    double getGearRatio() const { return GEAR_RATIO; }

private:
    double wheel_base_x_;      // 前后轮距的一半 (m)
    double wheel_base_y_;      // 左右轮距的一半 (m)
    double wheel_radius_;      // 轮子半径 (m)
    double install_angle_rad_; // 轮子安装角度 (弧度)
    
    // GM3508 减速比
    static constexpr double GEAR_RATIO = 19.0;
    
    // 底盘半径 (从中心到轮子的距离)
    double chassis_radius_;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__OMNI_WHEEL_KINEMATICS_HPP_
