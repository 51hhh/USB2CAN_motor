#include "motor_control_ros2/omni_wheel_kinematics.hpp"
#include <cmath>

namespace motor_control {

OmniWheelKinematics::OmniWheelKinematics(
    double wheel_base_x, 
    double wheel_base_y, 
    double wheel_radius,
    double install_angle)
    : wheel_base_x_(wheel_base_x / 2.0)  // 转换为半轮距
    , wheel_base_y_(wheel_base_y / 2.0)  // 转换为半轴距
    , wheel_radius_(wheel_radius)
    , install_angle_rad_(install_angle * M_PI / 180.0)
{
    // 计算底盘半径（中心到轮子的距离）
    chassis_radius_ = std::sqrt(wheel_base_x_ * wheel_base_x_ + wheel_base_y_ * wheel_base_y_);
}

std::array<double, 4> OmniWheelKinematics::inverseKinematics(
    double vx, double vy, double wz) const
{
    std::array<double, 4> wheel_velocities;
    
    // X 形布置全向轮逆运动学公式（45° 安装角）
    // 每个轮子只能在其安装方向上提供驱动力
    // 
    // 对于以 θ 角度安装的轮子，其速度为：
    // v_wheel = vx * cos(θ) + vy * sin(θ) + wz * L
    // 其中 L 是底盘中心到轮子的距离
    //
    // 坐标系定义：X轴向右，Y轴向前，逆时针为正
    
    // 计算各轮子的角速度分量（线速度贡献）
    double angular_component = wz * chassis_radius_;
    
    // 使用 sqrt(2)/2 = 0.707... 作为 cos(45°) 和 sin(45°)
    const double cos45 = std::cos(install_angle_rad_);  // cos(45°) ≈ 0.707
    const double sin45 = std::sin(install_angle_rad_);  // sin(45°) ≈ 0.707
    
    // 轮 1 (左前, FL): 安装角 45°
    // v1 = vx * cos(45°) + vy * sin(45°) + wz * L
    wheel_velocities[0] = vx * cos45 + vy * sin45 + angular_component;
    
    // 轮 2 (右前, FR): 安装角 135°
    // v2 = vx * cos(135°) + vy * sin(135°) + wz * L
    // cos(135°) = -cos(45°), sin(135°) = sin(45°)
    wheel_velocities[1] = -vx * cos45 + vy * sin45 + angular_component;
    
    // 轮 3 (左后, RL): 安装角 225° (或 -135°)
    // v3 = vx * cos(225°) + vy * sin(225°) + wz * L
    // cos(225°) = -cos(45°), sin(225°) = -sin(45°)
    wheel_velocities[2] = -vx * cos45 - vy * sin45 + angular_component;
    
    // 轮 4 (右后, RR): 安装角 315° (或 -45°)
    // v4 = vx * cos(315°) + vy * sin(315°) + wz * L
    // cos(315°) = cos(45°), sin(315°) = -sin(45°)
    wheel_velocities[3] = vx * cos45 - vy * sin45 + angular_component;
    
    return wheel_velocities;
}

void OmniWheelKinematics::forwardKinematics(
    const std::array<double, 4>& wheel_velocities,
    double& vx, double& vy, double& wz) const
{
    // X 形布置全向轮正运动学公式（从轮子速度推算底盘速度）
    // 
    // 雅可比矩阵的伪逆求解：
    // [vx]   [  cos(45°)   -cos(45°)  -cos(45°)   cos(45°) ]   [v1]
    // [vy] = [  sin(45°)    sin(45°)  -sin(45°)  -sin(45°) ] * [v2]
    // [wz]   [    1/L          1/L         1/L         1/L   ]   [v3]
    //                                                            [v4]
    // 
    // 简化后（考虑到对称性和最小二乘解）：
    
    double v1 = wheel_velocities[0];  // FL
    double v2 = wheel_velocities[1];  // FR
    double v3 = wheel_velocities[2];  // RL
    double v4 = wheel_velocities[3];  // RR
    
    const double cos45 = std::cos(install_angle_rad_);  // ≈ 0.707
    const double sin45 = std::sin(install_angle_rad_);  // ≈ 0.707
    
    // vx 分量：从各轮子的 x 方向投影求平均
    vx = (v1 * cos45 - v2 * cos45 - v3 * cos45 + v4 * cos45) / 4.0;
    
    // vy 分量：从各轮子的 y 方向投影求平均
    vy = (v1 * sin45 + v2 * sin45 - v3 * sin45 - v4 * sin45) / 4.0;
    
    // wz 角速度：从所有轮子的切向速度求平均
    wz = (v1 + v2 + v3 + v4) / (4.0 * chassis_radius_);
}

double OmniWheelKinematics::velocityToRPM(double linear_velocity) const
{
    // 线速度 (m/s) → 轮子角速度 (rad/s) → 电机角速度 (rad/s) → RPM
    double wheel_angular_velocity = linear_velocity / wheel_radius_;  // rad/s
    double motor_angular_velocity = wheel_angular_velocity * GEAR_RATIO;  // 考虑减速比
    double rpm = motor_angular_velocity * 60.0 / (2.0 * M_PI);  // 转换为 RPM
    return rpm;
}

double OmniWheelKinematics::rpmToVelocity(double rpm) const
{
    // RPM → 电机角速度 (rad/s) → 轮子角速度 (rad/s) → 线速度 (m/s)
    double motor_angular_velocity = rpm * 2.0 * M_PI / 60.0;  // rad/s
    double wheel_angular_velocity = motor_angular_velocity / GEAR_RATIO;  // 考虑减速比
    double linear_velocity = wheel_angular_velocity * wheel_radius_;  // m/s
    return linear_velocity;
}

} // namespace motor_control
