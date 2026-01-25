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
    
    // X 形布置全向轮逆运动学公式
    // 轮子速度 = vx * cos(θ) + vy * sin(θ) + wz * L
    // 其中 L 是底盘中心到轮子的距离，θ 是轮子的安装角度（轮子能转动的方向）
    //
    // 坐标系定义：X轴向右，Y轴向前，逆时针为正
    // 
    // X 形4轮布置（从车前看）：
    //     FL(45°)  ----  FR(-45°)
    //       |                |
    //       |    车辆        |
    //       |                |
    //     RL(-135°) ---- RR(135°)
    //
    // 安装角度（轮子能转动的方向）：
    // FL: 45° (向右上方)
    // FR: -45° (向左上方)  
    // RL: -135° (向左下方)
    // RR: 135° (向右下方)
    
    double angular_component = wz * chassis_radius_;
    
    // 左前轮 (FL): 安装角 45°
    double angle_fl = install_angle_rad_;  // 45°
    wheel_velocities[0] = vx * std::cos(angle_fl) + vy * std::sin(angle_fl) + angular_component;
    
    // 右前轮 (FR): 安装角 -45°
    double angle_fr = -install_angle_rad_;  // -45°
    wheel_velocities[1] = vx * std::cos(angle_fr) + vy * std::sin(angle_fr) + angular_component;
    
    // 左后轮 (RL): 安装角 -135° (= 180° - 45°)
    double angle_rl = M_PI - install_angle_rad_;  // -135°
    wheel_velocities[2] = vx * std::cos(angle_rl) + vy * std::sin(angle_rl) + angular_component;
    
    // 右后轮 (RR): 安装角 135° (= 180° + 45°)
    double angle_rr = M_PI + install_angle_rad_;  // 135°
    wheel_velocities[3] = vx * std::cos(angle_rr) + vy * std::sin(angle_rr) + angular_component;
    
    return wheel_velocities;
}

void OmniWheelKinematics::forwardKinematics(
    const std::array<double, 4>& wheel_velocities,
    double& vx, double& vy, double& wz) const
{
    // X 形布置全向轮正运动学公式（从轮子速度推算底盘速度）
    // 使用雅可比矩阵逆：
    // [vx]   [cos(45°)   cos(-45°)  cos(-135°)  cos(135°) ]^-1   [v1]
    // [vy] = [sin(45°)   sin(-45°)  sin(-135°)  sin(135°) ]   * [v2]
    // [wz]   [1/L         1/L         1/L         1/L      ]      [v3]
    //                                                               [v4]
    
    double v1 = wheel_velocities[0];  // FL
    double v2 = wheel_velocities[1];  // FR
    double v3 = wheel_velocities[2];  // RL
    double v4 = wheel_velocities[3];  // RR
    
    double angle_fl = install_angle_rad_;      // 45°
    double angle_fr = -install_angle_rad_;     // -45°
    double angle_rl = M_PI - install_angle_rad_;  // -135°
    double angle_rr = M_PI + install_angle_rad_;  // 135°
    
    // 雅可比矩阵的逆（通过求解最小二乘问题）
    // 对于对称的X型4轮配置，可以解析求解：
    double cos_fl = std::cos(angle_fl);
    double sin_fl = std::sin(angle_fl);
    double cos_fr = std::cos(angle_fr);
    double sin_fr = std::sin(angle_fr);
    double cos_rl = std::cos(angle_rl);
    double sin_rl = std::sin(angle_rl);
    double cos_rr = std::cos(angle_rr);
    double sin_rr = std::sin(angle_rr);
    
    // vx 分量：从各轮子的 x 方向投影求平均
    vx = (v1 * cos_fl + v2 * cos_fr + v3 * cos_rl + v4 * cos_rr) / 4.0;
    
    // vy 分量：从各轮子的 y 方向投影求平均
    vy = (v1 * sin_fl + v2 * sin_fr + v3 * sin_rl + v4 * sin_rr) / 4.0;
    
    // wz 角速度：从所有轮子的贡献求平均
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
