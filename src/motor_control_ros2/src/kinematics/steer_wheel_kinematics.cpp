#include "motor_control_ros2/steer_wheel_kinematics.hpp"
#include <cmath>
#include <algorithm>

namespace motor_control {

SteerWheelKinematics::SteerWheelKinematics(
    double wheel_base_x, 
    double wheel_base_y, 
    double wheel_radius)
    : wheel_base_x_(wheel_base_x),
      wheel_base_y_(wheel_base_y),
      wheel_radius_(wheel_radius),
      lx_(wheel_base_x / 2.0),
      ly_(wheel_base_y / 2.0)
{
}

void SteerWheelKinematics::inverseKinematics(
    double vx, double vy, double wz,
    WheelCommand& fl, WheelCommand& fr,
    WheelCommand& rl, WheelCommand& rr)
{
    // 底盘坐标系定义（ROS标准）：
    // X轴：前方为正
    // Y轴：左方为正
    // Z轴：上方为正（右手坐标系）
    // wz正：逆时针旋转（从上往下看）
    
    // 轮子位置（从底盘中心看）：
    // FL: (+lx_, +ly_)  前左
    // FR: (+lx_, -ly_)  前右
    // RL: (-lx_, +ly_)  后左
    // RR: (-lx_, -ly_)  后右
    
    // 每个轮子的速度 = 底盘平移速度 + 旋转引起的速度
    // 对于旋转，速度向量垂直于从中心到轮子的向量
    // v_rot = omega × r，其中 r 是轮子位置向量
    // 对于绕Z轴旋转: vx_rot = -wz * y, vy_rot = wz * x
    
    // 左前轮 (FL) 位置: (+lx_, +ly_)
    // 旋转速度: vx_rot = -wz * ly_, vy_rot = wz * lx_
    double vx_fl = vx - wz * ly_;
    double vy_fl = vy + wz * lx_;
    fl.velocity = std::sqrt(vx_fl * vx_fl + vy_fl * vy_fl);
    fl.angle = std::atan2(vy_fl, vx_fl) * 180.0 / M_PI;
    fl.angle = normalizeAngle(fl.angle);
    
    // 右前轮 (FR) 位置: (+lx_, -ly_)
    // 旋转速度: vx_rot = -wz * (-ly_) = +wz * ly_, vy_rot = wz * lx_
    double vx_fr = vx + wz * ly_;
    double vy_fr = vy + wz * lx_;
    fr.velocity = std::sqrt(vx_fr * vx_fr + vy_fr * vy_fr);
    fr.angle = std::atan2(vy_fr, vx_fr) * 180.0 / M_PI;
    fr.angle = normalizeAngle(fr.angle);
    
    // 左后轮 (RL) 位置: (-lx_, +ly_)
    // 旋转速度: vx_rot = -wz * ly_, vy_rot = wz * (-lx_) = -wz * lx_
    double vx_rl = vx - wz * ly_;
    double vy_rl = vy - wz * lx_;
    rl.velocity = std::sqrt(vx_rl * vx_rl + vy_rl * vy_rl);
    rl.angle = std::atan2(vy_rl, vx_rl) * 180.0 / M_PI;
    rl.angle = normalizeAngle(rl.angle);
    
    // 右后轮 (RR) 位置: (-lx_, -ly_)
    // 旋转速度: vx_rot = -wz * (-ly_) = +wz * ly_, vy_rot = wz * (-lx_) = -wz * lx_
    double vx_rr = vx + wz * ly_;
    double vy_rr = vy - wz * lx_;
    rr.velocity = std::sqrt(vx_rr * vx_rr + vy_rr * vy_rr);
    rr.angle = std::atan2(vy_rr, vx_rr) * 180.0 / M_PI;
    rr.angle = normalizeAngle(rr.angle);
}

void SteerWheelKinematics::forwardKinematics(
    const WheelCommand& fl, const WheelCommand& fr,
    const WheelCommand& rl, const WheelCommand& rr,
    double& vx, double& vy, double& wz)
{
    // 将轮子速度向量转换为底盘坐标系
    double fl_angle_rad = fl.angle * M_PI / 180.0;
    double fr_angle_rad = fr.angle * M_PI / 180.0;
    double rl_angle_rad = rl.angle * M_PI / 180.0;
    double rr_angle_rad = rr.angle * M_PI / 180.0;
    
    double vx_fl = fl.velocity * std::cos(fl_angle_rad);
    double vy_fl = fl.velocity * std::sin(fl_angle_rad);
    
    double vx_fr = fr.velocity * std::cos(fr_angle_rad);
    double vy_fr = fr.velocity * std::sin(fr_angle_rad);
    
    double vx_rl = rl.velocity * std::cos(rl_angle_rad);
    double vy_rl = rl.velocity * std::sin(rl_angle_rad);
    
    double vx_rr = rr.velocity * std::cos(rr_angle_rad);
    double vy_rr = rr.velocity * std::sin(rr_angle_rad);
    
    // 使用最小二乘法求解底盘速度
    // 简化方法：取 4 个轮子的平均值
    vx = (vx_fl + vx_fr + vx_rl + vx_rr) / 4.0;
    vy = (vy_fl + vy_fr + vy_rl + vy_rr) / 4.0;
    
    // 计算角速度（从轮子速度差）
    // wz = (vy_right - vy_left) / wheel_base_y
    double vy_left = (vy_fl + vy_rl) / 2.0;
    double vy_right = (vy_fr + vy_rr) / 2.0;
    wz = (vy_right - vy_left) / wheel_base_y_;
}

double SteerWheelKinematics::optimizeSteerAngle(
    double current_angle,
    double target_angle,
    double& velocity)
{
    // 归一化角度到 [0, 360)
    current_angle = normalizeAngle(current_angle);
    target_angle = normalizeAngle(target_angle);
    
    // 计算角度差
    double diff = angleDifference(target_angle, current_angle);
    
    // 如果角度差超过 90°，反转速度并调整角度
    if (std::abs(diff) > 90.0) {
        velocity = -velocity;
        target_angle = normalizeAngle(target_angle + 180.0);
    }
    
    return target_angle;
}

double SteerWheelKinematics::normalizeAngle(double angle) {
    angle = std::fmod(angle, 360.0);
    if (angle < 0.0) {
        angle += 360.0;
    }
    return angle;
}

double SteerWheelKinematics::angleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    
    // 归一化到 (-180, 180]
    while (diff > 180.0) {
        diff -= 360.0;
    }
    while (diff <= -180.0) {
        diff += 360.0;
    }
    
    return diff;
}

} // namespace motor_control
