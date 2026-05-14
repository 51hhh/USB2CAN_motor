#include "motor_control_ros2/parallel_arm_kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control {

ParallelArmKinematics::ParallelArmKinematics()
    : params_(), half_D_(params_.D / 2.0) {}

ParallelArmKinematics::ParallelArmKinematics(const Params& params)
    : params_(params), half_D_(params.D / 2.0) {}

// ======================== 角度转换 ========================

double ParallelArmKinematics::motorToKinTheta1(double motor_theta1) const {
    return motor_theta1 + params_.theta1_offset;
}
double ParallelArmKinematics::motorToKinTheta2(double motor_theta2) const {
    return motor_theta2 + params_.theta2_rel_offset;
}
double ParallelArmKinematics::kinToMotorTheta1(double kin_theta1) const {
    return kin_theta1 - params_.theta1_offset;
}
double ParallelArmKinematics::kinToMotorTheta2(double kin_theta2) const {
    return kin_theta2 - params_.theta2_rel_offset;
}

// ======================== 单臂正运动学 ========================

ParallelArmKinematics::EndPoint
ParallelArmKinematics::singleArmFK(double theta1, double theta2_rel) const {
    /*
     * 输入：电机角度（motor space）
     * 内部转换为运动学角度后用标准 RR 公式
     */
    double t1 = motorToKinTheta1(theta1);
    double t2 = motorToKinTheta2(theta2_rel);
    double L1 = params_.L1;
    double L2 = params_.L2;
    double abs_angle = t1 + t2;

    double x = L1 * std::cos(t1) + L2 * std::cos(abs_angle);
    double y = L1 * std::sin(t1) + L2 * std::sin(abs_angle);

    return EndPoint{x, y};
}

// ======================== 单臂逆运动学 ========================

std::optional<ParallelArmKinematics::SingleArmSolution>
ParallelArmKinematics::singleArmIK(double x, double y, bool elbow_up) const {
    /*
     * 标准 RR 逆运动学求运动学角度，然后转回电机角度输出。
     */
    double L1 = params_.L1;
    double L2 = params_.L2;

    double d2 = x * x + y * y;
    double d = std::sqrt(d2);

    if (d > (L1 + L2) || d < std::abs(L1 - L2)) {
        return std::nullopt;
    }

    double cos_theta2 = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);

    double theta2_kin = elbow_up ? std::acos(cos_theta2) : -std::acos(cos_theta2);

    double alpha = std::atan2(y, x);
    double beta = std::atan2(L2 * std::sin(theta2_kin), L1 + L2 * std::cos(theta2_kin));
    double theta1_kin = alpha - beta;

    // 转回电机角度
    double theta1_motor = kinToMotorTheta1(theta1_kin);
    double theta2_motor = kinToMotorTheta2(theta2_kin);

    return SingleArmSolution{theta1_motor, theta2_motor};
}

// ======================== 单臂雅可比 ========================

std::array<double, 4>
ParallelArmKinematics::singleArmJacobian(double theta1, double theta2_rel) const {
    /*
     * 输入：电机角度，内部转换为运动学角度计算 Jacobian
     * Jacobian 不受偏移影响（偏移是常量，导数为零）
     */
    double t1 = motorToKinTheta1(theta1);
    double t2 = motorToKinTheta2(theta2_rel);
    double L1 = params_.L1;
    double L2 = params_.L2;
    double s1 = std::sin(t1);
    double c1 = std::cos(t1);
    double s12 = std::sin(t1 + t2);
    double c12 = std::cos(t1 + t2);

    return {
        -L1 * s1 - L2 * s12,   // J11 = ∂x/∂θ1
        -L2 * s12,              // J12 = ∂x/∂θ2_rel
         L1 * c1 + L2 * c12,   // J21 = ∂y/∂θ1
         L2 * c12              // J22 = ∂y/∂θ2_rel
    };
}

// ======================== 单臂逆速度 ========================

std::optional<std::array<double, 2>>
ParallelArmKinematics::singleArmInverseVelocity(
    double theta1, double theta2_rel,
    double dx, double dy) const {
    /*
     * dq = J^{-1} · dp
     *
     * det(J) = L1·L2·sin(θ2_rel)
     *
     * J^{-1} = (1/det) · [  J22  -J12 ]
     *                     [ -J21   J11 ]
     */
    auto J = singleArmJacobian(theta1, theta2_rel);
    double det = J[0] * J[3] - J[1] * J[2];  // J11*J22 - J12*J21

    if (std::abs(det) < 1e-10) {
        return std::nullopt;  // 奇异构型（θ2_rel ≈ 0 或 π）
    }

    double inv_det = 1.0 / det;
    return std::array<double, 2>{
        inv_det * ( J[3] * dx - J[1] * dy),  // dθ1
        inv_det * (-J[2] * dx + J[0] * dy)   // dθ2_rel
    };
}

// ======================== 单臂逆力矩 ========================

std::array<double, 2>
ParallelArmKinematics::singleArmInverseTorque(
    double theta1, double theta2_rel,
    double Fx, double Fy) const {
    /*
     * τ = J^T · F（虚功原理）
     */
    auto J = singleArmJacobian(theta1, theta2_rel);
    return {
        J[0] * Fx + J[2] * Fy,  // τ1 = J11·Fx + J21·Fy
        J[1] * Fx + J[3] * Fy   // τ2 = J12·Fx + J22·Fy
    };
}

// ======================== 双臂并联正运动学 ========================

std::optional<std::array<double, 2>> ParallelArmKinematics::parallelFK(
    double left_theta1, double left_theta2_rel,
    double right_theta1, double right_theta2_rel) const {
    /*
     * 左臂肩在 (-D/2, 0)，正常安装（X正向右）
     * 右臂肩在 (+D/2, 0)，镜像安装（X正向左，即右臂局部x = -(全局x - D/2)）
     *
     * 左臂末端全局：(x_L - D/2,  y_L)
     * 右臂末端全局：(D/2 - x_R,  y_R)  ← x 翻转
     *
     * H = (P_Ly + P_Ry) / 2
     * φ = atan2(P_Ly - P_Ry, P_Rx - P_Lx)  左高右低为正
     */
    auto left_end  = singleArmFK(left_theta1, left_theta2_rel);
    auto right_end = singleArmFK(right_theta1, right_theta2_rel);

    double Lx = left_end.x - half_D_;
    double Ly = left_end.y;
    double Rx = half_D_ - right_end.x;  // 镜像
    double Ry = right_end.y;

    double H = (Ly + Ry) / 2.0;
    double dx = Rx - Lx;
    double dy = Ry - Ly;
    double phi = std::atan2(-dy, dx);  // 左高右低为正

    return std::array<double, 2>{H, phi};
}

// ======================== 双臂并联逆运动学 ========================

std::optional<ParallelArmKinematics::DualArmSolution>
ParallelArmKinematics::parallelIK(double H, double phi) const {
    /*
     * 板中心在 (0, H)，板长 D。
     * 板方向向量 (cos(φ), -sin(φ))。
     *
     * P_L = (-D/2·cos(φ),  H + D/2·sin(φ))
     * P_R = ( D/2·cos(φ),  H - D/2·sin(φ))
     *
     * 左臂局部：x = P_Lx + D/2,  y = P_Ly
     * 右臂局部（镜像）：x = D/2 - P_Rx,  y = P_Ry
     */
    double a = half_D_;
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);

    double PLx_local = -a * cos_phi + a;   // = a*(1 - cos(φ))
    double PLy_local = H + a * sin_phi;

    double PRx_local = a - a * cos_phi;    // = a*(1 - cos(φ))
    double PRy_local = H - a * sin_phi;

    auto left_sol  = singleArmIK(PLx_local, PLy_local);
    auto right_sol = singleArmIK(PRx_local, PRy_local);

    if (!left_sol || !right_sol) {
        return std::nullopt;
    }

    DualArmSolution sol{*left_sol, *right_sol};
    if (!checkLimits(sol)) {
        return std::nullopt;
    }

    return sol;
}

// ======================== 轨迹规划 ========================

std::vector<ParallelArmKinematics::TrajectoryPoint>
ParallelArmKinematics::quinticTrajectory(
    double p0, double v0, double a0,
    double pf, double vf, double af,
    double T, double dt) {

    if (T <= 0.0 || dt <= 0.0) {
        return {};
    }

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    double c0 = p0;
    double c1 = v0;
    double c2 = a0 / 2.0;

    double dp = pf - p0 - v0 * T - (a0 / 2.0) * T2;
    double dv = vf - v0 - a0 * T;
    double da = af - a0;

    double c3 = (20.0 * dp - 8.0 * T * dv + T2 * da) / (2.0 * T3);
    double c4 = (-30.0 * dp + 14.0 * T * dv - 2.0 * T2 * da) / (2.0 * T4);
    double c5 = (12.0 * dp - 6.0 * T * dv + T2 * da) / (2.0 * T5);

    std::vector<TrajectoryPoint> trajectory;
    size_t n = static_cast<size_t>(T / dt) + 1;
    trajectory.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        double t = i * dt;
        if (t > T) t = T;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        TrajectoryPoint pt;
        pt.time = t;
        pt.position     = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
        pt.velocity     = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
        pt.acceleration = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;

        trajectory.push_back(pt);
    }

    return trajectory;
}

// ======================== 限位检查 ========================

bool ParallelArmKinematics::checkLimits(const SingleArmSolution& sol) const {
    return sol.theta1 >= params_.theta1_min && sol.theta1 <= params_.theta1_max &&
           sol.theta2_rel >= params_.theta2_rel_min && sol.theta2_rel <= params_.theta2_rel_max;
}

bool ParallelArmKinematics::checkLimits(const DualArmSolution& sol) const {
    return checkLimits(sol.left) && checkLimits(sol.right);
}

}  // namespace motor_control
