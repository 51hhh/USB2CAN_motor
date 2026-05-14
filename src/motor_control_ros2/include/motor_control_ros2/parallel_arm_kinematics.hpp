#ifndef MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_
#define MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace motor_control {

/**
 * @brief 平面 RR 串联机械臂运动学解算器（双臂版本）
 *
 * 机构描述：
 *   平面两自由度串联机构，所有运动被约束在同一竖直平面内。
 *   整体结构模拟人体手臂的大臂与小臂，两个驱动电机共轴安装于肩关节。
 *
 *   大臂电机定子固定于基座，转子直接与大臂固连；
 *   小臂电机外壳固定在大臂电机转子上，随大臂整体摆动；
 *   小臂电机转子通过平行四边形四连杆机构驱动小臂绕肘关节转动，
 *   保证小臂电机转角与小臂相对大臂转角呈 1:1 线性关系（运动学解耦）。
 *
 *   等效于标准平面 RR 串联机械臂。
 *
 * 角度定义（全局坐标系：肩关节中心为原点，X 水平向右，Y 垂直向上）：
 *   θ1     = 大臂绝对角度（从 +X 轴逆时针为正）
 *   θ2_rel = 小臂相对大臂的转角（小臂电机经零位偏移后即为 θ2_rel）
 *
 * 电机零位：机械臂完全收紧时电机所在位置
 *   θ1 范围：0° ~ 110°
 *   θ2_rel 范围：0° ~ 80°
 *
 * 电机角度与运动学角度的映射（FK/IK 入口和出口均使用电机角度）：
 *   θ1_kin     = θ1_motor + theta1_offset        （offset = -90°，零位大臂垂直向下）
 *   θ2_rel_kin = θ2_motor + theta2_rel_offset     （offset = +70°，零位小臂相对大臂70°）
 *
 * 正运动学：
 *   x = L1·cos(θ1) + L2·cos(θ1 + θ2_rel)
 *   y = L1·sin(θ1) + L2·sin(θ1 + θ2_rel)
 *
 * 双臂系统：左右各一条独立臂，末端刚性连接击球板。
 *   肩关节中心距 = D，板中心距两肩对称。
 */
class ParallelArmKinematics {
public:
    // ===================== 机构几何常量 =====================
    struct Params {
        double L1 = 0.200;          // 大臂长度 (m)：肩关节到肘关节
        double L2 = 0.221;          // 小臂长度 (m)：肘关节到末端
        double D  = 0.3506;         // 肩关节中心距 (m)

        // 电机零位偏移 (rad)：kinematic_angle = motor_angle + offset
        // 零位 = 机械臂收紧（大臂垂直向下，小臂相对大臂 70°）
        double theta1_offset     = -M_PI / 2.0;            // 大臂：motor=0 → kin=-90°
        double theta2_rel_offset = 70.0 * M_PI / 180.0;    // 小臂：motor=0 → kin=70°

        // 单臂电机限位 (rad)  — 零位 = 机械臂收紧
        double theta1_min     = 0.0;
        double theta1_max     = 110.0 * M_PI / 180.0;   // 110°
        double theta2_rel_min = 0.0;
        double theta2_rel_max = 80.0 * M_PI / 180.0;     // 80°
    };

    // ===================== 结果类型 =====================
    struct EndPoint {
        double x;
        double y;
    };

    struct SingleArmSolution {
        double theta1;      // 大臂绝对角 (rad)
        double theta2_rel;  // 小臂相对角 (rad)
    };

    struct DualArmSolution {
        SingleArmSolution left;
        SingleArmSolution right;
    };

    struct TrajectoryPoint {
        double time;
        double position;
        double velocity;
        double acceleration;
    };

    ParallelArmKinematics();
    explicit ParallelArmKinematics(const Params& params);

    // ===================== 角度转换 =====================
    /// 电机角度 → 运动学角度
    double motorToKinTheta1(double motor_theta1) const;
    double motorToKinTheta2(double motor_theta2) const;
    /// 运动学角度 → 电机角度
    double kinToMotorTheta1(double kin_theta1) const;
    double kinToMotorTheta2(double kin_theta2) const;

    // ===================== 单臂运动学 =====================

    /**
     * @brief 单臂正运动学：(θ1, θ2_rel) → 末端坐标
     *        坐标系以该臂肩关节为原点
     */
    EndPoint singleArmFK(double theta1, double theta2_rel) const;

    /**
     * @brief 单臂逆运动学：末端 (x, y) → (θ1, θ2_rel)
     *        坐标系以该臂肩关节为原点
     * @param elbow_up true=肘上解（β取正），false=肘下解
     * @return 解算结果，不可达时返回 std::nullopt
     */
    std::optional<SingleArmSolution> singleArmIK(double x, double y,
                                                  bool elbow_up = true) const;

    // ===================== 单臂雅可比 =====================

    /**
     * @brief 单臂解析雅可比矩阵 J = ∂(x,y)/∂(θ1,θ2_rel)
     *        返回 [J11, J12, J21, J22]
     */
    std::array<double, 4> singleArmJacobian(double theta1, double theta2_rel) const;

    /**
     * @brief 单臂逆速度映射：末端速度 (dx, dy) → 关节速度 (dθ1, dθ2_rel)
     * @return 关节速度，奇异时返回 std::nullopt
     */
    std::optional<std::array<double, 2>> singleArmInverseVelocity(
        double theta1, double theta2_rel,
        double dx, double dy) const;

    /**
     * @brief 单臂逆力映射：末端力 (Fx, Fy) → 关节力矩 (τ1, τ2)
     */
    std::array<double, 2> singleArmInverseTorque(
        double theta1, double theta2_rel,
        double Fx, double Fy) const;

    // ===================== 双臂并联 =====================

    /**
     * @brief 并联正运动学：四个电机角 → 击球板中心高度 H 和俯仰角 φ
     *        左臂肩在 (-D/2, 0)，右臂肩在 (+D/2, 0)，右臂镜像安装
     * @return {H, phi}，无解时返回 std::nullopt
     */
    std::optional<std::array<double, 2>> parallelFK(
        double left_theta1, double left_theta2_rel,
        double right_theta1, double right_theta2_rel) const;

    /**
     * @brief 并联逆运动学：(H, φ) → 四个电机角
     * @param H   击球板中心高度 (m)，相对肩关节
     * @param phi 俯仰角 (rad)，左高右低为正
     * @return 四个电机角，无解时返回 std::nullopt
     */
    std::optional<DualArmSolution> parallelIK(double H, double phi) const;

    // ===================== 轨迹规划 =====================

    /**
     * @brief 五次多项式轨迹规划（位置+速度+加速度边界条件）
     */
    static std::vector<TrajectoryPoint> quinticTrajectory(
        double p0, double v0, double a0,
        double pf, double vf, double af,
        double T, double dt);

    // ===================== 限位检查 =====================
    bool checkLimits(const SingleArmSolution& sol) const;
    bool checkLimits(const DualArmSolution& sol) const;

    const Params& params() const { return params_; }

private:
    Params params_;
    double half_D_;  // D/2
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__PARALLEL_ARM_KINEMATICS_HPP_
