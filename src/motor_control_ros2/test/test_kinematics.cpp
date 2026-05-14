#include "motor_control_ros2/parallel_arm_kinematics.hpp"
#include <cstdio>
#include <cmath>

using motor_control::ParallelArmKinematics;

int main() {
    ParallelArmKinematics kin;
    
    printf("===== 四连杆可达范围扫描 =====\n");
    printf("参数: L1=200mm, r=30mm, l=200mm, d=20mm\n");
    printf("曲柄零点=大臂坐标系中垂直向下(-Y_local), 逆时针为正\n\n");
    
    for (int deg = 0; deg <= 180; deg += 5) {
        double theta2 = deg * M_PI / 180.0;
        auto q2 = kin.fourBarForward(theta2);
        if (q2) {
            printf("  theta2=%3d° -> q2=%7.2f°\n", deg, *q2 * 180.0 / M_PI);
        }
    }
    
    printf("\n===== 四连杆正逆解一致性（可达范围内）=====\n");
    bool ok = true;
    for (int deg = 55; deg <= 130; deg += 5) {
        double theta2 = deg * M_PI / 180.0;
        auto q2 = kin.fourBarForward(theta2);
        if (!q2) continue;
        auto theta2_back = kin.fourBarInverse(*q2);
        if (!theta2_back) {
            printf("theta2=%d° -> q2=%.2f° -> 逆解无解!\n", deg, *q2 * 180/M_PI);
            ok = false;
            continue;
        }
        double err = std::abs(*theta2_back - theta2);
        if (err > 1e-6) {
            printf("theta2=%d° -> q2=%.2f° -> back=%.2f° err=%.6f° FAIL\n",
                deg, *q2*180/M_PI, *theta2_back*180/M_PI, err*180/M_PI);
            ok = false;
        }
    }
    printf("四连杆正逆解一致性: %s\n", ok ? "PASS" : "FAIL");
    
    printf("\n===== 单臂FK/IK一致性（可达范围内）=====\n");
    bool arm_ok = true;
    struct { double q1_deg, theta2_deg; } cases[] = {
        {30, 65}, {45, 70}, {60, 60}, {75, 80}, {90, 75}, {100, 90}, {110, 80}
    };
    for (auto& tc : cases) {
        double q1 = tc.q1_deg * M_PI / 180.0;
        double theta2 = tc.theta2_deg * M_PI / 180.0;
        
        auto end = kin.singleArmFK(q1, theta2);
        if (!end) {
            printf("q1=%.0f° θ2=%.0f° -> FK无解\n", tc.q1_deg, tc.theta2_deg);
            arm_ok = false;
            continue;
        }
        auto sol = kin.singleArmIK(end->x, end->y);
        if (!sol) {
            printf("q1=%.0f° θ2=%.0f° -> (%.4f,%.4f) -> IK无解\n",
                tc.q1_deg, tc.theta2_deg, end->x, end->y);
            arm_ok = false;
            continue;
        }
        double q1_err = std::abs(sol->q1 - q1);
        double t2_err = std::abs(sol->theta2 - theta2);
        printf("q1=%3.0f° θ2=%3.0f° -> (%.4f, %.4f) -> q1=%.2f° θ2=%.2f° | err: %.6f° %.6f°",
            tc.q1_deg, tc.theta2_deg, end->x, end->y,
            sol->q1*180/M_PI, sol->theta2*180/M_PI, q1_err*180/M_PI, t2_err*180/M_PI);
        if (q1_err > 0.01 || t2_err > 0.01) {
            printf(" FAIL");
            arm_ok = false;
        }
        printf("\n");
    }
    printf("单臂FK/IK一致性: %s\n", arm_ok ? "PASS" : "FAIL");
    
    printf("\n===== 双臂并联FK/IK一致性 =====\n");
    bool par_ok = true;
    struct { double H; double phi_deg; } pcases[] = {
        {-0.15, 0}, {-0.20, 5}, {-0.25, -5}, {-0.10, 10}
    };
    for (auto& tc : pcases) {
        double phi = tc.phi_deg * M_PI / 180.0;
        auto sol = kin.parallelIK(tc.H, phi);
        if (!sol) {
            printf("H=%.3f phi=%.1f° -> IK无解\n", tc.H, tc.phi_deg);
            continue;
        }
        auto result = kin.parallelFK(sol->left.q1, sol->left.theta2,
                                      sol->right.q1, sol->right.theta2);
        if (!result) {
            printf("H=%.3f phi=%.1f° -> IK有解 but FK无解\n", tc.H, tc.phi_deg);
            par_ok = false;
            continue;
        }
        double H_err = std::abs((*result)[0] - tc.H);
        double phi_err = std::abs((*result)[1] - phi);
        printf("H=%.3f φ=%5.1f° -> L(q1=%5.1f°,θ2=%5.1f°) R(q1=%5.1f°,θ2=%5.1f°) -> H=%.4f φ=%.2f° err: %.6f %.6f°",
            tc.H, tc.phi_deg,
            sol->left.q1*180/M_PI, sol->left.theta2*180/M_PI,
            sol->right.q1*180/M_PI, sol->right.theta2*180/M_PI,
            (*result)[0], (*result)[1]*180/M_PI, H_err, phi_err*180/M_PI);
        if (H_err > 0.001 || phi_err > 0.001) {
            printf(" FAIL");
            par_ok = false;
        }
        printf("\n");
    }
    printf("双臂并联FK/IK一致性: %s\n", par_ok ? "PASS" : "FAIL");
    
    printf("\n===== 五次多项式轨迹 =====\n");
    auto traj = ParallelArmKinematics::quinticTrajectory(0,0,0, 1,0,0, 1.0, 0.001);
    auto& f = traj.front(); auto& b = traj.back();
    bool tok = std::abs(f.position)<1e-6 && std::abs(b.position-1.0)<1e-6
            && std::abs(f.velocity)<1e-6 && std::abs(b.velocity)<1e-6;
    printf("轨迹边界条件: %s\n", tok ? "PASS" : "FAIL");
    
    return 0;
}
