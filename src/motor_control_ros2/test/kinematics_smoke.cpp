#include "motor_control_ros2/steer_wheel_kinematics.hpp"

#include <cassert>
#include <cmath>

using motor_control::SteerWheelKinematics;
using motor_control::WheelCommand;

namespace {
bool close(double lhs, double rhs, double tolerance = 1e-9) {
  return std::abs(lhs - rhs) <= tolerance;
}
}

int main() {
  SteerWheelKinematics kinematics(0.65, 0.62, 0.055);
  WheelCommand fl, fr, rl, rr;
  const double expected_vx = 0.8;
  const double expected_vy = -0.35;
  const double expected_wz = 1.2;
  kinematics.inverseKinematics(expected_vx, expected_vy, expected_wz, fl, fr, rl, rr);

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  kinematics.forwardKinematics(fl, fr, rl, rr, vx, vy, wz);
  assert(close(vx, expected_vx));
  assert(close(vy, expected_vy));
  assert(close(wz, expected_wz));

  double velocity = 1.0;
  const double target = SteerWheelKinematics::optimizeSteerAngle(0.0, 170.0, velocity);
  assert(close(target, 350.0));
  assert(close(velocity, -1.0));
  return 0;
}
