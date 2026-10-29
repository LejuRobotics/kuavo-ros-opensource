#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Core>

#include "humanoid_wheel_interface/motion_planner/TorsoRotationTarget.h"

namespace {

const Eigen::Vector4d kLower(0.0, -2.81, -0.157, -3.14);
const Eigen::Vector4d kUpper(1.45, 0.0, 2.985, 3.14);

bool check(bool ok, const char* message) {
  if (!ok) {
    std::cerr << message << '\n';
  }
  return ok;
}

bool near(double actual, double expected, double tol = 1e-12) {
  return std::abs(actual - expected) <= tol;
}

bool testForwardLimitThenReverseKeepsKneeAndLegSnapshot() {
  bool ok = true;
  Eigen::Vector4d q(0.12, -0.20, 0.08, 0.0);
  for (int cycle = 0; cycle < 4; ++cycle) {
    for (int i = 0; i < 500; ++i) {
      q = ocs2::integrateTorsoRotationTarget(q, 1.5, 1.5, 0.02, kLower, kUpper);
      ok &= check(q[0] == 0.12, "knee snapshot must stay fixed while pushing forward");
      ok &= check(q[1] == -0.20, "leg snapshot must stay fixed while pushing forward");
      ok &= check(q.head<3>().sum() <= 0.5235 + 1e-12, "forward pitch workspace must not exceed limit");
    }
    ok &= check(near(q[2], 0.6035), "forward pitch must stop at workspace limit");
    ok &= check(near(q[3], 0.5235), "forward yaw must stop at workspace limit");
    // 在限位继续推不会累计越界目标，反向第一拍就离开限位。
    q = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
    ok &= check(near(q[2], 0.5735), "first reverse step must leave pitch limit");
    for (int i = 0; i < 500; ++i) {
      q = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
    }
    ok &= check(near(q[2], 0.08), "reverse pitch must return to start");
    ok &= check(near(q[3], -0.5235), "reverse yaw must stop at workspace limit");
  }
  return ok;
}

bool testModelLimitsTakePrecedence() {
  bool ok = true;
  Eigen::Vector4d lo = kLower;
  Eigen::Vector4d hi = kUpper;
  lo[2] = 0.10;
  hi[2] = 0.25;
  lo[3] = -0.10;
  hi[3] = 0.10;
  Eigen::Vector4d q(0.0, 0.0, 0.15, 0.0);
  q = ocs2::integrateTorsoRotationTarget(q, 1.5, 1.5, 1.0, lo, hi);
  ok &= check(q[2] == 0.25, "model pitch upper limit must take precedence");
  ok &= check(q[3] == 0.10, "model yaw upper limit must take precedence");
  q = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 1.0, lo, hi);
  ok &= check(q[2] == 0.10, "model pitch lower limit must take precedence");
  ok &= check(q[3] == -0.10, "model yaw lower limit must take precedence");
  return ok;
}

bool testOutsideWorkspaceEntryDoesNotJumpAndOnlyMovesInward() {
  bool ok = true;
  const Eigen::Vector4d q(0.0, 0.0, 0.8, 0.7);
  const auto held = ocs2::integrateTorsoRotationTarget(q, 1.5, 1.5, 0.02, kLower, kUpper);
  ok &= check(held.isApprox(q), "out-of-workspace hold must not jump");
  const auto inward = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
  ok &= check(near(inward[2], 0.77), "out-of-workspace pitch may only move inward");
  ok &= check(near(inward[3], 0.67), "out-of-workspace yaw may only move inward");
  return ok;
}

bool testStopAndInvalidInputPreserveTarget() {
  bool ok = true;
  const Eigen::Vector4d q(0.03, -0.06, 0.2, -0.2);
  ok &= check(ocs2::integrateTorsoRotationTarget(q, 0, 0, 0.02, kLower, kUpper).isApprox(q),
              "zero velocity must preserve target");
  ok &= check(ocs2::integrateTorsoRotationTarget(q, 1, 1, -0.02, kLower, kUpper).isApprox(q),
              "negative dt must preserve target");
  ok &= check(ocs2::integrateTorsoRotationTarget(q, std::numeric_limits<double>::quiet_NaN(), 1, 0.02,
                                                 kLower, kUpper)
                  .isApprox(q),
              "NaN velocity must preserve target");
  return ok;
}

bool testResetKeepsRaisedKneeLegAndRestoresOnlyWaist() {
  const Eigen::Vector4d held(0.12, -0.20, 0.6035, 0.4);
  const Eigen::Vector4d initial(0.0, 0.0, 0.08, -0.02);
  const auto reset = ocs2::makeTorsoRotationResetTarget(held, initial, kLower, kUpper);
  bool ok = check(reset[0] == held[0] && reset[1] == held[1],
                  "rotation reset must not restore initial knee/leg after torso translation");
  ok &= check(reset[2] == initial[2] && reset[3] == initial[3],
              "rotation reset must restore initial waist pitch/yaw");
  Eigen::Vector4d upper = kUpper;
  upper[2] = 0.05;
  const auto limited = ocs2::makeTorsoRotationResetTarget(held, initial, kLower, upper);
  ok &= check(limited[2] == upper[2], "rotation reset must respect model joint limits");
  return ok;
}

}  // namespace

int main() {
  bool ok = true;
  ok &= testForwardLimitThenReverseKeepsKneeAndLegSnapshot();
  ok &= testModelLimitsTakePrecedence();
  ok &= testOutsideWorkspaceEntryDoesNotJumpAndOnlyMovesInward();
  ok &= testStopAndInvalidInputPreserveTarget();
  ok &= testResetKeepsRaisedKneeLegAndRestoresOnlyWaist();
  if (!ok) {
    std::cerr << "torso_rotation_target_test failed\n";
    return 1;
  }
  std::cout << "torso_rotation_target_test passed\n";
  return 0;
}
