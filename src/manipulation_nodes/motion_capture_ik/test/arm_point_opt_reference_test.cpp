#include "DrakeElbowHandPointOpt.hpp"

#include <iostream>
#include <limits>

int main() {
  using namespace HighlyDynamic;
  for (double side : {-1.0, 1.0}) {
    const Eigen::Vector3d shoulder(0.1157, side * 0.2527, 1.1658);
    const Eigen::Vector3d elbow(0.13575, side * 0.2527, 0.88210321);
    const Eigen::Vector3d wrist(0.11575, side * 0.2527, 0.64860321);
    const Eigen::Vector3d lower(shoulder.x(), side < 0 ? shoulder.y() - 0.5 : -0.2, -0.3);
    const Eigen::Vector3d upper(shoulder.x() + 0.6, side < 0 ? 0.2 : shoulder.y() + 0.5,
                                shoulder.z() + 0.1);
    DrakeVelocityIKSolver solver(shoulder, 0.2844, 0.2344, lower, upper);
    solver.setWeights(DrakeVelocityIKWeights::HandPriority);
    if (!solver.seedFromMeasured(elbow, wrist)) return 1;
    const Eigen::Vector3d fixed = shoulder + Eigen::Vector3d(-0.2825, side * 0.2073, -0.1045);
    Eigen::Vector3d invalid = elbow;
    invalid.x() = std::numeric_limits<double>::quiet_NaN();
    bool success = true;
    const auto held = solver.solve(invalid, fixed, wrist, &success);
    if (success || !held.first.allFinite() || !held.second.allFinite()) return 2;
    // 无效参考不能污染历史；用本次实测肘部参考重新建锚后应恢复求解。
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (!solver.seedFromMeasured(elbow, wrist)) return 3;
      const auto result = solver.solve(elbow, fixed, wrist, &success);
      if (!success || !result.first.allFinite() || !result.second.allFinite()) {
        std::cerr << "Measured elbow reanchor failed for side " << side << '\n';
        return 4;
      }
    }
  }
  std::cout << "arm_point_opt_reference_test passed\n";
  return 0;
}
