#include "DrakeChestElbowHandPointOpt.hpp"

#include <iostream>

using namespace HighlyDynamic;

int main() {
  const Eigen::Vector3d leftShoulder(0.0, 0.25, 0.2);
  const Eigen::Vector3d rightShoulder(0.0, -0.25, 0.2);
  UpperBodyPoseList fk;
  fk[BODY][WAIST].p = Eigen::Vector3d(0.3, 0.0, 1.0);
  fk[BODY][WAIST].q = Eigen::Quaterniond::Identity();
  for (auto side : {LEFT, RIGHT}) {
    const auto shoulder = fk[BODY][WAIST].p + (side == LEFT ? leftShoulder : rightShoulder);
    fk[side][ELBOW].p = shoulder + Eigen::Vector3d(0.3, 0.0, 0.0);
    fk[side][HAND].p = fk[side][ELBOW].p + Eigen::Vector3d(0.0, 0.0, -0.3);
  }
  DrakeChestElbowHandPointOptSolver solver(leftShoulder, rightShoulder, 0.3, 0.3, &fk);
  // Isolate the cache/lock regression from wall-clock derivative penalties:
  // this test executes static frames without the ROS loop's sampling interval.
  DrakeChestElbowHandWeightConfig weights;
  weights.qv0.setZero();
  weights.qv1 = weights.qv2 = 0.0;
  weights.qa0 = weights.qa1 = weights.qa2 = 0.0;
  solver.setWeights(weights);
  auto solve = [&](bool updatePosition, bool updateOrientation) {
    return solver.solve(fk[BODY][WAIST].p, fk[BODY][WAIST].q,
                        fk[LEFT][ELBOW].p, fk[LEFT][HAND].p,
                        fk[RIGHT][ELBOW].p, fk[RIGHT][HAND].p,
                        updateOrientation, updatePosition, true);
  };
  // Reach normal operation: caller locks only take effect after initial warmup.
  for (int i = 0; i < 101; ++i) {
    if (!solve(true, true).success) {
      std::cerr << "Warmup solve failed at frame " << i << '\n';
      return 1;
    }
  }
  const auto beforePause = solve(false, false);
  // TUI translates the whole upper body and rotates the chest during pause.
  const Eigen::Vector3d translation(0.0, 0.0, 0.2);
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d oldChest = fk[BODY][WAIST].p;
  fk[BODY][WAIST].p += translation;
  fk[BODY][WAIST].q = rotation;
  for (auto side : {LEFT, RIGHT}) {
    for (auto point : {ELBOW, HAND}) {
      fk[side][point].p = fk[BODY][WAIST].p + rotation * (fk[side][point].p - oldChest);
    }
  }
  // Demonstrate the stale-cache failure before re-anchoring.
  const auto stale = solve(false, false);
  if (!stale.success || (stale.pChest - beforePause.pChest).norm() > 1e-6) {
    std::cerr << "Could not reproduce stale chest lock\n";
    return 2;
  }
  solver.reanchorFromFkResult(fk);
  // Supply a conflicting reference: locks must stay effective after re-anchor.
  const auto expectedChest = fk[BODY][WAIST].p;
  fk[BODY][WAIST].p.z() += 0.1;
  fk[BODY][WAIST].q = Eigen::Quaterniond::Identity();
  for (int i = 0; i < 3; ++i) {
    const auto resumed = solve(false, false);
    if (!resumed.success || (resumed.pChest - expectedChest).norm() > 1e-6 ||
        resumed.qChest.angularDistance(rotation) > 1e-6 ||
        (resumed.pLeftHand - fk[LEFT][HAND].p).norm() > 1e-4 ||
        (resumed.pRightHand - fk[RIGHT][HAND].p).norm() > 1e-4) {
      std::cerr << "Re-anchor failed to preserve TUI chest pose with caller locks enabled\n";
      return 3;
    }
  }
  std::cout << "Paused TUI motion: stale cache reproduced; re-anchor and caller locks passed\n";
  return 0;
}
