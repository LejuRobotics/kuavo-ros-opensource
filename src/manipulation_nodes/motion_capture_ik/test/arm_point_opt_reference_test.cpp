#include "DrakeElbowHandPointOpt.hpp"
#include "motion_capture_ik/WheelNaturalElbowGuide.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace {

bool nearlyEqual(double a, double b, double tol = 2e-2) { return std::abs(a - b) <= tol; }

// 与 Quest3IkIncrementalROS::applyNaturalElbowTrackingRefs 一致：
// 持久 guide + 躯干参考 + p1Ref/p1Fixed 同锁到 guide.elbowPosition。
Eigen::Vector3d applyNaturalRefsLikeProduction(HighlyDynamic::WheelNaturalElbowGuide& guide,
                                               const Eigen::Vector3d& shoulder,
                                               const Eigen::Vector3d& oppositeShoulder,
                                               const Eigen::Vector3d& hand,
                                               const Eigen::Vector3d& currentElbow,
                                               double sideSign) {
  HighlyDynamic::WheelNaturalElbowGuideInput in;
  in.shoulderPosition = shoulder;
  in.handPosition = hand;
  in.currentElbowPosition = currentElbow;
  in.humanPoseValid = false;
  in.torsoPosition = 0.5 * (shoulder + oppositeShoulder);
  // 肩/肘在 sideSign*Y 侧：外向取同号，与生产左+Y/右-Y（肩坐标同侧）一致。
  // 切勿写成 sideSign<0 ? +Y : -Y，否则初始肘 signed clearance 为负，触发反向腰部偏置。
  in.torsoOutwardDirection = sideSign * Eigen::Vector3d::UnitY();
  in.torsoFrameValid = true;
  return guide.update(in).elbowPosition;
}

}  // namespace

int main() {
  using namespace HighlyDynamic;
  for (double side : {-1.0, 1.0}) {
    const Eigen::Vector3d shoulder(0.1157, side * 0.2527, 1.1658);
    const Eigen::Vector3d oppositeShoulder(0.1157, -side * 0.2527, 1.1658);
    const Eigen::Vector3d elbow(0.13575, side * 0.2527, 0.88210321);
    const Eigen::Vector3d wrist(0.11575, side * 0.2527, 0.64860321);
    const Eigen::Vector3d lower(shoulder.x(), side < 0 ? shoulder.y() - 0.5 : -0.2, -0.3);
    const Eigen::Vector3d upper(shoulder.x() + 0.6, side < 0 ? 0.2 : shoulder.y() + 0.5,
                                shoulder.z() + 0.1);
    DrakeVelocityIKSolver solver(shoulder, 0.2844, 0.2344, lower, upper);
    solver.setWeights(DrakeVelocityIKWeights::HandPriority);
    if (!solver.seedFromMeasured(elbow, wrist)) return 1;
    const Eigen::Vector3d flaredFixed =
        shoulder + Eigen::Vector3d(-0.2825, side * 0.2073, -0.1045);
    Eigen::Vector3d invalid = elbow;
    invalid.x() = std::numeric_limits<double>::quiet_NaN();
    bool success = true;
    const auto held = solver.solve(invalid, flaredFixed, wrist, &success);
    if (success || !held.first.allFinite() || !held.second.allFinite()) return 2;
    // 无效参考不能污染历史；用本次实测肘部参考重新建锚后应恢复求解。
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (!solver.seedFromMeasured(elbow, wrist)) return 3;
      const auto result = solver.solve(elbow, flaredFixed, wrist, &success);
      if (!success || !result.first.allFinite() || !result.second.allFinite()) {
        std::cerr << "Measured elbow reanchor failed for side " << side << '\n';
        return 4;
      }
    }

    // #4746: bag 级略抬手（约 1–2cm up）。外张 p1Fixed 应明显拉大 |Y|；
    // 实测肘作 p1Fixed（且 p1Ref 锁定同点）时 |Y| 应贴近实测。
    const Eigen::Vector3d liftedWrist = wrist + Eigen::Vector3d(0.005, -side * 0.005, 0.015);
    DrakeVelocityIKSolver flaredSolver(shoulder, 0.2844, 0.2344, lower, upper);
    flaredSolver.setWeights(DrakeVelocityIKWeights::HandPriority);
    if (!flaredSolver.seedFromMeasured(elbow, wrist)) return 5;
    Eigen::Vector3d flaredElbow = elbow;
    for (int i = 0; i < 12; ++i) {
      const auto r = flaredSolver.solve(elbow, flaredFixed, liftedWrist, &success);
      if (!success) return 6;
      flaredElbow = r.first;
    }

    DrakeVelocityIKSolver anchoredSolver(shoulder, 0.2844, 0.2344, lower, upper);
    anchoredSolver.setWeights(DrakeVelocityIKWeights::HandPriority);
    if (!anchoredSolver.seedFromMeasured(elbow, wrist)) return 7;
    const Eigen::Vector3d measuredFixed = elbow;  // grip 上升沿刷新 p1Fixed + 锁定 p1Ref
    Eigen::Vector3d anchoredElbow = elbow;
    for (int i = 0; i < 12; ++i) {
      const auto r = anchoredSolver.solve(measuredFixed, measuredFixed, liftedWrist, &success);
      if (!success) return 8;
      anchoredElbow = r.first;
    }

    const double flaredAbsY = std::abs(flaredElbow.y());
    const double anchoredAbsY = std::abs(anchoredElbow.y());
    const double measuredAbsY = std::abs(elbow.y());
    if (!(flaredAbsY > measuredAbsY + 0.02)) {
      std::cerr << "Expected flared fixed-point to pull |Y| out, side=" << side
                << " flared=" << flaredAbsY << " measured=" << measuredAbsY << '\n';
      return 9;
    }
    if (!(anchoredAbsY < flaredAbsY - 0.015)) {
      std::cerr << "Expected measured fixed-point to reduce flare, side=" << side
                << " anchored=" << anchoredAbsY << " flared=" << flaredAbsY << '\n';
      return 10;
    }
    if (!nearlyEqual(anchoredAbsY, measuredAbsY, 0.03)) {
      std::cerr << "Anchored |Y| drifted too far from measured, side=" << side
                << " anchored=" << anchoredAbsY << " measured=" << measuredAbsY << '\n';
      return 11;
    }

    // #4746: 对齐生产 applyNaturalElbowTrackingRefs —
    // 持久 guide、躯干参考、p1Ref=p1Fixed=guide.elbowPosition；
    // 两轮 unlock 间显式 reset()+seed，覆盖首次／二次跟踪。
    {
      // 初始肘相对躯干沿外向的有符号净空必须为正（未侵入腰部）。
      {
        const Eigen::Vector3d torso = 0.5 * (shoulder + oppositeShoulder);
        const Eigen::Vector3d outward = side * Eigen::Vector3d::UnitY();
        const double signedClearance = (elbow - torso).dot(outward);
        if (!(signedClearance > 0.0)) {
          std::cerr << "Expected positive initial elbow signed clearance vs torso, side="
                    << side << " clearance=" << signedClearance << '\n';
          return 21;
        }
      }
      WheelNaturalElbowGuideConfig cfg;  // 默认与生产一致（blend=0.9, waist on）
      WheelNaturalElbowGuide guide(0.2844, 0.2344, cfg);

      auto runUnlockEpisode = [&](int failBase, Eigen::Vector3d* outElbow,
                                  Eigen::Vector3d* outHand, double* outHandErr) -> int {
        DrakeVelocityIKSolver naturalSolver(shoulder, 0.2844, 0.2344, lower, upper);
        naturalSolver.setWeights(DrakeVelocityIKWeights::HandPriority);
        if (!naturalSolver.seedFromMeasured(elbow, wrist)) return failBase;
        Eigen::Vector3d naturalElbow = elbow;
        Eigen::Vector3d naturalHand = wrist;
        double maxHandErr = 0.0;
        for (int i = 0; i < 12; ++i) {
          const Eigen::Vector3d ref = applyNaturalRefsLikeProduction(
              guide, shoulder, oppositeShoulder, liftedWrist, naturalElbow, side);
          const auto r = naturalSolver.solve(ref, ref, liftedWrist, &success);
          if (!success || !r.first.allFinite() || !r.second.allFinite()) {
            std::cerr << "Natural solve failed side=" << side << " step=" << i << '\n';
            return failBase + 1;
          }
          naturalElbow = r.first;
          naturalHand = r.second;
          maxHandErr = std::max(maxHandErr, (naturalHand - liftedWrist).norm());
        }
        *outElbow = naturalElbow;
        *outHand = naturalHand;
        *outHandErr = maxHandErr;
        return 0;
      };

      // 首次 unlock + 略抬手跟踪
      Eigen::Vector3d ep1Elbow, ep1Hand;
      double ep1HandErr = 0.0;
      if (const int rc = runUnlockEpisode(12, &ep1Elbow, &ep1Hand, &ep1HandErr)) return rc;

      // 模拟二次 unlock：reset 导引历史并以零位/建锚肘重 seed（对齐 mode2 进入）
      guide.reset();
      Eigen::Vector3d ep2Elbow, ep2Hand;
      double ep2HandErr = 0.0;
      if (const int rc = runUnlockEpisode(14, &ep2Elbow, &ep2Hand, &ep2HandErr)) return rc;

      const double ep1AbsY = std::abs(ep1Elbow.y());
      const double ep2AbsY = std::abs(ep2Elbow.y());
      const double measuredAbsYLocal = std::abs(elbow.y());

      // 不应像 flaredFixed 那样大幅外张
      if (!(ep1AbsY < flaredAbsY - 0.01) || !(ep2AbsY < flaredAbsY - 0.01)) {
        std::cerr << "Expected natural dual-ref |Y| below flared, side=" << side
                  << " ep1=" << ep1AbsY << " ep2=" << ep2AbsY << " flared=" << flaredAbsY
                  << '\n';
        return 16;
      }
      // 圆上允许少量 |Y| 变化，但不得大幅外漂
      if (!(ep1AbsY < measuredAbsYLocal + 0.05) || !(ep2AbsY < measuredAbsYLocal + 0.05)) {
        std::cerr << "Natural dual-ref |Y| drifted too far outward, side=" << side
                  << " ep1=" << ep1AbsY << " ep2=" << ep2AbsY
                  << " measured=" << measuredAbsYLocal << '\n';
        return 17;
      }
      // 首次／二次跟踪结果应接近（reset+重 seed 后不应二次放大）
      if (!nearlyEqual(ep1AbsY, ep2AbsY, 0.04)) {
        std::cerr << "1st/2nd natural |Y| diverged, side=" << side << " ep1=" << ep1AbsY
                  << " ep2=" << ep2AbsY << '\n';
        return 18;
      }
      // HandPriority 下手部跟踪误差应保持较小
      constexpr double kMaxHandErr = 0.03;  // 3 cm
      if (ep1HandErr > kMaxHandErr || ep2HandErr > kMaxHandErr) {
        std::cerr << "Hand tracking error too large, side=" << side
                  << " ep1Err=" << ep1HandErr << " ep2Err=" << ep2HandErr << '\n';
        return 19;
      }
      if (!ep1Hand.allFinite() || !ep2Hand.allFinite()) return 20;
    }
  }
  std::cout << "arm_point_opt_reference_test passed\n";
  return 0;
}
