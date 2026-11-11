#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Core>

#include "humanoid_wheel_interface/motion_planner/TorsoRotationTarget.h"
#include "humanoid_wheel_interface/motion_planner/TorsoReachability.h"
#include "humanoid_wheel_interface/motion_planner/TorsoYawTarget.h"

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
    ok &= check(near(q[3], kUpper[3]), "forward yaw must reach model limit beyond 30 degrees");
    // 在限位继续推不会累计越界目标，反向第一拍就离开限位。
    q = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
    ok &= check(near(q[2], 0.5735), "first reverse step must leave pitch limit");
    for (int i = 0; i < 500; ++i) {
      q = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
    }
    ok &= check(near(q[2], 0.08), "reverse pitch must return to start");
    ok &= check(near(q[3], kLower[3]), "reverse yaw must reach model limit beyond 30 degrees");
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
  const Eigen::Vector4d q(0.0, 0.0, 0.8, 3.3);
  const auto held = ocs2::integrateTorsoRotationTarget(q, 1.5, 1.5, 0.02, kLower, kUpper);
  ok &= check(held.isApprox(q), "out-of-workspace hold must not jump");
  const auto inward = ocs2::integrateTorsoRotationTarget(q, -1.5, -1.5, 0.02, kLower, kUpper);
  ok &= check(near(inward[2], 0.77), "out-of-workspace pitch may only move inward");
  ok &= check(near(inward[3], 3.27), "out-of-model-limit yaw may only move inward");
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

bool testApplyRotationJointDeltaRespectsYawButtonLimit() {
  bool ok = true;
  Eigen::Vector4d q(0.12, -0.20, 0.08, 0.0);
  const double full = ocs2::kTorsoYawTargetLimit;
  q = ocs2::applyTorsoRotationJointDelta(q, 0.0, full + 0.2, kLower, kUpper);
  ok &= check(near(q[3], full), "oneshot yaw must stop at button margin, not model ±pi");
  ok &= check(q[0] == 0.12 && q[1] == -0.20, "oneshot yaw must keep knee/leg snapshot");
  ok &= check(near(q[2], 0.08), "yaw-only oneshot must not change pitch");

  // Near-limit remainder must not overshoot.
  q[3] = full - 0.001;
  q = ocs2::applyTorsoRotationJointDelta(q, 0.0, 1.0, kLower, kUpper);
  ok &= check(near(q[3], full), "near-limit oneshot must clamp to button margin");

  q[3] = -full + 0.001;
  q = ocs2::applyTorsoRotationJointDelta(q, 0.0, -1.0, kLower, kUpper);
  ok &= check(near(q[3], -full), "negative near-limit oneshot must clamp to -button margin");

  // Already at limit + same direction → hold.
  const auto held = ocs2::applyTorsoRotationJointDelta(
      Eigen::Vector4d(0.12, -0.20, 0.08, full), 0.0, 0.5, kLower, kUpper);
  ok &= check(near(held[3], full), "at-limit same-direction oneshot must hold");

  // Reverse from +limit must move inward.
  const auto reverse = ocs2::applyTorsoRotationJointDelta(
      Eigen::Vector4d(0.12, -0.20, 0.08, full), 0.0, -0.2, kLower, kUpper);
  ok &= check(near(reverse[3], full - 0.2), "reverse oneshot from +limit must leave margin");
  return ok;
}

bool testApplyRotationJointDeltaPitchUsesWorkspace() {
  bool ok = true;
  Eigen::Vector4d q(0.12, -0.20, 0.08, 0.1);
  // pitchOffset = -0.08; upper = min(2.985, 0.5235+0.08)=0.6035
  q = ocs2::applyTorsoRotationJointDelta(q, 10.0, 0.0, kLower, kUpper);
  ok &= check(near(q[2], 0.6035), "oneshot pitch must respect workspace upper");
  ok &= check(near(q[3], 0.1), "pitch-only oneshot must not change yaw");
  ok &= check(q[0] == 0.12 && q[1] == -0.20, "oneshot pitch must keep knee/leg snapshot");
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

bool testClassifyOneshotDeltaDispositions() {
  bool ok = true;
  ok &= check(ocs2::classifyTorsoOneshotDelta(false, false, false) ==
                  ocs2::TorsoDeltaDisposition::kDeferPending,
              "idle yaw/pitch-only must defer");
  ok &= check(ocs2::classifyTorsoOneshotDelta(true, false, false) ==
                  ocs2::TorsoDeltaDisposition::kAsRotationJoint,
              "rotation-requested yaw/pitch-only must be joint delta");
  ok &= check(ocs2::classifyTorsoOneshotDelta(false, true, false) ==
                  ocs2::TorsoDeltaDisposition::kAsRotationJoint,
              "rotation-active yaw/pitch-only must be joint delta");
  ok &= check(ocs2::classifyTorsoOneshotDelta(false, false, true) ==
                  ocs2::TorsoDeltaDisposition::kAsCartesian,
              "cartesian x/z oneshot must stay cartesian");
  ok &= check(ocs2::classifyTorsoOneshotDelta(true, true, true) ==
                  ocs2::TorsoDeltaDisposition::kAsCartesian,
              "cartesian oneshot wins even inside rotation");
  return ok;
}

// 模拟 RM 时序：首次俯仰+yaw 同帧，delta 先到并中间跑一次更新，再持续俯仰。
// 旧路径会把 yaw 当笛卡尔消费后从 initState 重建关节目标 → 丢未完成 yaw；
// handoff 必须把 pending 折入进入旋转后的关节目标。
bool testFirstPitchYawRaceHandoffPreservesYaw() {
  bool ok = true;
  const double yawOneshot = 0.35;
  Eigen::Vector4d pending = Eigen::Vector4d::Zero();
  bool hasPending = false;

  // Frame 1: idle/trans, yaw-only delta arrives before rotation_vel.
  const bool rotReq1 = false;
  const bool rotActive1 = false;
  const bool deltaHasCartesian1 = false;
  ok &= check(ocs2::classifyTorsoOneshotDelta(rotReq1, rotActive1, deltaHasCartesian1) ==
                  ocs2::TorsoDeltaDisposition::kDeferPending,
              "race frame1 must defer yaw oneshot");
  pending[2] = yawOneshot;
  hasPending = true;
  const bool stashedThisFrame = true;
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(hasPending, stashedThisFrame, rotReq1),
              "race frame1 must not flush the just-stashed oneshot");

  // Frame 2: continuous pitch → rotation requested; enter from initState and handoff.
  const bool rotReq2 = true;
  const Eigen::Vector4d initJoints(0.12, -0.20, 0.08, 0.0);
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(hasPending, false, rotReq2),
              "entering rotation must handoff, not cartesian-flush");
  auto jointTarget = ocs2::handoffPendingTorsoRotationDelta(initJoints, pending, kLower, kUpper);
  // Then integrate one pitch step as continuous pitch would.
  jointTarget = ocs2::integrateTorsoRotationTarget(jointTarget, -0.5, 0.0, 0.02, kLower, kUpper);

  ok &= check(near(jointTarget[3], yawOneshot),
              "handoff after race must keep the unfinished yaw oneshot");
  ok &= check(jointTarget[0] == 0.12 && jointTarget[1] == -0.20,
              "handoff must keep knee/leg snapshot from initState");
  ok &= check(near(jointTarget[2], 0.08 - 0.01),
              "continuous pitch after handoff must still move pitch");

  // Without handoff (old bug): rebuild from initState alone loses yaw.
  const auto lost = ocs2::integrateTorsoRotationTarget(initJoints, -0.5, 0.0, 0.02, kLower, kUpper);
  ok &= check(near(lost[3], 0.0), "control: no-handoff path would have zero yaw");
  ok &= check(std::abs(jointTarget[3] - lost[3]) > 1e-6,
              "handoff yaw must differ from the lost no-handoff baseline");
  return ok;
}


// 生产路径：旧升降速度 sticky 保持时，yaw/pitch-only delta 先到。
// 旧 bug：stash 后 delta4 未清零，且 integrateVel 跳过空闲保护，笛卡尔末尾无条件冲刷
// → pending 被加回原 delta4（翻倍）并提前清空，后续 rotation_vel 无法交接。
// 现路径：transfer 清零 delta4；笛卡尔冲刷共用 shouldFlush（排除 stashedThisFrame）。
bool testStickyHeightVelStashNotDoubleFlushedThenHandoff() {
  bool ok = true;
  const double yawOneshot = 0.35;
  Eigen::Vector4d delta4(0.0, 0.0, yawOneshot, 0.0);
  bool hasDelta = true;
  Eigen::Vector4d pending = Eigen::Vector4d::Zero();
  bool hasPending = false;
  bool stashedThisFrame = false;
  const bool integrateVel = true;  // 旧 /cmd_torso_vel 升降仍 sticky
  const bool rotationRequested = false;
  const bool torsoRotationActive = false;

  ok &= check(ocs2::classifyTorsoOneshotDelta(rotationRequested, torsoRotationActive,
                                              /*deltaHasCartesian=*/false) ==
                  ocs2::TorsoDeltaDisposition::kDeferPending,
              "sticky-height frame must defer yaw oneshot");
  ocs2::transferOneshotToPendingTorsoRotationDelta(delta4, hasDelta, pending, hasPending,
                                                   stashedThisFrame);
  ok &= check(stashedThisFrame && hasPending && !hasDelta, "stash must own the oneshot");
  ok &= check(delta4.isZero(1e-15), "stashed delta4 must be zeroed to prevent double-count");
  ok &= check(near(pending[2], yawOneshot) && near(pending[3], 0.0),
              "pending must hold the oneshot exactly once");

  // 空闲早退因 integrateVel 被跳过；笛卡尔速度路径必须同样排除同帧 stash。
  ok &= check(integrateVel, "production path keeps integrating sticky height this frame");
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(hasPending, stashedThisFrame,
                                                           rotationRequested),
              "sticky-height cartesian path must not flush same-frame stash");

  // 控制：若误用旧逻辑（不清零 + 无条件冲刷），笛卡尔会吃到 2x 并清空 pending。
  Eigen::Vector4d buggyDelta = Eigen::Vector4d(0.0, 0.0, yawOneshot, 0.0);
  Eigen::Vector4d buggyPending = pending;
  buggyDelta[2] += buggyPending[2];
  buggyDelta[3] += buggyPending[3];
  ok &= check(near(buggyDelta[2], 2.0 * yawOneshot),
              "control: old path would double-count yaw on cartesian flush");

  // 下一帧俯仰速度到达：pending 仍在，handoff 只累计一次。
  const Eigen::Vector4d initJoints(0.12, -0.20, 0.08, 0.0);
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(hasPending, /*stashedThisFrame=*/false,
                                                           /*rotationRequested=*/true),
              "entering rotation must handoff, not cartesian-flush");
  auto jointTarget =
      ocs2::handoffPendingTorsoRotationDelta(initJoints, pending, kLower, kUpper);
  jointTarget =
      ocs2::integrateTorsoRotationTarget(jointTarget, -0.5, 0.0, 0.02, kLower, kUpper);
  ok &= check(near(jointTarget[3], yawOneshot),
              "handoff after sticky-height stash must apply yaw once");
  ok &= check(jointTarget[0] == 0.12 && jointTarget[1] == -0.20,
              "handoff must keep knee/leg snapshot");
  ok &= check(near(jointTarget[2], 0.08 - 0.01),
              "continuous pitch after handoff must still move pitch");
  return ok;
}

// 持续升降、本帧无新 stash：上一帧遗留 pending 仍应走笛卡尔冲刷（不切旋转时）。
bool testStickyHeightLaterFrameFlushesPriorPending() {
  bool ok = true;
  const bool integrateVel = true;
  const bool stashedThisFrame = false;
  const bool rotationRequested = false;
  ok &= check(integrateVel, "later sticky-height frame still integrating");
  ok &= check(ocs2::shouldFlushDeferredTorsoRotationDelta(/*hasPending=*/true, stashedThisFrame,
                                                          rotationRequested),
              "later sticky-height frame without new stash must cartesian-flush prior pending");
  return ok;
}

bool testFlushDeferredOnlyOnLaterIdleFrame() {
  bool ok = true;
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(false, false, false),
              "no pending → no flush");
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(true, true, false),
              "same-frame stash → no flush");
  ok &= check(!ocs2::shouldFlushDeferredTorsoRotationDelta(true, false, true),
              "rotation requested → handoff path, not flush");
  ok &= check(ocs2::shouldFlushDeferredTorsoRotationDelta(true, false, false),
              "later idle frame with pending → cartesian flush");
  return ok;
}

}  // namespace

int main() {
  bool ok = true;
  const ocs2::TorsoPlanarGeometry geometry{{0.098, 0.4155}, {-0.46521, 0.18325},
                                          {0.48296, 0.12941}, {0.0, 0.25}};
  const Eigen::Vector4d high(0.194, 1.298, 0.0, 0.524);
  const Eigen::Vector4d unreachable(0.164, 0.978, 0.0, 0.523);
  ok &= check(ocs2::isTorsoPoseReachable(high, geometry, kLower, kUpper),
              "bag high torso pose must be reachable");
  ok &= check(!ocs2::isTorsoPoseReachable(unreachable, geometry, kLower, kUpper),
              "bag low forward-pitch pose requiring negative knee must be rejected");
  const Eigen::Vector4d lowered(high[0], 0.978, high[2], high[3]);
  const auto limited = ocs2::limitTorsoReachableStep(high, lowered, geometry, kLower, kUpper);
  ok &= check(ocs2::isTorsoPoseReachable(limited, geometry, kLower, kUpper) && limited[1] > lowered[1],
              "lowering must stop at the reachable boundary");
  ok &= check(limited[0] == high[0] && limited[2] == high[2] && limited[3] == high[3],
              "pure lowering must preserve x, yaw and pitch");
  const auto reverse = ocs2::limitTorsoReachableStep(limited, high, geometry, kLower, kUpper);
  ok &= check(reverse.isApprox(high, 1e-10), "raising from lower boundary must remain responsive");
  const auto held = ocs2::limitTorsoReachableStep(limited, limited, geometry, kLower, kUpper);
  ok &= check(held.isApprox(limited, 1e-12), "zero input at reachable boundary must hold");
  Eigen::Vector4d recoveryRequest = unreachable;
  recoveryRequest[1] -= 0.001;
  const auto recovered = ocs2::limitTorsoReachableStep(unreachable, recoveryRequest, geometry,
                                                      kLower, kUpper, &high);
  Eigen::Vector4d expectedRecovery = high;
  expectedRecovery[1] -= 0.001;
  ok &= check(recovered.isApprox(expectedRecovery, 1e-10) &&
              ocs2::isTorsoPoseReachable(recovered, geometry, kLower, kUpper),
              "unreachable old target must resume a downward step from valid current FK");
  const auto blockedRecovery = ocs2::limitTorsoReachableStep(unreachable, recoveryRequest, geometry,
                                                            kLower, kUpper, &unreachable);
  ok &= check(blockedRecovery.isApprox(unreachable, 1e-12),
              "an invalid recovery anchor must never bypass joint limits");
  const Eigen::Vector2d rearXZ = geometry.knee +
      ocs2::rotateTorsoXZ(geometry.legOffset, 0.15) +
      ocs2::rotateTorsoXZ(geometry.waistOffset, -0.60) + geometry.torsoOffset;
  Eigen::Vector4d descending(rearXZ[0], rearXZ[1], 0.0, 0.0);
  const double originalX = descending[0];
  for (int i = 0; i < 500; ++i) {
    Eigen::Vector4d request = descending;
    request[1] = std::max(0.97816, request[1] - 0.004);
    const auto next = ocs2::limitTorsoCoupledDescent(descending, request, geometry, kLower, kUpper,
                                                   0.009, std::min(originalX, 0.11575), 0.36575);
    ok &= check(ocs2::isTorsoPoseReachable(next, geometry, kLower, kUpper),
                "every coupled descent target must remain reachable");
    ok &= check(next[1] <= descending[1] + 1e-12 && std::abs(next[0] - descending[0]) <= 0.009 + 1e-12,
                "coupled descent must not rise or exceed the x speed step");
    ok &= check(next[2] == 0.0 && next[3] == 0.0, "coupled descent must preserve yaw and upright pitch");
    descending = next;
  }
  ok &= check(near(descending[1], 0.97816, 1e-6) && descending[0] > originalX,
              "rearward upright torso must reach initial height through necessary x adjustment");
  const auto stopped = ocs2::limitTorsoCoupledDescent(descending, descending, geometry, kLower, kUpper,
                                                     0.009, originalX, 0.36575);
  ok &= check(stopped.isApprox(descending, 1e-12), "releasing descent input must stop x adjustment");
  ok &= testForwardLimitThenReverseKeepsKneeAndLegSnapshot();
  ok &= testModelLimitsTakePrecedence();
  ok &= testOutsideWorkspaceEntryDoesNotJumpAndOnlyMovesInward();
  ok &= testStopAndInvalidInputPreserveTarget();
  ok &= testApplyRotationJointDeltaRespectsYawButtonLimit();
  ok &= testApplyRotationJointDeltaPitchUsesWorkspace();
  ok &= testResetKeepsRaisedKneeLegAndRestoresOnlyWaist();
  ok &= testClassifyOneshotDeltaDispositions();
  ok &= testFirstPitchYawRaceHandoffPreservesYaw();
  ok &= testStickyHeightVelStashNotDoubleFlushedThenHandoff();
  ok &= testStickyHeightLaterFrameFlushesPriorPending();
  ok &= testFlushDeferredOnlyOnLaterIdleFrame();
  for (double direction : {-1.0, 1.0}) {
    const double previous = direction * 0.2;
    ok &= check(near(ocs2::clampTorsoTranslationFromRotation(previous, previous, -0.1, 0.1), previous),
                "rotation FK outside translation envelope must hold without jumping");
    ok &= check(near(ocs2::clampTorsoTranslationFromRotation(direction * 0.19, previous, -0.1, 0.1),
                     direction * 0.19), "inherited translation target must allow small inward steps");
    ok &= check(near(ocs2::clampTorsoTranslationFromRotation(direction * 0.21, previous, -0.1, 0.1), previous),
                "inherited translation target must reject outward steps");
    ok &= check(near(ocs2::clampTorsoTranslationFromRotation(direction * 0.12, direction * 0.05, -0.1, 0.1),
                     direction * 0.1), "translation target inside envelope must respect normal limits");
  }
  for (double direction : {-1.0, 1.0}) {
    const double target = direction * ocs2::kTorsoYawTargetLimit;
    ok &= check(near(ocs2::clampTorsoYawTarget(target), target),
                "VR A/B must reach pi minus margin without a height gate");
    ok &= check(near(ocs2::clampTorsoYawTarget(target * 0.5), target * 0.5),
                "VR must allow partial yaw targets in either direction");
    ok &= check(near(ocs2::clampTorsoYawTarget(-target), -target),
                "VR must allow yaw targets across zero");
    ok &= check(near(ocs2::clampTorsoYawTarget(direction * M_PI), target),
                "yaw target must retain the pi boundary margin");
  }
  ok &= check(near(ocs2::clampTorsoYawTarget(0.0), 0.0),
              "VR yaw recentering must remain available");
  if (!ok) {
    std::cerr << "torso_rotation_target_test failed\n";
    return 1;
  }
  std::cout << "torso_rotation_target_test passed\n";
  return 0;
}
