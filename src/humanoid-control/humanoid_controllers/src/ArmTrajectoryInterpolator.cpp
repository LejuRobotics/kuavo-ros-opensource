#include "humanoid_controllers/ArmTrajectoryInterpolator.h"

#include <algorithm>
#include <cmath>

#include "humanoid_wheel_interface/filters/HybridRuckigExtrapolator.h"
#include "leju_utils/math.hpp"

namespace humanoidController_wheel_wbc {

namespace {

}  // namespace

void ArmTrajectoryInterpolator::configure(const Config& config) {
  config_ = config;
  configured_ = true;
}

void ArmTrajectoryInterpolator::reset(const Eigen::VectorXd& currentQ, const ros::Time& now) {
  ensureBackend(static_cast<size_t>(currentQ.size()));
  if (backend_) {
    backend_->reset(currentQ);
  }
  smoothQ_ = currentQ;
  smoothV_ = Eigen::VectorXd::Zero(currentQ.size());
  fhanQ_ = currentQ;
  fhanV_ = Eigen::VectorXd::Zero(currentQ.size());
  lastTargetStamp_ = now;
  hasLastAppliedTargetSeq_ = false;
  initialized_ = true;
}

void ArmTrajectoryInterpolator::ingestRawTarget(const ros::Time& now, const Eigen::VectorXd& targetQ,
                                                const Eigen::VectorXd& targetV, const Eigen::VectorXd& measuredDq) {
  if (targetQ.size() == 0 || !isFiniteVector(targetQ)) {
    return;
  }

  Eigen::VectorXd mergedTargetV = Eigen::VectorXd::Zero(targetQ.size());
  bool hasTargetV = false;
  if (targetV.size() == targetQ.size() && isFiniteVector(targetV)) {
    mergedTargetV = targetV;
    hasTargetV = true;
  }

  // If input velocity looks like a missing field (all zeros), estimate it from target position delta.
  bool shouldEstimateVel = !hasTargetV;
  if (hasTargetV && mergedTargetV.cwiseAbs().maxCoeff() < 1e-6) {
    shouldEstimateVel = true;
  }
  if (shouldEstimateVel) {
    Eigen::VectorXd estimatedVel = Eigen::VectorXd::Zero(targetQ.size());
    if (hasLastRawTarget_ && lastRawTargetQ_.size() == targetQ.size() && !lastRawTargetStamp_.isZero()) {
      const double dt = (now - lastRawTargetStamp_).toSec();
      if (dt > 1e-4 && dt < 0.1) {
        estimatedVel = (targetQ - lastRawTargetQ_) / dt;
      }
    }
    for (Eigen::Index i = 0; i < estimatedVel.size(); ++i) {
      estimatedVel[i] = std::clamp(estimatedVel[i], -8.0, 8.0);
    }
    if (lastEstimatedTargetV_.size() == estimatedVel.size()) {
      estimatedVel = 0.2 * estimatedVel + 0.8 * lastEstimatedTargetV_;
    }
    mergedTargetV = estimatedVel;
    hasTargetV = true;
  }

  const bool targetChanged =
      !hasLastRawTarget_ || lastRawTargetQ_.size() != targetQ.size() ||
      (targetQ - lastRawTargetQ_).cwiseAbs().maxCoeff() > 1e-9 ||
      lastRawTargetV_.size() != mergedTargetV.size() ||
      (mergedTargetV - lastRawTargetV_).cwiseAbs().maxCoeff() > 1e-9;
  if (!targetChanged) {
    return;
  }

  lastRawTargetQ_ = targetQ;
  lastRawTargetV_ = mergedTargetV;
  lastEstimatedTargetV_ = mergedTargetV;
  lastRawTargetStamp_ = now;
  hasLastRawTarget_ = true;

  TargetSample sample;
  sample.targetQ = targetQ;
  sample.targetV = mergedTargetV;
  sample.hasTargetV = hasTargetV;
  sample.msgStamp = now;
  sample.msgSeq = ++localRawSeq_;
  sample.measuredDq = measuredDq;
  pushTarget(sample);
}

void ArmTrajectoryInterpolator::pushTarget(const TargetSample& sample) {
  if (!sample.targetQ.size()) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] ignore empty targetQ");
    return;
  }
  if (!isFiniteVector(sample.targetQ)) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] ignore non-finite targetQ");
    return;
  }
  if (sample.hasTargetV && sample.targetV.size() != sample.targetQ.size()) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] ignore mismatched targetV size");
    return;
  }
  if (hasLatestTargetSeq_ && sample.msgSeq <= latestTargetSeq_) {
    return;
  }
  latestTargetSeq_ = sample.msgSeq;
  hasLatestTargetSeq_ = true;
  target_ = sample;
  if (!sample.msgStamp.isZero()) {
    lastTargetStamp_ = sample.msgStamp;
  }
  hasTarget_ = true;
}

ArmTrajectoryInterpolator::Output ArmTrajectoryInterpolator::compute(
    const ros::Time& now, const ModeFlags& modeFlags, const Eigen::VectorXd& currentQ) {
  Output out;
  // 每周期以 smoothQ_（后端轨迹）为参考，用 fhan 跟踪微分器推进一步，
  // 输出天然满足加速度约束的 q/dq，作为本次默认输出（含所有 early-return）。
  if (initialized_ && fhanQ_.size() == smoothQ_.size()) {
    const double h  = config_.controlCycleSec;
    const double h0 = h * config_.fhanH0Ratio;
    const double r  = config_.fhanR;
    for (Eigen::Index i = 0; i < smoothQ_.size(); ++i) {
      leju_utils::fhanStepForward(fhanQ_(i), fhanV_(i), smoothQ_(i), r, h, h0);
    }
  }
  out.smoothQ = fhanQ_.size() == smoothQ_.size() ? fhanQ_ : smoothQ_;
  out.smoothV = fhanV_.size() == smoothV_.size() ? fhanV_ : smoothV_;

  const bool enabled = isEnabled(modeFlags);
  if (!enabled) {
    hasPrevModeFlags_ = true;
    prevModeFlags_ = modeFlags;
    return out;
  }

  const bool modeChanged = !hasPrevModeFlags_ ||
                           modeFlags.useArmTrajectoryControl != prevModeFlags_.useArmTrajectoryControl ||
                           modeFlags.quickMode != prevModeFlags_.quickMode ||
                           modeFlags.lbMpcMode != prevModeFlags_.lbMpcMode ||
                           modeFlags.armCtrlMode != prevModeFlags_.armCtrlMode;
  hasPrevModeFlags_ = true;
  prevModeFlags_ = modeFlags;

  if (!configured_) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] not configured");
    return out;
  }

  if (!initialized_ || modeChanged || smoothQ_.size() != currentQ.size()) {
    reset(currentQ, now);
    // reset() 已将 fhanQ_/fhanV_ 同步为 currentQ/zero，更新 out
    out.smoothQ = fhanQ_;
    out.smoothV = fhanV_;
    out.valid = true;
    return out;
  }

  if (!hasTarget_) {
    out.valid = true;
    return out;
  }
  if (target_.targetQ.size() != currentQ.size()) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] target size mismatch, keep previous output");
    out.valid = true;
    return out;
  }

  out.timeout = !target_.msgStamp.isZero() && (now - target_.msgStamp).toSec() > config_.timeoutSec;
  if (out.timeout) {
    out.valid = true;
    return out;
  }

  ensureBackend(static_cast<size_t>(currentQ.size()));
  if (!backend_) {
    out.valid = true;
    return out;
  }

  const bool hasNewTarget = !hasLastAppliedTargetSeq_ || (latestTargetSeq_ != lastAppliedTargetSeq_);
  if (hasNewTarget) {
    Eigen::VectorXd targetV = Eigen::VectorXd::Zero(target_.targetQ.size());
    if (target_.hasTargetV && target_.targetV.size() == target_.targetQ.size()) {
      targetV = target_.targetV;
    }
    bool updateOk = false;
    if (target_.measuredDq.size() == target_.targetQ.size()) {
      updateOk = backend_->updateTarget(target_.targetQ, targetV, target_.measuredDq);
    } else {
      updateOk = backend_->updateTarget(target_.targetQ, targetV);
    }
    if (!updateOk) {
      ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend updateTarget failed, hold previous output");
      out.valid = true;
      return out;
    }
    lastAppliedTargetSeq_ = latestTargetSeq_;
    hasLastAppliedTargetSeq_ = true;
  }

  Eigen::VectorXd nextQ = smoothQ_;
  Eigen::VectorXd nextV = smoothV_;
  if (!backend_->step(nextQ, nextV)) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend step failed, hold previous output");
    out.valid = true;
    return out;
  }
  if (!isFiniteVector(nextQ) || !isFiniteVector(nextV)) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend produced non-finite output");
    out.valid = true;
    return out;
  }

  // 更新后端轨迹状态；fhan 在下一周期顶部以新 smoothQ_ 为参考持续推进
  smoothQ_ = nextQ;
  smoothV_ = nextV;
  out.valid = true;
  return out;
}

bool ArmTrajectoryInterpolator::isFiniteVector(const Eigen::VectorXd& vec) {
  for (Eigen::Index i = 0; i < vec.size(); ++i) {
    if (!std::isfinite(vec[i])) {
      return false;
    }
  }
  return true;
}

bool ArmTrajectoryInterpolator::isEnabled(const ModeFlags& modeFlags) const {
  if (modeFlags.useArmTrajectoryControl) {
    return true;
  }
  const bool quickArm = (modeFlags.quickMode == 2 || modeFlags.quickMode == 3);
  const bool lbMpcValid = (modeFlags.lbMpcMode == 1 || modeFlags.lbMpcMode == 3);
  return quickArm && lbMpcValid;
}

void ArmTrajectoryInterpolator::ensureBackend(size_t dof) {
  if (dof == 0) {
    return;
  }
  if (!backend_) {
    const double cycle = std::clamp(config_.controlCycleSec, 1e-4, 0.02);
    backend_ = std::make_shared<ocs2::mobile_manipulator::HybridRuckigExtrapolator>(static_cast<int>(dof), cycle);
  }

  Eigen::VectorXd maxVel = config_.maxVel;
  Eigen::VectorXd maxAcc = config_.maxAcc;
  Eigen::VectorXd maxJerk = config_.maxJerk;
  Eigen::VectorXd dqRemapK = config_.dqRemapK;
  Eigen::VectorXd dqRemapOffset = config_.dqRemapOffset;
  if (maxVel.size() != static_cast<Eigen::Index>(dof)) maxVel = Eigen::VectorXd::Ones(dof) * 4.0;
  if (maxAcc.size() != static_cast<Eigen::Index>(dof)) maxAcc = Eigen::VectorXd::Ones(dof) * 10.0;
  if (maxJerk.size() != static_cast<Eigen::Index>(dof)) maxJerk = Eigen::VectorXd::Ones(dof) * 50.0;
  if (dqRemapK.size() != static_cast<Eigen::Index>(dof)) dqRemapK = Eigen::VectorXd::Ones(dof);
  if (dqRemapOffset.size() != static_cast<Eigen::Index>(dof)) dqRemapOffset = Eigen::VectorXd::Zero(dof);

  backend_->setKinematicLimits(maxVel.cwiseAbs(), maxAcc.cwiseAbs(), maxJerk.cwiseAbs());
  backend_->setOutputLowpassCutoff(config_.qCutoffHz, config_.vCutoffHz);
  backend_->setAccLowpassCutoff(config_.aCutoffHz);
  backend_->setDqRemapK(dqRemapK);
  backend_->setDqRemapOffset(dqRemapOffset);
}

}  // namespace humanoidController_wheel_wbc
