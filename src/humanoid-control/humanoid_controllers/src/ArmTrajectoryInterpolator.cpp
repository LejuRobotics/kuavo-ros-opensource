#include "humanoid_controllers/ArmTrajectoryInterpolator.h"

#include <algorithm>
#include <cmath>

#include "humanoid_wheel_interface/filters/ConstantVelocityCommandKalmanFilter.h"

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
  lastTargetStamp_ = now;
  hasLastAppliedTargetSeq_ = false;
  initialized_ = true;
}

void ArmTrajectoryInterpolator::ingestRawTarget(const ros::Time& now, const Eigen::VectorXd& targetQ,
                                                const Eigen::VectorXd& targetV) {
  if (targetQ.size() == 0 || !isFiniteVector(targetQ)) {
    return;
  }

  Eigen::VectorXd mergedTargetV = Eigen::VectorXd::Zero(targetQ.size());
  bool hasTargetV = false;
  if (targetV.size() == targetQ.size() && isFiniteVector(targetV)) {
    mergedTargetV = targetV;
    hasTargetV = true;
  }

  // Estimate velocity only when targetV is truly missing/invalid.
  bool shouldEstimateVel = !hasTargetV;
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
  for (Eigen::Index i = 0; i < mergedTargetV.size(); ++i) {
    mergedTargetV[i] = std::clamp(mergedTargetV[i], -config_.kalmanVLimit, config_.kalmanVLimit);
  }
  const double alpha = std::clamp(config_.targetVAlpha, 0.0, 1.0);
  if (alpha < 1.0 && lastRawTargetV_.size() == mergedTargetV.size()) {
    mergedTargetV = alpha * mergedTargetV + (1.0 - alpha) * lastRawTargetV_;
  }

  const bool targetChanged =
      !hasLastRawTarget_ || lastRawTargetQ_.size() != targetQ.size() ||
      (targetQ - lastRawTargetQ_).cwiseAbs().maxCoeff() > std::max(1e-9, config_.targetChangeEps) ||
      lastRawTargetV_.size() != mergedTargetV.size() ||
      (mergedTargetV - lastRawTargetV_).cwiseAbs().maxCoeff() > std::max(1e-9, config_.targetChangeEps);
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
  out.smoothQ = smoothQ_;
  out.smoothV = smoothV_;

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
    out.smoothQ = smoothQ_;
    out.smoothV = smoothV_;
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

  const bool hasNewTargetRaw = !hasLastAppliedTargetSeq_ || (latestTargetSeq_ != lastAppliedTargetSeq_);
  if (hasNewTargetRaw) {
    ++rawUpdateCandidateCount_;
  }
  const double updatePeriodSec = std::max(config_.referenceUpdatePeriodSec, config_.controlCycleSec);
  const bool periodGatePass = !hasLastAppliedTargetStamp_ ||
                              (now - lastAppliedTargetStamp_).toSec() >= updatePeriodSec;
  const bool hasNewTarget = hasNewTargetRaw && (config_.immediateUpdateOnNewTarget || periodGatePass);
  if (hasNewTargetRaw && !hasNewTarget) {
    ++gateBlockedCount_;
  }
  if (hasNewTarget) {
    Eigen::VectorXd targetV = Eigen::VectorXd::Zero(target_.targetQ.size());
    if (target_.hasTargetV && target_.targetV.size() == target_.targetQ.size()) {
      targetV = target_.targetV;
    }
    const bool updateOk = backend_->updateTarget(target_.targetQ, targetV);
    if (!updateOk) {
      ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend updateTarget failed (%s), hold previous output",
                        backend_->statusMessage().c_str());
      out.valid = true;
      return out;
    }
    lastAppliedTargetSeq_ = latestTargetSeq_;
    hasLastAppliedTargetSeq_ = true;
    lastAppliedTargetStamp_ = now;
    hasLastAppliedTargetStamp_ = true;
    ++gatedUpdateCount_;
  }

  Eigen::VectorXd nextQ = smoothQ_;
  Eigen::VectorXd nextV = smoothV_;
  if (hasNewTarget) {
    if (!backend_->step(nextQ, nextV)) {
      ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend step failed (%s), hold previous output",
                        backend_->statusMessage().c_str());
      out.valid = true;
      return out;
    }
  } else {
    if (!backend_->predict(nextQ, nextV)) {
      ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend predict failed (%s), hold previous output",
                        backend_->statusMessage().c_str());
      out.valid = true;
      return out;
    }
  }
  if (!isFiniteVector(nextQ) || !isFiniteVector(nextV)) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend produced non-finite output");
    out.valid = true;
    return out;
  }

  // Update smoothed command state from Kalman backend.
  smoothQ_ = nextQ;
  smoothV_ = nextV;
  if (backend_) {
    const uint64_t predictCount = backend_->predictCount();
    const uint64_t updateCount = backend_->updateCount();
    const double ratio = updateCount > 0 ? static_cast<double>(predictCount) / static_cast<double>(updateCount) : 0.0;
    ROS_INFO_THROTTLE(1.0,
                      "[ArmTrajectoryInterpolator] cmd-only multirate stats: predict=%lu update=%lu ratio=%.2f "
                      "rawUpdateCandidate=%lu gatedUpdate=%lu gateBlocked=%lu",
                      static_cast<unsigned long>(predictCount), static_cast<unsigned long>(updateCount), ratio,
                      static_cast<unsigned long>(rawUpdateCandidateCount_), static_cast<unsigned long>(gatedUpdateCount_),
                      static_cast<unsigned long>(gateBlockedCount_));
  }
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
    backend_ = std::make_shared<ocs2::mobile_manipulator::ConstantVelocityCommandKalmanFilter>(static_cast<int>(dof), cycle);
  }

  const bool limitValid = std::isfinite(config_.kalmanVLimit) && config_.kalmanVLimit > 0.0;
  if (!limitValid) {
    ROS_ERROR_THROTTLE(1.0, "[ArmTrajectoryInterpolator] Invalid kalman limit in armTrajInterpKinematicLimit, disable backend.");
    backend_.reset();
    return;
  }
  const Eigen::VectorXd maxVel = Eigen::VectorXd::Ones(dof) * config_.kalmanVLimit;

  const double refDt = std::max(config_.referenceUpdatePeriodSec, config_.controlCycleSec);
  const double derivedAcc = (refDt > 1e-6) ? (config_.kalmanVLimit / refDt) : config_.kalmanVLimit;
  const double nominalAcc = (std::isfinite(derivedAcc) && derivedAcc > 0.0) ? std::clamp(derivedAcc, 5.0, 80.0) : 10.0;
  const double processNoise = std::clamp(0.1 * nominalAcc * nominalAcc, 1e-9, 400.0);

  backend_->setKinematicLimits(maxVel.cwiseAbs());
  backend_->setKalmanParameters(processNoise,
                                config_.kalmanMeasurementQNoise,
                                config_.kalmanMeasurementDqNoise,
                                config_.kalmanInitialPosVar,
                                config_.kalmanInitialVelVar,
                                config_.fastUpdateRScale);
  if (!backend_->isOperational()) {
    ROS_WARN_THROTTLE(1.0, "[ArmTrajectoryInterpolator] backend not operational after configure (%s)",
                      backend_->statusMessage().c_str());
  }
}

}  // namespace humanoidController_wheel_wbc
