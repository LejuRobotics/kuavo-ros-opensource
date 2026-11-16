#include "humanoid_wheel_interface/filters/ConstantVelocityCommandKalmanFilter.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace ocs2 {
namespace mobile_manipulator {

namespace {
constexpr double kRawFallbackPositionDelta = 0.15;
constexpr double kRawFallbackVelocityDelta = 0.3;
}  // namespace

ConstantVelocityCommandKalmanFilter::ConstantVelocityCommandKalmanFilter(int dofNum, double cycleTime)
    : dofNum_(dofNum > 0 ? static_cast<size_t>(dofNum) : 1),
      cycleTime_(cycleTime > 1e-6 ? cycleTime : 0.002) {
  if (dofNum <= 0) {
    ROS_WARN_STREAM("[ConstantVelocityCommandKalmanFilter] Invalid dofNum=" << dofNum << ", fallback to 1.");
  }
  if (cycleTime <= 1e-6) {
    ROS_WARN_STREAM("[ConstantVelocityCommandKalmanFilter] Invalid cycleTime=" << cycleTime << ", fallback to 0.002.");
  }
  dofValid_ = (dofNum_ == 14);
  if (!dofValid_) {
    ROS_ERROR_STREAM("[ConstantVelocityCommandKalmanFilter] DoF must be 14, got " << dofNum_ << ".");
    statusMessage_ = "Invalid DoF, expected 14.";
  } else {
    statusMessage_ = "Constructed";
  }

  maxVel_ = Eigen::VectorXd::Ones(dofNum_) * 4.0;
  targetPosition_ = Eigen::VectorXd::Zero(dofNum_);
  targetVelocity_ = Eigen::VectorXd::Zero(dofNum_);
  state_ = Eigen::MatrixXd::Zero(dofNum_, 2);
  covariance_.assign(dofNum_, Matrix2::Identity());
  recovering_.assign(dofNum_, false);
  previousPositionBuffer_ = Eigen::VectorXd::Zero(dofNum_);
  stepPositionBuffer_ = Eigen::VectorXd::Zero(dofNum_);
  stepVelocityBuffer_ = Eigen::VectorXd::Zero(dofNum_);

  observationMatrix_.setIdentity();
  updateSystemMatrices();
}

bool ConstantVelocityCommandKalmanFilter::checkVectorSize(const Eigen::VectorXd& vec, const char* name) const {
  if (vec.size() != static_cast<Eigen::Index>(dofNum_)) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstantVelocityCommandKalmanFilter] " << name << " size mismatch, expected " << dofNum_
                                                                             << ", got " << vec.size());
    return false;
  }
  return true;
}

bool ConstantVelocityCommandKalmanFilter::validateAndReport(const char* source) const {
  if (dofValid_) {
    return true;
  }
  if (!dofErrorLogged_) {
    ROS_ERROR_STREAM("[ConstantVelocityCommandKalmanFilter] " << source << " rejected: DoF must be 14, got " << dofNum_ << ".");
    dofErrorLogged_ = true;
  } else {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[ConstantVelocityCommandKalmanFilter] " << source << " rejected due to invalid DoF.");
  }
  return false;
}

void ConstantVelocityCommandKalmanFilter::updateSystemMatrices(double elapsedSec) {
  const double dt = (std::isfinite(elapsedSec) && elapsedSec > 0.0) ? elapsedSec : cycleTime_;
  predictionDt_ = dt;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  transitionMatrix_.setIdentity();
  transitionMatrix_(0, 1) = dt;

  const double q = std::max(1e-9, processModelNoise_);
  // Discrete covariance from white acceleration noise in constant-velocity model.
  processNoise_.setZero();
  processNoise_(0, 0) = q * dt4 * 0.25;
  processNoise_(0, 1) = q * dt3 * 0.5;
  processNoise_(1, 0) = processNoise_(0, 1);
  processNoise_(1, 1) = q * dt2;

  measurementNoise_.setZero();
  measurementNoise_(0, 0) = std::max(1e-12, measurementQNoise_);
  measurementNoise_(1, 1) = std::max(1e-12, measurementDqNoise_);
  const double scale = std::clamp(fastUpdateRScale_, 1e-3, 1.0);
  measurementNoiseFast_ = measurementNoise_ * scale;
}

void ConstantVelocityCommandKalmanFilter::setKinematicLimits(const Eigen::VectorXd& maxVel) {
  if (!checkVectorSize(maxVel, "maxVel")) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  maxVel_ = maxVel.cwiseAbs();
}

void ConstantVelocityCommandKalmanFilter::setKalmanParameters(double processModelNoise,
                                                              double measurementQNoise,
                                                              double measurementDqNoise,
                                                              double initialPosVar,
                                                              double initialVelVar,
                                                              double fastUpdateRScale) {
  std::lock_guard<std::mutex> lock(mutex_);
  processModelNoise_ = std::max(1e-9, processModelNoise);
  measurementQNoise_ = std::max(1e-12, measurementQNoise);
  measurementDqNoise_ = std::max(1e-12, measurementDqNoise);
  initialPosVar_ = std::max(1e-12, initialPosVar);
  initialVelVar_ = std::max(1e-12, initialVelVar);
  fastUpdateRScale_ = std::clamp(fastUpdateRScale, 1e-3, 1.0);
  updateSystemMatrices();
}

void ConstantVelocityCommandKalmanFilter::reset(const Eigen::VectorXd& initialPosition) {
  if (!checkVectorSize(initialPosition, "initialPosition")) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!validateAndReport("reset")) {
    statusMessage_ = "Reset failed due to invalid DoF";
    initialized_ = false;
    return;
  }

  targetPosition_ = initialPosition;
  targetVelocity_.setZero();

  state_.setZero();
  for (size_t i = 0; i < dofNum_; ++i) {
    state_(static_cast<Eigen::Index>(i), 0) = initialPosition[static_cast<Eigen::Index>(i)];
  }

  Matrix2 initialCov = Matrix2::Zero();
  initialCov(0, 0) = initialPosVar_;
  initialCov(1, 1) = initialVelVar_;
  covariance_.assign(dofNum_, initialCov);
  recovering_.assign(dofNum_, false);
  hasPendingMeasurement_ = false;

  initialized_ = true;
  statusMessage_ = "Ready";
}

bool ConstantVelocityCommandKalmanFilter::updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity) {
  if (!checkVectorSize(targetPosition, "targetPosition") || !checkVectorSize(targetVelocity, "targetVelocity")) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    statusMessage_ = "updateTarget called before reset";
    return false;
  }
  if (!validateAndReport("updateTarget")) {
    statusMessage_ = "updateTarget failed due to invalid DoF";
    return false;
  }
  targetPosition_ = targetPosition;
  targetVelocity_ = targetVelocity;
  hasPendingMeasurement_ = true;
  statusMessage_ = "Target updated";
  return true;
}

void ConstantVelocityCommandKalmanFilter::sanitizeState(Vector2& state) const {
  if (!std::isfinite(state[0])) {
    state[0] = 0.0;
  }
  state[1] = std::isfinite(state[1]) ? state[1] : 0.0;
}

bool ConstantVelocityCommandKalmanFilter::predict(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity, double elapsedSec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    statusMessage_ = "predict called before reset";
    return false;
  }
  if (!validateAndReport("predict")) {
    statusMessage_ = "predict failed due to invalid DoF";
    return false;
  }

  if (outPosition.size() != static_cast<Eigen::Index>(dofNum_)) {
    outPosition.resize(static_cast<Eigen::Index>(dofNum_));
  }
  if (outVelocity.size() != static_cast<Eigen::Index>(dofNum_)) {
    outVelocity.resize(static_cast<Eigen::Index>(dofNum_));
  }
  updateSystemMatrices(elapsedSec);
  previousPositionBuffer_ = state_.col(0);
  outPosition.setZero();
  outVelocity.setZero();
  stepPositionBuffer_.setZero();
  stepVelocityBuffer_.setZero();

  for (size_t i = 0; i < dofNum_; ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    Vector2 state = state_.row(idx).transpose();
    Matrix2 covariance = covariance_[i];

    // Constant-velocity prediction. When no new rawV arrives, velocity remains unchanged.
    state = transitionMatrix_ * state;
    covariance = transitionMatrix_ * covariance * transitionMatrix_.transpose() + processNoise_;

    const double velLimit = maxVel_[idx];
    sanitizeState(state);
    state[1] = std::clamp(state[1], -velLimit, velLimit);
    if (recovering_[i]) {
      // Recovery is command tracking, not a new Kalman measurement. Continue even
      // when the caller deduplicates a static target and only runs predict().
      const double previousQ = previousPositionBuffer_[idx];
      const double maxDelta = velLimit * predictionDt_;
      const double rawQ = targetPosition_[idx];
      state[0] = std::clamp(rawQ, previousQ - maxDelta, previousQ + maxDelta);
      recovering_[i] = (state[0] != rawQ);
      state[1] = recovering_[i] ? (state[0] - previousQ) / predictionDt_
                                : std::clamp(targetVelocity_[idx], -velLimit, velLimit);
    }

    state_.row(idx) = state.transpose();
    covariance_[i] = covariance;
    stepPositionBuffer_[idx] = state[0];
    stepVelocityBuffer_[idx] = state[1];
  }
  outPosition = stepPositionBuffer_;
  outVelocity = stepVelocityBuffer_;
  ++predictCount_;
  statusMessage_ = "Predict ok";
  return true;
}

bool ConstantVelocityCommandKalmanFilter::update() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    statusMessage_ = "update called before reset";
    return false;
  }
  if (!validateAndReport("update")) {
    statusMessage_ = "update failed due to invalid DoF";
    return false;
  }
  if (!hasPendingMeasurement_) {
    statusMessage_ = "Update skipped (no pending measurement)";
    return true;
  }

  for (size_t i = 0; i < dofNum_; ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    Vector2 state = state_.row(idx).transpose();
    Matrix2 covariance = covariance_[i];

    const double commandQ = targetPosition_[idx];
    const double commandDq = targetVelocity_[idx];
    const Eigen::Vector2d measurement(commandQ, commandDq);
    const Eigen::Vector2d innovation = measurement - observationMatrix_ * state;
    Matrix2 innovationCov = observationMatrix_ * covariance * observationMatrix_.transpose() + measurementNoiseFast_;
    innovationCov = 0.5 * (innovationCov + innovationCov.transpose());
    const Eigen::Matrix<double, 2, 2> pht = covariance * observationMatrix_.transpose();
    Eigen::LDLT<Matrix2> ldlt(innovationCov);
    if (ldlt.info() != Eigen::Success) {
      innovationCov += 1e-9 * Matrix2::Identity();
      ldlt.compute(innovationCov);
    }
    if (ldlt.info() != Eigen::Success) {
      statusMessage_ = "Innovation covariance solve failed";
      return false;
    }
    const Eigen::Matrix<double, 2, 2> kalmanGain = ldlt.solve(pht.transpose()).transpose();

    state += kalmanGain * innovation;
    const Matrix2 iMinusKh = identityMatrix_ - kalmanGain * observationMatrix_;
    covariance =
        iMinusKh * covariance * iMinusKh.transpose() + kalmanGain * measurementNoiseFast_ * kalmanGain.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    for (int d = 0; d < 2; ++d) {
      covariance(d, d) = std::max(covariance(d, d), 1e-12);
    }

    const double velLimit = maxVel_[idx];
    sanitizeState(state);
    state[1] = std::clamp(state[1], -velLimit, velLimit);

    state_.row(idx) = state.transpose();
    covariance_[i] = covariance;
  }

  hasPendingMeasurement_ = false;
  ++updateCount_;
  statusMessage_ = "Update ok";
  return true;
}

bool ConstantVelocityCommandKalmanFilter::step(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity, double elapsedSec) {
  if (!predict(outPosition, outVelocity, elapsedSec)) {
    return false;
  }
  if (!update()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t fallbackPosCount = 0;
    size_t fallbackVelCount = 0;
    for (size_t i = 0; i < dofNum_; ++i) {
      const Eigen::Index idx = static_cast<Eigen::Index>(i);
      const double interpolatedQ = state_(idx, 0);
      const double rawQ = targetPosition_[idx];
      const bool fallbackQ = (std::abs(rawQ - interpolatedQ) > kRawFallbackPositionDelta);

      const double interpolatedDq = state_(idx, 1);
      const double rawDq = targetVelocity_[idx];
      const bool fallbackDq = (std::abs(rawDq - interpolatedDq) > kRawFallbackVelocityDelta);
      const bool fallbackAny = (fallbackQ || fallbackDq);
      if (fallbackAny || recovering_[i]) {
        // Recover toward raw, then apply the same position slew limit as normal updates.
        // Keep the latent state aligned with the bounded output.
        state_(idx, 0) = rawQ;
        state_(idx, 1) = std::clamp(rawDq, -maxVel_[idx], maxVel_[idx]);
        outPosition[idx] = rawQ;
        outVelocity[idx] = state_(idx, 1);
        if (fallbackQ) ++fallbackPosCount;
        if (fallbackDq) ++fallbackVelCount;
      } else {
        outPosition[idx] = interpolatedQ;
        outVelocity[idx] = interpolatedDq;
      }
      const double previousQ = previousPositionBuffer_[idx];
      const double maxDelta = maxVel_[idx] * predictionDt_;
      const double boundedQ = std::clamp(outPosition[idx], previousQ - maxDelta, previousQ + maxDelta);
      if (boundedQ != outPosition[idx]) {
        recovering_[i] = true;
        outPosition[idx] = boundedQ;
        outVelocity[idx] = (boundedQ - previousQ) / predictionDt_;
        state_(idx, 0) = outPosition[idx];
        state_(idx, 1) = outVelocity[idx];
      }
    }
    if (fallbackPosCount > 0 || fallbackVelCount > 0) {
      statusMessage_ = "Step ok with raw fallback";
      ROS_WARN_STREAM_THROTTLE(1.0, "[ConstantVelocityCommandKalmanFilter] Rate-limited raw recovery triggered: q="
                                        << fallbackPosCount << " DoF(s) (th="
                                        << kRawFallbackPositionDelta << "), dq="
                                        << fallbackVelCount << " DoF(s) (th="
                                        << kRawFallbackVelocityDelta << ")");
    } else {
      statusMessage_ = "Step ok";
    }
  }
  return true;
}

bool ConstantVelocityCommandKalmanFilter::isOperational() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_ && dofValid_;
}

std::string ConstantVelocityCommandKalmanFilter::statusMessage() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return statusMessage_;
}

uint64_t ConstantVelocityCommandKalmanFilter::predictCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return predictCount_;
}

uint64_t ConstantVelocityCommandKalmanFilter::updateCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return updateCount_;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
