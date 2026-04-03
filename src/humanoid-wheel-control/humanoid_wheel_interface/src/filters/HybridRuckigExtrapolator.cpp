#include "humanoid_wheel_interface/filters/HybridRuckigExtrapolator.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace ocs2 {
namespace mobile_manipulator {

HybridRuckigExtrapolator::HybridRuckigExtrapolator(int dofNum, double cycleTime)
    : dofNum_(dofNum > 0 ? static_cast<size_t>(dofNum) : 1),
      cycleTime_(cycleTime > 1e-6 ? cycleTime : 0.002),
      otg_(dofNum_, cycleTime_),
      input_(dofNum_),
      output_(dofNum_) {
    if (dofNum <= 0) {
        ROS_WARN_STREAM("Invalid DOF number: " << dofNum << ", fallback to 1");
    }
    if (cycleTime <= 1e-6) {
        ROS_WARN_STREAM("Invalid cycleTime: " << cycleTime << ", fallback to 0.002");
    }

    currentExtrapolatedPos_ = Eigen::VectorXd::Zero(dofNum_);
    velHistoryFifo_.resize(5, Eigen::VectorXd::Zero(dofNum_));
    activeVelProfile_.resize(5, Eigen::VectorXd::Zero(dofNum_));
    qX1_ = Eigen::VectorXd::Zero(dofNum_);
    qX2_ = Eigen::VectorXd::Zero(dofNum_);
    qY1_ = Eigen::VectorXd::Zero(dofNum_);
    qY2_ = Eigen::VectorXd::Zero(dofNum_);
    vX1_ = Eigen::VectorXd::Zero(dofNum_);
    vX2_ = Eigen::VectorXd::Zero(dofNum_);
    vY1_ = Eigen::VectorXd::Zero(dofNum_);
    vY2_ = Eigen::VectorXd::Zero(dofNum_);
    prevFilteredPos_ = Eigen::VectorXd::Zero(dofNum_);
    aX1_ = Eigen::VectorXd::Zero(dofNum_);
    aX2_ = Eigen::VectorXd::Zero(dofNum_);
    aY1_ = Eigen::VectorXd::Zero(dofNum_);
    aY2_ = Eigen::VectorXd::Zero(dofNum_);
    prevMeasuredDq_ = Eigen::VectorXd::Zero(dofNum_);
    smoothedMeasuredDt_ = cycleTime_;
    dqRemapK_ = Eigen::VectorXd::Ones(dofNum_);
    dqRemapOffset_ = Eigen::VectorXd::Zero(dofNum_);
    remapedDqScale_ = Eigen::VectorXd::Zero(dofNum_);
}

void HybridRuckigExtrapolator::setKinematicLimits(const Eigen::VectorXd& maxVel, const Eigen::VectorXd& maxAcc,
                                                  const Eigen::VectorXd& maxJerk) {
    if (maxVel.size() != static_cast<Eigen::Index>(dofNum_) ||
        maxAcc.size() != static_cast<Eigen::Index>(dofNum_) ||
        maxJerk.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_ERROR_STREAM("Kinematic limits dimension mismatch. expected " << dofNum_ << ", got v="
                         << maxVel.size() << ", a=" << maxAcc.size() << ", j=" << maxJerk.size());
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < dofNum_; ++i) {
        input_.max_velocity[i] = maxVel[i];
        input_.max_acceleration[i] = maxAcc[i];
        input_.max_jerk[i] = maxJerk[i];
    }
}

void HybridRuckigExtrapolator::setOutputLowpassCutoff(double qCutoffHz, double vCutoffHz) {
    std::lock_guard<std::mutex> lock(mutex_);
    qLowpassCoeff_ = computeSecondOrderLowpassCoeff(qCutoffHz, cycleTime_);
    vLowpassCoeff_ = computeSecondOrderLowpassCoeff(vCutoffHz, cycleTime_);
}

void HybridRuckigExtrapolator::setAccLowpassCutoff(double aCutoffHz) {
    std::lock_guard<std::mutex> lock(mutex_);
    aLowpassCoeff_ = computeSecondOrderLowpassCoeff(aCutoffHz, cycleTime_);
}

void HybridRuckigExtrapolator::setDqRemapK(const Eigen::VectorXd& dqRemapK) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dqRemapK.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_WARN_STREAM("Invalid dq remap k size: " << dqRemapK.size() << ", expected " << dofNum_ << ", fallback to ones.");
        dqRemapK_.setOnes();
        return;
    }
    for (Eigen::Index i = 0; i < dqRemapK.size(); ++i) {
        if (!std::isfinite(dqRemapK[i]) || dqRemapK[i] < 0.0) {
            ROS_WARN_STREAM("Invalid dq remap k[" << i << "]: " << dqRemapK[i] << ", fallback to ones.");
            dqRemapK_.setOnes();
            return;
        }
    }
    dqRemapK_ = dqRemapK;
}

void HybridRuckigExtrapolator::setDqRemapOffset(const Eigen::VectorXd& dqRemapOffset) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dqRemapOffset.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_WARN_STREAM("Invalid dq remap offset size: " << dqRemapOffset.size() << ", expected " << dofNum_ << ", fallback to zeros.");
        dqRemapOffset_.setZero();
        return;
    }
    for (Eigen::Index i = 0; i < dqRemapOffset.size(); ++i) {
        if (!std::isfinite(dqRemapOffset[i])) {
            ROS_WARN_STREAM("Invalid dq remap offset[" << i << "]: " << dqRemapOffset[i] << ", fallback to zeros.");
            dqRemapOffset_.setZero();
            return;
        }
    }
    dqRemapOffset_ = dqRemapOffset;
}

void HybridRuckigExtrapolator::reset(const Eigen::VectorXd& initialPosition) {
    if (initialPosition.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_ERROR_STREAM("Reset position dimension mismatch. expected " << dofNum_ << ", got " << initialPosition.size());
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < dofNum_; ++i) {
        input_.current_position[i] = initialPosition[i];
        input_.current_velocity[i] = 0.0;
        input_.current_acceleration[i] = 0.0;
        input_.target_position[i] = initialPosition[i];
        input_.target_velocity[i] = 0.0;
        input_.target_acceleration[i] = 0.0;
    }

    currentExtrapolatedPos_ = initialPosition;
    std::fill(velHistoryFifo_.begin(), velHistoryFifo_.end(), Eigen::VectorXd::Zero(dofNum_));
    std::fill(activeVelProfile_.begin(), activeVelProfile_.end(), Eigen::VectorXd::Zero(dofNum_));
    microStepIndex_ = 0;
    qX1_ = initialPosition;
    qX2_ = initialPosition;
    qY1_ = initialPosition;
    qY2_ = initialPosition;
    vX1_.setZero();
    vX2_.setZero();
    vY1_.setZero();
    vY2_.setZero();
    prevFilteredPos_ = initialPosition;
    hasPrevFilteredPos_ = true;
    aX1_.setZero();
    aX2_.setZero();
    aY1_.setZero();
    aY2_.setZero();
    prevMeasuredDq_.setZero();
    hasPrevMeasuredDq_ = false;
    smoothedMeasuredDt_ = cycleTime_;
    prevMeasuredDqStampSec_ = 0.0;
    hasSmoothedMeasuredDt_ = false;
    hasPrevMeasuredDqStamp_ = false;
    dqRemapK_.setOnes();
    dqRemapOffset_.setZero();
    remapedDqScale_.setZero();
    initialized_ = true;
}

bool HybridRuckigExtrapolator::updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity) {
    if (targetPosition.size() != static_cast<Eigen::Index>(dofNum_) ||
        targetVelocity.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Target dimension mismatch. expected " << dofNum_ << ", got p="
                                   << targetPosition.size() << ", v=" << targetVelocity.size());
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return false;
    }

    for (size_t i = 0; i < dofNum_; ++i) {
        input_.target_position[i] = targetPosition[i];
        input_.target_velocity[i] = targetVelocity[i];
    }
    // No sensor dq path: reset measured-acc chain to avoid stale differentiation.
    aX1_.setZero();
    aX2_.setZero();
    aY1_.setZero();
    aY2_.setZero();
    prevMeasuredDq_.setZero();
    hasPrevMeasuredDq_ = false;
    smoothedMeasuredDt_ = cycleTime_;
    prevMeasuredDqStampSec_ = 0.0;
    hasSmoothedMeasuredDt_ = false;
    hasPrevMeasuredDqStamp_ = false;

    std::copy(velHistoryFifo_.begin(), velHistoryFifo_.end(), activeVelProfile_.begin());
    currentExtrapolatedPos_ = targetPosition;
    microStepIndex_ = 0;
    return true;
}

bool HybridRuckigExtrapolator::updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity,
                                          const Eigen::VectorXd& currentMeasuredVelocity) {
    if (targetPosition.size() != static_cast<Eigen::Index>(dofNum_) ||
        targetVelocity.size() != static_cast<Eigen::Index>(dofNum_) ||
        currentMeasuredVelocity.size() != static_cast<Eigen::Index>(dofNum_)) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Target dimension mismatch. expected " << dofNum_ << ", got p="
                                   << targetPosition.size() << ", tv=" << targetVelocity.size()
                                   << ", dq=" << currentMeasuredVelocity.size());
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return false;
    }

    constexpr double kDtSmoothAlpha = 0.05;  // Smooth message interval jitter.
    const double nowSec =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    double measuredDt = cycleTime_;
    if (hasPrevMeasuredDqStamp_) {
        measuredDt = nowSec - prevMeasuredDqStampSec_;
    }
    prevMeasuredDqStampSec_ = nowSec;
    hasPrevMeasuredDqStamp_ = true;
    if (!std::isfinite(measuredDt) || measuredDt <= 1e-6) {
        measuredDt = cycleTime_;
    }
    measuredDt = std::clamp(measuredDt, 0.5 * cycleTime_, 0.1);
    if (!hasSmoothedMeasuredDt_) {
        smoothedMeasuredDt_ = measuredDt;
        hasSmoothedMeasuredDt_ = true;
    } else {
        smoothedMeasuredDt_ = kDtSmoothAlpha * measuredDt + (1.0 - kDtSmoothAlpha) * smoothedMeasuredDt_;
    }

    Eigen::VectorXd rawAcc = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dofNum_));
    if (hasPrevMeasuredDq_) {
        rawAcc = (currentMeasuredVelocity - prevMeasuredDq_) / std::max(6e-3, smoothedMeasuredDt_);
        // print smoothedMeasuredDt_
        // ROS_INFO_STREAM("smoothedMeasuredDt_: " << smoothedMeasuredDt_);
    }
    Eigen::VectorXd filteredAcc = applySecondOrderLowpass(rawAcc, aLowpassCoeff_, aX1_, aX2_, aY1_, aY2_);

    for (size_t i = 0; i < dofNum_; ++i) {
        input_.target_position[i] = targetPosition[i];
        input_.target_velocity[i] = currentMeasuredVelocity[i];
        // input_.current_velocity[i] = currentMeasuredVelocity[i];
        // input_.target_velocity[i] = 0.0;
        // input_.current_acceleration[i] = filteredAcc[static_cast<Eigen::Index>(i)];
        // input_.target_acceleration[i] = filteredAcc[static_cast<Eigen::Index>(i)];
        input_.current_acceleration[i] = 0.0;
    }

    prevMeasuredDq_ = currentMeasuredVelocity;
    hasPrevMeasuredDq_ = true;

    std::copy(velHistoryFifo_.begin(), velHistoryFifo_.end(), activeVelProfile_.begin());
    currentExtrapolatedPos_ = targetPosition;
    microStepIndex_ = 0;
    return true;
}

bool HybridRuckigExtrapolator::step(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity) {
    constexpr double kVelocityDiffDt = 0.002;  // 2 ms
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return false;
    }

    auto result = otg_.update(input_, output_);
    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
        Eigen::VectorXd currentRuckigV =
            Eigen::Map<const Eigen::VectorXd>(output_.new_velocity.data(), static_cast<Eigen::Index>(dofNum_));
        velHistoryFifo_.push_back(currentRuckigV);
        velHistoryFifo_.pop_front();
        output_.pass_to_input(input_);
    }

    Eigen::VectorXd vExtrapolate = Eigen::VectorXd::Zero(dofNum_);
    if (microStepIndex_ < static_cast<int>(activeVelProfile_.size())) {
        vExtrapolate = activeVelProfile_[microStepIndex_];
        microStepIndex_++;
    } else {
        vExtrapolate = activeVelProfile_.back();
    }

    currentExtrapolatedPos_ += vExtrapolate * cycleTime_;
    Eigen::VectorXd currentFilteredPos = applySecondOrderLowpass(currentExtrapolatedPos_, qLowpassCoeff_, qX1_, qX2_, qY1_, qY2_);

    outPosition = currentFilteredPos;
    Eigen::VectorXd rawDiffVelocity = Eigen::VectorXd::Zero(dofNum_);
    if (!hasPrevFilteredPos_) {
        hasPrevFilteredPos_ = true;
    } else {
        rawDiffVelocity = (currentFilteredPos - prevFilteredPos_) / kVelocityDiffDt;
    }
    Eigen::VectorXd dqLp = applySecondOrderLowpass(rawDiffVelocity, vLowpassCoeff_, vX1_, vX2_, vY1_, vY2_);
    constexpr double kRemapDenom = 3.14159265358979323846;
    outVelocity.resize(static_cast<Eigen::Index>(dofNum_));
    for (Eigen::Index i = 0; i < dqLp.size(); ++i) {
        const double absDq = std::abs(dqLp[i]);
        remapedDqScale_[i] = std::atan(dqRemapK_[i] * (absDq - dqRemapOffset_[i])) / kRemapDenom + 0.5;
        outVelocity[i] = remapedDqScale_[i] * dqLp[i];
    }
    prevFilteredPos_ = currentFilteredPos;
    return true;
}

HybridRuckigExtrapolator::BiquadCoeff HybridRuckigExtrapolator::computeSecondOrderLowpassCoeff(double cutoffHz, double sampleTime) {
    BiquadCoeff coeff;
    if (cutoffHz <= 0.0 || sampleTime <= 1e-9) {
        return coeff;
    }

    const double fs = 1.0 / sampleTime;
    const double nyquist = 0.5 * fs;
    const double fc = std::min(cutoffHz, 0.95 * nyquist);
    if (fc <= 1e-6) {
        return coeff;
    }

    constexpr double kSqrt2 = 1.4142135623730951;
    constexpr double kPi = 3.14159265358979323846;
    const double k = std::tan(kPi * fc / fs);
    const double k2 = k * k;
    const double norm = 1.0 / (1.0 + kSqrt2 * k + k2);

    coeff.b0 = k2 * norm;
    coeff.b1 = 2.0 * coeff.b0;
    coeff.b2 = coeff.b0;
    coeff.a1 = 2.0 * (k2 - 1.0) * norm;
    coeff.a2 = (1.0 - kSqrt2 * k + k2) * norm;
    coeff.enabled = true;
    return coeff;
}

Eigen::VectorXd HybridRuckigExtrapolator::applySecondOrderLowpass(const Eigen::VectorXd& x, const BiquadCoeff& coeff, Eigen::VectorXd& x1,
                                                                  Eigen::VectorXd& x2, Eigen::VectorXd& y1, Eigen::VectorXd& y2) {
    if (!coeff.enabled) {
        return x;
    }

    Eigen::VectorXd y = coeff.b0 * x + coeff.b1 * x1 + coeff.b2 * x2 - coeff.a1 * y1 - coeff.a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
