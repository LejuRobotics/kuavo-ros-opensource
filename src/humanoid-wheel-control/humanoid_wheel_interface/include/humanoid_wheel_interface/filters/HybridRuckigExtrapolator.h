#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <ros/ros.h>
#include "ruckig/ruckig.hpp"

namespace ocs2 {
namespace mobile_manipulator {

class HybridRuckigExtrapolator {
public:
    explicit HybridRuckigExtrapolator(int dofNum, double cycleTime = 0.002);
    ~HybridRuckigExtrapolator() = default;

    void setKinematicLimits(const Eigen::VectorXd& maxVel, const Eigen::VectorXd& maxAcc, const Eigen::VectorXd& maxJerk);
    void setOutputLowpassCutoff(double qCutoffHz, double vCutoffHz);
    /** Second-order lowpass on measured acc (from dq difference); 0 disables filtering (raw acc). */
    void setAccLowpassCutoff(double aCutoffHz);
    /** Per-joint gain k for dq remap; x = |dq_lp[i]|. */
    void setDqRemapK(const Eigen::VectorXd& dqRemapK);
    /** Per-joint offset in rad/s for dq remap; x = |dq_lp[i]|. */
    void setDqRemapOffset(const Eigen::VectorXd& dqRemapOffset);
    void reset(const Eigen::VectorXd& initialPosition);
    bool updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity);
    /** Sets target from args; uses measured dq for current_velocity and input.target_velocity (sensor semantics).
     *  Measured acc is computed inside: diff(dq)/dt + second-order lowpass -> current_acceleration. */
    bool updateTarget(const Eigen::VectorXd& targetPosition, const Eigen::VectorXd& targetVelocity,
                      const Eigen::VectorXd& currentMeasuredVelocity);
    bool step(Eigen::VectorXd& outPosition, Eigen::VectorXd& outVelocity);

private:
    struct BiquadCoeff {
        double b0{1.0};
        double b1{0.0};
        double b2{0.0};
        double a1{0.0};
        double a2{0.0};
        bool enabled{false};
    };

    static BiquadCoeff computeSecondOrderLowpassCoeff(double cutoffHz, double sampleTime);
    static Eigen::VectorXd applySecondOrderLowpass(const Eigen::VectorXd& x, const BiquadCoeff& coeff, Eigen::VectorXd& x1,
                                                   Eigen::VectorXd& x2, Eigen::VectorXd& y1, Eigen::VectorXd& y2);

    size_t dofNum_{1};
    double cycleTime_{0.002};
    mutable std::mutex mutex_;
    bool initialized_{false};

    ruckig::Ruckig<ruckig::DynamicDOFs> otg_;
    ruckig::InputParameter<ruckig::DynamicDOFs> input_;
    ruckig::OutputParameter<ruckig::DynamicDOFs> output_;

    Eigen::VectorXd currentExtrapolatedPos_;
    int microStepIndex_{0};

    std::deque<Eigen::VectorXd> velHistoryFifo_;
    std::vector<Eigen::VectorXd> activeVelProfile_;

    BiquadCoeff qLowpassCoeff_;
    BiquadCoeff vLowpassCoeff_;
    Eigen::VectorXd qX1_, qX2_, qY1_, qY2_;
    Eigen::VectorXd vX1_, vX2_, vY1_, vY2_;
    Eigen::VectorXd prevFilteredPos_;
    bool hasPrevFilteredPos_{false};
    Eigen::VectorXd dqRemapK_;
    Eigen::VectorXd dqRemapOffset_;
    Eigen::VectorXd remapedDqScale_;

    BiquadCoeff aLowpassCoeff_;
    Eigen::VectorXd aX1_, aX2_, aY1_, aY2_;
    Eigen::VectorXd prevMeasuredDq_;
    bool hasPrevMeasuredDq_{false};
    double smoothedMeasuredDt_{0.0};
    double prevMeasuredDqStampSec_{0.0};
    bool hasSmoothedMeasuredDt_{false};
    bool hasPrevMeasuredDqStamp_{false};
};

}  // namespace mobile_manipulator
}  // namespace ocs2
