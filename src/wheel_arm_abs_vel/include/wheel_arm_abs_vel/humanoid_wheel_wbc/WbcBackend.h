#pragma once
#include "wheel_arm_abs_vel/humanoid_wheel_wbc/DynamicsBackend.h"
#include "wheel_arm_abs_vel/humanoid_wheel_wbc/absolute/AbsoluteWeightedWbc.h"

namespace ocs2 { namespace mobile_manipulator {

// Adapter only: preserve the original tasks, solver and feedback unchanged.
class WbcBackend final : public DynamicsBackend {
 public:
  WbcBackend(std::shared_ptr<AbsoluteWeightedWbc> wbc, size_t joints, size_t arms)
      : wbc_(std::move(wbc)), joints_(joints), arms_(arms) {}
  const char* name() const override { return "wbc"; }
  DynamicsResult update(const DynamicsReference& r, const SystemObservation& m,
                        double, double) override {
    if (r.wholeBodyAcceleration)
      wbc_->setJointAccelerationFeedforward(r.acceleration, r.accelerationValid);
    else
      wbc_->setArmAccelerationFeedforward(r.acceleration.tail(arms_), r.accelerationValid);
    wbc_->setTaskAuditReference(r.session, r.generation, r.sequence, r.epoch,
                               r.atomicReference, r.accelerationGain);
    const vector_t x = wbc_->update(r.q, r.v, m);
    DynamicsResult result;
    if (x.size() >= static_cast<Eigen::Index>(joints_) && x.allFinite()) {
      result.torque = x.tail(joints_);
      result.rawTorque = result.torque;
      result.solvedAcceleration = x.head(m.state.size());
      result.valid = true;
      result.reason = "ok";
    } else result.reason = "invalid_wbc_solution";
    return result;
  }
 private:
  std::shared_ptr<AbsoluteWeightedWbc> wbc_;
  size_t joints_, arms_;
};
}}
