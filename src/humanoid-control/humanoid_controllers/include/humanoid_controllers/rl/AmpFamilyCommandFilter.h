#pragma once

#include "humanoid_controllers/rl/RlGaitReceiver.h"
#include "humanoid_controllers/rl/ForwardVelocitySmoother.h"
#include <algorithm>
#include <limits>

namespace humanoid_controller {

// Family-specific RUN envelope and retirement limits. Raw receiver input remains
// available for switching intent; only the executed command is shaped here.
class AmpFamilyCommandFilter final : public ocs2::humanoid::GaitCommandFilter {
 public:
  void setThermalVelocityLimit(double limit) {
    std::lock_guard<std::mutex> lock(mutex_);
    thermal_limit_ = limit;
  }
  void setRunToWalkSpeedLimit(double limit) {
    std::lock_guard<std::mutex> lock(mutex_);
    retirement_limit_ = limit;
  }
  void setForwardVelocitySmoothing(double decel_tau, double accel_tau = 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    smoother_.configure(decel_tau, accel_tau);
  }
  void resumeExternalVelocityAfterThermalExit() {
    std::lock_guard<std::mutex> lock(mutex_);
    thermal_limit_ = std::numeric_limits<double>::infinity();
    smoother_.reset();
  }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    smoother_.reset();
  }
  geometry_msgs::Twist update(const geometry_msgs::Twist& input, double time, bool bypass) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto output = input;
    output.linear.x = std::clamp(output.linear.x, -thermal_limit_, thermal_limit_);
    output.linear.x = std::min(output.linear.x, retirement_limit_);
    if (thermal_limit_ == 0) {
      output.linear.y = output.linear.z = 0;
      output.angular.x = output.angular.y = output.angular.z = 0;
    }
    // Reversal, stance and execution gates must never retain a forward tail.
    if (bypass) smoother_.reset();
    else output.linear.x = smoother_.update(output.linear.x, time);
    return output;
  }
  ocs2::humanoid::CommandDataRL filterOutput(const ocs2::humanoid::CommandDataRL& input) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto output = input;
    if (thermal_limit_ == 0) {
      output.cmdVelLineX_ = output.cmdVelLineY_ = output.cmdVelLineZ_ = 0;
      output.cmdVelAngularX_ = output.cmdVelAngularY_ = output.cmdVelAngularZ_ = 0;
    }
    return output;
  }
  void inherit(const ocs2::humanoid::GaitCommandFilter* source) override {
    if (source == this) return;
    const auto* family = dynamic_cast<const AmpFamilyCommandFilter*>(source);
    ForwardVelocitySmoother snapshot;
    if (family) {
      std::lock_guard<std::mutex> lock(family->mutex_);
      snapshot = family->smoother_;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (family) smoother_.inherit(snapshot);
    else smoother_.reset();
  }
  double inheritedForwardVelocity(double fallback) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return smoother_.active() ? smoother_.value() : fallback;
  }

 private:
  mutable std::mutex mutex_;
  ForwardVelocitySmoother smoother_;
  double thermal_limit_ = std::numeric_limits<double>::infinity();
  double retirement_limit_ = std::numeric_limits<double>::infinity();
};
}  // namespace humanoid_controller
