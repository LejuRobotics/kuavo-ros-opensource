#pragma once
#include <pinocchio/fwd.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <humanoid_wheel_interface/ManipulatorModelInfo.h>
#include "wheel_arm_abs_vel/humanoid_wheel_wbc/DynamicsBackend.h"
#include <string>
#include <vector>

namespace ocs2 { namespace mobile_manipulator {

struct RneaSettings {
  std::string mode = "full";  // gravity | bias | full
  vector_t torqueLimit, torqueRateLimit, accelerationLimit, jerkLimit;
  double velocityCutoffHz = 30.0;
  double maxMeasurementAge = 0.05;
  double maxPeriod = 0.05;
  double dynamicRampSeconds = 0.2;
};

// Fixed planar base, free-space rigid-body feedforward. No WBC/QP dependency.
// q/v use the model coordinates; actuated outputs are mapped by joint name.
class RneaBackend final : public DynamicsBackend {
 public:
  RneaBackend(const PinocchioInterface& interface, const ManipulatorModelInfo& info,
              const RneaSettings& settings);
  const char* name() const override { return "rnea"; }
  DynamicsResult update(const DynamicsReference&, const SystemObservation&,
                        double dt, double measurementAge) override;
 private:
  PinocchioInterface interface_;
  RneaSettings settings_;
  std::vector<int> velocityIndices_;
  vector_t q_, v_, a_, filteredV_, filteredA_, previousTorque_;
  vector_t gravity_, bias_, full_;
  bool initialized_ = false;
  double elapsed_ = 0.0, saturationSeconds_ = 0.0;
};
}}
