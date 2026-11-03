#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_capture_ik::whole_body_ik {

// All arguments use radians and seconds. Hardware peak protection is NOT a
// degree-valued trajectory setting and must never supply the normal drive cap.
inline double effectiveDriveVelocity(double requested, double operating_cap,
                                     double hardware_peak) {
  for (double value : {requested, operating_cap, hardware_peak}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("joint velocity limits must be positive rad/s");
    }
  }
  return std::min({requested, operating_cap, hardware_peak});
}

// A fixed normalization scale, deliberately independent of operating limits.
inline double driveRegularization(double effort, double normalization_speed) {
  if (!std::isfinite(effort) || effort <= 0.0 ||
      !std::isfinite(normalization_speed) || normalization_speed <= 0.0) {
    throw std::invalid_argument("invalid fixed drive-cost normalization");
  }
  return 0.001 * effort / (normalization_speed * normalization_speed);
}

struct ReadyJointEnvelope {
  double lower;
  double upper;
  double lower_stopping_guard;
  double upper_stopping_guard;
};

// Make the explicitly approved rest endpoint reachable with zero velocity.
// Only its side of the constant inset is reduced; physical URDF limits and
// velocity-dependent stopping distance remain mandatory, including in ACTIVE.
inline ReadyJointEnvelope readyJointEnvelope(double physical_lower,
    double physical_upper, double ready, double margin, double guard) {
  for (double value : {physical_lower, physical_upper, ready, margin, guard}) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite ready envelope");
  }
  if (margin < 0.0 || guard < 0.0 ||
      physical_lower + 2.0 * (margin + guard) >= physical_upper ||
      ready < physical_lower || ready > physical_upper) {
    throw std::invalid_argument("ready pose outside a valid physical joint interval");
  }
  const double lower = std::min(physical_lower + margin, ready);
  const double upper = std::max(physical_upper - margin, ready);
  return {lower, upper, std::min(guard, ready - lower),
                        std::min(guard, upper - ready)};
}

}  // namespace motion_capture_ik::whole_body_ik
