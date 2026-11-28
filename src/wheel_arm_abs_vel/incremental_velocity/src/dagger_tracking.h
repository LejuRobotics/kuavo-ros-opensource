#pragma once

#include <algorithm>
#include <cmath>

namespace velocity_ik {
inline bool projectFixedDaggerJoint(double measured, double lower, double upper,
                                    double tolerance, double& seed) {
  seed = std::clamp(measured, lower, upper);
  return std::isfinite(measured) && std::abs(measured - seed) <= tolerance;
}

inline bool allowDaggerBrake(bool was_ready, bool solved, bool feasible, int limit, int& count) {
  if (was_ready && !solved && feasible && count < limit) {
    ++count;
    return true;
  }
  count = 0;
  return false;
}

inline double daggerBandwidth(double error, double onset, double full, double slow, double fast) {
  const double u = std::clamp((error-onset)/(full-onset), 0., 1.);
  return slow + (std::max(slow, fast)-slow) * u*u*(3.-2.*u);
}
}  // namespace velocity_ik
