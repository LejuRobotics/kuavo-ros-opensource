#pragma once

#include <cmath>
#include <stdexcept>

namespace velocity_ik {
struct TemporalCost {
  double acceleration_weight{0.}, delta_acceleration_weight{0.};
  double acceleration_scale{1.}, delta_acceleration_scale{1.};
};

inline bool validTemporalCost(const TemporalCost& p) {
  return std::isfinite(p.acceleration_weight) && p.acceleration_weight >= 0 &&
      std::isfinite(p.delta_acceleration_weight) && p.delta_acceleration_weight >= 0 &&
      std::isfinite(p.acceleration_scale) && p.acceleration_scale > 0 &&
      std::isfinite(p.delta_acceleration_scale) && p.delta_acceleration_scale > 0;
}

// Convention: 0.5*u'Hu + g'u. No persistent history: v/a MUST be the
// last committed reference, including actual fallback/owner-echo acceleration.
inline bool addTemporalCost(const TemporalCost& p, double dt, double v,
                            double a, double* h_diagonal, double* gradient) {
  if (!validTemporalCost(p) || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(v) || !std::isfinite(a)) return false;
  if (p.acceleration_weight == 0 && p.delta_acceleration_weight == 0) return true;
  const double sa = p.acceleration_scale * dt;
  const double sd = p.delta_acceleration_scale * dt;
  const double A = p.acceleration_weight / (sa * sa);
  const double B = p.delta_acceleration_weight / (sd * sd);
  const double h = *h_diagonal + 2. * (A + B);
  const double g = *gradient - 2. * (A * v + B * (v + dt * a));
  if (!std::isfinite(h) || !std::isfinite(g)) return false;
  *h_diagonal = h; *gradient = g;
  return true;
}

inline double temporalAccelerationCost(const TemporalCost& p, double a) {
  return p.acceleration_weight * std::pow(a / p.acceleration_scale, 2);
}
inline double temporalDeltaAccelerationCost(const TemporalCost& p, double da) {
  return p.delta_acceleration_weight * std::pow(da / p.delta_acceleration_scale, 2);
}
}  // namespace velocity_ik
