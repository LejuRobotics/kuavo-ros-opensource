#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include "stop_reserve.h"

namespace velocity_ik {

// Behavioural constraints only. The caller retains the original hard domain.
struct OperatingJoint {
  double q{}, v{}, dt{.002}, lower{}, upper{}, vmax{};
  double lower_guard{}, upper_guard{}, delay{}, acceleration{}, braking{}, outward{};
  double extra_position{}, extra_time{};
  double lower_scale{1.}, upper_scale{1.};
  double current_acceleration{}, jerk{}; // jerk==0 retains the original contract.
  bool separate_stop{false};
  // Approved ready endpoints disable only this SIDE's comfort preference.
  bool lower_comfort{true}, upper_comfort{true};
};

inline bool validOperatingJoint(const OperatingJoint& p) {
  for (double x : {p.q, p.v, p.dt, p.lower, p.upper, p.vmax,
                  p.lower_guard, p.upper_guard, p.delay, p.acceleration,
                  p.braking, p.outward, p.extra_position, p.extra_time,
                  p.lower_scale, p.upper_scale, p.current_acceleration, p.jerk})
    if (!std::isfinite(x)) return false;
  return p.dt > 0 && p.lower <= p.upper && p.vmax >= 0 &&
      p.acceleration > 0 && p.braking > 0 && p.outward >= 0 && p.jerk >= 0 &&
      p.lower_guard >= 0 && p.upper_guard >= 0 && p.delay >= 0 &&
      p.extra_position >= 0 && p.extra_time >= 0 &&
      p.lower_scale >= 0 && p.lower_scale <= 1 &&
      p.upper_scale >= 0 && p.upper_scale <= 1;
}

inline double operatingStopReserve(const OperatingJoint& p, double speed,
                                   bool upper, bool comfort) {
  const double guard = upper ? p.upper_guard : p.lower_guard;
  const double hard_distance = p.separate_stop
      ? discreteStopDistance(speed, p.delay, p.outward, p.braking, p.dt)
      : p.delay * speed + .5 * p.acceleration * p.delay * p.delay +
        discreteStopDistance(speed, 0., 0., p.acceleration, p.dt);
  // Physical guards are unchanged. Anticipate the acceleration-to-braking
  // transition only in the behavioral preview, without creating a new hard
  // domain or asking the reference owner to change its integration rule.
  if (!comfort) return guard + hard_distance;
  const double preview = p.jerk > 0.
      ? std::max(hard_distance, jerkLimitedStopDistance(speed,
          upper ? p.current_acceleration : -p.current_acceleration,
          p.delay, p.outward, p.braking, p.jerk, p.dt))
      : hard_distance;
  return guard + preview + p.extra_position + p.extra_time * speed;
}

inline double operatingStopMargin(const OperatingJoint& p, double u,
                                  bool upper, bool comfort) {
  const double qnext = p.q + .5 * p.dt * (p.v + u);
  auto next=p;
  if(p.jerk>0.) {
    // Preview starts at this candidate's endpoint acceleration, in the
    // same finite-difference convention as jointJerkInterval and the owner.
    next.current_acceleration=(u-p.v)/p.dt;
  }
  return (upper ? p.upper - qnext : qnext - p.lower) -
      operatingStopReserve(next, std::max(0., upper ? u : -u), upper, comfort);
}

// Telemetry only: a rejected candidate has no stopping margin. Never pass its
// NaN velocity into the finite physical contract or throw before rejection.
inline double operatingStopMarginDiagnostic(const OperatingJoint& p, double u,
                                             bool upper, bool comfort) {
  if (!std::isfinite(u) || !validOperatingJoint(p))
    return std::numeric_limits<double>::quiet_NaN();
  return operatingStopMargin(p, u, upper, comfort);
}

inline double predictiveOutwardCap(const OperatingJoint& p, bool upper) {
  const auto feasible = [&](double speed) {
    const double u = upper ? speed : -speed;
    const auto segment = trapezoidSegmentBounds(p.q, p.v, u, p.dt);
    return (upper ? segment.second <= p.upper + 1.e-12
                  : segment.first >= p.lower - 1.e-12) &&
        operatingStopMargin(p, u, upper, true) >= 0.;
  };
  // Zero is a comfort request, not an instantaneous hard stopping command.
  if (!feasible(0.)) return 0.;
  if (feasible(p.vmax)) return p.vmax;
  double lo = 0., hi = p.vmax;
  for (int k = 0; k < 40; ++k) {
    const double mid = .5 * (lo + hi);
    if (feasible(mid)) lo = mid; else hi = mid;
  }
  return lo;
}

struct OperatingInterval {
  double lower{}, upper{}, baseline{}, soft_lower{}, soft_upper{};
  double static_lower{}, static_upper{}, predictive_lower{}, predictive_upper{};
  double lower_relaxation{}, upper_relaxation{};
  bool valid{false};
};

inline OperatingInterval feasibleOperatingInterval(double hard_lower,
    double hard_upper, double soft_lower, double soft_upper) {
  OperatingInterval r;
  if (!std::isfinite(hard_lower) || !std::isfinite(hard_upper) ||
      !std::isfinite(soft_lower) || !std::isfinite(soft_upper) ||
      hard_lower > hard_upper || soft_lower > soft_upper) return r;
  r.baseline = std::clamp(0., hard_lower, hard_upper);
  r.soft_lower = soft_lower; r.soft_upper = soft_upper;
  r.lower = std::max(hard_lower, std::min(soft_lower, r.baseline));
  r.upper = std::min(hard_upper, std::max(soft_upper, r.baseline));
  r.lower_relaxation = std::max(0., soft_lower - r.lower);
  r.upper_relaxation = std::max(0., r.upper - soft_upper);
  r.valid = true;
  return r;
}

inline OperatingInterval operatingInterval(const OperatingJoint& p,
    double hard_lower, double hard_upper) {
  if (!validOperatingJoint(p)) return {};
  const double sl = -p.vmax * (p.lower_comfort ? p.lower_scale : 1.);
  const double su = p.vmax * (p.upper_comfort ? p.upper_scale : 1.);
  const double pl = -(p.lower_comfort ? predictiveOutwardCap(p, false) : p.vmax);
  const double pu = p.upper_comfort ? predictiveOutwardCap(p, true) : p.vmax;
  auto r = feasibleOperatingInterval(hard_lower, hard_upper,
                                    std::max(sl, pl), std::min(su, pu));
  r.static_lower = sl; r.static_upper = su;
  r.predictive_lower = pl; r.predictive_upper = pu;
  return r;
}
}  // namespace velocity_ik
