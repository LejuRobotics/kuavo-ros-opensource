#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace velocity_ik {
// Distance from a candidate outward velocity until rest, including an interval
// before braking can begin. Command acceleration and guaranteed braking are
// different contracts. a_out is a bound on plant acceleration during that delay.
inline double stopDistance(double outward_velocity, double delay,
                           double outward_acceleration, double braking) {
  if (!std::isfinite(outward_velocity) || !std::isfinite(delay) ||
      !std::isfinite(outward_acceleration) || !std::isfinite(braking) ||
      delay < 0 || outward_acceleration < 0 || braking <= 0)
    throw std::invalid_argument("invalid stopping contract");
  const double v = std::max(0.0, outward_velocity);
  const double after_delay = v + outward_acceleration * delay;
  return v * delay + 0.5 * outward_acceleration * delay * delay +
         after_delay * after_delay / (2.0 * braking);
}
// Exact distance for the execution owner's nominal trapezoidal stop. The last
// partial-speed segment lasts a whole tick; continuous v^2/(2*b) misses it.
inline double discreteStopDistance(double outward_velocity, double delay,
                                   double outward_acceleration, double braking,
                                   double h) {
  const double continuous = stopDistance(outward_velocity, delay,
                                         outward_acceleration, braking);
  if (!std::isfinite(h) || h <= 0)
    throw std::invalid_argument("invalid stopping period");
  const double speed = std::max(0.0, outward_velocity) + outward_acceleration * delay;
  const double step = braking * h;
  const double remainder = std::fmod(speed, step);
  return continuous + 0.5 * h * remainder * (1.0 - remainder / step);
}

// Conservative preview for the comfort envelope under finite command jerk.
// The physical stopping certificate remains discreteStopDistance. During the
// execution delay retain its worst-outward-acceleration contract; afterwards
// ramp acceleration down to -braking. Continuous ramp integration bounds the
// owner's left-applied discrete acceleration ramp. The last partial-speed
// tick can add at most max(|a|, braking)*h^2/8 to its continuous counterpart.
inline double jerkLimitedStopDistance(double speed, double current_acceleration,
    double delay, double outward, double braking, double jerk, double h) {
  for (double x : {speed, current_acceleration, delay, outward, braking, jerk, h})
    if (!std::isfinite(x)) throw std::invalid_argument("invalid jerk stopping contract");
  if (delay < 0. || outward < 0. || braking <= 0. || jerk <= 0. || h <= 0.)
    throw std::invalid_argument("invalid jerk stopping contract");
  speed = std::max(0., speed);
  double a = delay > 0. ? std::max({0., current_acceleration, outward})
                       : current_acceleration;
  double distance = speed * delay + .5 * a * delay * delay;
  double v = speed + a * delay;
  // Weaker braking is conservative when the current deceleration is stronger.
  a = std::max(a, -braking);
  if (v <= 0. && a <= 0.) return distance;
  const double root = std::sqrt(a * a + 2. * jerk * v);
  const double stop_time = a >= 0. ? (a + root) / jerk
                                  : 2. * v / (root - a);
  const double ramp_time = (a + braking) / jerk;
  const double t = std::min(stop_time, ramp_time);
  distance += v * t + .5 * a * t * t - jerk * t * t * t / 6.;
  if (stop_time > ramp_time) {
    v = std::max(0., v + a * t - .5 * jerk * t * t);
    distance += v * v / (2. * braking);
  }
  return distance + .125 * std::max(std::abs(a), braking) * h * h;
}

// Both endpoints and the only possible interior extremum of this exact
// constant-acceleration segment. A sign reversal can cross a limit between
// valid endpoints and must be constrained in the candidate velocity domain.
inline std::pair<double, double> trapezoidSegmentBounds(
    double q, double v, double next_v, double h) {
  const double end = q + 0.5 * h * (v + next_v);
  double lower = std::min(q, end), upper = std::max(q, end);
  if (v * next_v < 0.0) {
    const double t = -v * h / (next_v - v);
    const double extremum = q + 0.5 * v * t;
    lower = std::min(lower, extremum);
    upper = std::max(upper, extremum);
  }
  return {lower, upper};
}

}
