#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace motion_capture_ik::whole_body_ik {

struct ArmPressure {
  double reach{0.0};
  double singularity{0.0};
  double joint{0.0};
  double total{0.0};
};

struct Recruitment {
  double pressure{0.0};
  double lower_body{0.0};
  double base{0.0};
};

inline double smoothStep01(double value) {
  const double clamped = std::clamp(value, 0.0, 1.0);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

// Pressure rises from zero at start to one at full as value increases.
inline double risingPressure(double value, double start, double full) {
  if (!std::isfinite(value) || !std::isfinite(start) ||
      !std::isfinite(full) || full <= start) {
    return 1.0;
  }
  return smoothStep01((value - start) / (full - start));
}

// Pressure rises from zero at start to one at full as value decreases.
inline double fallingPressure(double value, double start, double full) {
  if (!std::isfinite(value) || !std::isfinite(start) ||
      !std::isfinite(full) || start <= full) {
    return 1.0;
  }
  return smoothStep01((start - value) / (start - full));
}

inline ArmPressure calculateArmPressure(
    double reach_distance, double reach_start, double reach_full,
    double sigma_min, double singularity_start, double singularity_full,
    double minimum_safe_joint_margin, double joint_margin_start,
    double joint_margin_full) {
  ArmPressure result;
  result.reach = risingPressure(reach_distance, reach_start, reach_full);
  result.singularity = fallingPressure(
      sigma_min, singularity_start, singularity_full);
  result.joint = fallingPressure(
      minimum_safe_joint_margin, joint_margin_start, joint_margin_full);
  result.total = std::max({result.reach, result.singularity, result.joint});
  return result;
}

template <std::size_t ArmCount>
inline Recruitment calculateRecruitment(
    const std::array<ArmPressure, ArmCount>& arm_pressure,
    const std::array<bool, ArmCount>& arm_controlled,
    double lower_body_start, double lower_body_full,
    double base_start, double base_full, bool enabled) {
  Recruitment result;
  if (!enabled) {
    return result;
  }
  for (std::size_t arm = 0; arm < ArmCount; ++arm) {
    if (arm_controlled.at(arm)) {
      result.pressure = std::max(result.pressure,
                                 arm_pressure.at(arm).total);
    }
  }
  result.lower_body = risingPressure(
      result.pressure, lower_body_start, lower_body_full);
  result.base = risingPressure(result.pressure, base_start, base_full);
  return result;
}

inline double slewRecruitment(double current, double target,
                              double rise_rate, double fall_rate,
                              double dt) {
  current = std::clamp(current, 0.0, 1.0);
  target = std::clamp(target, 0.0, 1.0);
  if (!std::isfinite(dt) || dt <= 0.0) {
    return current;
  }
  const double rate = target >= current ? std::max(0.0, rise_rate)
                                        : std::max(0.0, fall_rate);
  const double maximum_change = rate * dt;
  return std::clamp(target, current - maximum_change,
                    current + maximum_change);
}

}  // namespace motion_capture_ik::whole_body_ik
