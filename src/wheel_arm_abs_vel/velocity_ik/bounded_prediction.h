#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
namespace velocity_ik {
// Bound displacement without modifying measured-motion feedforward or coupling
// channels. Fixed velocity gives a continuous displacement at either cap.
inline Eigen::Vector3d boundedPredictionStep(const Eigen::Vector3d& velocity,
                                            double age, double space_cap) {
  if (!velocity.allFinite() || !std::isfinite(age) || age < 0.) return Eigen::Vector3d::Zero();
  Eigen::Vector3d step = std::min(age, .020) * velocity;
  const double length = step.norm();
  if (length > space_cap) step *= space_cap / length;
  return step;
}
inline Eigen::Vector4d boundedBodyPredictionStep(const Eigen::Vector4d& velocity,
                                                 double age) {
  if (!velocity.allFinite() || !std::isfinite(age) || age < 0.) return Eigen::Vector4d::Zero();
  return (std::min(age, .020) * velocity).cwiseMax(-.020).cwiseMin(.020);
}
}
