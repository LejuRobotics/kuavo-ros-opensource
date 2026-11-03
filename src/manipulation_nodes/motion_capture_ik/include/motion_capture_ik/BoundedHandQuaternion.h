#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace HighlyDynamic {

// A conservative rotation-space error ball replaces the Euler box and its
// pitch bypass. Apply it to emitted commands without overwriting the human
// source accumulator, so clipped out-and-back motion remains reversible.
inline Eigen::Quaterniond boundHandQuaternion(const Eigen::Quaterniond& target,
                                             const Eigen::Quaterniond& measured,
                                             double radius) {
  const Eigen::Quaterniond reference = measured.normalized();
  Eigen::Quaterniond goal = target.normalized();
  if (reference.dot(goal) < 0.0) goal.coeffs() *= -1.0;
  const double angle = reference.angularDistance(goal);
  if (angle <= radius) return goal;
  if (radius <= 0.0) return reference;
  return reference.slerp(radius / angle, goal).normalized();
}

// Undo the chest follow used for active hands before committing a bounded
// world command back into the grip-frame source accumulator.
inline Eigen::Quaterniond handCommandInSourceFrame(const Eigen::Quaterniond& command,
                                                   const Eigen::Quaterniond& chest,
                                                   const Eigen::Quaterniond& chestAnchor) {
  return (chestAnchor.normalized() * chest.normalized().conjugate() * command.normalized()).normalized();
}

}  // namespace HighlyDynamic
