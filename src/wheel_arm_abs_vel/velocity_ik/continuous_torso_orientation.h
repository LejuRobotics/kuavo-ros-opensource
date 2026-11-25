#pragma once

#include "torso_heading.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace velocity_ik {

// Continuous anatomical ZYX coordinates, not an incremental control target.
// The caller copies this observation into the fixed human session anchor.
struct ContinuousTorsoOrientation {
  Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
  Eigen::Vector2d pitch_yaw_rate{Eigen::Vector2d::Zero()};
  double source_time{0.0};
  bool valid{false};

  static double nearest(double angle, double reference) {
    return reference + std::atan2(std::sin(angle - reference),
                                  std::cos(angle - reference));
  }

  // One update per accepted source observation; repeated lifecycle packets
  // and solver/feasibility reads must not advance the representation.
  void update(const Eigen::Matrix3d& rotation, double stamp) {
    if (!rotation.allFinite() || !std::isfinite(stamp) ||
        (valid && stamp <= source_time)) return;
    constexpr double pi = 3.14159265358979323846;
    const double horizontal = std::hypot(rotation(0, 0), rotation(1, 0));
    const double pitch = std::atan2(-rotation(2, 0), horizontal);
    Eigen::Vector3d first(std::atan2(rotation(2, 1), rotation(2, 2)),
                          pitch, std::atan2(rotation(1, 0), rotation(0, 0)));
    Eigen::Vector3d second(first.x() + pi, pi - pitch, first.z() + pi);
    const Eigen::Vector3d previous = rpy;
    if (valid) {
      for (int i = 0; i < 3; ++i) {
        first(i) = nearest(first(i), previous(i));
        second(i) = nearest(second(i), previous(i));
      }
    } else {
      // Anatomical side lean stays within +/-90 degrees. This also permits
      // starting a session already bent past 90 degrees without a pi yaw flip.
      first.x() = nearest(first.x(), 0.0);
      second.x() = nearest(second.x(), 0.0);
      second.y() = nearest(second.y(), 0.0);
      second.z() = nearest(second.z(), 0.0);
    }
    const double first_cost = valid ? (first - previous).squaredNorm()
                                    : first.x() * first.x();
    const double second_cost = valid ? (second - previous).squaredNorm()
                                     : second.x() * second.x();
    rpy = first_cost <= second_cost ? first : second;
    // ZYX yaw copies the singular roll once the chest faces the floor
    // (yaw ≈ roll). Use the forward/left heading instead; it keeps the
    // compass direction while pitch above still follows this ZYX branch.
    const double anatomical_yaw = torsoHeading(rotation).yaw;
    rpy.z() = valid ? nearest(anatomical_yaw, previous.z())
                    : nearest(anatomical_yaw, 0.0);

    if (horizontal < 0.10) {
      // Below |cos(pitch)|≈0.10, yaw and roll are the same axis. Hold the
      // preceding roll instead of following that split.
      const double t = std::clamp((horizontal - 0.01) / 0.09, 0.0, 1.0);
      const double weight = t * t * (3.0 - 2.0 * t);
      const double reference_roll = valid ? previous.x() : 0.0;
      rpy.x() = reference_roll + weight * (rpy.x() - reference_roll);
    }

    // Differentiate the same coordinates as the absolute position target.
    // No tan(pitch) inversion of the angular velocity at the ZYX singularity.
    pitch_yaw_rate.setZero();
    if (valid) pitch_yaw_rate = (rpy.tail<2>() - previous.tail<2>()) / (stamp - source_time);
    source_time = stamp;
    valid = true;
  }
};

}  // namespace velocity_ik
