#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace velocity_ik {
struct TorsoHeading {
  double pitch{0.0}, yaw{0.0}, pitch_rate{0.0}, yaw_rate{0.0};
};

// Anatomical columns are forward/left/up; omega is in the world frame.
// Keep forward heading below 60 deg pitch, blend to left-axis heading by
// 75 deg and retain it past horizontal (up.z < 0). Left stays observable
// when forward points at the floor. Rates differentiate the same blend,
// avoiding the Euler tan(pitch) singularity.
inline TorsoHeading torsoHeading(
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& omega = Eigen::Vector3d::Zero()) {
  const Eigen::Vector3d forward = rotation.col(0), left = rotation.col(1);
  const Eigen::Vector3d forward_dot = omega.cross(forward), left_dot = omega.cross(left);
  const Eigen::Vector2d f = forward.head<2>(), fd = forward_dot.head<2>();
  const Eigen::Vector2d side(left.y(), -left.x()), sd(left_dot.y(), -left_dot.x());
  const double h = f.norm(), sh = side.norm();
  const double hd = h > 1e-8 ? f.dot(fd) / h : 0.0;
  TorsoHeading result;
  result.pitch = std::atan2(-forward.z(), h);
  result.pitch_rate = (-h * forward_dot.z() + forward.z() * hd) / forward.squaredNorm();
  constexpr double kFullForward = 0.5;  // cos(60 deg)
  constexpr double kFullSide = 0.25881904510252074;  // cos(75 deg)
  // Euler pitch folds back below 90 deg after over-bending. Do not switch
  // back to the now-reversed forward projection on that side of vertical.
  const double x = std::clamp((h - kFullSide) / (kFullForward - kFullSide), 0.0, 1.0);
  double w = x * x * (3.0 - 2.0 * x);
  double wd = (x > 0.0 && x < 1.0)
      ? 6.0 * x * (1.0 - x) * hd / (kFullForward - kFullSide) : 0.0;
  // Fade the forward contribution out on inverted/sideways poses as well.
  // A smooth fade avoids a hemisphere switch at up.z == 0 during side lean.
  const double up_z = rotation(2,2);
  const double up_z_dot = omega.cross(Eigen::Vector3d(rotation.col(2))).z();
  const double u = std::clamp(up_z / kFullSide, 0.0, 1.0);
  const double upright = u*u*(3.0-2.0*u);
  const double upright_dot = u>0.0 && u<1.0
      ? 6.0*u*(1.0-u)*up_z_dot/kFullSide : 0.0;
  wd = wd*upright + w*upright_dot;
  w *= upright;
  // If lateral lean makes left nearly vertical, fade back to forward.
  const double sh_dot = sh > 1e-8 ? side.dot(sd)/sh : 0.0;
  const double s = std::clamp((sh-kFullSide)/(kFullForward-kFullSide),0.0,1.0);
  const double reliable_side = s*s*(3.0-2.0*s);
  const double reliable_side_dot = s>0.0 && s<1.0
      ? 6.0*s*(1.0-s)*sh_dot/(kFullForward-kFullSide) : 0.0;
  wd = wd*reliable_side - (1.0-w)*reliable_side_dot;
  w = 1.0 - (1.0-w)*reliable_side;
  Eigen::Vector2d fn = Eigen::Vector2d::Zero(), fnd = Eigen::Vector2d::Zero();
  Eigen::Vector2d sn = Eigen::Vector2d::Zero(), snd = Eigen::Vector2d::Zero();
  if (h > 1e-8) { fn = f / h; fnd = (fd - fn * hd) / h; }
  if (sh > 1e-8) { sn = side / sh; snd = (sd - sn * sn.dot(sd)) / sh; }
  Eigen::Vector2d heading = w * fn + (1.0 - w) * sn;
  Eigen::Vector2d heading_dot = w * fnd + (1.0 - w) * snd + wd * (fn - sn);
  // Opposite projections can cancel in inverted poses. Select the more
  // observable axis instead of dividing by a near-zero heading vector.
  if (heading.squaredNorm() < 1e-8) {
    heading = w >= 0.5 ? fn : sn;
    heading_dot = w >= 0.5 ? fnd : snd;
  }
  result.yaw = std::atan2(heading.y(), heading.x());
  result.yaw_rate = (heading.x() * heading_dot.y() - heading.y() * heading_dot.x()) /
      heading.squaredNorm();
  return result;
}
}  // namespace velocity_ik
