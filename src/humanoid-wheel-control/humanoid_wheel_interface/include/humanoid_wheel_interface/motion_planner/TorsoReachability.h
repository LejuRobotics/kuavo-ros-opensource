#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ocs2 {

// 轮臂三个 Y 轴关节的平面链，坐标为 base_link 下的 x/z，长度单位 m。
struct TorsoPlanarGeometry {
  Eigen::Vector2d knee, legOffset, waistOffset, torsoOffset;
};

inline Eigen::Vector2d rotateTorsoXZ(const Eigen::Vector2d& p, double angle) {
  return {std::cos(angle) * p.x() + std::sin(angle) * p.y(),
          -std::sin(angle) * p.x() + std::cos(angle) * p.y()};
}

inline bool isTorsoPoseReachable(const Eigen::Vector4d& pose, const TorsoPlanarGeometry& g,
                                 const Eigen::Vector4d& lower, const Eigen::Vector4d& upper) {
  if (!pose.allFinite()) return false;
  const Eigen::Vector2d wrist = pose.head<2>() - g.knee - rotateTorsoXZ(g.torsoOffset, pose[3]);
  const double a = g.legOffset.norm(), b = g.waistOffset.norm();
  if (a < 1e-8 || b < 1e-8) return false;
  const double cosine = (wrist.squaredNorm() - a * a - b * b) / (2.0 * a * b);
  if (cosine < -1.0 - 1e-9 || cosine > 1.0 + 1e-9) return false;
  const double bend = std::acos(std::max(-1.0, std::min(1.0, cosine)));
  const double offset = std::atan2(g.waistOffset.y(), g.waistOffset.x()) -
                        std::atan2(g.legOffset.y(), g.legOffset.x());
  for (double direction : {-1.0, 1.0}) {
    const double leg = std::remainder(offset + direction * bend, 2.0 * M_PI);
    const Eigen::Vector2d chain = g.legOffset + rotateTorsoXZ(g.waistOffset, leg);
    const double knee = std::remainder(std::atan2(chain.y(), chain.x()) -
                                       std::atan2(wrist.y(), wrist.x()), 2.0 * M_PI);
    const Eigen::Vector4d q(knee, leg, pose[3] - knee - leg, pose[2]);
    if ((q.array() >= lower.array() - 1e-8).all() &&
        (q.array() <= upper.array() + 1e-8).all()) return true;
  }
  return false;
}

// 沿用户请求的线段截断，不能通过修改未操作轴来凑出可达目标。
inline Eigen::Vector4d limitTorsoReachableStep(const Eigen::Vector4d& previous,
                                             const Eigen::Vector4d& requested,
                                             const TorsoPlanarGeometry& g,
                                             const Eigen::Vector4d& lower,
                                             const Eigen::Vector4d& upper) {
  if (!requested.allFinite() || !isTorsoPoseReachable(previous, g, lower, upper)) return previous;
  double accepted = 0.0;
  for (int step = 1; step <= 16; ++step) {
    const double fraction = step / 16.0;
    if (!isTorsoPoseReachable(previous + fraction * (requested - previous), g, lower, upper)) {
      double rejected = fraction;
      for (int i = 0; i < 20; ++i) {
        const double mid = (accepted + rejected) * 0.5;
        if (isTorsoPoseReachable(previous + mid * (requested - previous), g, lower, upper)) accepted = mid;
        else rejected = mid;
      }
      return previous + accepted * (requested - previous);
    }
    accepted = fraction;
  }
  return requested;
}

}  // namespace ocs2
