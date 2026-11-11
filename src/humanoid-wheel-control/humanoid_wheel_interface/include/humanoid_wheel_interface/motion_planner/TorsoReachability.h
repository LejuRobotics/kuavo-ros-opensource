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
                                             const Eigen::Vector4d& upper,
                                             const Eigen::Vector4d* reachableFallback = nullptr) {
  if (!requested.allFinite()) return previous;
  Eigen::Vector4d start = previous;
  Eigen::Vector4d end = requested;
  if (!isTorsoPoseReachable(start, g, lower, upper)) {
    // 旧目标越界时从当前有效 FK 接续，保留本拍增量，不要求用户先复位。
    if (!reachableFallback || !isTorsoPoseReachable(*reachableFallback, g, lower, upper)) return previous;
    start = *reachableFallback;
    end = start + (requested - previous);
  }
  double accepted = 0.0;
  for (int step = 1; step <= 16; ++step) {
    const double fraction = step / 16.0;
    if (!isTorsoPoseReachable(start + fraction * (end - start), g, lower, upper)) {
      double rejected = fraction;
      for (int i = 0; i < 20; ++i) {
        const double mid = (accepted + rejected) * 0.5;
        if (isTorsoPoseReachable(start + mid * (end - start), g, lower, upper)) accepted = mid;
        else rejected = mid;
      }
      return start + accepted * (end - start);
    }
    accepted = fraction;
  }
  return end;
}

// 纯下降允许必要的 x 联动：先尝试保持 x，再找本拍速度范围内最近的可达 x。
// 保持 yaw/pitch；若本拍所需 x 变化过大，减少下降量，不积累不可达目标。
inline Eigen::Vector4d limitTorsoCoupledDescent(
    const Eigen::Vector4d& previous, const Eigen::Vector4d& requested,
    const TorsoPlanarGeometry& g, const Eigen::Vector4d& lower, const Eigen::Vector4d& upper,
    double maxXStep, double minX, double maxX, const Eigen::Vector4d* reachableFallback = nullptr) {
  Eigen::Vector4d start = previous;
  if (!isTorsoPoseReachable(start, g, lower, upper)) {
    if (!reachableFallback || !isTorsoPoseReachable(*reachableFallback, g, lower, upper)) return previous;
    start = *reachableFallback;
  }
  const double dz = requested[1] - previous[1];
  if (!requested.allFinite() || dz >= 0.0 || !std::isfinite(maxXStep) || maxXStep <= 0.0) {
    return limitTorsoReachableStep(previous, requested, g, lower, upper, reachableFallback);
  }
  auto candidate = [&](double fraction, Eigen::Vector4d& result) {
    result = start;
    result[1] += fraction * dz;
    if (!isTorsoPoseReachable(result, g, lower, upper)) {
      bool found = false;
      double bestX = start[0], bestDistance = maxXStep + 1.0;
      for (double direction : {-1.0, 1.0}) {
        const double bound = std::max(minX, std::min(maxX, start[0] + direction * maxXStep));
        for (int probe = 1; probe <= 8; ++probe) {
          Eigen::Vector4d trial = result;
          trial[0] = start[0] + (bound - start[0]) * probe / 8.0;
          if (!isTorsoPoseReachable(trial, g, lower, upper)) continue;
          double rejected = start[0], accepted = trial[0];
          for (int i = 0; i < 24; ++i) {
            const double mid = (rejected + accepted) * 0.5;
            trial[0] = mid;
            if (isTorsoPoseReachable(trial, g, lower, upper)) accepted = mid;
            else rejected = mid;
          }
          // 在可达边界内侧留 10 微米，避免插值与浮点误差反复擦过边界。
          const double inset = std::min(1e-5, std::abs(start[0] + (bound - start[0]) * probe / 8.0 - accepted));
          accepted += direction * inset;
          if (std::abs(accepted - start[0]) < bestDistance) {
            bestDistance = std::abs(accepted - start[0]);
            bestX = accepted;
            found = true;
          }
          break;
        }
      }
      if (!found) return false;
      result[0] = bestX;
    }
    // 本拍起点与终点之间也必须可达，不能只验证终点。
    for (int i = 1; i <= 16; ++i) {
      if (!isTorsoPoseReachable(start + (result - start) * (i / 16.0), g, lower, upper)) return false;
    }
    return true;
  };
  Eigen::Vector4d result;
  if (candidate(1.0, result)) return result;
  double accepted = 0.0, rejected = 1.0;
  Eigen::Vector4d best = start;
  for (int i = 0; i < 24; ++i) {
    const double mid = (accepted + rejected) * 0.5;
    if (candidate(mid, result)) {
      accepted = mid;
      best = result;
    } else rejected = mid;
  }
  return best;
}

}  // namespace ocs2
