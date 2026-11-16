#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace HighlyDynamic {

// 连续三个有效候选帧后恢复跟随，恢复时仍限制输出单步角度。
struct GripQuaternionRecovery {
  Eigen::Quaterniond candidate = Eigen::Quaterniond::Identity();
  int continuousFrames = 0;

  void reset() { continuousFrames = 0; }

  Eigen::Quaterniond update(const Eigen::Quaterniond& previous,
                           const Eigen::Quaterniond& target) {
    constexpr double maxStepRad = 0.5;
    const auto angleBetween = [](const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
      return 2.0 * std::acos(std::min(1.0, std::abs(a.dot(b))));
    };
    const double angle = angleBetween(previous, target);
    if (angle <= maxStepRad) {
      reset();
      return target;
    }
    // 与原始候选比较，不能始终与被冻结的输出比较，否则无法恢复。
    continuousFrames = continuousFrames > 0 && angleBetween(candidate, target) <= maxStepRad
                           ? std::min(continuousFrames + 1, 3)
                           : 1;
    candidate = target;
    if (continuousFrames < 3) return previous;
    return previous.slerp(maxStepRad / angle, target).normalized();
  }
};

}  // namespace HighlyDynamic
