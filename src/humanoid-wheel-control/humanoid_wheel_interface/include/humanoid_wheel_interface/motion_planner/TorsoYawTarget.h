#pragma once

#include <algorithm>

namespace ocs2 {

// 与 torso_joystick_controller.py 的 YAW_BUTTON_LIMIT = pi - 0.05 对齐，
// 保留四元数双覆盖边界的余量；普通摇杆仍由 Joy 侧限制到 ±30°。
constexpr double kTorsoYawTargetLimit = 3.091592653589793;
constexpr double kTorsoYawMinHeightIncrement = 0.05;

inline double clampTorsoYawTarget(double yaw) {
  return std::max(-kTorsoYawTargetLimit, std::min(kTorsoYawTargetLimit, yaw));
}

// 高度不足时只允许主动向零回收；跨零请求停在零，单独升降保持已有 yaw。
inline double applyTorsoYawRequest(double currentYaw, double requestedYaw,
                                  double currentHeightIncrement, double requestedHeightIncrement) {
  if (std::min(currentHeightIncrement, requestedHeightIncrement) < kTorsoYawMinHeightIncrement) {
    const double heldYaw = clampTorsoYawTarget(currentYaw);
    return std::max(std::min(0.0, heldYaw), std::min(std::max(0.0, heldYaw), requestedYaw));
  }
  return clampTorsoYawTarget(requestedYaw);
}

}  // namespace ocs2
