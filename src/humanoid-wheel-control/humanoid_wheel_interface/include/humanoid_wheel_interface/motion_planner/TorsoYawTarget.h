#pragma once

#include <algorithm>

namespace ocs2 {

// 与 torso_joystick_controller.py 的 YAW_BUTTON_LIMIT = pi - 0.05 对齐，
// 保留四元数双覆盖边界的余量；普通摇杆仍由 Joy 侧限制到 ±30°。
constexpr double kTorsoYawTargetLimit = 3.091592653589793;

inline double clampTorsoYawTarget(double yaw) {
  return std::max(-kTorsoYawTargetLimit, std::min(kTorsoYawTargetLimit, yaw));
}

}  // namespace ocs2
