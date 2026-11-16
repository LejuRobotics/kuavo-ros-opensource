#pragma once
#include <array>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace HighlyDynamic {
// Joint order in each arm: J1..J7. Shared by all wheel models and both
// normal/VR gain tables; head and lower-body gains remain machine-specific.
inline constexpr std::array<double, 7> kWheelArmKp{{250, 210, 250, 180, 120, 120, 120}};
inline constexpr std::array<double, 7> kWheelArmKd{{20, 10, 18, 10, 6, 6, 6}};

template<class Settings, class Drivers, class Driver>
void applyWheelArmGains(Settings& settings, const Drivers& drivers,
                        int lower_dof, int arm_dof, Driver ec, Driver ruiwo) {
  if (lower_dof < 0 || arm_dof != 14 ||
      drivers.size() < static_cast<std::size_t>(lower_dof + arm_dof))
    throw std::runtime_error("Wheel arm gains require two ordered seven-joint arms");
  std::size_t ec_count = 0, rw_count = 0;
  for (const auto driver : drivers) {
    if (driver == ec) ++ec_count;
    else if (driver == ruiwo) ++rw_count;
  }
  auto next = settings;  // Validate completely before changing live settings.
  auto validate = [](auto& kp, auto& kd, std::size_t count) {
    if (kp.size() != count || kd.size() != count)
      throw std::runtime_error("Wheel arm gain table does not match motor driver layout");
  };
  validate(next.joint_kp, next.joint_kd, ec_count);
  validate(next.ruiwo_kp, next.ruiwo_kd, rw_count);
  if (next.vr_joint_kp.empty() && next.vr_joint_kd.empty()) {
    next.vr_joint_kp = next.joint_kp; next.vr_joint_kd = next.joint_kd;
  }
  if (next.vr_ruiwo_kp.empty() && next.vr_ruiwo_kd.empty()) {
    next.vr_ruiwo_kp = next.ruiwo_kp; next.vr_ruiwo_kd = next.ruiwo_kd;
  }
  validate(next.vr_joint_kp, next.vr_joint_kd, ec_count);
  validate(next.vr_ruiwo_kp, next.vr_ruiwo_kd, rw_count);
  ec_count = rw_count = 0;
  for (std::size_t i = 0; i < drivers.size(); ++i) {
    if (i >= static_cast<std::size_t>(lower_dof) &&
        i < static_cast<std::size_t>(lower_dof + arm_dof)) {
      const auto joint = (i - lower_dof) % 7;
      if (drivers[i] == ec) {
        next.joint_kp[ec_count] = next.vr_joint_kp[ec_count] = kWheelArmKp[joint];
        next.joint_kd[ec_count] = next.vr_joint_kd[ec_count] = kWheelArmKd[joint];
      } else if (drivers[i] == ruiwo) {
        next.ruiwo_kp[rw_count] = next.vr_ruiwo_kp[rw_count] = kWheelArmKp[joint];
        next.ruiwo_kd[rw_count] = next.vr_ruiwo_kd[rw_count] = kWheelArmKd[joint];
      } else throw std::runtime_error("Unsupported wheel arm motor driver");
    }
    if (drivers[i] == ec) ++ec_count;
    else if (drivers[i] == ruiwo) ++rw_count;
  }
  settings = std::move(next);
}
}  // namespace HighlyDynamic
