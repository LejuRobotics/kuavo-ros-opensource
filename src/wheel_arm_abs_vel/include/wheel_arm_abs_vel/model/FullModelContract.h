#pragma once
#include <filesystem>
#include <stdexcept>
#include <string>

namespace wheel_arm_abs_vel {
inline std::string fullModelRelativePath(int version) {
  if (version <= 0)
    throw std::runtime_error("Absolute velocity IK requires a positive resolved model version");
  const auto model = "biped_s" + std::to_string(version);
  return model + "/urdf/" + model + ".urdf";
}

// Startup-only contract shared by the private IK and controller. No Drake fallback.
inline void requireFullModel(int version, const std::string& assets,
                             const std::string& urdf) {
  const auto expected = std::filesystem::path(assets) / "models" /
      fullModelRelativePath(version);
  if (assets.empty() || urdf.empty() ||
      std::filesystem::canonical(urdf) != std::filesystem::canonical(expected))
    throw std::runtime_error("Absolute velocity IK version " + std::to_string(version) +
                             " requires its own complete URDF: " + expected.string());
}
}  // namespace wheel_arm_abs_vel
