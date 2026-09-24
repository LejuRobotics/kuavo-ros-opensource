#pragma once

#include <string>
#include <vector>

namespace HighlyDynamic {

// 11-DOF SG100 关节限位（从 SG100 URDF <limit lower upper> 加载）。
// 关节顺序与 SG100HandCommand 一致；URDF 解析用 ROS urdf 库，不依赖 roscpp。
class SG100JointLimits {
 public:
  static constexpr int kDof = 11;

  // 从 URDF 文件加载 11 个 revolute 关节的 lower/upper。
  // joint_prefix 拼在 kUrdfJoints 关节名之前(如 "l_" / "r_"),用于区分左右手。
  // 返回 false 当文件无法解析,或任一关节缺失/无 <limit> —— 不套用任何默认区间
  // (默认区间会与真实限位冲突),由调用方据此拒绝启动。
  bool load(const std::string& urdf_path, const std::string& joint_prefix);

  // 将 positions clamp 到 [lower, upper]。返回 false 当未加载或维度 != kDof。
  bool clamp(std::vector<float>& positions) const;

 private:
  std::vector<double> lower_;
  std::vector<double> upper_;
  bool loaded_ = false;
};

}  // namespace HighlyDynamic
