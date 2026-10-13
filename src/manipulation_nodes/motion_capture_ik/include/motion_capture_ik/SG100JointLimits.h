#pragma once

#include <string>
#include <vector>

namespace HighlyDynamic {

// 11-DOF SG100 关节限位（整机 URDF 或 joint_angle_map.json）。
// 关节顺序与 SG100HandCommand 一致；不依赖 roscpp。
class SG100JointLimits {
 public:
  static constexpr int kDof = 11;

  // 从 URDF 文件加载 11 个 revolute 关节的 lower/upper。
  // joint_prefix 拼在 kUrdfJoints 关节名之前(如 "l_" / "r_"),用于区分左右手。
  // 返回 false 当文件无法解析,或任一关节缺失/无 <limit>。
  bool load(const std::string& urdf_path, const std::string& joint_prefix);

  // 从 SG100 joint_angle_map.json 加载 min_angle/max_angle（单位：度），
  // 内部换算为弧度，与话题命令及 URDF 限位保持一致。
  bool loadJson(const std::string& json_path);

  // 将 positions clamp 到 [lower, upper]。返回 false 当未加载或维度 != kDof。
  bool clamp(std::vector<float>& positions) const;

 private:
  std::vector<double> lower_;
  std::vector<double> upper_;
  bool loaded_ = false;
};

}  // namespace HighlyDynamic
