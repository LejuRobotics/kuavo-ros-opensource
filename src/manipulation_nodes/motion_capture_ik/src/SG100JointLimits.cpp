#include "motion_capture_ik/SG100JointLimits.h"

#include <urdf/model.h>

#include <array>
#include <cstddef>
#include <cstdio>

namespace HighlyDynamic {

namespace {

// 命令关节名 → URDF 关节名（与 sg100_hand_config.py COMMAND_TO_URDF 一致）。
// 顺序 = SG100HandCommand 的 11 关节顺序。
constexpr std::size_t kDof = SG100JointLimits::kDof;

const std::array<std::string, kDof> kCommandJoints = {
    "TH_CMC_ABD_Joint",   // 0  thumb_j1
    "TH_MCP_FLEX_Joint",  // 1  thumb_j2
    "TH_IP_FLEX_Joint",   // 2  thumb_j3
    "IF_MCP_ABD_Joint",   // 3  index_j1
    "IF_MCP_FLEX_Joint",  // 4  index_j2
    "IF_PIP_FLEX_Joint",  // 5  index_j3
    "MF_MCP_FLEX_Joint",  // 6  middle_j1
    "MF_PIP_FLEX_Joint",  // 7  middle_j2
    "LF_MCP_ABD_Joint",   // 8  little_j1
    "LF_MCP_FLEX_Joint",  // 9  little_j2
    "LF_PIP_FLEX_Joint",  // 10 little_j3
};

const std::array<std::string, kDof> kUrdfJoints = {
    "thumb_j1", "thumb_j2", "thumb_j3",
    "index_j1", "index_j2", "index_j3",
    "middle_j1", "middle_j2",
    "little_j1", "little_j2", "little_j3",
};

}  // namespace

bool SG100JointLimits::load(const std::string& urdf_path,
                            const std::string& joint_prefix) {
  loaded_ = false;
  lower_.clear();
  upper_.clear();

  urdf::Model model;
  if (!model.initFile(urdf_path)) {
    std::fprintf(stderr, "[SG100JointLimits] initFile failed: %s\n",
                 urdf_path.c_str());
    return false;
  }

  lower_.reserve(kDof);
  upper_.reserve(kDof);

  for (std::size_t i = 0; i < kDof; ++i) {
    const std::string joint_name = joint_prefix + kUrdfJoints[i];
    urdf::JointConstSharedPtr joint = model.getJoint(joint_name);
    if (!joint || !joint->limits) {
      // 关节缺失或无 <limit>：不再套用 [0, 1.57] 默认区间 —— 该区间与真实限位
      // （例如 thumb_j2 实为 [-2.62, 0]）方向相反，套用会把指令推向反方向。
      // 直接判定加载失败，由调用方拒绝启动。
      std::fprintf(stderr,
                   "[SG100JointLimits] joint '%s' (urdf '%s') missing <limit>; "
                   "refusing to guess joint limits\n",
                   kCommandJoints[i].c_str(), joint_name.c_str());
      lower_.clear();
      upper_.clear();
      return false;
    }
    lower_.push_back(joint->limits->lower);
    upper_.push_back(joint->limits->upper);
  }

  loaded_ = true;
  return true;
}

bool SG100JointLimits::clamp(std::vector<float>& positions) const {
  if (!loaded_ || positions.size() != kDof) {
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    const double lo = lower_[i];
    const double hi = upper_[i];
    double v = static_cast<double>(positions[i]);
    if (v < lo) {
      v = lo;
    }
    if (v > hi) {
      v = hi;
    }
    positions[i] = static_cast<float>(v);
  }
  return true;
}

}  // namespace HighlyDynamic
