#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace motion_capture_ik {
namespace whole_body_ik {

constexpr std::uint32_t kDofLayoutVersion = 1;
constexpr int kBaseDof = 3;
constexpr int kLowerBodyDof = 4;
constexpr int kArmCount = 2;
constexpr int kJointsPerArm = 7;
constexpr int kDualArmDof = kArmCount * kJointsPerArm;
constexpr int kWholeBodyDof = kBaseDof + kLowerBodyDof + kDualArmDof;

enum class DofGroup : std::uint8_t {
  kBase = 0,
  kLowerBody = 1,
  kLeftArm = 2,
  kRightArm = 3,
};

struct DofSpec {
  std::string name;
  std::string model_joint_name;
  DofGroup group{DofGroup::kBase};
  bool decision_enabled{false};
};

class DofLayout {
 public:
  static DofLayout ArmOnlyStage() {
    return Build(false, false);
  }

  static DofLayout LowerBodyStage() {
    return Build(false, true);
  }

  static DofLayout WholeBodyStage() {
    return Build(true, true);
  }

  int fullDofCount() const { return static_cast<int>(specs_.size()); }

  int decisionDofCount() const {
    return static_cast<int>(decision_to_full_.size());
  }

  const DofSpec& fullSpec(int full_index) const {
    return specs_.at(static_cast<std::size_t>(full_index));
  }

  const DofSpec& decisionSpec(int decision_index) const {
    return fullSpec(
        decision_to_full_.at(static_cast<std::size_t>(decision_index)));
  }

  int decisionToFullIndex(int decision_index) const {
    return decision_to_full_.at(static_cast<std::size_t>(decision_index));
  }

  int decisionIndexByName(const std::string& name) const {
    for (int decision_index = 0; decision_index < decisionDofCount();
         ++decision_index) {
      if (decisionSpec(decision_index).name == name) {
        return decision_index;
      }
    }
    throw std::out_of_range("DoF is not enabled in the decision layout: " +
                            name);
  }

  std::vector<int> decisionIndices(DofGroup group) const {
    std::vector<int> indices;
    for (int decision_index = 0; decision_index < decisionDofCount();
         ++decision_index) {
      if (decisionSpec(decision_index).group == group) {
        indices.push_back(decision_index);
      }
    }
    return indices;
  }

  int armDecisionIndex(int arm, int joint) const {
    if (arm < 0 || arm >= kArmCount || joint < 0 ||
        joint >= kJointsPerArm) {
      throw std::out_of_range("invalid arm/joint index");
    }
    return arm_decision_indices_.at(static_cast<std::size_t>(arm))
        .at(static_cast<std::size_t>(joint));
  }

  int armForDecisionIndex(int decision_index) const {
    const DofGroup group = decisionSpec(decision_index).group;
    if (group == DofGroup::kLeftArm) {
      return 0;
    }
    if (group == DofGroup::kRightArm) {
      return 1;
    }
    return -1;
  }

  std::vector<std::string> decisionNames() const {
    std::vector<std::string> names;
    names.reserve(decision_to_full_.size());
    for (int decision_index = 0; decision_index < decisionDofCount();
         ++decision_index) {
      names.push_back(decisionSpec(decision_index).name);
    }
    return names;
  }

  std::vector<std::uint8_t> decisionGroups() const {
    std::vector<std::uint8_t> groups;
    groups.reserve(decision_to_full_.size());
    for (int decision_index = 0; decision_index < decisionDofCount();
         ++decision_index) {
      groups.push_back(
          static_cast<std::uint8_t>(decisionSpec(decision_index).group));
    }
    return groups;
  }

 private:
  static DofLayout Build(bool enable_base, bool enable_lower_body) {
    std::vector<DofSpec> specs;
    specs.reserve(kWholeBodyDof);
    specs.push_back({"base_x", "whole_body_base_x_joint",
                     DofGroup::kBase, enable_base});
    specs.push_back({"base_y", "whole_body_base_y_joint",
                     DofGroup::kBase, enable_base});
    specs.push_back({"base_yaw", "whole_body_base_yaw_joint",
                     DofGroup::kBase, enable_base});
    specs.push_back(
        {"knee_joint", "knee_joint", DofGroup::kLowerBody,
         enable_lower_body});
    specs.push_back(
        {"leg_joint", "leg_joint", DofGroup::kLowerBody,
         enable_lower_body});
    specs.push_back({"waist_pitch_joint", "waist_pitch_joint",
                     DofGroup::kLowerBody, enable_lower_body});
    specs.push_back({"waist_yaw_joint", "waist_yaw_joint",
                     DofGroup::kLowerBody, enable_lower_body});
    for (int arm = 0; arm < kArmCount; ++arm) {
      const char side = arm == 0 ? 'l' : 'r';
      const DofGroup group =
          arm == 0 ? DofGroup::kLeftArm : DofGroup::kRightArm;
      for (int joint = 0; joint < kJointsPerArm; ++joint) {
        const std::string name = "zarm_" + std::string(1, side) +
                                 std::to_string(joint + 1) + "_joint";
        specs.push_back({name, name, group, true});
      }
    }
    return DofLayout(std::move(specs));
  }
  explicit DofLayout(std::vector<DofSpec> specs) : specs_(std::move(specs)) {
    if (specs_.size() != static_cast<std::size_t>(kWholeBodyDof)) {
      throw std::logic_error("whole-body layout must contain 21 DoFs");
    }
    std::unordered_set<std::string> names;
    for (int full_index = 0; full_index < fullDofCount(); ++full_index) {
      const DofSpec& spec = fullSpec(full_index);
      if (spec.name.empty() || !names.insert(spec.name).second) {
        throw std::logic_error("whole-body layout contains an invalid name");
      }
      if (!spec.decision_enabled) {
        continue;
      }
      const int decision_index = decisionDofCount();
      decision_to_full_.push_back(full_index);
      if (spec.group == DofGroup::kLeftArm) {
        arm_decision_indices_.at(0).push_back(decision_index);
      } else if (spec.group == DofGroup::kRightArm) {
        arm_decision_indices_.at(1).push_back(decision_index);
      }
    }
    for (const auto& arm_indices : arm_decision_indices_) {
      if (arm_indices.size() != static_cast<std::size_t>(kJointsPerArm)) {
        throw std::logic_error(
            "every decision layout must enable seven joints per arm");
      }
    }
  }

  std::vector<DofSpec> specs_;
  std::vector<int> decision_to_full_;
  std::array<std::vector<int>, kArmCount> arm_decision_indices_;
};

}  // namespace whole_body_ik
}  // namespace motion_capture_ik
