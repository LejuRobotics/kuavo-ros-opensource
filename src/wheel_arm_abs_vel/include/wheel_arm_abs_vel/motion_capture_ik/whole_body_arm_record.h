#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <sensor_msgs/JointState.h>
#include <wheel_arm_abs_vel/WholeBodyIkReference.h>

namespace wheel_arm_abs_vel {
namespace arm_record {

// Legacy topic format for recording only. Command receivers must ignore it.
constexpr char kFrameId[] = "whole_body_ik_record_only";

inline bool convertJoints(const WholeBodyIkReference& reference,
                          const std::vector<std::string>& names,
                          const std::string& prefix,
                          sensor_msgs::JointState& output) {
  if (!reference.valid || !reference.takeover_requested ||
      reference.q.size() != reference.dof_names.size() ||
      reference.dq.size() != reference.dof_names.size()) {
    return false;
  }
  sensor_msgs::JointState record;
  record.header = reference.header;
  record.header.frame_id = kFrameId;
  record.effort.assign(names.size(), 0.0);  // Legacy placeholder, not measured torque.
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto& name = names[i];
    if (std::count(reference.dof_names.begin(), reference.dof_names.end(), name) != 1) {
      return false;
    }
    const auto found = std::find(reference.dof_names.begin(), reference.dof_names.end(), name);
    const auto index = std::distance(reference.dof_names.begin(), found);
    if (!std::isfinite(reference.q[index]) || !std::isfinite(reference.dq[index])) {
      return false;
    }
    record.name.push_back(prefix + std::to_string(i + 1));
    record.position.push_back(reference.q[index] * kRadToDeg);
    record.velocity.push_back(reference.dq[index] * kRadToDeg);
  }
  output = std::move(record);
  return true;
}

inline bool convert(const WholeBodyIkReference& reference,
                    sensor_msgs::JointState& output) {
  static const std::vector<std::string> names = [] {
    std::vector<std::string> result;
    for (int i = 0; i < 14; ++i) {
      result.push_back(std::string("zarm_") + (i < 7 ? "l" : "r") +
                       std::to_string(i % 7 + 1) + "_joint");
    }
    return result;
  }();
  return convertJoints(reference, names, "arm_joint_", output);
}

inline bool convertLowerBody(const WholeBodyIkReference& reference,
                             sensor_msgs::JointState& output) {
  static const std::vector<std::string> names{
      "knee_joint", "leg_joint", "waist_pitch_joint", "waist_yaw_joint"};
  return convertJoints(reference, names, "joint", output);
}

}  // namespace arm_record
}  // namespace wheel_arm_abs_vel
