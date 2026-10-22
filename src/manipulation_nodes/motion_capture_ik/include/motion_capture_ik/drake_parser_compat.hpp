#pragma once

#include <drake/multibody/parsing/parser.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace motion_capture_ik {
namespace drake_parser_compat {

// Drake 1.19 (x86): AddModelFromFile() returns ModelInstanceIndex.
// Drake 1.51 (aarch64): AddModelFromFile removed; use AddModels() instead.
// arms_ik_node already uses AddModels on 1.19 — safe on both sides.
inline drake::multibody::ModelInstanceIndex AddUrdfModel(drake::multibody::Parser& parser,
                                                         const std::string& urdf_path) {
  std::vector<drake::multibody::ModelInstanceIndex> instances = parser.AddModels(urdf_path);
  if (instances.empty()) {
    throw std::runtime_error("AddModels returned no instances for: " + urdf_path);
  }
  return instances.front();
}

}  // namespace drake_parser_compat
}  // namespace motion_capture_ik
