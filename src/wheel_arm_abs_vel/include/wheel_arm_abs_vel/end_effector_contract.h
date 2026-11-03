#pragma once
#include "kuavo_common/common/json.hpp"
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace kuavo_common {
// Input is produced by the shared Python resolver before nodelet initialization.
// No file probing, bus I/O or dependence on stale ROS parameters here.
inline std::vector<std::string> endEffectorContract(const char* text, int version) {
  if (!text || !*text) return {};
  const auto data = nlohmann::json::parse(text);
  if (!data.is_object() || !data.at("robot_version").is_number_integer() ||
      data.at("robot_version").get<int>() != version || !data.at("real").is_boolean() ||
      !data.at("source").is_string() || data.at("source").get<std::string>().empty())
    throw std::runtime_error("Invalid or mismatched end-effector contract");
  const auto types = data.at("types").get<std::vector<std::string>>();
  static const std::set<std::string> supported = {
    "none","jodell","qiangnao","qiangnao_touch","lejuclaw","revo2","heiman","linker_hand"};
  if (types.size()!=2 || types[0]!=types[1] || !supported.count(types[0]))
    throw std::runtime_error("End-effector contract requires two matching supported types");
  return types;
}
} // namespace kuavo_common
