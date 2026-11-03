#pragma once

#include "kuavo_common/common/json.hpp"
#include <sstream>
#include <string>

namespace kuavo_common {

//nlohmann::json already does everything the old SeatConfig class did
using SeatConfig = nlohmann::json;

/** 加载并合并座椅配置：shared_json (seat_config_v5.json) + per_version_json (kuavo.json 覆盖) */
inline SeatConfig loadSeatConfig(const std::string& shared_json, const std::string& per_version_json = "") {
  SeatConfig data = SeatConfig::object();

  auto parse = [](const std::string& str, SeatConfig& out) {
    if (str.empty()) return false;
    try { std::istringstream(str) >> out; return out.is_object(); }
    catch (...) { return false; }
  };

  parse(shared_json, data);

  if (!per_version_json.empty()) {
    SeatConfig overrides;
    if (parse(per_version_json, overrides)) {
      for (auto it = overrides.begin(); it != overrides.end(); ++it)
        data[it.key()] = it.value();
    }
  }
  return data;
}

}  // namespace kuavo_common
