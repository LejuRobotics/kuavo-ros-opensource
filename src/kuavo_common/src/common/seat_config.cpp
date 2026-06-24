#include "kuavo_common/common/seat_config.h"
#include <iostream>
#include <sstream>

namespace kuavo_common {

namespace {
bool parseJson(const std::string& str, nlohmann::json& out) {
  if (str.empty())
    return false;
  try {
    std::istringstream(str) >> out;
    return out.is_object();
  } catch (const std::exception& e) {
    std::cerr << "[SeatConfig] parse failed: " << e.what() << std::endl;
    return false;
  }
}
}  // namespace

void SeatConfig::mergeOverride(const nlohmann::json& overrides) {
  // 浅层合并：overrides 中的 key 直接覆盖 data_ 中的同名字段
  for (auto it = overrides.begin(); it != overrides.end(); ++it) {
    // 只合并 seat/sit 相关 key，避免污染
    const std::string& key = it.key();
    if (key.rfind("sit_", 0) == 0 || key.rfind("seat_", 0) == 0 ||
        key == "sitDownWeight" || key == "sitDownJointAccelTask" ||
        key == "stand_up_boot_interpolation") {
      data_[key] = it.value();
    }
  }
}

bool SeatConfig::load(const std::string& shared_json, const std::string& per_version_json) {
  valid_ = false;

  // 1. 加载共享默认配置
  nlohmann::json shared;
  if (parseJson(shared_json, shared)) {
    data_ = shared;
    std::cout << "[SeatConfig] loaded shared seat config" << std::endl;
  } else {
    data_ = nlohmann::json::object();
    std::cerr << "[SeatConfig] shared config not available, using empty defaults" << std::endl;
  }

  // 2. 用 kuavo.json 中的 seat/sit 字段覆盖
  nlohmann::json per_version;
  if (parseJson(per_version_json, per_version)) {
    mergeOverride(per_version);
    std::cout << "[SeatConfig] merged per-version overrides" << std::endl;
  }

  valid_ = true;
  return true;
}

double SeatConfig::getDouble(const std::string& key, double default_val) const {
  if (!data_.contains(key))
    return default_val;
  try {
    return data_[key].get<double>();
  } catch (...) {
    return default_val;
  }
}

bool SeatConfig::getBool(const std::string& key, bool default_val) const {
  if (!data_.contains(key))
    return default_val;
  try {
    return data_[key].get<bool>();
  } catch (...) {
    return default_val;
  }
}

std::string SeatConfig::getString(const std::string& key, const std::string& default_val) const {
  if (!data_.contains(key))
    return default_val;
  try {
    return data_[key].get<std::string>();
  } catch (...) {
    return default_val;
  }
}

size_t SeatConfig::getDoubleArray(const std::string& key, double* out, size_t size) const {
  if (!data_.contains(key) || !data_[key].is_array())
    return 0;
  const auto& arr = data_[key];
  const size_t n = std::min(arr.size(), size);
  for (size_t i = 0; i < n; ++i) {
    try {
      out[i] = arr[i].get<double>();
    } catch (...) {
      out[i] = 0.0;
    }
  }
  // 不足部分填 0
  for (size_t i = n; i < size; ++i)
    out[i] = 0.0;
  return n;
}

std::vector<double> SeatConfig::getDoubleVector(const std::string& key) const {
  std::vector<double> out;
  if (!data_.contains(key) || !data_[key].is_array())
    return out;
  for (const auto& v : data_[key]) {
    try {
      out.push_back(v.get<double>());
    } catch (...) {
      out.push_back(0.0);
    }
  }
  return out;
}

const nlohmann::json* SeatConfig::getObject(const std::string& key) const {
  if (!data_.contains(key) || !data_[key].is_object())
    return nullptr;
  return &data_[key];
}

bool SeatConfig::hasKey(const std::string& key) const {
  return data_.contains(key);
}

}  // namespace kuavo_common
