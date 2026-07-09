#pragma once

#include "kuavo_common/common/json.hpp"
#include <string>
#include <vector>

namespace kuavo_common {

/**
 * 座椅配置管理器：合并共享 seat_config_v5.json + 各版本 kuavo.json 中的 seat/sit 字段。
 * 调用方负责从 ROS param 读取 JSON 字符串，传入 load()。
 *
 * 合并规则：
 *   1. shared_json 作为默认值（来自 seat_config_v5.json）
 *   2. per_version_json 中的 seat_/sit_ 前缀字段覆盖（来自 kuavo.json）
 */
class SeatConfig {
 public:
  SeatConfig() = default;

  /**
   * 加载并合并配置。
   * @param shared_json      共享默认 seat_config_v5.json 内容
   * @param per_version_json 各版本 kuavo.json 内容（可为空）
   */
  bool load(const std::string& shared_json, const std::string& per_version_json = "");

  /** 是否加载成功 */
  bool valid() const { return valid_; }

  // ── 类型化访问器 ──
  double getDouble(const std::string& key, double default_val = 0.0) const;
  bool getBool(const std::string& key, bool default_val = false) const;
  std::string getString(const std::string& key, const std::string& default_val = "") const;

  /** 读取 JSON 数组到 C++ double 数组。size 为期望长度，不足填 0。返回实际读到的元素数 */
  size_t getDoubleArray(const std::string& key, double* out, size_t size) const;

  /** 读取 JSON 数组为 std::vector<double> */
  std::vector<double> getDoubleVector(const std::string& key) const;

  /** 获取嵌套子对象 */
  const nlohmann::json* getObject(const std::string& key) const;

  /** 是否存在指定 key（合并后的视图） */
  bool hasKey(const std::string& key) const;

  /** 直接访问合并后的 JSON（高级用法） */
  const nlohmann::json& data() const { return data_; }

 private:
  void mergeOverride(const nlohmann::json& overrides);

  nlohmann::json data_;
  bool valid_{false};
};

}  // namespace kuavo_common
