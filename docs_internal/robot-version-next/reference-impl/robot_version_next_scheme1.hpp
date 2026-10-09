/**
 * @file robot_version_next_scheme1.hpp
 * @brief ROBOT_VERSION 新格式解析模块 — 参考实现
 * @note 最终实现以 src/robot_version/ 包为准，本文件供实现时参考
 *
 * 格式：<brand>-<product>-<locomotion>[-<end_effector>[-<camera>]][+tags...]
 *
 * 示例：
 *   export ROBOT_VERSION="kuavo-4pro-biped-revo1hand"
 *   export ROBOT_VERSION="kuavo-4pro-biped-revo1hand_lejuclaw-Gemini330_none_none_none"
 *   export ROBOT_VERSION="kuavo-4pro-biped-dummy"
 *   export ROBOT_VERSION="kuavo-5-wheeled-lejuclaw"
 */

#pragma once

#include <string>
#include <set>
#include <map>
#include <regex>
#include <stdexcept>
#include <cstdlib>

namespace robot_version {

// ==================== Enum 定义 ====================

enum class Brand { Kuavo, Roban };

enum class Locomotion { Biped, Wheeled };

enum class EndEffectorType {
    None, Lejuclaw, Revo1hand, Revo2hand, LinkhandO6, Revo1touch, Dummy
};

enum class CameraModel {
    None, D435, D405, Gemini330, Gemini335L
};

class RobotVersion {
public:
    /// 构造函数，自动检测输入类型（int 旧格式 / string 新格式）
    explicit RobotVersion(const std::string& version_str) {
        raw_ = resolve_input(version_str);
        parse();
    }

    static RobotVersion from_env(const std::string& default_version = "") {
        const char* env = std::getenv("ROBOT_VERSION");
        return RobotVersion(env ? std::string(env) : default_version);
    }

    // ==================== 核心属性 ====================

    const std::string& raw() const { return raw_; }
    const std::string& robot_id() const { return robot_id_; }
    const std::string& product() const { return product_; }
    const std::string& end_effector_str() const { return end_effector_str_; }
    const std::set<std::string>& tags() const { return tags_; }

    // ==================== Enum 字段 ====================

    Brand brand() const { return brand_; }
    Locomotion locomotion() const { return locomotion_; }
    EndEffectorType left_end() const { return left_end_; }
    EndEffectorType right_end() const { return right_end_; }

    // ==================== 末端判断 ====================

    // 末端判断通过宏提供（见文件末尾 HAS_REVO1HAND 等）

    // ==================== 标签判断 ====================

    bool has_tag(const std::string& tag) const {
        return tags_.find(tag) != tags_.end();
    }

    // ==================== 硬件信息 ====================

    std::string get_upper_computer() const {
        for (const auto& tag : tags_) {
            if (tag.rfind("upper_", 0) == 0) return tag.substr(6);
        }
        return "";
    }

    std::string get_lower_computer() const {
        for (const auto& tag : tags_) {
            if (tag.rfind("lower_", 0) == 0) return tag.substr(6);
        }
        return "";
    }

    // ==================== 语义路径字段（来自映射表） ====================

    /// 模型目录名，如 "biped_s45"
    std::string model_name() const {
        auto* info = get_model_info();
        return info ? info->model_name : "";
    }

    /// 配置目录名，如 "kuavo_v45"
    std::string config_name() const {
        auto* info = get_model_info();
        return info ? info->config_name : "";
    }

    /// 旧版本号整数，用于 /robot_version int 参数兼容。新格式专属机器人返回 0。
    int legacy_int() const {
        for (const auto& [k, v] : legacy_map()) {
            if (v == raw_) return k;
        }
        return 0;
    }

    explicit operator bool() const { return !raw_.empty(); }

private:
    std::string raw_;
    std::string robot_id_;
    std::string product_;
    std::string end_effector_str_;
    Brand brand_ = Brand::Kuavo;
    Locomotion locomotion_ = Locomotion::Biped;
    EndEffectorType left_end_ = EndEffectorType::None;
    EndEffectorType right_end_ = EndEffectorType::None;
    std::set<std::string> tags_;

    // ==================== 映射表定义 ====================

    struct ModelInfo {
        const char* model_name;      // "biped_s45"
        const char* config_name;     // "kuavo_v45"
    };

    /// Legacy 兼容表：旧 int → 新格式字符串
    static const std::map<int, std::string>& legacy_map() {
        static const std::map<int, std::string> m = {
            {40,     "kuavo-4-biped-none"},
            {42,     "kuavo-4V2-biped-none"},
            {45,     "kuavo-4pro-biped-revo1hand"},
            {47,     "kuavo-4pro-biped-lejuclaw"},
            {100045, "kuavo-4pro-biped-dummy"},
            {49,     "kuavo-4proEDU-biped-revo1hand"},
            {100049, "kuavo-4proEDU-biped-dummy"},
            {50,     "kuavo-5-biped-none"},
            {52,     "kuavo-5V2-biped-none"},
            {60,     "kuavo-5-wheeled-none"},
            {14,     "roban-2v1-biped-none"},
            // ... 实际需覆盖全部 33 个版本号
        };
        return m;
    }

    /// 模型信息表：brand-product-locomotion → 路径字段
    static const std::map<std::string, ModelInfo>& model_map() {
        static const std::map<std::string, ModelInfo> m = {
            {"kuavo-4-biped",       {"biped_s40", "kuavo_v40"}},
            {"kuavo-4V2-biped",     {"biped_s42", "kuavo_v42"}},
            {"kuavo-4pro-biped",    {"biped_s45", "kuavo_v45"}},
            {"kuavo-4proEDU-biped", {"biped_s49", "kuavo_v49"}},
            {"kuavo-5-biped",       {"biped_s50", "kuavo_v50"}},
            {"kuavo-5V2-biped",     {"biped_s52", "kuavo_v52"}},
            {"kuavo-5-wheeled",     {"biped_s60", "kuavo_v60"}},
            {"roban-2v1-biped",     {"biped_s14", "kuavo_v14"}},
            // ... 实际需覆盖全部 base model
        };
        return m;
    }

    // ==================== 内部方法 ====================

    static std::string resolve_input(const std::string& input) {
        if (input.empty()) return input;
        bool all_digits = true;
        for (char c : input) {
            if (!std::isdigit(c)) { all_digits = false; break; }
        }
        if (all_digits) {
            int legacy = std::stoi(input);
            const auto& m = legacy_map();
            auto it = m.find(legacy);
            if (it != m.end()) return it->second;
            throw std::invalid_argument("Unknown legacy version: " + input);
        }
        return input;
    }

    const ModelInfo* get_model_info() const {
        auto last_dash = robot_id_.rfind('-');
        std::string base_key = (last_dash != std::string::npos) ? robot_id_.substr(0, last_dash) : robot_id_;
        const auto& m = model_map();
        auto it = m.find(base_key);
        return (it != m.end()) ? &it->second : nullptr;
    }

    static Brand parse_brand(const std::string& s) {
        if (s == "kuavo") return Brand::Kuavo;
        if (s == "roban") return Brand::Roban;
        throw std::invalid_argument("Unknown brand: " + s);
    }

    static Locomotion parse_locomotion(const std::string& s) {
        if (s == "biped") return Locomotion::Biped;
        if (s == "wheeled") return Locomotion::Wheeled;
        throw std::invalid_argument("Unknown locomotion: " + s);
    }

    static EndEffectorType parse_end_effector(const std::string& s) {
        if (s == "none" || s.empty()) return EndEffectorType::None;
        if (s == "lejuclaw") return EndEffectorType::Lejuclaw;
        if (s == "revo1hand") return EndEffectorType::Revo1hand;
        if (s == "revo2hand") return EndEffectorType::Revo2hand;
        if (s == "linkhandO6") return EndEffectorType::LinkhandO6;
        if (s == "revo1touch") return EndEffectorType::Revo1touch;
        if (s == "dummy") return EndEffectorType::Dummy;
        throw std::invalid_argument("Unknown end effector: " + s);
    }

    void parse() {
        if (raw_.empty()) return;

        static const std::regex pattern(
            R"(^(kuavo|roban)-([a-zA-Z0-9]+)-(biped|wheeled)(?:-((none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy)(_(none|lejuclaw|revo1hand|revo2hand|linkhandO6|revo1touch|dummy))?))?(?:-((none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)_(none|D435|D405|Gemini330|Gemini335L)))?((?:\+[a-zA-Z0-9_]+)*)$)"
        );

        std::smatch match;
        if (!std::regex_match(raw_, match, pattern)) {
            throw std::invalid_argument("Invalid ROBOT_VERSION format: " + raw_);
        }

        brand_ = parse_brand(match[1].str());
        product_ = match[2].str();
        locomotion_ = parse_locomotion(match[3].str());
        end_effector_str_ = match[4].str();
        robot_id_ = match[1].str() + "-" + product_ + "-" + match[3].str() + "-" + end_effector_str_;

        size_t pos = end_effector_str_.find('_');
        if (pos != std::string::npos) {
            left_end_ = parse_end_effector(end_effector_str_.substr(0, pos));
            right_end_ = parse_end_effector(end_effector_str_.substr(pos + 1));
        } else {
            left_end_ = parse_end_effector(end_effector_str_);
            right_end_ = left_end_;
        }

        std::string tags_str = match[8].str();
        if (!tags_str.empty()) {
            size_t start = 0;
            while ((start = tags_str.find('+', start)) != std::string::npos) {
                size_t end = tags_str.find('+', start + 1);
                std::string tag = (end == std::string::npos)
                    ? tags_str.substr(start + 1)
                    : tags_str.substr(start + 1, end - start - 1);
                if (!tag.empty()) tags_.insert(tag);
                start = (end == std::string::npos) ? tags_str.length() : end;
            }
        }
    }
};

// ==================== 兼容宏 ====================

#define IS_KUAVO(rv)            (rv.brand() == robot_version::Brand::Kuavo)
#define IS_ROBAN(rv)            (rv.brand() == robot_version::Brand::Roban)
#define IS_KUAVO_BIPED(rv)      (IS_KUAVO(rv) && rv.locomotion() == robot_version::Locomotion::Biped)
#define IS_KUAVO_WHEELED(rv)    (IS_KUAVO(rv) && rv.locomotion() == robot_version::Locomotion::Wheeled)
#define IS_ROBAN_BIPED(rv)      (IS_ROBAN(rv) && rv.locomotion() == robot_version::Locomotion::Biped)
#define IS_KUAVO4(rv)           (IS_KUAVO(rv) && rv.product()[0] == '4')
#define IS_KUAVO5(rv)           (IS_KUAVO(rv) && rv.product()[0] == '5')
#define IS_KUAVO4PRO(rv)        (IS_KUAVO(rv) && rv.product().find("4pro") == 0)
#define IS_KUAVO4V2(rv)         (IS_KUAVO(rv) && rv.product() == "4V2")
#define IS_KUAVO4PRO_EDU(rv)    (IS_KUAVO(rv) && rv.product() == "4proEDU")
#define IS_KUAVO5V2(rv)         (IS_KUAVO(rv) && rv.product() == "5V2")
#define IS_KUAVO5V3(rv)         (IS_KUAVO(rv) && rv.product() == "5V3")
#define IS_KUAVO5V4(rv)         (IS_KUAVO(rv) && rv.product() == "5V4")
#define IS_ROBAN2V1(rv)         (IS_ROBAN(rv) && rv.product() == "2v1")

// 末端执行器
#define HAS_REVO1HAND(rv)       (rv.left_end() == robot_version::EndEffectorType::Revo1hand || rv.right_end() == robot_version::EndEffectorType::Revo1hand)
#define HAS_REVO2HAND(rv)       (rv.left_end() == robot_version::EndEffectorType::Revo2hand || rv.right_end() == robot_version::EndEffectorType::Revo2hand)
#define HAS_LINKHANDO6(rv)      (rv.left_end() == robot_version::EndEffectorType::LinkhandO6 || rv.right_end() == robot_version::EndEffectorType::LinkhandO6)
#define HAS_REVO1TOUCH(rv)      (rv.left_end() == robot_version::EndEffectorType::Revo1touch || rv.right_end() == robot_version::EndEffectorType::Revo1touch)
#define HAS_LEJUCLAW(rv)        (rv.left_end() == robot_version::EndEffectorType::Lejuclaw || rv.right_end() == robot_version::EndEffectorType::Lejuclaw)
#define HAS_DUMMY(rv)           (rv.left_end() == robot_version::EndEffectorType::Dummy || rv.right_end() == robot_version::EndEffectorType::Dummy)
#define IS_ASYMMETRIC_END(rv)   (rv.left_end() != rv.right_end())

}  // namespace robot_version
