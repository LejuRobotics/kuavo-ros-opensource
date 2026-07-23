#include "kuavo_solver/common/solver_tools.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace kuavo_solver {

// ============================================================================
// YamlLoader 实现
// ============================================================================

SolverTools::YamlLoader SolverTools::YamlLoader::Open(const std::string& dir, const char* filename,
                                                      int expected_version) {
    const std::string path = SolverTools::JoinPath(dir, filename);

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("YamlLoader: failed to load ") + path + " — " + e.what());
    }

    if (!root["version"] || !root["version"].IsScalar() || root["version"].as<int>() != expected_version) {
        throw std::runtime_error("YamlLoader: unsupported version (expected " + std::to_string(expected_version) + ")");
    }

    YamlLoader loader;
    loader.source_path_ = path;
    loader.expected_version_ = expected_version;
    loader.variant_token_.clear();
    loader.key_path_.clear();
    loader.refresh_current_();
    return loader;
}

SolverTools::YamlLoader SolverTools::YamlLoader::OpenVariant(const std::string& dir, const char* filename,
                                                             int expected_version,
                                                             const std::string& variant_token) {
    const std::string path = SolverTools::JoinPath(dir, filename);

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("YamlLoader: failed to load ") + path + " — " + e.what());
    }

    if (!root["version"] || !root["version"].IsScalar() || root["version"].as<int>() != expected_version) {
        throw std::runtime_error("YamlLoader: unsupported version (expected " + std::to_string(expected_version) + ")");
    }

    if (!root["variants"] || !root["variants"].IsMap()) {
        throw std::runtime_error("YamlLoader: missing variants map");
    }

    const YAML::Node variant = root["variants"][variant_token];
    if (!variant || !variant.IsMap()) {
        throw std::runtime_error(std::string("YamlLoader: variant not found: ") + variant_token);
    }

    YamlLoader loader;
    loader.source_path_ = path;
    loader.expected_version_ = expected_version;
    loader.variant_token_ = variant_token;
    loader.key_path_.clear();
    loader.refresh_current_();
    return loader;
}

void SolverTools::YamlLoader::refresh_current_() {
    if (source_path_.empty()) {
        throw std::runtime_error("YamlLoader: internal error (missing source path)");
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(source_path_);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("YamlLoader: failed to load ") + source_path_ + " — " + e.what());
    }

    if (!root["version"] || !root["version"].IsScalar() || root["version"].as<int>() != expected_version_) {
        throw std::runtime_error("YamlLoader: unsupported version (expected " + std::to_string(expected_version_) + ")");
    }

    YAML::Node n = root;
    if (!variant_token_.empty()) {
        if (!root["variants"] || !root["variants"].IsMap()) {
            throw std::runtime_error("YamlLoader: missing variants map");
        }
        n = root["variants"][variant_token_];
        if (!n || !n.IsMap()) {
            throw std::runtime_error(std::string("YamlLoader: variant not found: ") + variant_token_);
        }
    }

    for (const auto& pk : key_path_) {
        n = n[pk];
    }
    current_ = std::move(n);
}

YAML::Node SolverTools::YamlLoader::root() const {
    if (source_path_.empty()) {
        return {};
    }
    try {
        return YAML::LoadFile(source_path_);
    } catch (const YAML::Exception&) {
        return {};
    }
}

YAML::Node SolverTools::YamlLoader::variant() const {
    if (source_path_.empty() || variant_token_.empty()) {
        return {};
    }
    try {
        YAML::Node r = YAML::LoadFile(source_path_);
        return r["variants"][variant_token_];
    } catch (const YAML::Exception&) {
        return {};
    }
}

bool SolverTools::YamlLoader::has(const char* key) const {
    return has(current_, key);
}

bool SolverTools::YamlLoader::has(const YAML::Node& parent, const char* key) const {
    const std::string k(key ? key : "");
    const YAML::Node& p = parent;
    const YAML::Node v = p[k];
    return v && v.IsDefined();
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, double* out) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsScalar()) {
        throw std::runtime_error(std::string("YamlLoader: required scalar missing: ") + key);
    }
    *out = v.as<double>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, int* out) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsScalar()) {
        throw std::runtime_error(std::string("YamlLoader: required scalar missing: ") + k);
    }
    *out = v.as<int>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, bool* out) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsScalar()) {
        throw std::runtime_error(std::string("YamlLoader: required scalar missing: ") + k);
    }
    *out = v.as<bool>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, std::string* out) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsScalar()) {
        throw std::runtime_error(std::string("YamlLoader: required scalar missing: ") + k);
    }
    *out = v.as<std::string>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::optional(const char* key, double* out, double default_val) {
    if (has(key)) {
        const std::string k(key ? key : "");
        const YAML::Node& c = current_;
        const YAML::Node v = c[k];
        if (v && v.IsScalar()) {
            *out = v.as<double>();
        }
    } else {
        *out = default_val;
    }
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::optional(const char* key, int* out, int default_val) {
    if (has(key)) {
        const std::string k(key ? key : "");
        const YAML::Node& c = current_;
        const YAML::Node v = c[k];
        if (v && v.IsScalar()) {
            *out = v.as<int>();
        }
    } else {
        *out = default_val;
    }
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::optional(const char* key, bool* out, bool default_val) {
    if (has(key)) {
        const std::string k(key ? key : "");
        const YAML::Node& c = current_;
        const YAML::Node v = c[k];
        if (v && v.IsScalar()) {
            *out = v.as<bool>();
        }
    } else {
        *out = default_val;
    }
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, LimitRange* out) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsSequence() || v.size() != 2) {
        throw std::runtime_error(std::string("YamlLoader: required [min,max] missing: ") + k);
    }
    out->min = v[0].as<double>();
    out->max = v[1].as<double>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, double* a, double* b) {
    const std::string k(key ? key : "");
    const YAML::Node& c = current_;
    const YAML::Node v = c[k];
    if (!v || !v.IsSequence() || v.size() != 2) {
        throw std::runtime_error(std::string("YamlLoader: required [a,b] missing: ") + k);
    }
    *a = v[0].as<double>();
    *b = v[1].as<double>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, double* a, double* b, double* c) {
    const std::string k(key ? key : "");
    const YAML::Node& cur = current_;
    const YAML::Node v = cur[k];
    if (!v || !v.IsSequence() || v.size() != 3) {
        throw std::runtime_error(std::string("YamlLoader: required [a,b,c] missing: ") + k);
    }
    *a = v[0].as<double>();
    *b = v[1].as<double>();
    *c = v[2].as<double>();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::require(const char* key, Vec3* out) {
    return require(key, &out->x, &out->y, &out->z);
}

SolverTools::YamlLoader& SolverTools::YamlLoader::child(const char* key) {
    const std::string k(key ? key : "");
    key_path_.push_back(k);
    refresh_current_();
    return *this;
}

SolverTools::YamlLoader& SolverTools::YamlLoader::parent() {
    if (!key_path_.empty()) {
        key_path_.pop_back();
        refresh_current_();
    }
    return *this;
}

// ============================================================================
// 路径与编译期宏
// ============================================================================

std::string SolverTools::JoinPath(const std::string& dir, const char* filename) {
    if (dir.empty()) {
        return filename;
    }
    if (dir.back() == '/') {
        return dir + filename;
    }
    return dir + '/' + filename;
}

std::string SolverTools::RequireConfigDir(const char* macro_value, const char* solver_name) {
    if (macro_value == nullptr || macro_value[0] == '\0') {
        const std::string name = solver_name ? solver_name : "solver";
        throw std::runtime_error(
            "[" + name + "] config_dir macro is required (no fallback)");
    }
    return std::string(macro_value);
}

// ============================================================================
// YAML 辅助
// ============================================================================

bool SolverTools::GetScalar(const YAML::Node& n, const char* key, double* out) {
    const std::string k(key ? key : "");
    const YAML::Node v = n[k];
    if (!v || !v.IsScalar()) {
        return false;
    }
    *out = v.as<double>();
    return true;
}

bool SolverTools::GetScalar(const YAML::Node& n, const char* key, int* out) {
    const std::string k(key ? key : "");
    const YAML::Node v = n[k];
    if (!v || !v.IsScalar()) {
        return false;
    }
    *out = v.as<int>();
    return true;
}

bool SolverTools::GetScalar(const YAML::Node& n, const char* key, bool* out) {
    const std::string k(key ? key : "");
    const YAML::Node v = n[k];
    if (!v || !v.IsScalar()) {
        return false;
    }
    *out = v.as<bool>();
    return true;
}

bool SolverTools::GetSeq2(const YAML::Node& n, const char* key, double* a, double* b) {
    const std::string k(key ? key : "");
    const YAML::Node v = n[k];
    if (!v || !v.IsSequence() || v.size() != 2) {
        return false;
    }
    *a = v[0].as<double>();
    *b = v[1].as<double>();
    return true;
}

bool SolverTools::GetVec3(const YAML::Node& n, const char* key, double* x, double* y, double* z) {
    const std::string k(key ? key : "");
    const YAML::Node v = n[k];
    if (!v || !v.IsSequence() || v.size() != 3) {
        return false;
    }
    *x = v[0].as<double>();
    *y = v[1].as<double>();
    *z = v[2].as<double>();
    return true;
}

bool SolverTools::GetVec3(const YAML::Node& n, const char* key, double out[3]) {
    return GetVec3(n, key, &out[0], &out[1], &out[2]);
}

bool SolverTools::GetVec3(const YAML::Node& n, const char* key, Vec3* out) {
    return GetVec3(n, key, &out->x, &out->y, &out->z);
}

// ============================================================================
// 数学工具
// ============================================================================

double SolverTools::Clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

double SolverTools::ClampSafe(double v, double lo, double hi, double default_val) {
    if (IsValidLimit(lo, hi)) {
        return Clamp(v, lo, hi);
    }
    return default_val;
}

double SolverTools::RequireInRange(const char* name, double v, double lo, double hi, double eps) {
    if (!(v >= lo - eps) || !(v <= hi + eps)) {
        std::ostringstream oss;
        oss << "SolverTools::RequireInRange: '" << (name ? name : "<unnamed>")
            << "'=" << v << " 越界，期望 [" << lo << ", " << hi << "]（eps=" << eps << "）";
        throw std::runtime_error(oss.str());
    }
    return v;
}

bool SolverTools::IsFinite(double v) {
    return std::isfinite(v);
}

bool SolverTools::IsValidLimit(double lo, double hi) {
    return std::isfinite(lo) && std::isfinite(hi) && lo < hi;
}

bool SolverTools::IsValidLimit(const LimitRange& limit) {
    return IsValidLimit(limit.min, limit.max);
}

double SolverTools::SafeAcos(double x) {
    return std::acos(std::max(-1.0, std::min(1.0, x)));
}

// ============================================================================
// 几何工具
// ============================================================================

Eigen::Vector3d SolverTools::ToEigen(const Vec3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

Eigen::Matrix3d SolverTools::RotX(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, c, -s,
         0, s,  c;
    return R;
}

Eigen::Matrix3d SolverTools::RotY(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d R;
    R <<  c, 0, s,
          0, 1, 0,
         -s, 0, c;
    return R;
}

Eigen::Matrix3d SolverTools::RotZ(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

Eigen::Matrix3d SolverTools::RotXDerivative(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d dR;
    dR << 0,  0,  0,
          0, -s, -c,
          0,  c, -s;
    return dR;
}

Eigen::Matrix3d SolverTools::RotYDerivative(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d dR;
    dR << -s, 0,  c,
           0, 0,  0,
          -c, 0, -s;
    return dR;
}

Eigen::Matrix3d SolverTools::RotZDerivative(double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Eigen::Matrix3d dR;
    dR << -s, -c, 0,
           c, -s, 0,
           0,  0, 0;
    return dR;
}

Eigen::Matrix3d SolverTools::RotationAxisAngle(const Eigen::Vector3d& axis, double theta) {
    const Eigen::Vector3d k = axis.normalized();
    Eigen::Matrix3d K;
    K <<  0.0, -k(2),  k(1),
          k(2),  0.0, -k(0),
         -k(1),  k(0),  0.0;
    return Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * (K * K);
}

// ============================================================================
// JacobianSystem 对偶代数
//
// Convention: J_constraint * dq + J_actuator * dp = 0
//             J_constraint^T * λ = τ_joint, J_actuator^T * λ = -τ_motor
// ============================================================================

Eigen::VectorXd SolverTools::MotorVelocityFromJoint(const JacobianSystem& jac,
                                                    const Eigen::VectorXd& dq) {
    return -jac.J_actuator.colPivHouseholderQr().solve(jac.J_constraint * dq);
}

Eigen::VectorXd SolverTools::JointVelocityFromMotor(const JacobianSystem& jac,
                                                    const Eigen::VectorXd& dp) {
    return -jac.J_constraint.colPivHouseholderQr().solve(jac.J_actuator * dp);
}

Eigen::VectorXd SolverTools::MotorCurrentFromJointTorque(const JacobianSystem& jac,
                                                         const Eigen::VectorXd& tau_joint) {
    const Eigen::VectorXd lambda =
        jac.J_constraint.transpose().colPivHouseholderQr().solve(tau_joint);
    return -jac.J_actuator.transpose() * lambda;
}

Eigen::VectorXd SolverTools::JointTorqueFromMotorCurrent(const JacobianSystem& jac,
                                                         const Eigen::VectorXd& i_motor) {
    const Eigen::VectorXd lambda =
        jac.J_actuator.colPivHouseholderQr().solve(i_motor);
    return -jac.J_constraint.transpose() * lambda;
}

}  // namespace kuavo_solver
