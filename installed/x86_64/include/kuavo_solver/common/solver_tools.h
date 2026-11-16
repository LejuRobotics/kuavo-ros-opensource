#ifndef KUAVO_SOLVER_SOLVER_TOOLS_H_
#define KUAVO_SOLVER_SOLVER_TOOLS_H_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace kuavo_solver {

/**
 * @brief 通用 LimitRange 结构
 */
struct LimitRange {
    double min{0.0};
    double max{0.0};

    bool is_valid() const { return std::isfinite(min) && std::isfinite(max) && min < max; }
};

/**
 * @brief 通用 Vec3 结构
 */
struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};
};

/**
 * @brief JacobianSystem: unify closed-loop kinematic constraints.
 *
 * Convention:
 *   J_constraint (m×nq), J_actuator (m×np)
 *   J_constraint * dq + J_actuator * dp = 0
 *
 * Dual (force/torque):
 *   J_constraint^T * λ = τ_joint
 *   J_actuator^T  * λ = -τ_motor
 */
struct JacobianSystem {
    Eigen::MatrixXd J_constraint;
    Eigen::MatrixXd J_actuator;
};

/**
 * @brief YAML 参数加载器
 *
 * 使用示例：
 *   auto loader = SolverTools::YamlLoader::Open("config.yaml", 1, "ankle_solver");
 *   loader.require("key1", &value1);
 *   loader.optional("key2", &value2, 0.0);
 *   auto variant = loader.variant("my_variant");
 */
class SolverTools {
public:
    // =========================================================================
    // YamlLoader：YAML 文件加载和解析
    // =========================================================================

    class YamlLoader {
    public:
        /** 打开 YAML 文件并验证版本 */
        static YamlLoader Open(const std::string& dir, const char* filename, int expected_version);

        /** 打开 YAML 文件并获取指定变体 */
        static YamlLoader OpenVariant(const std::string& dir, const char* filename,
                                     int expected_version, const std::string& variant_token);

        /** 默认构造函数（用于链式调用） */
        YamlLoader() = default;

        // -------------------------------------------------------------------------
        // 链式读取：检查 / 读取 / 读取必填字段
        // -------------------------------------------------------------------------

        /** 检查字段是否存在 */
        bool has(const char* key) const;

        /** 检查子节点是否存在 */
        bool has(const YAML::Node& parent, const char* key) const;

        /** 读取必填标量字段，失败抛异常 */
        YamlLoader& require(const char* key, double* out);
        YamlLoader& require(const char* key, int* out);
        YamlLoader& require(const char* key, bool* out);
        YamlLoader& require(const char* key, std::string* out);

        /** 读取必填二元组 [a, b] 到两个独立 double，失败抛异常 */
        YamlLoader& require(const char* key, double* a, double* b);
        YamlLoader& require(const char* key, double* a, double* b, double* c);

        /** 读取可选标量字段，失败使用默认值 */
        YamlLoader& optional(const char* key, double* out, double default_val);
        YamlLoader& optional(const char* key, int* out, int default_val);
        YamlLoader& optional(const char* key, bool* out, bool default_val);

        /** 读取必填二元组 [min, max]，失败抛异常 */
        YamlLoader& require(const char* key, LimitRange* out);

        /** 读取必填三元组 [x, y, z]，失败抛异常 */
        YamlLoader& require(const char* key, Vec3* out);

        // -------------------------------------------------------------------------
        // 节点访问
        // -------------------------------------------------------------------------

        /** 获取解析树根节点（每次从磁盘重新解析，避免 yaml-cpp 遍历破坏内存树） */
        YAML::Node root() const;

        /** 获取 variants 下当前变体节点（每次从磁盘重新解析） */
        YAML::Node variant() const;

        /** 获取当前加载器关联的节点（用于链式调用） */
        const YAML::Node& node() const { return current_; }

        /** 切换到子节点（链式调用） */
        YamlLoader& child(const char* key);

        /** 返回父节点（用于退出子节点） */
        YamlLoader& parent();

    private:
        /** 从 source_path_ + variant_token_ + key_path_ 重建 current_（每次重新 LoadFile） */
        void refresh_current_();

        std::string source_path_;
        std::string variant_token_;
        std::vector<std::string> key_path_;
        YAML::Node current_;
        int expected_version_{-1};
    };

    // =========================================================================
    // 路径与编译期宏
    // =========================================================================

    static std::string JoinPath(const std::string& dir, const char* filename);

    /**
     * @brief 校验编译期宏提供的配置目录字符串
     *
     * 语义：若宏指针为 nullptr 或空字符串，则抛出 runtime_error（禁用 fallback）。
     * 典型用法：SolverTools::RequireConfigDir(KUAVO_ANKLE_CONFIG_DIR, "AnkleSolver")
     */
    static std::string RequireConfigDir(const char* macro_value, const char* solver_name);

    // =========================================================================
    // YAML 辅助
    // =========================================================================

    /** 读取 double 标量，失败返回 false */
    static bool GetScalar(const YAML::Node& n, const char* key, double* out);

    /** 读取 int 标量，失败返回 false */
    static bool GetScalar(const YAML::Node& n, const char* key, int* out);

    /** 读取 bool 标量，失败返回 false */
    static bool GetScalar(const YAML::Node& n, const char* key, bool* out);

    /** 读取 [min, max] 二元组，失败返回 false */
    static bool GetSeq2(const YAML::Node& n, const char* key, double* a, double* b);

    /** 读取 [x, y, z] 三元组，失败返回 false */
    static bool GetVec3(const YAML::Node& n, const char* key, double* x, double* y, double* z);

    /** 读取 [x, y, z] 三元组到数组，失败返回 false */
    static bool GetVec3(const YAML::Node& n, const char* key, double out[3]);

    /** 读取 [x, y, z] 三元组到 Vec3，失败返回 false */
    static bool GetVec3(const YAML::Node& n, const char* key, Vec3* out);

    // =========================================================================
    // 数学工具
    // =========================================================================

    /** 限幅：v \in [lo, hi] */
    static double Clamp(double v, double lo, double hi);

    /** 安全限幅：lo > hi 时返回默认值 */
    static double ClampSafe(double v, double lo, double hi, double default_val = 0.0);

    /**
     * 严格区间检查（fail-fast，替代 Clamp 的静默饱和）：
     *   若 v 不在 [lo - eps, hi + eps] 内，抛 std::runtime_error，
     *   消息中包含 name / 当前值 / [lo, hi] 区间，便于定位越界来源。
     *   通过时原样返回 v（不做截断），保证数学可逆。
     */
    static double RequireInRange(const char* name, double v, double lo, double hi,
                                 double eps = 1e-12);

    /** 检查数值是否有限（非 NaN/Inf） */
    static bool IsFinite(double v);

    /** 检查限位是否有效（lo < hi 且均为有限值） */
    static bool IsValidLimit(double lo, double hi);

    /** 检查限位是否有效 */
    static bool IsValidLimit(const LimitRange& limit);

    /** 限幅辅助：超出则钳位（限位无效时跳过） */
    template <typename Scalar>
    static Scalar ApplyLimit(Scalar v, double lo, double hi) {
        if (IsValidLimit(lo, hi)) {
            return static_cast<Scalar>(Clamp(v, lo, hi));
        }
        return v;
    }

    /** 限幅辅助：使用 LimitRange 结构 */
    template <typename Scalar>
    static Scalar ApplyLimit(Scalar v, const LimitRange& limit) {
        return ApplyLimit(v, limit.min, limit.max);
    }

    /** 向量标准化（返回单位向量） */
    template <typename Derived>
    static Eigen::Vector3d Normalized(const Eigen::MatrixBase<Derived>& v) {
        return v.normalized();
    }

    /** 安全反余弦：把 x 截到 [-1, 1] 再取 acos，避免浮点越界 */
    static double SafeAcos(double x);

    // =========================================================================
    // 几何工具
    // =========================================================================

    /** Vec3 -> Eigen::Vector3d */
    static Eigen::Vector3d ToEigen(const Vec3& v);

    /** 基本旋转矩阵 */
    static Eigen::Matrix3d RotX(double a);
    static Eigen::Matrix3d RotY(double a);
    static Eigen::Matrix3d RotZ(double a);

    /** 基本旋转矩阵对角度的导数（∂R/∂a） */
    static Eigen::Matrix3d RotXDerivative(double a);
    static Eigen::Matrix3d RotYDerivative(double a);
    static Eigen::Matrix3d RotZDerivative(double a);

    /** Rodrigues 公式：绕任意单位轴 axis 旋转 theta 的旋转矩阵 */
    static Eigen::Matrix3d RotationAxisAngle(const Eigen::Vector3d& axis, double theta);

    // =========================================================================
    // JacobianSystem 对偶代数
    //
    // 约束：J_constraint * dq + J_actuator * dp = 0
    // 对偶：J_constraint^T * λ = τ_joint, J_actuator^T * λ = -τ_motor
    // =========================================================================

    /** 关节速度 → 电机速度：dp = -J_actuator^{-1} * J_constraint * dq */
    static Eigen::VectorXd MotorVelocityFromJoint(const JacobianSystem& jac,
                                                  const Eigen::VectorXd& dq);

    /** 电机速度 → 关节速度：dq = -J_constraint^{-1} * J_actuator * dp */
    static Eigen::VectorXd JointVelocityFromMotor(const JacobianSystem& jac,
                                                  const Eigen::VectorXd& dp);

    /** 关节力矩 → 电机电流（正比力矩）：τ_motor = -J_actuator^T * (J_constraint^T)^{-1} * τ_joint */
    static Eigen::VectorXd MotorCurrentFromJointTorque(const JacobianSystem& jac,
                                                       const Eigen::VectorXd& tau_joint);

    /** 电机电流（正比力矩） → 关节力矩：τ_joint = -J_constraint^T * J_actuator^{-1} * τ_motor */
    static Eigen::VectorXd JointTorqueFromMotorCurrent(const JacobianSystem& jac,
                                                       const Eigen::VectorXd& i_motor);

    // =========================================================================
    // Round-trip 验证
    // =========================================================================

    /** 统一的 Mujoco 往返验证结果（各解算器共用） */
    struct VerificationResult {
        double position_error_norm{0.0};
        double velocity_error_norm{0.0};
        double torque_error_norm{0.0};
        bool is_valid{false};
    };

    /**
     * @brief 使用 round-trip 方式验证正逆解的一致性
     *
     * 步骤：
     *   p_computed = fwd_pos(q);              q_back     = inv_pos(p_computed)
     *   dp_computed = fwd_vel(q, p, dq);      dq_back    = inv_vel(q, p, dp_computed)
     *   i_computed  = fwd_cur(q, p, tau);     tau_back   = inv_torque(q, p, i_computed)
     *
     * 速度/力矩项在对应输入为空或范数过小时跳过（视为 0 误差）。
     * is_valid 要求三项误差均小于各自容差。
     */
    template <typename FwdPos, typename InvPos,
              typename FwdVel, typename InvVel,
              typename FwdCur, typename InvTorque>
    static VerificationResult VerifyRoundTrip(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p,
        const Eigen::VectorXd& dq, const Eigen::VectorXd& tau,
        double tol_pos, double tol_vel, double tol_torque,
        FwdPos fwd_pos, InvPos inv_pos,
        FwdVel fwd_vel, InvVel inv_vel,
        FwdCur fwd_cur, InvTorque inv_torque) {
        VerificationResult r;

        const Eigen::VectorXd p_computed = fwd_pos(q);
        const Eigen::VectorXd q_back = inv_pos(p_computed);
        r.position_error_norm = (q - q_back).norm();

        if (dq.size() == q.size() && dq.norm() > 1e-10) {
            const Eigen::VectorXd dp_computed = fwd_vel(q, p, dq);
            const Eigen::VectorXd dq_back = inv_vel(q, p, dp_computed);
            r.velocity_error_norm = (dq - dq_back).norm();
        } else {
            r.velocity_error_norm = 0.0;
        }

        if (tau.size() == q.size() && tau.norm() > 1e-10) {
            const Eigen::VectorXd i_computed = fwd_cur(q, p, tau);
            const Eigen::VectorXd tau_back = inv_torque(q, p, i_computed);
            r.torque_error_norm = (tau - tau_back).norm();
        } else {
            r.torque_error_norm = 0.0;
        }

        r.is_valid = (r.position_error_norm < tol_pos) &&
                     (r.velocity_error_norm < tol_vel) &&
                     (r.torque_error_norm < tol_torque);
        return r;
    }
};

}  // namespace kuavo_solver

#endif  // KUAVO_SOLVER_SOLVER_TOOLS_H_
