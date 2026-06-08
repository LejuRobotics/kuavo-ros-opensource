#include "kuavo_solver/waist/waist_solver.h"

#include <iostream>
#include <stdexcept>
#include <utility>

#include "kuavo_solver/common/config_dir_macros.h"
#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

namespace {

/** 当前 WaistSolver 仅支持 parallel_rotate 子机构，统一通过 "7gen" token 触发。 */
constexpr const char* kParallelRotateToken = "7gen";
constexpr const char* kDisabledToken       = "none";
constexpr const char* kDisabledAliasToken  = "disabled";

bool IsDisabledToken(const std::string& tok) {
    return tok == kDisabledToken || tok == kDisabledAliasToken;
}

}  // namespace

WaistSolver::~WaistSolver() = default;

// ============================================================================
// 配置加载
// ============================================================================

void WaistSolver::getconfig(const std::string& waist_solver_type_token) {
    const std::string waist_config_dir =
        SolverTools::RequireConfigDir(
            KUAVO_WAIST_CONFIG_DIR, "WaistSolver");

    waist_solver_type_token_ = waist_solver_type_token;
    parallel_rotate_solver_.reset();
    std::cout << "[WaistSolver] waist_solver_type: " << waist_solver_type_token_ << std::endl;

    if (IsDisabledToken(waist_solver_type_token_)) {
        return;
    }

    if (waist_solver_type_token_ == kParallelRotateToken) {
        const auto loaded =
            ParallelRotateWaistSolver::loadParam(waist_solver_type_token_, waist_config_dir);
        parallel_rotate_solver_ =
            std::make_unique<ParallelRotateWaistSolver>(loaded.params);
        return;
    }

    throw std::runtime_error("[WaistSolver] Unknown waist_solver_type: " + waist_solver_type_token_);
}

// ============================================================================
// 3D 分派 helper：将 [yaw, pitch/l_bar, roll/r_bar] 的 3D 接口折叠到
// ParallelRotateWaistSolver 的 2D 接口。disabled 时返回输入的直通副本。
// ============================================================================

namespace {

/** 以 lambda fn 调用 2D solver，其签名为 Vector2d(const Vector2d&)。 */
template <typename Fn2>
Eigen::VectorXd Dispatch1(int yaw_idx, const Eigen::VectorXd& in_3, Fn2 fn2) {
    Eigen::VectorXd out(3);
    out[yaw_idx] = in_3[yaw_idx];
    const Eigen::Vector2d in2(in_3[1], in_3[2]);
    const Eigen::Vector2d out2 = fn2(in2);
    out[1] = out2[0];
    out[2] = out2[1];
    return out;
}

/** fn 签名：Vector2d(const Vector2d& q2, const Vector2d& p2, const Vector2d& x2) */
template <typename Fn3>
Eigen::VectorXd Dispatch3(int yaw_idx,
                          const Eigen::VectorXd& q, const Eigen::VectorXd& p,
                          const Eigen::VectorXd& x, Fn3 fn3) {
    Eigen::VectorXd out(3);
    out[yaw_idx] = x[yaw_idx];
    const Eigen::Vector2d q2(q[1], q[2]);
    const Eigen::Vector2d p2(p[1], p[2]);
    const Eigen::Vector2d x2(x[1], x[2]);
    const Eigen::Vector2d y2 = fn3(q2, p2, x2);
    out[1] = y2[0];
    out[2] = y2[1];
    return out;
}

}  // namespace

// ============================================================================
// 位置 / 速度 / 力矩 接口
// ============================================================================

Eigen::VectorXd WaistSolver::joint_to_motor_position(const Eigen::VectorXd& q) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return q;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch1(kYawIdx, q,
        [this](const Eigen::Vector2d& q2) { return parallel_rotate_solver_->joint_to_motor_position(q2); });
}

Eigen::VectorXd WaistSolver::motor_to_joint_position(const Eigen::VectorXd& p) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return p;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch1(kYawIdx, p,
        [this](const Eigen::Vector2d& p2) { return parallel_rotate_solver_->motor_to_joint_position(p2); });
}

Eigen::VectorXd WaistSolver::motor_to_joint_position(const Eigen::VectorXd& p,
                                                     const Eigen::VectorXd& q_joint_hint) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return p;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    Eigen::VectorXd out(3);
    out[kYawIdx] = p[kYawIdx];
    const Eigen::Vector2d p2(p[1], p[2]);
    const Eigen::Vector2d qh(q_joint_hint[1], q_joint_hint[2]);
    const Eigen::Vector2d q2_out = parallel_rotate_solver_->motor_to_joint_position(p2, qh);
    out[1] = q2_out[0];
    out[2] = q2_out[1];
    return out;
}

Eigen::VectorXd WaistSolver::joint_to_motor_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return dq;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch3(kYawIdx, q, p, dq,
        [this](const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dq2) {
            return parallel_rotate_solver_->joint_to_motor_velocity(q2, p2, dq2);
        });
}

Eigen::VectorXd WaistSolver::motor_to_joint_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return dp;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch3(kYawIdx, q, p, dp,
        [this](const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dp2) {
            return parallel_rotate_solver_->motor_to_joint_velocity(q2, p2, dp2);
        });
}

Eigen::VectorXd WaistSolver::joint_to_motor_current(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return tau;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch3(kYawIdx, q, p, tau,
        [this](const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& tau2) {
            return parallel_rotate_solver_->joint_to_motor_current(q2, p2, tau2);
        });
}

Eigen::VectorXd WaistSolver::motor_to_joint_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) return i;
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    return Dispatch3(kYawIdx, q, p, i,
        [this](const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& i2) {
            return parallel_rotate_solver_->motor_to_joint_torque(q2, p2, i2);
        });
}

// ============================================================================
// 约束雅可比（解析，供严格验证）
// ============================================================================

std::pair<Eigen::Matrix2d, Eigen::Matrix2d> WaistSolver::parallel_jacobian_system(
    const Eigen::VectorXd& q3, const Eigen::VectorXd& p3) {
    if (!parallel_rotate_solver_) {
        if (IsDisabledToken(waist_solver_type_token_)) {
            throw std::logic_error("[WaistSolver] parallel_jacobian_system not available for disabled solver");
        }
        throw std::logic_error("[WaistSolver] parallel_rotate_solver_ not initialized");
    }
    const Eigen::Vector2d q2(q3[1], q3[2]);
    const Eigen::Vector2d p2(p3[1], p3[2]);
    const JacobianSystem jac = parallel_rotate_solver_->jacobian_system(q2, p2);
    // ParallelRotateWaistSolver 的 2D 子机构保证这两个块是 2x2；在此做类型固化。
    return std::make_pair(Eigen::Matrix2d(jac.J_constraint), Eigen::Matrix2d(jac.J_actuator));
}

}  // namespace kuavo_solver
