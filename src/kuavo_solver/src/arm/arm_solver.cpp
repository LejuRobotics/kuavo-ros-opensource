#include "kuavo_solver/arm/arm_solver.h"

#include <iostream>
#include <memory>
#include <stdexcept>

#include "kuavo_solver/common/config_dir_macros.h"
#include "kuavo_solver/common/solver_tools.h"

namespace {

/**
 * @brief 判断当前 ArmSolver 是否处于“直通”模式（type == NONE）
 *
 * 契约：直通模式下所有接口按原值返回；非直通模式下若底层 solver 未初始化则报错。
 */
inline bool IsArmDisabled(const std::string& token) {
    return ArmSolverTypeFromString(token) == ArmSolverType::ARM_SOLVER_TYPE_NONE;
}

}  // namespace

ArmSolver::~ArmSolver() = default;

// ============================================================================
// 配置加载
// ============================================================================

void ArmSolver::getconfig(const std::string& arm_solver_type_token) {
    const std::string arm_config_dir =
        kuavo_solver::SolverTools::RequireConfigDir(KUAVO_ARM_CONFIG_DIR, "ArmSolver");

    arm_solver_type_token_ = arm_solver_type_token;
    parallel_linear_solver_.reset();
    std::cout << "[ArmSolver] arm_solver_type: " << arm_solver_type_token_ << std::endl;

    const ArmSolverType type = ArmSolverTypeFromString(arm_solver_type_token);

    if (type == ArmSolverType::ARM_SOLVER_TYPE_NONE) {
        return;
    }

    if (type == ArmSolverType::ARM_SOLVER_TYPE_7GEN) {
        const auto loaded =
            kuavo_solver::ParallelLinearArmSolver::loadParam(arm_solver_type_token_, arm_config_dir);
        parallel_linear_solver_ =
            std::make_unique<kuavo_solver::ParallelLinearArmSolver>(loaded.params);
        return;
    }

    throw std::runtime_error("[ArmSolver] Unknown arm_solver_type: " + arm_solver_type_token_);
}

// ============================================================================
// 位置 / 速度 / 力矩 接口
//
// 统一契约：
//   - 若 solver_type == NONE：所有接口直通；
//   - 若 solver_type 已知但底层 unique_ptr 为空：以 logic_error 报告初始化错误；
//   - 否则委托给 ParallelLinearArmSolver。
// ============================================================================

Eigen::VectorXd ArmSolver::joint_to_motor_position(const Eigen::VectorXd& q) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return q;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->joint_to_motor_position(q);
}

Eigen::VectorXd ArmSolver::motor_to_joint_position(const Eigen::VectorXd& p) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return p;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->motor_to_joint_position(p);
}

Eigen::VectorXd ArmSolver::joint_to_motor_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return dq;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->joint_to_motor_velocity(q, p, dq);
}

Eigen::VectorXd ArmSolver::motor_to_joint_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return dp;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->motor_to_joint_velocity(q, p, dp);
}

Eigen::VectorXd ArmSolver::joint_to_motor_current(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return tau;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->joint_to_motor_current(q, p, tau);
}

Eigen::VectorXd ArmSolver::motor_to_joint_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i) {
    if (!parallel_linear_solver_) {
        if (IsArmDisabled(arm_solver_type_token_)) return i;
        throw std::logic_error("[ArmSolver] parallel_linear_solver_ not initialized");
    }
    return parallel_linear_solver_->motor_to_joint_torque(q, p, i);
}
