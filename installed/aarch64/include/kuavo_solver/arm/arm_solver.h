#ifndef _arm_solver_h_
#define _arm_solver_h_

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "kuavo_solver/arm/parallel_linear_arm_solver.h"

enum ArmSolverType {
    ARM_SOLVER_TYPE_7GEN = 0,
    ARM_SOLVER_TYPE_NONE = -1,
};

struct ArmSolverTypeEntry {
    const char* token;
    ArmSolverType type;
};

inline const std::vector<ArmSolverTypeEntry>& GetArmSolverTypeTable() {
    static const std::vector<ArmSolverTypeEntry> kTable = {
        {"7gen", ArmSolverType::ARM_SOLVER_TYPE_7GEN},
        {"none", ArmSolverType::ARM_SOLVER_TYPE_NONE},
    };
    return kTable;
}

inline ArmSolverType ArmSolverTypeFromString(const std::string& arm_solver_type) {
    for (const auto& entry : GetArmSolverTypeTable()) {
        if (arm_solver_type == entry.token) {
            return entry.type;
        }
    }
    throw std::invalid_argument("unknown arm_solver_type token: " + arm_solver_type);
}

/**
 * @brief ArmSolver — 手臂运动学解算器工厂（统一 14D 接口）
 *
 * 类似 AnkleSolver 的工厂模式，通过 getconfig() 选择具体实现。
 * 当前支持 7gen (ParallelLinearArmSolver)。
 *
 * 对外契约:
 *   - 所有接口仅接受 14D 向量（双臂拼接: [L7, R7]）。
 *   - 单臂(7D)适配由调用边界负责，不在 ArmSolver 内部做维度分叉。
 */
class ArmSolver
{
public:
    Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);

    Eigen::VectorXd joint_to_motor_velocity(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq);
    Eigen::VectorXd motor_to_joint_velocity(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);

    Eigen::VectorXd joint_to_motor_current(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau);
    Eigen::VectorXd motor_to_joint_torque(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i);

    /** arm_solver_type_token 由上层版本映射提供；config_dir 由编译期宏 KUAVO_ARM_CONFIG_DIR 提供（缺失直接 throw）。 */
    void getconfig(const std::string& arm_solver_type_token);

    ~ArmSolver();

private:
    std::string arm_solver_type_token_;
    std::unique_ptr<kuavo_solver::ParallelLinearArmSolver> parallel_linear_solver_;
};

#endif // _arm_solver_h_
