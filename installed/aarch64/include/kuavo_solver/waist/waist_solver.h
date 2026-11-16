#ifndef KUAVO_SOLVER_WAIST_SOLVER_H_
#define KUAVO_SOLVER_WAIST_SOLVER_H_

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>
#include <string>

#include "kuavo_solver/waist/parallel_rotate_waist_solver.h"

namespace kuavo_solver {

enum class WaistSolverType {
    WAIST_SOLVER_TYPE_PARALLEL_ROTATE,
    WAIST_SOLVER_TYPE_NONE,
};

inline const char* WaistSolverTypeToString(WaistSolverType type) {
    if (type == WaistSolverType::WAIST_SOLVER_TYPE_PARALLEL_ROTATE) return "parallel_rotate";
    if (type == WaistSolverType::WAIST_SOLVER_TYPE_NONE) return "none";
    return "unknown";
}

/**
 * @brief WaistSolver — 腰部 3D 接口
 *
 * q3 = [yaw, pitch, roll]，p3 = [yaw, l_bar, r_bar]，单位 rad。
 */
class WaistSolver
{
public:
    Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);
    /** 由 p 求 q；pitch/roll 迭代初值取 q_joint_hint 的对应分量。 */
    Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p, const Eigen::VectorXd& q_joint_hint);

    Eigen::VectorXd joint_to_motor_velocity(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq);
    Eigen::VectorXd motor_to_joint_velocity(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);

    Eigen::VectorXd joint_to_motor_current(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau);
    Eigen::VectorXd motor_to_joint_torque(
        const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i);

    /** 约束雅可比 (J_constraint, J_actuator)，2×2 块。 */
    std::pair<Eigen::Matrix2d, Eigen::Matrix2d> parallel_jacobian_system(
        const Eigen::VectorXd& q3, const Eigen::VectorXd& p3);

    void getconfig(const std::string& waist_solver_type_token);

    ~WaistSolver();

private:
    static constexpr int kYawIdx = 0;

    std::string waist_solver_type_token_;
    std::unique_ptr<ParallelRotateWaistSolver> parallel_rotate_solver_;
};

}  // namespace kuavo_solver

#endif  // KUAVO_SOLVER_WAIST_SOLVER_H_
