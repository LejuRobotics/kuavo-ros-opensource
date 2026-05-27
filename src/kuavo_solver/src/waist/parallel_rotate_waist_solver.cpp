#include "kuavo_solver/waist/parallel_rotate_waist_solver.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

// ============================================================================
// 0) 构造 / 参数加载
// ============================================================================

ParallelRotateWaistSolver::ParallelRotateWaistSolver(const ParallelRotateWaistParams& params) : params_(params) {
    bar_pivot_left_       = SolverTools::ToEigen(params_.bar_pivot_left);
    bar_pivot_right_      = SolverTools::ToEigen(params_.bar_pivot_right);
    tendon_joint_in_bar_  = SolverTools::ToEigen(params_.tendon_joint_in_bar);
    pitch_origin_in_yaw_  = SolverTools::ToEigen(params_.pitch_origin_in_yaw);
    roll_origin_in_pitch_ = SolverTools::ToEigen(params_.roll_origin_in_pitch);
    ball_left_in_roll_    = SolverTools::ToEigen(params_.ball_center_left_in_roll);
    ball_right_in_roll_   = SolverTools::ToEigen(params_.ball_center_right_in_roll);
    L_m_ = params_.rod_length_m;
    if (!(L_m_ > 0.0) || !std::isfinite(L_m_)) {
        throw std::invalid_argument("ParallelRotateWaistSolver: rod_length_m must be > 0");
    }
    default_tolerance_ = params_.default_tolerance;
    max_iterations_ = params_.max_iterations;
}

ParallelRotateWaistSolver::LoadedParam ParallelRotateWaistSolver::loadParam(
    const std::string& waist_solver_type_token, const std::string& waist_config_dir) {
    auto loader = SolverTools::YamlLoader::OpenVariant(
        waist_config_dir, "parallel_rotate_waist_solver.yaml", 1, waist_solver_type_token);

    ParallelRotateWaistParams p;
    loader.require("bar_pivot_left",            &p.bar_pivot_left)
          .require("bar_pivot_right",           &p.bar_pivot_right)
          .require("tendon_joint_in_bar",       &p.tendon_joint_in_bar)
          .require("pitch_origin_in_yaw",       &p.pitch_origin_in_yaw)
          .require("roll_origin_in_pitch",      &p.roll_origin_in_pitch)
          .require("ball_center_left_in_roll",  &p.ball_center_left_in_roll)
          .require("ball_center_right_in_roll", &p.ball_center_right_in_roll)
          .require("rod_length_m",              &p.rod_length_m);

    loader.child("ik")
          .require("default_tolerance", &p.default_tolerance)
          .require("max_iterations", &p.max_iterations)
          .parent();

    loader.child("joint_limits")
          .require("pitch", &p.joint_pitch)
          .require("roll",  &p.joint_roll);
    loader.parent();

    loader.child("motor_limits")
          .require("bar", &p.motor_bar);

    return {p};
}

Eigen::Vector3d ParallelRotateWaistSolver::ToEigen(const Vec3& v) {
    return SolverTools::ToEigen(v);
}

// ============================================================================
// 1) 连杆向量（Linkage Vectors）
// ----------------------------------------------------------------------------
// 单侧刚性杆：A(bar) − B(pitch, roll)，均表达在 waist_yaw 坐标系。
//   A = bar_pivot + R_y(bar) * tendon_joint_in_bar
//   B = pitch_origin_in_yaw + R_y(pitch) * (roll_origin_in_pitch + R_x(roll) * ball_in_roll)
// ============================================================================

Eigen::Vector3d ParallelRotateWaistSolver::bar_end_in_yaw_(double bar_angle, bool is_left) const {
    const Eigen::Vector3d& pivot = is_left ? bar_pivot_left_ : bar_pivot_right_;
    return pivot + SolverTools::RotY(bar_angle) * tendon_joint_in_bar_;
}

Eigen::Vector3d ParallelRotateWaistSolver::ball_center_in_yaw_(double pitch, double roll, bool is_left) const {
    const Eigen::Vector3d& ball_in_roll = is_left ? ball_left_in_roll_ : ball_right_in_roll_;
    const Eigen::Vector3d p_in_pitch = roll_origin_in_pitch_ + SolverTools::RotX(roll) * ball_in_roll;
    return pitch_origin_in_yaw_ + SolverTools::RotY(pitch) * p_in_pitch;
}

Eigen::Vector3d ParallelRotateWaistSolver::linkage_vector_(
    double pitch, double roll, double bar_angle, bool is_left) const {
    return bar_end_in_yaw_(bar_angle, is_left) - ball_center_in_yaw_(pitch, roll, is_left);
}

// ============================================================================
// 2) 雅可比（Jacobians & JacobianSystem）
// ----------------------------------------------------------------------------
// 约束（长度形式）：f_side = |A − B| − L_rod = 0
//   ∂L/∂pitch = (1/L) * (A − B) · (-∂B/∂pitch)
//   ∂L/∂roll  = (1/L) * (A − B) · (-∂B/∂roll)
//   ∂L/∂bar   = (1/L) * (A − B) ·  ∂A/∂bar
// ============================================================================

ParallelRotateWaistSolver::LinkageLengthJacobian
ParallelRotateWaistSolver::linkage_length_jacobian_(
    double pitch, double roll, double bar_angle, bool is_left) const {
    const Eigen::Vector3d& ball_in_roll = is_left ? ball_left_in_roll_ : ball_right_in_roll_;

    const Eigen::Vector3d A = bar_end_in_yaw_(bar_angle, is_left);
    const Eigen::Vector3d B = ball_center_in_yaw_(pitch, roll, is_left);
    const Eigen::Vector3d e = A - B;
    const double L = e.norm();
    if (L < 1e-12) {
        throw std::runtime_error("linkage_length_jacobian_: 刚性杆长度过小（构型奇异）");
    }
    const Eigen::Vector3d n_hat = e / L;

    const Eigen::Vector3d p_in_pitch = roll_origin_in_pitch_ + SolverTools::RotX(roll) * ball_in_roll;
    const Eigen::Vector3d dB_dpitch  = SolverTools::RotYDerivative(pitch) * p_in_pitch;
    const Eigen::Vector3d dB_droll   = SolverTools::RotY(pitch) * SolverTools::RotXDerivative(roll) * ball_in_roll;
    const Eigen::Vector3d dA_dbar    = SolverTools::RotYDerivative(bar_angle) * tendon_joint_in_bar_;

    LinkageLengthJacobian j;
    j.joint(0) = -n_hat.dot(dB_dpitch);
    j.joint(1) = -n_hat.dot(dB_droll);
    j.actuator =  n_hat.dot(dA_dbar);
    j.length   = L;
    return j;
}

JacobianSystem ParallelRotateWaistSolver::jacobian_system_impl_(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2) const {
    const LinkageLengthJacobian jl = linkage_length_jacobian_(q2[0], q2[1], p2[0], /*is_left=*/true);
    const LinkageLengthJacobian jr = linkage_length_jacobian_(q2[0], q2[1], p2[1], /*is_left=*/false);

    JacobianSystem out;
    out.J_constraint = Eigen::Matrix2d::Zero();
    out.J_constraint.row(0) = jl.joint;
    out.J_constraint.row(1) = jr.joint;

    out.J_actuator = Eigen::Matrix2d::Zero();
    out.J_actuator(0, 0) = jl.actuator;
    out.J_actuator(1, 1) = jr.actuator;
    return out;
}

JacobianSystem ParallelRotateWaistSolver::jacobian_system(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2) const {
    return jacobian_system_impl_(q2, p2);
}

// 3) FK：joint→motor；IK：motor→joint。FK 余弦闭式；IK 残差 ‖e‖−L，Gauss–Newton。

double ParallelRotateWaistSolver::forward_kinematics_impl_side_(
    double pitch, double roll, bool is_left) const {
    // D = bar_pivot − B(pitch, roll)
    const Eigen::Vector3d B = ball_center_in_yaw_(pitch, roll, is_left);
    const Eigen::Vector3d& pivot = is_left ? bar_pivot_left_ : bar_pivot_right_;
    const Eigen::Vector3d D = pivot - B;

    const double tj_x = tendon_joint_in_bar_[0];
    const double tj_z = tendon_joint_in_bar_[2];
    const double a    = std::sqrt(tj_x * tj_x + tj_z * tj_z);
    const double phi  = std::atan2(tj_z, tj_x);
    const double R    = std::sqrt(D[0] * D[0] + D[2] * D[2]);
    const double psi  = std::atan2(D[2], D[0]);

    if (a < 1e-12 || R < 1e-12) {
        throw std::runtime_error(
            "ParallelRotateWaistSolver::forward_kinematics_impl_side_: 几何退化（a≈0 或 R≈0）");
    }

    const double arg = (L_m_ * L_m_ - D.squaredNorm() - a * a) / (2.0 * a * R);
    return phi - psi + SolverTools::SafeAcos(arg);
}

Eigen::Vector2d ParallelRotateWaistSolver::inverse_kinematics_impl_(
    const Eigen::Vector2d& bars, const Eigen::Vector2d& init_q) const {
    // Gauss–Newton；无 LM damping、无 line search。
    Eigen::Vector2d q = init_q;
    const double tol = default_tolerance_;
    const int max_iter = max_iterations_;

    for (int it = 0; it < max_iter; ++it) {
        const LinkageLengthJacobian jl = linkage_length_jacobian_(q[0], q[1], bars[0], /*is_left=*/true);
        const LinkageLengthJacobian jr = linkage_length_jacobian_(q[0], q[1], bars[1], /*is_left=*/false);

        Eigen::Vector2d residual;
        residual << jl.length - L_m_, jr.length - L_m_;
        if (residual.norm() < tol) break;

        Eigen::Matrix2d J_constraint;
        J_constraint.row(0) = jl.joint;
        J_constraint.row(1) = jr.joint;

        const Eigen::Vector2d delta = J_constraint.colPivHouseholderQr().solve(residual);
        q -= delta;

        if (delta.norm() < tol) break;
    }
    return q;
}

// 4) 速度 / 力矩：SolverTools::*FromJoint / *FromMotor

Eigen::Vector2d ParallelRotateWaistSolver::joint_to_motor_velocity(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dq2) const {
    const JacobianSystem jac = jacobian_system_impl_(q2, p2);
    return SolverTools::MotorVelocityFromJoint(jac, dq2);
}

Eigen::Vector2d ParallelRotateWaistSolver::motor_to_joint_velocity(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dp2) const {
    const JacobianSystem jac = jacobian_system_impl_(q2, p2);
    return SolverTools::JointVelocityFromMotor(jac, dp2);
}

Eigen::Vector2d ParallelRotateWaistSolver::joint_to_motor_current(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& tau2) const {
    const JacobianSystem jac = jacobian_system_impl_(q2, p2);
    return SolverTools::MotorCurrentFromJointTorque(jac, tau2);
}

Eigen::Vector2d ParallelRotateWaistSolver::motor_to_joint_torque(
    const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& i2) const {
    const JacobianSystem jac = jacobian_system_impl_(q2, p2);
    return SolverTools::JointTorqueFromMotorCurrent(jac, i2);
}

// 5) 2D 路由

Eigen::Vector2d ParallelRotateWaistSolver::joint_to_motor_position(const Eigen::Vector2d& q2) const {
    Eigen::Vector2d bars;
    bars[0] = forward_kinematics_impl_side_(q2[0], q2[1], /*is_left=*/true);
    bars[1] = forward_kinematics_impl_side_(q2[0], q2[1], /*is_left=*/false);
    return bars;
}

double ParallelRotateWaistSolver::linkage_residual_l2_(const Eigen::Vector2d& q2,
                                                     const Eigen::Vector2d& bars2) const {
    const LinkageLengthJacobian jl = linkage_length_jacobian_(q2[0], q2[1], bars2[0], /*is_left=*/true);
    const LinkageLengthJacobian jr = linkage_length_jacobian_(q2[0], q2[1], bars2[1], /*is_left=*/false);
    Eigen::Vector2d r;
    r << jl.length - L_m_, jr.length - L_m_;
    return r.norm();
}

Eigen::Vector2d ParallelRotateWaistSolver::motor_to_joint_global_search_(const Eigen::Vector2d& p2) const {
    auto residual_norm = [this](const Eigen::Vector2d& q, const Eigen::Vector2d& bars) -> double {
        return linkage_residual_l2_(q, bars);
    };

    std::vector<Eigen::Vector2d> seeds;
    seeds.reserve(10);
    seeds.emplace_back(0.0, 0.0);
    if (params_.joint_pitch.is_valid() && params_.joint_roll.is_valid()) {
        const double pm = 0.5 * (params_.joint_pitch.min + params_.joint_pitch.max);
        const double rm = 0.5 * (params_.joint_roll.min + params_.joint_roll.max);
        seeds.emplace_back(pm, 0.0);
        seeds.emplace_back(0.0, rm);
        seeds.emplace_back(pm, rm);
        seeds.emplace_back(params_.joint_pitch.min, params_.joint_roll.min);
        seeds.emplace_back(params_.joint_pitch.min, params_.joint_roll.max);
        seeds.emplace_back(params_.joint_pitch.max, params_.joint_roll.min);
        seeds.emplace_back(params_.joint_pitch.max, params_.joint_roll.max);
    }

    Eigen::Vector2d best_q = inverse_kinematics_impl_(p2, seeds.front());
    double best_rn = residual_norm(best_q, p2);
    for (size_t i = 1; i < seeds.size(); ++i) {
        const Eigen::Vector2d qi = inverse_kinematics_impl_(p2, seeds[i]);
        const double rn = residual_norm(qi, p2);
        if (rn < best_rn) {
            best_rn = rn;
            best_q = qi;
        }
    }

    const double tol_fail = std::max(default_tolerance_ * 1.0e3, 1.0e-6);
    if (!std::isfinite(best_rn) || best_rn > tol_fail) {
        throw std::runtime_error(
            "ParallelRotateWaistSolver::motor_to_joint_global_search_: IK residual too large (" +
            std::to_string(best_rn) + ")");
    }
    return best_q;
}

Eigen::Vector2d ParallelRotateWaistSolver::motor_to_joint_position(const Eigen::Vector2d& p2) const {
    // 初值 (0,0)。
    return inverse_kinematics_impl_(p2, Eigen::Vector2d::Zero());
}

Eigen::Vector2d ParallelRotateWaistSolver::motor_to_joint_position(
    const Eigen::Vector2d& bars2,
    const Eigen::Vector2d& q_joint_pitch_roll_hint2) const {
    Eigen::Vector2d qh = inverse_kinematics_impl_(bars2, q_joint_pitch_roll_hint2);
    const double rn = linkage_residual_l2_(qh, bars2);
    const double tol_ok = std::max(default_tolerance_ * 100.0, 1.0e-8);
    if (std::isfinite(rn) && rn <= tol_ok) {
        return qh;
    }
    return motor_to_joint_global_search_(bars2);
}

}  // namespace kuavo_solver
