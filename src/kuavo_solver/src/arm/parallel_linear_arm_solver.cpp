#include "kuavo_solver/arm/parallel_linear_arm_solver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

namespace {

// 14D 直通索引（非平行耦合的肩/腕外自由度）：
//   0 shoulder_yaw, 1 shoulder_roll, 2 shoulder_pitch, 4 elbow_rotation,
//   7..9 右肩三个自由度, 11 右肘_rotation
constexpr int kPassthroughIdx[] = {0, 1, 2, 4, 7, 8, 9, 11};
constexpr int kNumPassthrough = 8;

inline double side_sign_(bool is_left) { return is_left ? 1.0 : -1.0; }

}  // namespace

Eigen::Vector3d ParallelLinearArmSolver::ToEigen(const Vec3& v) {
    return SolverTools::ToEigen(v);
}

double ParallelLinearArmSolver::Clamp(double v, double lo, double hi) {
    return SolverTools::Clamp(v, lo, hi);
}

// ============================================================================
// 0) 构造 / 参数加载
// ============================================================================

ParallelLinearArmSolver::ParallelLinearArmSolver(const ParallelLinearArmParams& params) : params_(params) {
    // 左肘
    elbow_left_.O_f_x = params.elbow_left.O_f_x;
    elbow_left_.O_f_z = params.elbow_left.O_f_z;
    elbow_left_.O_p_x = params.elbow_left.O_p_x;
    elbow_left_.O_p_z = params.elbow_left.O_p_z;
    elbow_left_.a_x = params.elbow_left.a_x;
    elbow_left_.a_z = params.elbow_left.a_z;
    elbow_left_.t_x = params.elbow_left.t_x;
    elbow_left_.t_z = params.elbow_left.t_z;
    elbow_left_.L = std::sqrt(elbow_left_.t_x * elbow_left_.t_x + elbow_left_.t_z * elbow_left_.t_z);
    elbow_left_.L2 = elbow_left_.L * elbow_left_.L;
    elbow_left_.sign = params.elbow_left.sign;
    elbow_left_.theta_sign = params.elbow_left.theta_sign;

    // 右肘
    elbow_right_.O_f_x = params.elbow_right.O_f_x;
    elbow_right_.O_f_z = params.elbow_right.O_f_z;
    elbow_right_.O_p_x = params.elbow_right.O_p_x;
    elbow_right_.O_p_z = params.elbow_right.O_p_z;
    elbow_right_.a_x = params.elbow_right.a_x;
    elbow_right_.a_z = params.elbow_right.a_z;
    elbow_right_.t_x = params.elbow_right.t_x;
    elbow_right_.t_z = params.elbow_right.t_z;
    elbow_right_.L = std::sqrt(elbow_right_.t_x * elbow_right_.t_x + elbow_right_.t_z * elbow_right_.t_z);
    elbow_right_.L2 = elbow_right_.L * elbow_right_.L;
    elbow_right_.sign = params.elbow_right.sign;
    elbow_right_.theta_sign = params.elbow_right.theta_sign;

    // 左腕
    wrist_left_.base_A = ToEigen(params.wrist_left.base_A);
    wrist_left_.base_B = ToEigen(params.wrist_left.base_B);
    wrist_left_.roll_center = ToEigen(params.wrist_left.roll_center);
    wrist_left_.roll_axis = ToEigen(params.wrist_left.roll_axis);
    wrist_left_.pitch_center_in_roll = ToEigen(params.wrist_left.pitch_center_in_roll);
    wrist_left_.pitch_axis = ToEigen(params.wrist_left.pitch_axis);
    wrist_left_.anchor_A_pitch = ToEigen(params.wrist_left.anchor_A_pitch);
    wrist_left_.anchor_B_pitch = ToEigen(params.wrist_left.anchor_B_pitch);
    wrist_left_.len_A_zero = params.wrist_left.len_A_zero;
    wrist_left_.len_B_zero = params.wrist_left.len_B_zero;

    // 右腕
    wrist_right_.base_A = ToEigen(params.wrist_right.base_A);
    wrist_right_.base_B = ToEigen(params.wrist_right.base_B);
    wrist_right_.roll_center = ToEigen(params.wrist_right.roll_center);
    wrist_right_.roll_axis = ToEigen(params.wrist_right.roll_axis);
    wrist_right_.pitch_center_in_roll = ToEigen(params.wrist_right.pitch_center_in_roll);
    wrist_right_.pitch_axis = ToEigen(params.wrist_right.pitch_axis);
    wrist_right_.anchor_A_pitch = ToEigen(params.wrist_right.anchor_A_pitch);
    wrist_right_.anchor_B_pitch = ToEigen(params.wrist_right.anchor_B_pitch);
    wrist_right_.len_A_zero = params.wrist_right.len_A_zero;
    wrist_right_.len_B_zero = params.wrist_right.len_B_zero;
    default_tolerance_ = params.default_tolerance;
    max_iterations_ = params.max_iterations;
}

ParallelLinearArmSolver::LoadedParam ParallelLinearArmSolver::loadParam(
    const std::string& arm_solver_type_token, const std::string& arm_config_dir) {
    auto loader = SolverTools::YamlLoader::OpenVariant(
        arm_config_dir, "parallel_linear_arm_solver.yaml", 1, arm_solver_type_token);

    ParallelLinearArmParams p;

    // 肘部几何：左 + 右
    loader.child("elbow");
    {
        loader.child("left")
            .require("O_f_x", &p.elbow_left.O_f_x)
            .require("O_f_z", &p.elbow_left.O_f_z)
            .require("O_p_x", &p.elbow_left.O_p_x)
            .require("O_p_z", &p.elbow_left.O_p_z)
            .require("a_x", &p.elbow_left.a_x)
            .require("a_z", &p.elbow_left.a_z)
            .require("t_x", &p.elbow_left.t_x)
            .require("t_z", &p.elbow_left.t_z)
            .require("sign", &p.elbow_left.sign)
            .require("theta_sign", &p.elbow_left.theta_sign);
        loader.parent();

        loader.child("right")
            .require("O_f_x", &p.elbow_right.O_f_x)
            .require("O_f_z", &p.elbow_right.O_f_z)
            .require("O_p_x", &p.elbow_right.O_p_x)
            .require("O_p_z", &p.elbow_right.O_p_z)
            .require("a_x", &p.elbow_right.a_x)
            .require("a_z", &p.elbow_right.a_z)
            .require("t_x", &p.elbow_right.t_x)
            .require("t_z", &p.elbow_right.t_z)
            .require("sign", &p.elbow_right.sign)
            .require("theta_sign", &p.elbow_right.theta_sign);
        loader.parent();
    }
    loader.parent();

    // 腕部几何：左 + 右
    loader.child("wrist");
    {
        loader.child("left")
            .require("base_A", &p.wrist_left.base_A.x, &p.wrist_left.base_A.y, &p.wrist_left.base_A.z)
            .require("base_B", &p.wrist_left.base_B.x, &p.wrist_left.base_B.y, &p.wrist_left.base_B.z)
            .require("roll_center", &p.wrist_left.roll_center.x, &p.wrist_left.roll_center.y, &p.wrist_left.roll_center.z)
            .require("roll_axis", &p.wrist_left.roll_axis.x, &p.wrist_left.roll_axis.y, &p.wrist_left.roll_axis.z)
            .require("pitch_center_in_roll", &p.wrist_left.pitch_center_in_roll.x, &p.wrist_left.pitch_center_in_roll.y, &p.wrist_left.pitch_center_in_roll.z)
            .require("pitch_axis", &p.wrist_left.pitch_axis.x, &p.wrist_left.pitch_axis.y, &p.wrist_left.pitch_axis.z)
            .require("anchor_A_pitch", &p.wrist_left.anchor_A_pitch.x, &p.wrist_left.anchor_A_pitch.y, &p.wrist_left.anchor_A_pitch.z)
            .require("anchor_B_pitch", &p.wrist_left.anchor_B_pitch.x, &p.wrist_left.anchor_B_pitch.y, &p.wrist_left.anchor_B_pitch.z)
            .require("len_A_zero", &p.wrist_left.len_A_zero)
            .require("len_B_zero", &p.wrist_left.len_B_zero);
        loader.parent();

        loader.child("right")
            .require("base_A", &p.wrist_right.base_A.x, &p.wrist_right.base_A.y, &p.wrist_right.base_A.z)
            .require("base_B", &p.wrist_right.base_B.x, &p.wrist_right.base_B.y, &p.wrist_right.base_B.z)
            .require("roll_center", &p.wrist_right.roll_center.x, &p.wrist_right.roll_center.y, &p.wrist_right.roll_center.z)
            .require("roll_axis", &p.wrist_right.roll_axis.x, &p.wrist_right.roll_axis.y, &p.wrist_right.roll_axis.z)
            .require("pitch_center_in_roll", &p.wrist_right.pitch_center_in_roll.x, &p.wrist_right.pitch_center_in_roll.y, &p.wrist_right.pitch_center_in_roll.z)
            .require("pitch_axis", &p.wrist_right.pitch_axis.x, &p.wrist_right.pitch_axis.y, &p.wrist_right.pitch_axis.z)
            .require("anchor_A_pitch", &p.wrist_right.anchor_A_pitch.x, &p.wrist_right.anchor_A_pitch.y, &p.wrist_right.anchor_A_pitch.z)
            .require("anchor_B_pitch", &p.wrist_right.anchor_B_pitch.x, &p.wrist_right.anchor_B_pitch.y, &p.wrist_right.anchor_B_pitch.z)
            .require("len_A_zero", &p.wrist_right.len_A_zero)
            .require("len_B_zero", &p.wrist_right.len_B_zero);
        loader.parent();
    }
    loader.parent();

    // 关节限位
    loader.child("joint_limits")
        .require("elbow_left", &p.joint_elbow_left)
        .require("elbow_right", &p.joint_elbow_right)
        .require("wrist_roll_left", &p.joint_wrist_roll_left)
        .require("wrist_roll_right", &p.joint_wrist_roll_right)
        .require("wrist_pitch_left", &p.joint_wrist_pitch_left)
        .require("wrist_pitch_right", &p.joint_wrist_pitch_right);
    loader.parent();

    // 电机限位
    loader.child("motor_limits")
        .require("elbow", &p.motor_elbow)
        .require("wrist_a", &p.motor_wrist_a)
        .require("wrist_b", &p.motor_wrist_b);
    loader.parent();

    loader.child("ik")
        .require("default_tolerance", &p.default_tolerance)
        .require("max_iterations", &p.max_iterations);

    return {p};
}

// ============================================================================
// 1) 连杆向量（Linkage Vectors）
// ----------------------------------------------------------------------------
// 肘：2D 平面 slider-crank
//   A = O_f + R(theta) * (a_x, a_z)  —— 摆杆末端
//   P = O_p + (0, -d)                —— 滑块位置
//   约束 |A − P|² − L² = 0
//   所需几何向量 e = A − P 通过 rotate2d_() 推出
//
// 腕：3D 双拉杆
//   anchor_cf = pitch_center_cf + R_roll * R_pitch * anchor_local（A 或 B）
//   约束 |anchor − base| − (len_zero + d) = 0
// ============================================================================

void ParallelLinearArmSolver::rotate2d_(double x, double z, double cos_t, double sin_t, int sign,
                                       double& x_out, double& z_out) {
    x_out = x * cos_t - sign * z * sin_t;
    z_out = sign * x * sin_t + z * cos_t;
}

Eigen::Matrix3d ParallelLinearArmSolver::rotation_axis_angle_(const Eigen::Vector3d& axis, double theta) {
    return SolverTools::RotationAxisAngle(axis, theta);
}

Eigen::Vector3d ParallelLinearArmSolver::wrist_linkage_vector_(
    double roll, double pitch, WireSide side, const WristParams& params) {
    const Eigen::Matrix3d R_roll = rotation_axis_angle_(params.roll_axis, roll);
    const Eigen::Matrix3d R_pitch = rotation_axis_angle_(params.pitch_axis, pitch);
    const Eigen::Vector3d pitch_center_cf = params.roll_center + R_roll * params.pitch_center_in_roll;
    const Eigen::Matrix3d R_total = R_roll * R_pitch;

    const Eigen::Vector3d& anchor_local = (side == WireSide::A) ? params.anchor_A_pitch : params.anchor_B_pitch;
    const Eigen::Vector3d& base         = (side == WireSide::A) ? params.base_A         : params.base_B;
    const Eigen::Vector3d  anchor_cf    = pitch_center_cf + R_total * anchor_local;
    return anchor_cf - base;
}

// ============================================================================
// 2) 雅可比（Jacobians & JacobianSystem）
// ----------------------------------------------------------------------------
// 统一用"长度形式"的约束 f_i = |linkage_vector_i| − L_i = 0，组装 JacobianSystem
// 使得直线执行器的 Ja = −1（腕）/ Ja 恒定几何系数（肘）。
// ============================================================================

ParallelLinearArmSolver::LinkageLengthJacobian
ParallelLinearArmSolver::elbow_linkage_length_jacobian_(double theta_internal, double d, const ElbowParams& p) {
    const double c = std::cos(theta_internal);
    const double s = std::sin(theta_internal);
    double a_rot_x = 0.0, a_rot_z = 0.0;
    rotate2d_(p.a_x, p.a_z, c, s, p.sign, a_rot_x, a_rot_z);
    const double A_x = p.O_f_x + a_rot_x;
    const double A_z = p.O_f_z + a_rot_z;
    const double P_z = p.O_p_z - d;
    const double ex = A_x - p.O_p_x;
    const double ez = A_z - P_z;

    // 使用长度形式 f = sqrt(ex² + ez²) − L = 0（等价于 squared 形式，差一个 2L 因子，
    // 但保持与踝/腕 LinkageLengthJacobian 同构的语义：joint=∂L/∂q, actuator=∂L/∂d）
    const double L = std::sqrt(ex * ex + ez * ez);
    if (L < 1e-12) {
        throw std::runtime_error("elbow_linkage_length_jacobian_: 连杆长度过小");
    }

    const double dAx_dtheta = -p.a_x * s - p.sign * p.a_z * c;
    const double dAz_dtheta =  p.sign * p.a_x * c - p.a_z * s;

    LinkageLengthJacobian j;
    j.joint.resize(1);
    j.joint(0) = (ex * dAx_dtheta + ez * dAz_dtheta) / L;  // ∂L/∂theta_internal
    // ∂L/∂d：P_z = O_p_z − d → ez = A_z − P_z = A_z − O_p_z + d → ∂ez/∂d = 1
    //        ∂L/∂d = ez / L
    j.actuator = ez / L;
    j.length = L;
    return j;
}

ParallelLinearArmSolver::LinkageLengthJacobian
ParallelLinearArmSolver::wrist_linkage_length_jacobian_(
    double roll, double pitch, WireSide side, const WristParams& params) {
    const Eigen::Matrix3d R_roll = rotation_axis_angle_(params.roll_axis, roll);
    const Eigen::Matrix3d R_pitch = rotation_axis_angle_(params.pitch_axis, pitch);
    const Eigen::Vector3d pitch_center_cf = params.roll_center + R_roll * params.pitch_center_in_roll;
    const Eigen::Matrix3d R_total = R_roll * R_pitch;

    const Eigen::Vector3d& anchor_local = (side == WireSide::A) ? params.anchor_A_pitch : params.anchor_B_pitch;
    const Eigen::Vector3d& base         = (side == WireSide::A) ? params.base_A         : params.base_B;
    const Eigen::Vector3d  anchor_cf    = pitch_center_cf + R_total * anchor_local;

    const Eigen::Vector3d vec = anchor_cf - base;
    const double L = vec.norm();
    if (L < 1e-12) {
        throw std::runtime_error("wrist_linkage_length_jacobian_: 拉杆长度过小");
    }
    const Eigen::Vector3d n_hat = vec / L;

    // d|vec|/dθ = n_hat · (ω × r)
    const Eigen::Vector3d omega_roll  = params.roll_axis;
    const Eigen::Vector3d omega_pitch = R_roll * params.pitch_axis;
    const Eigen::Vector3d r_roll      = anchor_cf - params.roll_center;
    const Eigen::Vector3d r_pitch     = anchor_cf - pitch_center_cf;

    LinkageLengthJacobian j;
    j.joint.resize(2);
    j.joint(0) = n_hat.dot(omega_roll.cross(r_roll));
    j.joint(1) = n_hat.dot(omega_pitch.cross(r_pitch));
    j.actuator = 1.0;  // 直线执行器：d_X = L_X − len_X_zero，∂L/∂d = 1
    j.length = L;
    return j;
}

JacobianSystem ParallelLinearArmSolver::elbow_jacobian_system_impl_(
    double theta_joint, double d_m, bool is_left) const {
    const ElbowParams& p = is_left ? elbow_left_ : elbow_right_;
    const double theta_internal = theta_joint * p.theta_sign;

    // 内部坐标下的长度雅可比：joint = ∂L/∂theta_internal, actuator = ∂L/∂d
    const LinkageLengthJacobian J = elbow_linkage_length_jacobian_(theta_internal, d_m, p);

    // 外部坐标 theta_joint = theta_internal / theta_sign → ∂L/∂theta_joint = theta_sign * ∂L/∂theta_internal
    // 约束 f = L − L0 = 0：
    //   Jc = ∂f/∂theta_joint = theta_sign * J.joint
    //   Ja = ∂f/∂d           = J.actuator   (直线执行器，L0 对 d 不依赖)
    //
    // 注意：elbow 的约束"实际长度 = L0 = const"内置于几何（肘杆定长），
    // 因此 Ja = ∂L/∂d = ez/L 并不是常量 −1，而是随构型变化的标量。
    // 速度对偶：dd = −(Jc/Ja)·dtheta_joint，与原实现一致。
    JacobianSystem js;
    js.J_constraint = Eigen::Matrix<double, 1, 1>();
    js.J_actuator   = Eigen::Matrix<double, 1, 1>();
    js.J_constraint(0, 0) = p.theta_sign * J.joint(0);
    js.J_actuator(0, 0)   = J.actuator;

    if (std::abs(js.J_actuator(0, 0)) < 1e-15) {
        throw std::runtime_error(
            "elbow_jacobian_system_impl_: Ja ≈ 0（死点奇异），theta=" + std::to_string(theta_joint) +
            ", d=" + std::to_string(d_m));
    }
    return js;
}

JacobianSystem ParallelLinearArmSolver::wrist_jacobian_system_impl_(
    double roll_joint, double pitch_joint, bool is_left) const {
    const WristParams& p = is_left ? wrist_left_ : wrist_right_;
    const double s = side_sign_(is_left);
    const double roll_solver  = s * roll_joint;
    const double pitch_solver = s * pitch_joint;

    // 按 A/B 两根"拉杆"分别计算长度雅可比（solver 内部坐标）
    const LinkageLengthJacobian J_A = wrist_linkage_length_jacobian_(roll_solver, pitch_solver, WireSide::A, p);
    const LinkageLengthJacobian J_B = wrist_linkage_length_jacobian_(roll_solver, pitch_solver, WireSide::B, p);

    // 外部坐标：roll_joint = (1/s)·roll_solver → ∂/∂roll_joint = s · ∂/∂roll_solver
    // 每列乘 s（s² = 1，不影响范数）
    JacobianSystem js;
    js.J_constraint = Eigen::Matrix2d::Zero();
    // 统一约束口径：Jc*dq + Ja*dp = 0
    // wrist 这里定义 d_X = L_X - len_X_zero，等价约束写作
    //   f_X = L_X - (len_X_zero + d_X) = 0
    // 因此 ∂f_X/∂d_X = -1，Ja 必须为 -I。
    js.J_actuator   = -Eigen::Matrix2d::Identity();
    js.J_constraint.row(0) = s * J_A.joint;
    js.J_constraint.row(1) = s * J_B.joint;
    return js;
}

// ---- 公有几何接口（简单转发） ----

JacobianSystem ParallelLinearArmSolver::elbow_jacobian_system(
    double theta, double d_m, bool is_left) const {
    return elbow_jacobian_system_impl_(theta, d_m, is_left);
}

JacobianSystem ParallelLinearArmSolver::wrist_jacobian_system(
    double roll, double pitch, bool is_left) const {
    return wrist_jacobian_system_impl_(roll, pitch, is_left);
}

// ============================================================================
// 3) 正逆运动学（FK = joint→motor，IK = motor→joint）
// ----------------------------------------------------------------------------
// 肘：FK 解 |A(theta) − P(d)| = L，对 d 闭式 dz = −sqrt(L² − dx²)
//     IK 解给定 d 时 theta，闭式余弦定理
// 腕：FK 闭式（直接计算 L = |anchor − base|）
//     IK 牛顿迭代（残差 = 目标 d − 当前 d，用 wrist_jacobian_system 的 Jc 解线性）
// ============================================================================

double ParallelLinearArmSolver::elbow_forward_kinematics_(double theta_internal, const ElbowParams& p) {
    const double c = std::cos(theta_internal);
    const double s = std::sin(theta_internal);
    double a_rot_x = 0.0, a_rot_z = 0.0;
    rotate2d_(p.a_x, p.a_z, c, s, p.sign, a_rot_x, a_rot_z);
    const double A_x = p.O_f_x + a_rot_x;
    const double A_z = p.O_f_z + a_rot_z;
    const double dx = A_x - p.O_p_x;
    if (dx * dx > p.L2) {
        throw std::runtime_error("elbow FK (joint→motor) out of range");
    }
    const double dz = -std::sqrt(p.L2 - dx * dx);
    return dz - A_z + p.O_p_z;
}

double ParallelLinearArmSolver::elbow_inverse_kinematics_(double d, const ElbowParams& p) {
    const double dx0 = p.O_f_x - p.O_p_x;
    const double P_z = p.O_p_z - d;
    const double dz0 = p.O_f_z - P_z;
    const double C = p.L2 - (dx0 * dx0 + dz0 * dz0) - (p.a_x * p.a_x + p.a_z * p.a_z);
    const double A = 2 * (dx0 * p.a_x + dz0 * p.a_z);
    const double B = 2 * p.sign * (-dx0 * p.a_z + dz0 * p.a_x);
    const double R = std::sqrt(A * A + B * B);
    if (R < 1e-12) {
        return 0.0;
    }
    const double C_clamped = std::max(-R, std::min(R, C));
    const double phi = std::atan2(B, A);
    const double delta = std::acos(C_clamped / R);
    return phi - delta;
}

std::pair<double, double> ParallelLinearArmSolver::wrist_forward_kinematics_(
    double roll, double pitch, const WristParams& params) {
    const Eigen::Vector3d vec_A = wrist_linkage_vector_(roll, pitch, WireSide::A, params);
    const Eigen::Vector3d vec_B = wrist_linkage_vector_(roll, pitch, WireSide::B, params);
    const double d_A = vec_A.norm() - params.len_A_zero;
    const double d_B = vec_B.norm() - params.len_B_zero;
    return std::make_pair(d_A, d_B);
}

std::pair<double, double> ParallelLinearArmSolver::wrist_inverse_kinematics_(
    double d_A, double d_B, const WristParams& params, double tol, int max_iter) {
    double roll = 0.0, pitch = 0.0;
    for (int i = 0; i < max_iter; ++i) {
        const LinkageLengthJacobian J_A = wrist_linkage_length_jacobian_(roll, pitch, WireSide::A, params);
        const LinkageLengthJacobian J_B = wrist_linkage_length_jacobian_(roll, pitch, WireSide::B, params);
        Eigen::Vector2d residual(J_A.length - params.len_A_zero - d_A,
                                J_B.length - params.len_B_zero - d_B);
        if (residual.norm() < tol) break;

        Eigen::Matrix2d J_constraint;
        J_constraint.row(0) = J_A.joint;
        J_constraint.row(1) = J_B.joint;
        const Eigen::Vector2d delta = J_constraint.colPivHouseholderQr().solve(residual);
        roll  -= delta(0);
        pitch -= delta(1);
        if (delta.norm() < tol) break;
    }
    return std::make_pair(roll, pitch);
}

// ---- 带 side_sign / theta_sign 的单侧位置映射（供公共 API 使用） ----

double ParallelLinearArmSolver::elbow_forward_kinematics_side_(double theta_joint, bool is_left) const {
    const ElbowParams& p = is_left ? elbow_left_ : elbow_right_;
    return elbow_forward_kinematics_(theta_joint * p.theta_sign, p);
}

double ParallelLinearArmSolver::elbow_inverse_kinematics_side_(double d_m, bool is_left) const {
    const ElbowParams& p = is_left ? elbow_left_ : elbow_right_;
    return elbow_inverse_kinematics_(d_m, p) * p.theta_sign;
}

Eigen::Vector2d ParallelLinearArmSolver::wrist_forward_kinematics_side_(
    double roll, double pitch, bool is_left) const {
    const WristParams& p = is_left ? wrist_left_ : wrist_right_;
    const double s = side_sign_(is_left);
    const auto result = wrist_forward_kinematics_(s * roll, s * pitch, p);
    return Eigen::Vector2d(result.first, result.second);
}

Eigen::Vector2d ParallelLinearArmSolver::wrist_inverse_kinematics_side_(
    double dA, double dB, bool is_left) const {
    const WristParams& p = is_left ? wrist_left_ : wrist_right_;
    const double s = side_sign_(is_left);
    const auto result = wrist_inverse_kinematics_(dA, dB, p, default_tolerance_, max_iterations_);
    return s * Eigen::Vector2d(result.first, result.second);
}

// ============================================================================
// 4) 速度 / 力矩映射（派生自 JacobianSystem，走 SolverTools::*FromJoint/Motor）
// ============================================================================

double ParallelLinearArmSolver::joint_to_motor_velocity_elbow_side_(
    double theta, double d_m, double dtheta, bool is_left) const {
    const JacobianSystem jac = elbow_jacobian_system_impl_(theta, d_m, is_left);
    Eigen::VectorXd dq(1); dq(0) = dtheta;
    return SolverTools::MotorVelocityFromJoint(jac, dq)(0);
}

double ParallelLinearArmSolver::motor_to_joint_velocity_elbow_side_(
    double theta, double d_m, double dd, bool is_left) const {
    const JacobianSystem jac = elbow_jacobian_system_impl_(theta, d_m, is_left);
    Eigen::VectorXd dp(1); dp(0) = dd;
    return SolverTools::JointVelocityFromMotor(jac, dp)(0);
}

double ParallelLinearArmSolver::joint_to_motor_current_elbow_side_(
    double theta, double d_m, double tau, bool is_left) const {
    const JacobianSystem jac = elbow_jacobian_system_impl_(theta, d_m, is_left);
    Eigen::VectorXd tj(1); tj(0) = tau;
    return SolverTools::MotorCurrentFromJointTorque(jac, tj)(0);
}

double ParallelLinearArmSolver::motor_to_joint_torque_elbow_side_(
    double theta, double d_m, double i_m, bool is_left) const {
    const JacobianSystem jac = elbow_jacobian_system_impl_(theta, d_m, is_left);
    Eigen::VectorXd im(1); im(0) = i_m;
    return SolverTools::JointTorqueFromMotorCurrent(jac, im)(0);
}

Eigen::Vector2d ParallelLinearArmSolver::joint_to_motor_velocity_wrist_side_(
    double roll, double pitch, const Eigen::Vector2d& dq_rp, bool is_left) const {
    const JacobianSystem jac = wrist_jacobian_system_impl_(roll, pitch, is_left);
    return SolverTools::MotorVelocityFromJoint(jac, dq_rp);
}

Eigen::Vector2d ParallelLinearArmSolver::motor_to_joint_velocity_wrist_side_(
    double roll, double pitch, const Eigen::Vector2d& dd_ab, bool is_left) const {
    const JacobianSystem jac = wrist_jacobian_system_impl_(roll, pitch, is_left);
    return SolverTools::JointVelocityFromMotor(jac, dd_ab);
}

Eigen::Vector2d ParallelLinearArmSolver::joint_to_motor_current_wrist_side_(
    double roll, double pitch, const Eigen::Vector2d& tau_rp, bool is_left) const {
    const JacobianSystem jac = wrist_jacobian_system_impl_(roll, pitch, is_left);
    return SolverTools::MotorCurrentFromJointTorque(jac, tau_rp);
}

Eigen::Vector2d ParallelLinearArmSolver::motor_to_joint_torque_wrist_side_(
    double roll, double pitch, const Eigen::Vector2d& i_ab, bool is_left) const {
    const JacobianSystem jac = wrist_jacobian_system_impl_(roll, pitch, is_left);
    return SolverTools::JointTorqueFromMotorCurrent(jac, i_ab);
}

// ============================================================================
// 5) 公共 API：14D 路由
// ============================================================================

Eigen::VectorXd ParallelLinearArmSolver::joint_to_motor_position(const Eigen::VectorXd& q) {
    Eigen::VectorXd result(14);
    for (int i = 0; i < kNumPassthrough; ++i) {
        result[kPassthroughIdx[i]] = q[kPassthroughIdx[i]];
    }

    // 左肘
    {
        const double theta = SolverTools::RequireInRange(
            "q[3] L_elbow", q[3], params_.joint_elbow_left.min, params_.joint_elbow_left.max);
        result[3] = elbow_forward_kinematics_side_(theta, /*is_left=*/true);
    }
    // 右肘
    {
        const double theta = SolverTools::RequireInRange(
            "q[10] R_elbow", q[10], params_.joint_elbow_right.min, params_.joint_elbow_right.max);
        result[10] = elbow_forward_kinematics_side_(theta, /*is_left=*/false);
    }
    // 左腕
    {
        const double roll = SolverTools::RequireInRange(
            "q[5] L_wrist_roll", q[5], params_.joint_wrist_roll_left.min, params_.joint_wrist_roll_left.max);
        const double pitch = SolverTools::RequireInRange(
            "q[6] L_wrist_pitch", q[6], params_.joint_wrist_pitch_left.min, params_.joint_wrist_pitch_left.max);
        const auto d = wrist_forward_kinematics_side_(roll, pitch, /*is_left=*/true);
        result[5] = d[0];
        result[6] = d[1];
    }
    // 右腕
    {
        const double roll = SolverTools::RequireInRange(
            "q[12] R_wrist_roll", q[12], params_.joint_wrist_roll_right.min, params_.joint_wrist_roll_right.max);
        const double pitch = SolverTools::RequireInRange(
            "q[13] R_wrist_pitch", q[13], params_.joint_wrist_pitch_right.min, params_.joint_wrist_pitch_right.max);
        const auto d = wrist_forward_kinematics_side_(roll, pitch, /*is_left=*/false);
        result[12] = d[0];
        result[13] = d[1];
    }

    return result;
}

Eigen::VectorXd ParallelLinearArmSolver::motor_to_joint_position(const Eigen::VectorXd& p) {
    Eigen::VectorXd result(14);
    for (int i = 0; i < kNumPassthrough; ++i) {
        result[kPassthroughIdx[i]] = p[kPassthroughIdx[i]];
    }

    result[3]  = elbow_inverse_kinematics_side_(p[3],  /*is_left=*/true);
    result[10] = elbow_inverse_kinematics_side_(p[10], /*is_left=*/false);

    {
        const auto r = wrist_inverse_kinematics_side_(p[5], p[6], /*is_left=*/true);
        result[5] = r[0];
        result[6] = r[1];
    }
    {
        const auto r = wrist_inverse_kinematics_side_(p[12], p[13], /*is_left=*/false);
        result[12] = r[0];
        result[13] = r[1];
    }
    return result;
}

Eigen::VectorXd ParallelLinearArmSolver::joint_to_motor_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq) {
    Eigen::VectorXd result(14);
    for (int i = 0; i < kNumPassthrough; ++i) {
        result[kPassthroughIdx[i]] = dq[kPassthroughIdx[i]];
    }

    result[3]  = joint_to_motor_velocity_elbow_side_(q[3],  p[3],  dq[3],  /*is_left=*/true);
    result[10] = joint_to_motor_velocity_elbow_side_(q[10], p[10], dq[10], /*is_left=*/false);

    {
        const Eigen::Vector2d dp_l = joint_to_motor_velocity_wrist_side_(
            q[5], q[6], Eigen::Vector2d(dq[5], dq[6]), /*is_left=*/true);
        result[5] = dp_l[0];
        result[6] = dp_l[1];
    }
    {
        const Eigen::Vector2d dp_r = joint_to_motor_velocity_wrist_side_(
            q[12], q[13], Eigen::Vector2d(dq[12], dq[13]), /*is_left=*/false);
        result[12] = dp_r[0];
        result[13] = dp_r[1];
    }
    return result;
}

Eigen::VectorXd ParallelLinearArmSolver::motor_to_joint_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) {
    Eigen::VectorXd result(14);
    for (int i = 0; i < kNumPassthrough; ++i) {
        result[kPassthroughIdx[i]] = dp[kPassthroughIdx[i]];
    }

    result[3]  = motor_to_joint_velocity_elbow_side_(q[3],  p[3],  dp[3],  /*is_left=*/true);
    result[10] = motor_to_joint_velocity_elbow_side_(q[10], p[10], dp[10], /*is_left=*/false);

    {
        const Eigen::Vector2d dq_l = motor_to_joint_velocity_wrist_side_(
            q[5], q[6], Eigen::Vector2d(dp[5], dp[6]), /*is_left=*/true);
        result[5] = dq_l[0];
        result[6] = dq_l[1];
    }
    {
        const Eigen::Vector2d dq_r = motor_to_joint_velocity_wrist_side_(
            q[12], q[13], Eigen::Vector2d(dp[12], dp[13]), /*is_left=*/false);
        result[12] = dq_r[0];
        result[13] = dq_r[1];
    }
    return result;
}

Eigen::VectorXd ParallelLinearArmSolver::joint_to_motor_current(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau) {
    Eigen::VectorXd result(14);
    for (int i = 0; i < kNumPassthrough; ++i) {
        result[kPassthroughIdx[i]] = tau[kPassthroughIdx[i]];
    }

    result[3]  = joint_to_motor_current_elbow_side_(q[3],  p[3],  tau[3],  /*is_left=*/true);
    result[10] = joint_to_motor_current_elbow_side_(q[10], p[10], tau[10], /*is_left=*/false);

    {
        const Eigen::Vector2d i_l = joint_to_motor_current_wrist_side_(
            q[5], q[6], Eigen::Vector2d(tau[5], tau[6]), /*is_left=*/true);
        result[5] = i_l[0];
        result[6] = i_l[1];
    }
    {
        const Eigen::Vector2d i_r = joint_to_motor_current_wrist_side_(
            q[12], q[13], Eigen::Vector2d(tau[12], tau[13]), /*is_left=*/false);
        result[12] = i_r[0];
        result[13] = i_r[1];
    }
    return result;
}

Eigen::VectorXd ParallelLinearArmSolver::motor_to_joint_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i) {
    Eigen::VectorXd result(14);
    for (int k = 0; k < kNumPassthrough; ++k) {
        result[kPassthroughIdx[k]] = i[kPassthroughIdx[k]];
    }

    result[3]  = motor_to_joint_torque_elbow_side_(q[3],  p[3],  i[3],  /*is_left=*/true);
    result[10] = motor_to_joint_torque_elbow_side_(q[10], p[10], i[10], /*is_left=*/false);

    {
        const Eigen::Vector2d tau_l = motor_to_joint_torque_wrist_side_(
            q[5], q[6], Eigen::Vector2d(i[5], i[6]), /*is_left=*/true);
        result[5] = tau_l[0];
        result[6] = tau_l[1];
    }
    {
        const Eigen::Vector2d tau_r = motor_to_joint_torque_wrist_side_(
            q[12], q[13], Eigen::Vector2d(i[12], i[13]), /*is_left=*/false);
        result[12] = tau_r[0];
        result[13] = tau_r[1];
    }
    return result;
}

ParallelLinearArmSolver::VerificationResult ParallelLinearArmSolver::verify_with_mujoco(
    const Eigen::VectorXd& q_mujoco, const Eigen::VectorXd& p_mujoco,
    const Eigen::VectorXd& dq_mujoco, const Eigen::VectorXd& tau_mujoco,
    double position_tolerance, double velocity_tolerance, double torque_tolerance) {
    return SolverTools::VerifyRoundTrip(
        q_mujoco, p_mujoco, dq_mujoco, tau_mujoco,
        position_tolerance, velocity_tolerance, torque_tolerance,
        [this](const Eigen::VectorXd& q)                             { return joint_to_motor_position(q); },
        [this](const Eigen::VectorXd& p)                             { return motor_to_joint_position(p); },
        [this](const Eigen::VectorXd& q, const Eigen::VectorXd& p,
               const Eigen::VectorXd& dq)                            { return joint_to_motor_velocity(q, p, dq); },
        [this](const Eigen::VectorXd& q, const Eigen::VectorXd& p,
               const Eigen::VectorXd& dp)                            { return motor_to_joint_velocity(q, p, dp); },
        [this](const Eigen::VectorXd& q, const Eigen::VectorXd& p,
               const Eigen::VectorXd& tau)                           { return joint_to_motor_current(q, p, tau); },
        [this](const Eigen::VectorXd& q, const Eigen::VectorXd& p,
               const Eigen::VectorXd& i)                             { return motor_to_joint_torque(q, p, i); });
}

}  // namespace kuavo_solver
