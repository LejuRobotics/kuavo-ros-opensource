#include "kuavo_solver/roban_ankle_solver.h"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <map>

namespace kuavo_solver {

Roban_ankle_solver::Roban_ankle_solver(const RobanAnkleParams& params) {
    // 关节偏移参数
    z_pitch_ = params.z_pitch;
    z_roll_ = params.z_roll;
    x_pitch_ = params.x_pitch;
    
    // 左脚踝参数
    x_lleq_ = params.x_lleq; y_lleq_ = params.y_lleq; z_lleq_ = params.z_lleq;
    x_lreq_ = params.x_lreq; y_lreq_ = params.y_lreq; z_lreq_ = params.z_lreq;
    x_llbar_ = params.x_llbar; z_llbar_ = params.z_llbar;
    x_lrbar_ = params.x_lrbar; z_lrbar_ = params.z_lrbar;
    x_lltd_ = params.x_lltd; y_lltd_ = params.y_lltd; z_lltd_ = params.z_lltd;
    x_lrtd_ = params.x_lrtd; y_lrtd_ = params.y_lrtd; z_lrtd_ = params.z_lrtd;
    l0_ll_eqtd_ = params.l0_ll_eqtd; l0_lr_eqtd_ = params.l0_lr_eqtd;
    
    // 右脚踝参数
    x_rleq_ = params.x_rleq; y_rleq_ = params.y_rleq; z_rleq_ = params.z_rleq;
    x_rreq_ = params.x_rreq; y_rreq_ = params.y_rreq; z_rreq_ = params.z_rreq;
    x_rlbar_ = params.x_rlbar; z_rlbar_ = params.z_rlbar;
    x_rrbar_ = params.x_rrbar; z_rrbar_ = params.z_rrbar;
    x_rltd_ = params.x_rltd; y_rltd_ = params.y_rltd; z_rltd_ = params.z_rltd;
    x_rrtd_ = params.x_rrtd; y_rrtd_ = params.y_rrtd; z_rrtd_ = params.z_rrtd;
    l0_rl_eqtd_ = params.l0_rl_eqtd; l0_rr_eqtd_ = params.l0_rr_eqtd;
    
    // 计算 bar 长度
    l_llbar_ = std::sqrt(y_lltd_ * y_lltd_ + z_lltd_ * z_lltd_);
    l_lrbar_ = std::sqrt(y_lrtd_ * y_lrtd_ + z_lrtd_ * z_lrtd_);
    l_rlbar_ = std::sqrt(y_rltd_ * y_rltd_ + z_rltd_ * z_rltd_);
    l_rrbar_ = std::sqrt(y_rrtd_ * y_rrtd_ + z_rrtd_ * z_rrtd_);
    
    default_tolerance_ = params.default_tolerance;
    max_iterations_ = params.max_iterations;
}

Eigen::Vector3d Roban_ankle_solver::compute_tendon_vector(
    double pitch, double roll, double actuator_angle,
    AnkleSide ankle_side, TendonSide tendon_side) const {
    TendonParams p = get_tendon_params(ankle_side, tendon_side);
    
    double c1 = std::cos(pitch);
    double s1 = std::sin(pitch);
    double c2 = std::cos(roll);
    double s2 = std::sin(roll);
    double c_act = std::cos(actuator_angle);
    double s_act = std::sin(actuator_angle);
    
    // 正确的运动学：考虑 pitch 和 roll 关节之间的偏移
    // z_pitch_raw = z_pitch_ - z_roll_（z_pitch_ 是 roll 关节相对于 knee 的 z 偏移）
    double z_pitch_raw = z_pitch_ - z_roll_;
    
    // eq 点在 knee 坐标系中的位置（考虑关节偏移）
    // p_eq_knee = T_pitch + R_pitch * (T_roll + R_roll * p_eq_roll)
    double p_eq_x = x_pitch_ + z_roll_ * s1 + c1 * p.x_eq + s1 * s2 * p.y_eq + s1 * c2 * p.z_eq;
    double p_eq_y = c2 * p.y_eq - s2 * p.z_eq;
    double p_eq_z = z_pitch_raw + z_roll_ * c1 - s1 * p.x_eq + c1 * s2 * p.y_eq + c1 * c2 * p.z_eq;
    
    // td 点在 knee 坐标系中的位置
    // p_td_knee = T_bar + R_actuator * p_td_bar
    double p_td_x = p.x_bar + p.x_td;
    double p_td_y = c_act * p.y_td - s_act * p.z_td;
    double p_td_z = p.z_bar + s_act * p.y_td + c_act * p.z_td;
    
    // 肌腱向量 = td - eq
    Eigen::Vector3d p_eqtd;
    p_eqtd << p_td_x - p_eq_x, p_td_y - p_eq_y, p_td_z - p_eq_z;
    
    return p_eqtd;
}


Eigen::Matrix<double, 3, 2> Roban_ankle_solver::compute_jacobian_ankle(
    double pitch, double roll, AnkleSide ankle_side, TendonSide tendon_side) const {
    TendonParams p = get_tendon_params(ankle_side, tendon_side);
    
    double c1 = std::cos(pitch);
    double s1 = std::sin(pitch);
    double c2 = std::cos(roll);
    double s2 = std::sin(roll);
    
    // 计算 eq 点位置相对于 [pitch, roll] 的雅可比
    // p_eq_x = x_pitch_ + z_roll_*s1 + c1*x_eq + s1*s2*y_eq + s1*c2*z_eq
    // p_eq_y = c2*y_eq - s2*z_eq
    // p_eq_z = z_pitch_raw + z_roll_*c1 - s1*x_eq + c1*s2*y_eq + c1*c2*z_eq
    
    // ∂p_eq/∂pitch
    double dp_eq_x_dpitch = z_roll_ * c1 - s1 * p.x_eq + c1 * s2 * p.y_eq + c1 * c2 * p.z_eq;
    double dp_eq_y_dpitch = 0.0;
    double dp_eq_z_dpitch = -z_roll_ * s1 - c1 * p.x_eq - s1 * s2 * p.y_eq - s1 * c2 * p.z_eq;
    
    // ∂p_eq/∂roll
    double dp_eq_x_droll = s1 * c2 * p.y_eq - s1 * s2 * p.z_eq;
    double dp_eq_y_droll = -s2 * p.y_eq - c2 * p.z_eq;
    double dp_eq_z_droll = c1 * c2 * p.y_eq - c1 * s2 * p.z_eq;
    
    // p_eqtd = p_td - p_eq，p_td 不依赖于 pitch/roll
    // ∂p_eqtd/∂[pitch, roll] = -∂p_eq/∂[pitch, roll]
    Eigen::Matrix<double, 3, 2> jacp_ankle;
    jacp_ankle << 
        -dp_eq_x_dpitch, -dp_eq_x_droll,
        -dp_eq_y_dpitch, -dp_eq_y_droll,
        -dp_eq_z_dpitch, -dp_eq_z_droll;
    
    return jacp_ankle;
}


Eigen::Vector3d Roban_ankle_solver::compute_jacobian_actuator(
    double pitch, double roll, double actuator_angle,
    AnkleSide ankle_side, TendonSide tendon_side) const {
    TendonParams p = get_tendon_params(ankle_side, tendon_side);
    
    double c_act = std::cos(actuator_angle);
    double s_act = std::sin(actuator_angle);
    
    Eigen::Vector3d jacp_actuator;
    jacp_actuator << 0, -p.y_td * s_act - p.z_td * c_act, p.y_td * c_act - p.z_td * s_act;
    
    return jacp_actuator;
}


std::pair<double, double> Roban_ankle_solver::inverse_kinematics_(
    double lbar, double rbar, AnkleSide side, double tol, int max_iter) {
    // 使用默认值
    if (tol < 0) {
        tol = default_tolerance_;
    }
    if (max_iter < 0) {
        max_iter = max_iterations_;
    }
    

    
    // 使用雅可比方法
    return inverse_kinematics_jacobian_(lbar, rbar, side, tol, max_iter);
}

TendonParams Roban_ankle_solver::get_tendon_params(
    AnkleSide ankle_side, TendonSide tendon_side) const {
    int ankle_idx = (ankle_side == AnkleSide::LEFT) ? 0 : 1;
    int tendon_idx = (tendon_side == TendonSide::LEFT) ? 0 : 1;
    int idx = ankle_idx * 2 + tendon_idx;
    
    struct ParamRefs {
        const double* x_eq, *y_eq, *z_eq;
        const double* x_td, *y_td, *z_td;
        const double* x_bar, *z_bar, *l_bar, *l0_eqtd;
    };
    
    // 注意：使用 this 指针来访问成员变量
    const ParamRefs refs[] = {
        {&x_lleq_, &y_lleq_, &z_lleq_, &x_lltd_, &y_lltd_, &z_lltd_, &x_llbar_, &z_llbar_, &l_llbar_, &l0_ll_eqtd_},
        {&x_lreq_, &y_lreq_, &z_lreq_, &x_lrtd_, &y_lrtd_, &z_lrtd_, &x_lrbar_, &z_lrbar_, &l_lrbar_, &l0_lr_eqtd_},
        {&x_rleq_, &y_rleq_, &z_rleq_, &x_rltd_, &y_rltd_, &z_rltd_, &x_rlbar_, &z_rlbar_, &l_rlbar_, &l0_rl_eqtd_},
        {&x_rreq_, &y_rreq_, &z_rreq_, &x_rrtd_, &y_rrtd_, &z_rrtd_, &x_rrbar_, &z_rrbar_, &l_rrbar_, &l0_rr_eqtd_}
    };
    
    TendonParams params;
    params.x_eq = *refs[idx].x_eq;
    params.y_eq = *refs[idx].y_eq;
    params.z_eq = *refs[idx].z_eq;
    params.x_td = *refs[idx].x_td;
    params.y_td = *refs[idx].y_td;
    params.z_td = *refs[idx].z_td;
    params.x_bar = *refs[idx].x_bar;
    params.z_bar = *refs[idx].z_bar;
    params.l_bar = *refs[idx].l_bar;
    params.l0_eqtd = *refs[idx].l0_eqtd;
    
    return params;
}

Roban_ankle_solver::AnkleJacobianSystem Roban_ankle_solver::compute_ankle_jacobian_system_(
    double pitch, double roll, double lbar, double rbar, AnkleSide ankle_side) const {
    AnkleJacobianSystem jac_sys;
    
    Eigen::Vector3d p_l_eqtd = compute_tendon_vector(pitch, roll, lbar, ankle_side, TendonSide::LEFT);
    double l_l_eqtd = p_l_eqtd.norm();
    if (l_l_eqtd < 1e-10) {
        throw std::runtime_error("compute_ankle_jacobian_system_: 左肌腱长度过小");
    }
    Eigen::Vector3d hat_l_eqtd = p_l_eqtd / l_l_eqtd;
    
    Eigen::Matrix<double, 3, 2> J_ankle_l = compute_jacobian_ankle(pitch, roll, ankle_side, TendonSide::LEFT);
    Eigen::Vector3d J_actuator_l = compute_jacobian_actuator(pitch, roll, lbar, ankle_side, TendonSide::LEFT);
    
    Eigen::RowVector2d J_l_ankle_l = hat_l_eqtd.transpose() * J_ankle_l;
    double J_l_actuator_l = hat_l_eqtd.transpose() * J_actuator_l;
    
    Eigen::Vector3d p_r_eqtd = compute_tendon_vector(pitch, roll, rbar, ankle_side, TendonSide::RIGHT);
    double l_r_eqtd = p_r_eqtd.norm();
    if (l_r_eqtd < 1e-10) {
        throw std::runtime_error("compute_ankle_jacobian_system_: 右肌腱长度过小");
    }
    Eigen::Vector3d hat_r_eqtd = p_r_eqtd / l_r_eqtd;
    
    Eigen::Matrix<double, 3, 2> J_ankle_r = compute_jacobian_ankle(pitch, roll, ankle_side, TendonSide::RIGHT);
    Eigen::Vector3d J_actuator_r = compute_jacobian_actuator(pitch, roll, rbar, ankle_side, TendonSide::RIGHT);
    
    Eigen::RowVector2d J_l_ankle_r = hat_r_eqtd.transpose() * J_ankle_r;
    double J_l_actuator_r = hat_r_eqtd.transpose() * J_actuator_r;
    
    jac_sys.J_constraint << J_l_ankle_l(0), J_l_ankle_l(1), J_l_ankle_r(0), J_l_ankle_r(1);
    jac_sys.J_actuator = Eigen::Matrix2d::Zero();
    jac_sys.J_actuator(0, 0) = J_l_actuator_l;
    jac_sys.J_actuator(1, 1) = J_l_actuator_r;
    
    return jac_sys;
}

Eigen::Vector2d Roban_ankle_solver::joint_to_motor_velocity_single_(
    double pitch, double roll, double lbar, double rbar,
    double dpitch, double droll, AnkleSide ankle_side) const {
    AnkleJacobianSystem jac_sys = compute_ankle_jacobian_system_(
        pitch, roll, lbar, rbar, ankle_side);
    
    Eigen::Vector2d dq(dpitch, droll);
    // 约束方程：dl/dt = J_constraint * dq + J_actuator * dp = 0
    // 所以：dp = -J_actuator^(-1) * J_constraint * dq
    Eigen::Vector2d dp = -jac_sys.J_actuator.colPivHouseholderQr().solve(jac_sys.J_constraint * dq);
    
    return dp;
}

Eigen::Vector2d Roban_ankle_solver::motor_to_joint_velocity_single_(
    double pitch, double roll, double llbar, double lrbar,
    double dllbar, double dlrbar, AnkleSide ankle_side) const {
    AnkleJacobianSystem jac_sys = compute_ankle_jacobian_system_(
        pitch, roll, llbar, lrbar, ankle_side);
    
    Eigen::Vector2d dp;
    dp << dllbar, dlrbar;
    
    // 约束方程：dl/dt = J_constraint * dq + J_actuator * dp = 0
    // 所以：dq = -J_constraint^(-1) * J_actuator * dp
    Eigen::Vector2d dq = -jac_sys.J_constraint.colPivHouseholderQr().solve(
        jac_sys.J_actuator * dp);
    
    return dq;
}

Eigen::Vector2d Roban_ankle_solver::joint_to_motor_current_single_(
    double pitch, double roll, double llbar, double lrbar,
    double tau_pitch, double tau_roll, AnkleSide ankle_side) const {
    AnkleJacobianSystem jac_sys = compute_ankle_jacobian_system_(pitch, roll, llbar, lrbar, ankle_side);
    // 虚功原理: τ_joint^T * dq = τ_motor^T * dp
    // 由于 dp = -J_actuator^{-1} * J_constraint * dq
    // 所以 τ_motor = -(J_constraint^{-1} * J_actuator)^T * τ_joint
    Eigen::Matrix2d J_actuator_to_ankle = jac_sys.J_actuator.colPivHouseholderQr().solve(jac_sys.J_constraint);
    Eigen::Matrix2d J_ankle_to_actuator = J_actuator_to_ankle.inverse();
    Eigen::Vector2d tau(tau_pitch, tau_roll);
    return -J_ankle_to_actuator.transpose() * tau;
}

Eigen::Vector2d Roban_ankle_solver::motor_to_joint_torque_single_(
    double pitch, double roll, double llbar, double lrbar,
    double i_llbar, double i_lrbar, AnkleSide ankle_side) const {
    AnkleJacobianSystem jac_sys = compute_ankle_jacobian_system_(pitch, roll, llbar, lrbar, ankle_side);
    Eigen::Matrix2d J_actuator_to_ankle = jac_sys.J_actuator.colPivHouseholderQr().solve(jac_sys.J_constraint);
    Eigen::Vector2d i(i_llbar, i_lrbar);
    // 虚功原理: τ_joint^T * dq = τ_motor^T * dp
    // 由于 dq = -J_constraint^{-1} * J_actuator * dp
    // 所以 τ_joint = -(J_actuator^{-1} * J_constraint)^T * τ_motor
    return -J_actuator_to_ankle.transpose() * i;
}


std::pair<double, double> Roban_ankle_solver::forward_kinematics_(
    double pitch, double roll, AnkleSide ankle_side, double tol, int max_iter) {
    return forward_kinematics_jacobian_(pitch, roll, ankle_side, tol, max_iter);
}


std::pair<double, double> Roban_ankle_solver::forward_kinematics_jacobian_(
    double pitch, double roll, AnkleSide ankle_side, double tol, int max_iter) {
    TendonParams p_l = get_tendon_params(ankle_side, TendonSide::LEFT);
    TendonParams p_r = get_tendon_params(ankle_side, TendonSide::RIGHT);
    
    double pitch_phys = pitch;
    double roll_phys = roll;
    
    auto safe_acos = [](double x) {
        return std::acos(std::max(-1.0, std::min(1.0, x)));
    };
    
    double c1 = std::cos(pitch_phys);
    double s1 = std::sin(pitch_phys);
    double c2 = std::cos(roll_phys);
    double s2 = std::sin(roll_phys);
    
    // 正确的运动学：考虑关节偏移
    double z_pitch_raw = z_pitch_ - z_roll_;
    
    // eq 点在 knee 坐标系中的位置（考虑关节偏移）
    double p_eq_l_x = x_pitch_ + z_roll_ * s1 + c1 * p_l.x_eq + s1 * s2 * p_l.y_eq + s1 * c2 * p_l.z_eq;
    double p_eq_l_y = c2 * p_l.y_eq - s2 * p_l.z_eq;
    double p_eq_l_z = z_pitch_raw + z_roll_ * c1 - s1 * p_l.x_eq + c1 * s2 * p_l.y_eq + c1 * c2 * p_l.z_eq;
    
    double p_eq_r_x = x_pitch_ + z_roll_ * s1 + c1 * p_r.x_eq + s1 * s2 * p_r.y_eq + s1 * c2 * p_r.z_eq;
    double p_eq_r_y = c2 * p_r.y_eq - s2 * p_r.z_eq;
    double p_eq_r_z = z_pitch_raw + z_roll_ * c1 - s1 * p_r.x_eq + c1 * s2 * p_r.y_eq + c1 * c2 * p_r.z_eq;
    
    // eq 点相对于 bar 的位置（在 yz 平面上的投影）
    double y_LlbarEq = p_eq_l_y;  // bar 的 y 为 0
    double z_LlbarEq = p_eq_l_z - p_l.z_bar;  // 相对于 bar 的 z 位置
    
    double y_LrbarEq = p_eq_r_y;
    double z_LrbarEq = p_eq_r_z - p_r.z_bar;
    
    // x 方向的差值（td - eq）
    double x_LltdEq = (p_l.x_bar + p_l.x_td) - p_eq_l_x;
    double x_LrtdEq = (p_r.x_bar + p_r.x_td) - p_eq_r_x;
    
    // 使用余弦定理求解 actuator angle
    double b_ll = std::sqrt(y_LlbarEq * y_LlbarEq + z_LlbarEq * z_LlbarEq);
    double a_ll = p_l.l_bar;
    double c_ll_sq = p_l.l0_eqtd * p_l.l0_eqtd - x_LltdEq * x_LltdEq;
    double c_ll = c_ll_sq > 0 ? std::sqrt(c_ll_sq) : 0.0;
    double theta_llbar_phys = std::atan2(z_LlbarEq, y_LlbarEq) + 
                               safe_acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2.0 * a_ll * b_ll)) - 
                               std::atan2(p_l.z_td, p_l.y_td);
    
    double b_lr = std::sqrt(y_LrbarEq * y_LrbarEq + z_LrbarEq * z_LrbarEq);
    double a_lr = p_r.l_bar;
    double c_lr_sq = p_r.l0_eqtd * p_r.l0_eqtd - x_LrtdEq * x_LrtdEq;
    double c_lr = c_lr_sq > 0 ? std::sqrt(c_lr_sq) : 0.0;
    double theta_lrbar_phys = std::atan2(z_LrbarEq, y_LrbarEq) - 
                               safe_acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2.0 * a_lr * b_lr)) - 
                               std::atan2(p_r.z_td, p_r.y_td);
    
    double llbar = theta_llbar_phys;
    double lrbar = theta_lrbar_phys;
    
    return std::make_pair(llbar, lrbar);
}



Eigen::VectorXd Roban_ankle_solver::joint_to_motor_position(const Eigen::VectorXd& q) {
    if (q.size() != 4) {
        throw std::runtime_error("joint_to_motor_position: 输入向量必须是4维 [左pitch, 左roll, 右pitch, 右roll]");
    }
    
    auto left_result = forward_kinematics_(q[0], q[1], AnkleSide::LEFT);
    auto right_result = forward_kinematics_(q[2], q[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << left_result.first, left_result.second, 
              right_result.first, right_result.second;
    return result;
}

Eigen::VectorXd Roban_ankle_solver::motor_to_joint_position(const Eigen::VectorXd& p) {
    if (p.size() != 4) {
        throw std::runtime_error("motor_to_joint_position: 输入向量必须是4维 [左llbar, 左lrbar, 右llbar, 右lrbar]");
    }
    
    auto left_result = inverse_kinematics_(p[0], p[1], AnkleSide::LEFT);
    auto right_result = inverse_kinematics_(p[2], p[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << left_result.first, left_result.second,
              right_result.first, right_result.second;
    return result;
}

Eigen::VectorXd Roban_ankle_solver::joint_to_motor_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq) {
    if (q.size() != 4 || p.size() != 4 || dq.size() != 4) {
        throw std::runtime_error("joint_to_motor_velocity: 输入向量必须是4维");
    }
    
    Eigen::Vector2d dp_left = joint_to_motor_velocity_single_(
        q[0], q[1], p[0], p[1], dq[0], dq[1], AnkleSide::LEFT);
    Eigen::Vector2d dp_right = joint_to_motor_velocity_single_(
        q[2], q[3], p[2], p[3], dq[2], dq[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << dp_left(0), dp_left(1), dp_right(0), dp_right(1);
    return result;
}

Eigen::VectorXd Roban_ankle_solver::motor_to_joint_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) {
    if (q.size() != 4 || p.size() != 4 || dp.size() != 4) {
        throw std::runtime_error("motor_to_joint_velocity: 输入向量必须是4维");
    }
    
    Eigen::Vector2d dq_left = motor_to_joint_velocity_single_(
        q[0], q[1], p[0], p[1], dp[0], dp[1], AnkleSide::LEFT);
    Eigen::Vector2d dq_right = motor_to_joint_velocity_single_(
        q[2], q[3], p[2], p[3], dp[2], dp[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << dq_left(0), dq_left(1), dq_right(0), dq_right(1);
    return result;
}

Eigen::VectorXd Roban_ankle_solver::joint_to_motor_current(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau) {
    if (q.size() != 4 || p.size() != 4 || tau.size() != 4) {
        throw std::runtime_error("joint_to_motor_current: 输入向量必须是4维");
    }
    
    Eigen::Vector2d i_left = joint_to_motor_current_single_(
        q[0], q[1], p[0], p[1], tau[0], tau[1], AnkleSide::LEFT);
    Eigen::Vector2d i_right = joint_to_motor_current_single_(
        q[2], q[3], p[2], p[3], tau[2], tau[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << i_left(0), i_left(1), i_right(0), i_right(1);
    return result;
}

Eigen::VectorXd Roban_ankle_solver::motor_to_joint_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i) {
    if (q.size() != 4 || p.size() != 4 || i.size() != 4) {
        throw std::runtime_error("motor_to_joint_torque: 输入向量必须是4维");
    }
    
    Eigen::Vector2d tau_left = motor_to_joint_torque_single_(
        q[0], q[1], p[0], p[1], i[0], i[1], AnkleSide::LEFT);
    Eigen::Vector2d tau_right = motor_to_joint_torque_single_(
        q[2], q[3], p[2], p[3], i[2], i[3], AnkleSide::RIGHT);
    
    Eigen::VectorXd result(4);
    result << tau_left(0), tau_left(1), tau_right(0), tau_right(1);
    return result;
}

std::pair<double, double> Roban_ankle_solver::inverse_kinematics_jacobian_(
    double llbar, double lrbar, AnkleSide ankle_side, double tol, int max_iter) {
    TendonParams p_l = get_tendon_params(ankle_side, TendonSide::LEFT);
    TendonParams p_r = get_tendon_params(ankle_side, TendonSide::RIGHT);
    
    double llbar_phys = llbar;
    double lrbar_phys = lrbar;
    double pitch_phys = 0.0;
    double roll_phys = 0.0;
    
    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d p_ll_eqtd = compute_tendon_vector(pitch_phys, roll_phys, llbar_phys, ankle_side, TendonSide::LEFT);
        Eigen::Vector3d p_lr_eqtd = compute_tendon_vector(pitch_phys, roll_phys, lrbar_phys, ankle_side, TendonSide::RIGHT);
        
        double l_ll_eqtd = p_ll_eqtd.norm();
        double l_lr_eqtd = p_lr_eqtd.norm();
        
        if (l_ll_eqtd < 1e-10 || l_lr_eqtd < 1e-10) {
            throw std::runtime_error("inverse_kinematics_jacobian_: 肌腱长度过小");
        }
        
        Eigen::Vector2d residual(l_ll_eqtd - p_l.l0_eqtd, l_lr_eqtd - p_r.l0_eqtd);
        
        if (residual.norm() < tol) {
            break;
        }
        
        Eigen::Vector3d hat_ll_eqtd = p_ll_eqtd / l_ll_eqtd;
        Eigen::Vector3d hat_lr_eqtd = p_lr_eqtd / l_lr_eqtd;
        
        Eigen::Matrix<double, 3, 2> J_ankle_ll = compute_jacobian_ankle(pitch_phys, roll_phys, ankle_side, TendonSide::LEFT);
        Eigen::Matrix<double, 3, 2> J_ankle_lr = compute_jacobian_ankle(pitch_phys, roll_phys, ankle_side, TendonSide::RIGHT);
        
        Eigen::RowVector2d J_l_ankle_ll = hat_ll_eqtd.transpose() * J_ankle_ll;
        Eigen::RowVector2d J_l_ankle_lr = hat_lr_eqtd.transpose() * J_ankle_lr;
        
        Eigen::Matrix2d J_constraint;
        J_constraint << J_l_ankle_ll(0), J_l_ankle_ll(1), J_l_ankle_lr(0), J_l_ankle_lr(1);
        
        Eigen::Vector2d delta = J_constraint.colPivHouseholderQr().solve(residual);
        pitch_phys -= delta(0);
        roll_phys -= delta(1);
        
        if (delta.norm() < tol) {
            break;
        }
    }
    
    double pitch = pitch_phys;
    double roll = roll_phys;
    
    return std::make_pair(pitch, roll);
}

Roban_ankle_solver::VerificationResult Roban_ankle_solver::verify_with_mujoco(
    const Eigen::VectorXd& q_mujoco,
    const Eigen::VectorXd& p_mujoco,
    const Eigen::VectorXd& dq_mujoco,
    const Eigen::VectorXd& tau_mujoco,
    double position_tolerance,
    double velocity_tolerance,
    double torque_tolerance) {
    
    VerificationResult result;
    
    Eigen::VectorXd p_computed = joint_to_motor_position(q_mujoco);
    Eigen::VectorXd q_back = motor_to_joint_position(p_computed);
    result.position_error_norm = (q_mujoco - q_back).norm();
    
    if (dq_mujoco.size() == 4 && dq_mujoco.norm() > 1e-10) {
        Eigen::VectorXd dp_computed = joint_to_motor_velocity(q_mujoco, p_mujoco, dq_mujoco);
        Eigen::VectorXd dq_back = motor_to_joint_velocity(q_mujoco, p_mujoco, dp_computed);
        result.velocity_error_norm = (dq_mujoco - dq_back).norm();
    } else {
        result.velocity_error_norm = 0.0;
    }
    
    if (tau_mujoco.size() == 4 && tau_mujoco.norm() > 1e-10) {
        Eigen::VectorXd i_computed = joint_to_motor_current(q_mujoco, p_mujoco, tau_mujoco);
        Eigen::VectorXd tau_back = motor_to_joint_torque(q_mujoco, p_mujoco, i_computed);
        result.torque_error_norm = (tau_mujoco - tau_back).norm();
    } else {
        result.torque_error_norm = 0.0;
    }
    
    result.is_valid = (result.position_error_norm < position_tolerance) &&
                      (result.velocity_error_norm < velocity_tolerance) &&
                      (result.torque_error_norm < torque_tolerance);
    
    return result;
}

} // namespace kuavo_solver
