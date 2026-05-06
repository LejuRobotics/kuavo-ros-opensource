#include "kuavo_solver/ankle/axis_offset_ankle_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

AxisOffsetAnkleSolver::AxisOffsetLoadedParam AxisOffsetAnkleSolver::loadOffsetParam(
    const std::string& ankle_solver_type_token, const std::string& ankle_config_dir) {
    auto loader = SolverTools::YamlLoader::OpenVariant(
        ankle_config_dir, "axisoffsetanklesolver.yaml", 1, ankle_solver_type_token);

    AxisOffsetLoadedParam out;
    AxisOffsetAnkleParams& p = out.params;

    // 限位 [min, max]
    loader.require("ankle_pitch_limits", &p.ankle_pitch_lo, &p.ankle_pitch_hi);
    loader.require("ankle_roll_limits", &p.ankle_roll_lo, &p.ankle_roll_hi);

    // 几何标量
    loader.require("z_pitch", &p.z_pitch)
          .require("z_roll", &p.z_roll)
          .require("x_pitch", &p.x_pitch)
          .require("x_lleq", &p.x_lleq)
          .require("y_lleq", &p.y_lleq)
          .require("z_lleq", &p.z_lleq)
          .require("x_lreq", &p.x_lreq)
          .require("y_lreq", &p.y_lreq)
          .require("z_lreq", &p.z_lreq)
          .require("x_llbar", &p.x_llbar)
          .require("z_llbar", &p.z_llbar)
          .require("x_lrbar", &p.x_lrbar)
          .require("z_lrbar", &p.z_lrbar)
          .require("x_lltd", &p.x_lltd)
          .require("y_lltd", &p.y_lltd)
          .require("z_lltd", &p.z_lltd)
          .require("x_lrtd", &p.x_lrtd)
          .require("y_lrtd", &p.y_lrtd)
          .require("z_lrtd", &p.z_lrtd)
          .require("l0_ll_eqtd", &p.l0_ll_eqtd)
          .require("l0_lr_eqtd", &p.l0_lr_eqtd)
          .require("x_rleq", &p.x_rleq)
          .require("y_rleq", &p.y_rleq)
          .require("z_rleq", &p.z_rleq)
          .require("x_rreq", &p.x_rreq)
          .require("y_rreq", &p.y_rreq)
          .require("z_rreq", &p.z_rreq)
          .require("x_rlbar", &p.x_rlbar)
          .require("z_rlbar", &p.z_rlbar)
          .require("x_rrbar", &p.x_rrbar)
          .require("z_rrbar", &p.z_rrbar)
          .require("x_rltd", &p.x_rltd)
          .require("y_rltd", &p.y_rltd)
          .require("z_rltd", &p.z_rltd)
          .require("x_rrtd", &p.x_rrtd)
          .require("y_rrtd", &p.y_rrtd)
          .require("z_rrtd", &p.z_rrtd)
          .require("l0_rl_eqtd", &p.l0_rl_eqtd)
          .require("l0_rr_eqtd", &p.l0_rr_eqtd)
          .require("default_tolerance", &p.default_tolerance)
          .require("max_iterations", &p.max_iterations);

    // bar 长度以 td 的 (y,z) 推导为准（不从 YAML 读取 l_*bar）。
    out.l_llbar = std::sqrt(p.y_lltd * p.y_lltd + p.z_lltd * p.z_lltd);
    out.l_lrbar = std::sqrt(p.y_lrtd * p.y_lrtd + p.z_lrtd * p.z_lrtd);
    out.l_rlbar = std::sqrt(p.y_rltd * p.y_rltd + p.z_rltd * p.z_rltd);
    out.l_rrbar = std::sqrt(p.y_rrtd * p.y_rrtd + p.z_rrtd * p.z_rrtd);

    // 膝关节联动（可选）
    p.knee_linkage_enabled = loader.has("knee_linkage_enabled");
    if (p.knee_linkage_enabled) {
        loader.require("z_bar_knee", &p.z_bar_knee)
              .require("l_tendon", &p.l_tendon)
              .require("l_bar_td", &p.l_bar_td)
              .require("l_knee_eq", &p.l_knee_eq)
              .require("q_offset_knee", &p.q_offset_knee)
              .require("q_offset_bar", &p.q_offset_bar);
        // x_bar_knee 是新引入的机架 x 偏置，为保持向后兼容旧 YAML：未提供时保持 0。
        if (loader.has("x_bar_knee")) {
            loader.require("x_bar_knee", &p.x_bar_knee);
        } else {
            p.x_bar_knee = 0.0;
        }
    }

    return out;
}

AxisOffsetAnkleSolver::AxisOffsetAnkleSolver(const AxisOffsetAnkleParams& params) {
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

    knee_linkage_enabled_ = params.knee_linkage_enabled;
    x_bar_knee_ = params.x_bar_knee;
    z_bar_knee_ = params.z_bar_knee;
    l_tendon_ = params.l_tendon;
    l_bar_td_ = params.l_bar_td;
    l_knee_eq_ = params.l_knee_eq;
    q_offset_knee_ = params.q_offset_knee;
    q_offset_bar_ = params.q_offset_bar;

    ankle_pitch_lo_ = params.ankle_pitch_lo;
    ankle_pitch_hi_ = params.ankle_pitch_hi;
    ankle_roll_lo_ = params.ankle_roll_lo;
    ankle_roll_hi_ = params.ankle_roll_hi;
}

AxisOffsetAnkleSolver::KneeJacobians AxisOffsetAnkleSolver::knee_jacobians_(double q_knee, double p_knee) const {
    KneeJacobians out;
    if (!knee_linkage_enabled_) {
        return out;
    }
    // W frame: l_knee_bar 轴为原点；l_knee 轴位置 = (x_bar_knee_, z_bar_knee_)。
    const double sq4 = std::sin(q_knee + q_offset_knee_);
    const double cq4 = std::cos(q_knee + q_offset_knee_);
    const double x_BarEq_W = x_bar_knee_ + l_knee_eq_ * sq4;
    const double z_BarEq_W = z_bar_knee_ + l_knee_eq_ * cq4;
    const double sp4 = std::sin(-p_knee - q_offset_bar_);
    const double cp4 = std::cos(-p_knee - q_offset_bar_);
    const double x_BarTd_W = l_bar_td_ * sp4;
    const double z_BarTd_W = l_bar_td_ * cp4;
    Eigen::Vector2d Jbar_p_WTd(-l_bar_td_ * cp4, l_bar_td_ * sp4);
    Eigen::Vector2d Jknee_p_WEq(l_knee_eq_ * cq4, -l_knee_eq_ * sq4);
    Eigen::Vector2d Jxx_l_Td_Vector(x_BarEq_W - x_BarTd_W, z_BarEq_W - z_BarTd_W);
    const double den_bar = Jxx_l_Td_Vector.dot(Jbar_p_WTd);
    const double den_knee = Jxx_l_Td_Vector.dot(Jknee_p_WEq);
    // 闭环 4-bar 的隐式 Jacobian 分母为 0 表示构型奇异；fail-fast 上抛，避免静默返回 1.0 掩盖问题。
    if (std::abs(den_bar) < 1e-14) {
        throw std::runtime_error(
            "AxisOffsetAnkleSolver::knee_jacobians_: Jknee_bar 分母奇异 (den_bar≈0)；"
            "当前构型 q_knee=" + std::to_string(q_knee) + ", p_knee=" + std::to_string(p_knee));
    }
    if (std::abs(den_knee) < 1e-14) {
        throw std::runtime_error(
            "AxisOffsetAnkleSolver::knee_jacobians_: Jbar_knee 分母奇异 (den_knee≈0)；"
            "当前构型 q_knee=" + std::to_string(q_knee) + ", p_knee=" + std::to_string(p_knee));
    }
    out.Jknee_bar = Jxx_l_Td_Vector.dot(Jknee_p_WEq) / den_bar;
    out.Jbar_knee = Jxx_l_Td_Vector.dot(Jbar_p_WTd) / den_knee;
    return out;
}

double AxisOffsetAnkleSolver::knee_joint_to_motor(double q_knee) const {
    if (!knee_linkage_enabled_) {
        return q_knee;
    }
    // W frame: l_knee_bar 轴为原点；l_knee 轴位置 = (x_bar_knee_, z_bar_knee_)。
    const double sq4 = std::sin(q_knee + q_offset_knee_);
    const double cq4 = std::cos(q_knee + q_offset_knee_);
    const double x_BarEq_W = x_bar_knee_ + l_knee_eq_ * sq4;
    const double z_BarEq_W = z_bar_knee_ + l_knee_eq_ * cq4;
    const double l_BarEq = std::sqrt(x_BarEq_W * x_BarEq_W + z_BarEq_W * z_BarEq_W);
    const double acos_arg = (l_bar_td_ * l_bar_td_ + l_BarEq * l_BarEq - l_tendon_ * l_tendon_) / (2 * l_bar_td_ * l_BarEq);
    return M_PI - SolverTools::SafeAcos(acos_arg) + std::atan2(x_BarEq_W, -z_BarEq_W) - q_offset_bar_;
}

double AxisOffsetAnkleSolver::knee_motor_to_joint(double p_knee) const {
    if (!knee_linkage_enabled_) {
        return p_knee;
    }
    // Knee frame: l_knee 轴为原点；td anchor 在 W（bar frame）下 = (l_bar_td*sp4, l_bar_td*cp4)，
    // 减去 knee 轴在 W 的位置 (x_bar_knee_, z_bar_knee_) 得到 td 在 knee frame 的坐标。
    const double sp4 = std::sin(-p_knee - q_offset_bar_);
    const double cp4 = std::cos(-p_knee - q_offset_bar_);
    const double x_KneeTd_W = l_bar_td_ * sp4 - x_bar_knee_;
    const double z_KneeTd_W = l_bar_td_ * cp4 - z_bar_knee_;
    const double l_KneeTd = std::sqrt(x_KneeTd_W * x_KneeTd_W + z_KneeTd_W * z_KneeTd_W);
    const double acos_arg = (l_knee_eq_ * l_knee_eq_ + l_KneeTd * l_KneeTd - l_tendon_ * l_tendon_) / (2 * l_knee_eq_ * l_KneeTd);
    return SolverTools::SafeAcos(acos_arg) - std::atan2(-x_KneeTd_W, z_KneeTd_W) - q_offset_knee_;
}

double AxisOffsetAnkleSolver::knee_joint_to_motor_velocity(double q_knee, double p_knee, double dq_knee) const {
    if (!knee_linkage_enabled_) {
        return dq_knee;
    }
    return knee_jacobians_(q_knee, p_knee).Jknee_bar * dq_knee;
}

double AxisOffsetAnkleSolver::knee_motor_to_joint_velocity(double q_knee, double p_knee, double dp_knee) const {
    if (!knee_linkage_enabled_) {
        return dp_knee;
    }
    return knee_jacobians_(q_knee, p_knee).Jbar_knee * dp_knee;
}

double AxisOffsetAnkleSolver::knee_joint_to_motor_current(double q_knee, double p_knee, double tau_joint) const {
    if (!knee_linkage_enabled_) {
        return tau_joint;
    }
    return knee_jacobians_(q_knee, p_knee).Jbar_knee * tau_joint;
}

double AxisOffsetAnkleSolver::knee_motor_to_joint_torque(double q_knee, double p_knee, double i_motor) const {
    if (!knee_linkage_enabled_) {
        return i_motor;
    }
    return knee_jacobians_(q_knee, p_knee).Jknee_bar * i_motor;
}

Eigen::Vector3d AxisOffsetAnkleSolver::compute_tendon_vector(
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


Eigen::Matrix<double, 3, 2> AxisOffsetAnkleSolver::compute_jacobian_ankle(
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


Eigen::Vector3d AxisOffsetAnkleSolver::compute_jacobian_actuator(
    double pitch, double roll, double actuator_angle,
    AnkleSide ankle_side, TendonSide tendon_side) const {
    TendonParams p = get_tendon_params(ankle_side, tendon_side);
    
    double c_act = std::cos(actuator_angle);
    double s_act = std::sin(actuator_angle);
    
    Eigen::Vector3d jacp_actuator;
    jacp_actuator << 0, -p.y_td * s_act - p.z_td * c_act, p.y_td * c_act - p.z_td * s_act;
    
    return jacp_actuator;
}


std::pair<double, double> AxisOffsetAnkleSolver::inverse_kinematics_(
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

TendonParams AxisOffsetAnkleSolver::get_tendon_params(
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

JacobianSystem AxisOffsetAnkleSolver::compute_ankle_jacobian_system_(
    double pitch, double roll, double lbar, double rbar, AnkleSide ankle_side) const {
    JacobianSystem jac_sys;
    
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
    
    jac_sys.J_constraint = Eigen::Matrix2d::Zero();
    jac_sys.J_constraint << J_l_ankle_l(0), J_l_ankle_l(1), J_l_ankle_r(0), J_l_ankle_r(1);
    jac_sys.J_actuator = Eigen::Matrix2d::Zero();
    jac_sys.J_actuator(0, 0) = J_l_actuator_l;
    jac_sys.J_actuator(1, 1) = J_l_actuator_r;
    
    return jac_sys;
}

JacobianSystem AxisOffsetAnkleSolver::jacobian_system(
    double pitch, double roll, double lbar, double rbar, AnkleSide ankle_side) const {
    return compute_ankle_jacobian_system_(pitch, roll, lbar, rbar, ankle_side);
}

Eigen::Vector2d AxisOffsetAnkleSolver::joint_to_motor_velocity_single_(
    double pitch, double roll, double lbar, double rbar,
    double dpitch, double droll, AnkleSide ankle_side) const {
    const JacobianSystem jac = compute_ankle_jacobian_system_(pitch, roll, lbar, rbar, ankle_side);
    const Eigen::Vector2d dq(dpitch, droll);
    return SolverTools::MotorVelocityFromJoint(jac, dq);
}

Eigen::Vector2d AxisOffsetAnkleSolver::motor_to_joint_velocity_single_(
    double pitch, double roll, double llbar, double lrbar,
    double dllbar, double dlrbar, AnkleSide ankle_side) const {
    const JacobianSystem jac = compute_ankle_jacobian_system_(pitch, roll, llbar, lrbar, ankle_side);
    const Eigen::Vector2d dp(dllbar, dlrbar);
    return SolverTools::JointVelocityFromMotor(jac, dp);
}

Eigen::Vector2d AxisOffsetAnkleSolver::joint_to_motor_current_single_(
    double pitch, double roll, double llbar, double lrbar,
    double tau_pitch, double tau_roll, AnkleSide ankle_side) const {
    const JacobianSystem jac = compute_ankle_jacobian_system_(pitch, roll, llbar, lrbar, ankle_side);
    const Eigen::Vector2d tau(tau_pitch, tau_roll);
    return SolverTools::MotorCurrentFromJointTorque(jac, tau);
}

Eigen::Vector2d AxisOffsetAnkleSolver::motor_to_joint_torque_single_(
    double pitch, double roll, double llbar, double lrbar,
    double i_llbar, double i_lrbar, AnkleSide ankle_side) const {
    const JacobianSystem jac = compute_ankle_jacobian_system_(pitch, roll, llbar, lrbar, ankle_side);
    const Eigen::Vector2d i(i_llbar, i_lrbar);
    return SolverTools::JointTorqueFromMotorCurrent(jac, i);
}


std::pair<double, double> AxisOffsetAnkleSolver::forward_kinematics_(
    double pitch, double roll, AnkleSide ankle_side, double tol, int max_iter) {
    return forward_kinematics_jacobian_(pitch, roll, ankle_side, tol, max_iter);
}


std::pair<double, double> AxisOffsetAnkleSolver::forward_kinematics_jacobian_(
    double pitch, double roll, AnkleSide ankle_side, double tol, int max_iter) {
    TendonParams p_l = get_tendon_params(ankle_side, TendonSide::LEFT);
    TendonParams p_r = get_tendon_params(ankle_side, TendonSide::RIGHT);
    
    double pitch_phys = pitch;
    double roll_phys = roll;

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
                               SolverTools::SafeAcos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2.0 * a_ll * b_ll)) -
                               std::atan2(p_l.z_td, p_l.y_td);
    
    double b_lr = std::sqrt(y_LrbarEq * y_LrbarEq + z_LrbarEq * z_LrbarEq);
    double a_lr = p_r.l_bar;
    double c_lr_sq = p_r.l0_eqtd * p_r.l0_eqtd - x_LrtdEq * x_LrtdEq;
    double c_lr = c_lr_sq > 0 ? std::sqrt(c_lr_sq) : 0.0;
    double theta_lrbar_phys = std::atan2(z_LrbarEq, y_LrbarEq) - 
                               SolverTools::SafeAcos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2.0 * a_lr * b_lr)) - 
                               std::atan2(p_r.z_td, p_r.y_td);
    
    double llbar = theta_llbar_phys;
    double lrbar = theta_lrbar_phys;
    
    return std::make_pair(llbar, lrbar);
}



Eigen::VectorXd AxisOffsetAnkleSolver::joint_to_motor_position(const Eigen::VectorXd& q) {
    if (q.size() != 4) {
        throw std::runtime_error(
            "joint_to_motor_position: 输入须为 4 维 [左pitch,左roll,右pitch,右roll]");
    }

    auto left_result = forward_kinematics_(q[0], q[1], AnkleSide::LEFT);
    auto right_result = forward_kinematics_(q[2], q[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << left_result.first, left_result.second, right_result.first, right_result.second;
    return result;
}

Eigen::VectorXd AxisOffsetAnkleSolver::motor_to_joint_position(const Eigen::VectorXd& p) {
    if (p.size() != 4) {
        throw std::runtime_error("motor_to_joint_position: 输入须为 4 维");
    }

    auto left_result = inverse_kinematics_(p[0], p[1], AnkleSide::LEFT);
    auto right_result = inverse_kinematics_(p[2], p[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << left_result.first, left_result.second, right_result.first, right_result.second;
    return result;
}

Eigen::VectorXd AxisOffsetAnkleSolver::joint_to_motor_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq) {
    if (q.size() != 4 || p.size() != 4 || dq.size() != 4) {
        throw std::runtime_error("joint_to_motor_velocity: 输入须全为 4 维");
    }

    Eigen::Vector2d dp_left = joint_to_motor_velocity_single_(
        q[0], q[1], p[0], p[1], dq[0], dq[1], AnkleSide::LEFT);
    Eigen::Vector2d dp_right = joint_to_motor_velocity_single_(
        q[2], q[3], p[2], p[3], dq[2], dq[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << dp_left(0), dp_left(1), dp_right(0), dp_right(1);
    return result;
}

Eigen::VectorXd AxisOffsetAnkleSolver::motor_to_joint_velocity(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp) {
    if (q.size() != 4 || p.size() != 4 || dp.size() != 4) {
        throw std::runtime_error("motor_to_joint_velocity: 输入须全为 4 维");
    }

    Eigen::Vector2d dq_left = motor_to_joint_velocity_single_(
        q[0], q[1], p[0], p[1], dp[0], dp[1], AnkleSide::LEFT);
    Eigen::Vector2d dq_right = motor_to_joint_velocity_single_(
        q[2], q[3], p[2], p[3], dp[2], dp[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << dq_left(0), dq_left(1), dq_right(0), dq_right(1);
    return result;
}

Eigen::VectorXd AxisOffsetAnkleSolver::joint_to_motor_current(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau) {
    if (q.size() != 4 || p.size() != 4 || tau.size() != 4) {
        throw std::runtime_error("joint_to_motor_current: 输入须全为 4 维");
    }

    Eigen::Vector2d i_left = joint_to_motor_current_single_(
        q[0], q[1], p[0], p[1], tau[0], tau[1], AnkleSide::LEFT);
    Eigen::Vector2d i_right = joint_to_motor_current_single_(
        q[2], q[3], p[2], p[3], tau[2], tau[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << i_left(0), i_left(1), i_right(0), i_right(1);
    return result;
}

Eigen::VectorXd AxisOffsetAnkleSolver::motor_to_joint_torque(
    const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i) {
    if (q.size() != 4 || p.size() != 4 || i.size() != 4) {
        throw std::runtime_error("motor_to_joint_torque: 输入须全为 4 维");
    }

    Eigen::Vector2d tau_left = motor_to_joint_torque_single_(
        q[0], q[1], p[0], p[1], i[0], i[1], AnkleSide::LEFT);
    Eigen::Vector2d tau_right = motor_to_joint_torque_single_(
        q[2], q[3], p[2], p[3], i[2], i[3], AnkleSide::RIGHT);

    Eigen::VectorXd result(4);
    result << tau_left(0), tau_left(1), tau_right(0), tau_right(1);
    return result;
}

std::pair<double, double> AxisOffsetAnkleSolver::inverse_kinematics_jacobian_(
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

AxisOffsetAnkleSolver::VerificationResult AxisOffsetAnkleSolver::verify_with_mujoco(
    const Eigen::VectorXd& q_mujoco,
    const Eigen::VectorXd& p_mujoco,
    const Eigen::VectorXd& dq_mujoco,
    const Eigen::VectorXd& tau_mujoco,
    double position_tolerance,
    double velocity_tolerance,
    double torque_tolerance) {
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

} // namespace kuavo_solver
