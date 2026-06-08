#include "kuavo_solver/ankle/ankle_solver.h"
#include "kuavo_solver/common/config_dir_macros.h"
#include "kuavo_solver/common/solver_tools.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace kuavo_solver {

// ============================================================================
// 12D leg 布局索引（axis_offset / 5gen_2、s2gen_2）
// ----------------------------------------------------------------------------
// 与 AxisOffsetAnkleSolver 历史上的 joint_to_motor_*_leg12_ 完全一致。
//
//   idx 0..2 : 左 hip yaw / roll / pitch        （直通）
//   idx 3    : 左 knee 关节 ↔ 电机 1D
//   idx 4,5  : 左踝 pitch / roll（关节）；joint→motor 后同槽写入左右 bar，且 [4]、[5]
//             与内部 bar 向量 (p₀,p₁) 交叉赋值（result[5]=p₀, result[4]=p₁）
//   idx 6..8 : 中段（历史中踝正解后保持与输入一致的直通分量）
//   idx 9    : 右 knee
//   idx 10,11: 右踝 pitch / roll（关节）；电机侧无交叉，对应内部 (p₂,p₃)
//
// AxisOffsetAnkleSolver 4D：[L_pitch,L_roll,R_pitch,R_roll] ↔ [L_l_bar,L_r_bar,R_l_bar,R_r_bar]
// ============================================================================

namespace {

// ---- knee 1D 索引 ----------------------------------------------------------
constexpr int kIdxLKnee = 3;
constexpr int kIdxRKnee = 9;

// ---- ankle 关节 / 电机 在 leg12 中的槽位-----------
constexpr int kIdxLAnklePitch = 4;
constexpr int kIdxLAnkleRoll  = 5;
constexpr int kIdxRAnklePitch = 10;
constexpr int kIdxRAnkleRoll  = 11;

/** 把 leg12 向量在"ankle joint 侧"的 4 维抽出：[L_pitch, L_roll, R_pitch, R_roll] */
inline Eigen::VectorXd PickAnkleJoint4(const Eigen::VectorXd& v12) {
    Eigen::VectorXd v4(4);
    v4 << v12[kIdxLAnklePitch], v12[kIdxLAnkleRoll],
          v12[kIdxRAnklePitch], v12[kIdxRAnkleRoll];
    return v4;
}

/** 把 leg12 向量在电机侧装进 4D：p₄ = leg[5], leg[4], leg[10], leg[11] */
inline Eigen::VectorXd PickAnkleMotor4(const Eigen::VectorXd& v12) {
    Eigen::VectorXd v4(4);
    v4 << v12[5], v12[4], v12[kIdxRAnklePitch], v12[kIdxRAnkleRoll];
    return v4;
}

/** 把 4D（joint 侧）写回 leg12：v4 = [L_pitch, L_roll, R_pitch, R_roll] */
inline void WriteAnkleJoint4(Eigen::VectorXd& out12, const Eigen::VectorXd& v4) {
    out12[kIdxLAnklePitch] = v4[0];
    out12[kIdxLAnkleRoll]  = v4[1];
    out12[kIdxRAnklePitch] = v4[2];
    out12[kIdxRAnkleRoll]  = v4[3];
}

/** 把 4D（motor）写回 leg12：左腿交叉 out[5]=v₀、out[4]=v₁；右腿 out[10]=v₂、out[11]=v₃ */
inline void WriteAnkleMotor4(Eigen::VectorXd& out12, const Eigen::VectorXd& v4) {
    out12[5] = v4[0];
    out12[4] = v4[1];
    out12[kIdxRAnklePitch] = v4[2];
    out12[kIdxRAnkleRoll]  = v4[3];
}

/** motor_to_joint 后对踝 pitch/roll 钳位（与 YAML ankle_*_limits 一致）。 */
inline void ClampAnkleMotorToJointDegreesOfFreedom(const AxisOffsetAnkleSolver& sol, Eigen::VectorXd& q12) {
    const double p_lo = sol.ankle_pitch_lo();
    const double p_hi = sol.ankle_pitch_hi();
    const double r_lo = sol.ankle_roll_lo();
    const double r_hi = sol.ankle_roll_hi();
    q12[kIdxLAnklePitch] = std::max(std::min(q12[kIdxLAnklePitch], p_hi), p_lo);
    q12[kIdxLAnkleRoll]  = std::max(std::min(q12[kIdxLAnkleRoll], r_hi), r_lo);
    q12[kIdxRAnklePitch] = std::max(std::min(q12[kIdxRAnklePitch], p_hi), p_lo);
    q12[kIdxRAnkleRoll]  = std::max(std::min(q12[kIdxRAnkleRoll], r_hi), r_lo);
}

/** disabled（"none"）模式下的 joint→motor 位置口径：两侧 knee 不得小于 0。 */
Eigen::VectorXd IdentityLegJointToMotorPosition(const Eigen::VectorXd& q) {
    Eigen::VectorXd joint_q = q;
    joint_q[kIdxLKnee] = std::max(joint_q[kIdxLKnee], 0.0);
    joint_q[kIdxRKnee] = std::max(joint_q[kIdxRKnee], 0.0);
    return joint_q;
}

}  // namespace

// ============================================================================
// 析构 / 构造
// ============================================================================

AnkleSolver::~AnkleSolver() = default;

// ============================================================================
// 配置加载：从 KUAVO_ANKLE_CONFIG_DIR 加载对应变体（axis_offset / fixed_axis）
// ============================================================================

void AnkleSolver::getconfig(const std::string& ankle_solver_type_token) {
    const std::string ankle_config_dir =
        kuavo_solver::SolverTools::RequireConfigDir(KUAVO_ANKLE_CONFIG_DIR, "AnkleSolver");

    ankle_solver_type_token_ = ankle_solver_type_token;
    axis_offset_solver_.reset();
    fixed_axis_solver_.reset();
    std::cout << "[AnkleSolver] ankle_solver_type: " << ankle_solver_type_token_ << std::endl;

    const kuavo_solver::AnkleSolverType type =
        kuavo_solver::AnkleSolverTypeFromString(ankle_solver_type_token);

    if (type == kuavo_solver::AnkleSolverType::ANKLE_SOLVER_TYPE_NONE) {
        return;
    }

    if (kuavo_solver::IsAxisOffsetSolverType(type)) {
        const auto loaded =
            kuavo_solver::AxisOffsetAnkleSolver::loadOffsetParam(ankle_solver_type_token_, ankle_config_dir);
        axis_offset_solver_ = std::make_unique<kuavo_solver::AxisOffsetAnkleSolver>(loaded.params);
        return;
    }

    // fixed-axis family
    const auto loaded =
        kuavo_solver::FixedAxisAnkleSolver::loadFixedParam(ankle_solver_type_token_, ankle_config_dir);
    fixed_axis_solver_ =
        std::unique_ptr<kuavo_solver::FixedAxisAnkleSolver, kuavo_solver::FixedAxisAnkleSolverDeleter>(
            new kuavo_solver::FixedAxisAnkleSolver(loaded.layout, loaded.config, N_ITER,
                                                   loaded.bypass_knee_floor_for_joint_to_motor_position));
}

// ============================================================================
// AnkleSolver 公有接口 — 12D 分派
// ----------------------------------------------------------------------------
// 统一实现策略：
//   1) fixed_axis 优先直接委派；
//   2) axis_offset 存在且尺寸为 12：在本类内"代码级平级"地分别调用
//      knee 业务（1D，左/右各一次）与 ankle 业务（4D 双边一次性），
//      不再嵌套到 AxisOffsetAnkleSolver 的 12D 包装；
//   3) axis_offset 存在且尺寸非 12：视为透传到 axis_offset 4D 接口
//      （供底层单元测试 / pybind 直连使用）；
//   4) 均无解算器且 type==NONE：返回输入直通（必要时做关节下限保护）。
// ============================================================================

Eigen::VectorXd AnkleSolver::joint_to_motor_position(const Eigen::VectorXd& q) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->joint_to_motor_position(q);
    }
    if (axis_offset_solver_) {
        if (q.size() != 12) {
            return axis_offset_solver_->joint_to_motor_position(q);
        }
        Eigen::VectorXd p = q;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D（平级调用，左/右各一次）
        p[kIdxLKnee] = axis_offset_solver_->knee_joint_to_motor(q[kIdxLKnee]);
        p[kIdxRKnee] = axis_offset_solver_->knee_joint_to_motor(q[kIdxRKnee]);

        // 业务 B：ankle 4D 双边（平级调用，一次性）
        const Eigen::VectorXd q_ankle4 = PickAnkleJoint4(q);
        const Eigen::VectorXd p_ankle4 = axis_offset_solver_->joint_to_motor_position(q_ankle4);
        WriteAnkleMotor4(p, p_ankle4);
        return p;
    }
    if (ankle_solver_type_token_ == "none") {
        return IdentityLegJointToMotorPosition(q);
    }
    throw std::logic_error("AnkleSolver::joint_to_motor_position: ankle_solver_type not mapped");
}

Eigen::VectorXd AnkleSolver::motor_to_joint_position(const Eigen::VectorXd& p) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->motor_to_joint_position(p);
    }
    if (axis_offset_solver_) {
        if (p.size() != 12) {
            return axis_offset_solver_->motor_to_joint_position(p);
        }
        Eigen::VectorXd q = p;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D
        q[kIdxLKnee] = axis_offset_solver_->knee_motor_to_joint(p[kIdxLKnee]);
        q[kIdxRKnee] = axis_offset_solver_->knee_motor_to_joint(p[kIdxRKnee]);

        // 业务 B：ankle 4D 双边
        const Eigen::VectorXd p_ankle4 = PickAnkleMotor4(p);
        const Eigen::VectorXd q_ankle4 = axis_offset_solver_->motor_to_joint_position(p_ankle4);
        WriteAnkleJoint4(q, q_ankle4);
        ClampAnkleMotorToJointDegreesOfFreedom(*axis_offset_solver_, q);
        return q;
    }
    if (ankle_solver_type_token_ == "none") {
        return p;
    }
    throw std::logic_error("AnkleSolver::motor_to_joint_position: ankle_solver_type not mapped");
}

Eigen::VectorXd AnkleSolver::joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p,
                                                    const Eigen::VectorXd& dq) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->joint_to_motor_velocity(q, p, dq);
    }
    if (axis_offset_solver_) {
        if (!(q.size() == 12 && p.size() == 12 && dq.size() == 12)) {
            return axis_offset_solver_->joint_to_motor_velocity(q, p, dq);
        }
        Eigen::VectorXd dp = dq;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D
        dp[kIdxLKnee] = axis_offset_solver_->knee_joint_to_motor_velocity(
            q[kIdxLKnee], p[kIdxLKnee], dq[kIdxLKnee]);
        dp[kIdxRKnee] = axis_offset_solver_->knee_joint_to_motor_velocity(
            q[kIdxRKnee], p[kIdxRKnee], dq[kIdxRKnee]);

        // 业务 B：ankle 4D 双边
        const Eigen::VectorXd q_ankle4  = PickAnkleJoint4(q);
        const Eigen::VectorXd p_ankle4  = PickAnkleMotor4(p);
        const Eigen::VectorXd dq_ankle4 = PickAnkleJoint4(dq);
        const Eigen::VectorXd dp_ankle4 =
            axis_offset_solver_->joint_to_motor_velocity(q_ankle4, p_ankle4, dq_ankle4);
        WriteAnkleMotor4(dp, dp_ankle4);
        return dp;
    }
    if (ankle_solver_type_token_ == "none") {
        return dq;
    }
    throw std::logic_error("AnkleSolver::joint_to_motor_velocity: ankle_solver_type not mapped");
}

Eigen::VectorXd AnkleSolver::motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p,
                                                     const Eigen::VectorXd& dp) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->motor_to_joint_velocity(q, p, dp);
    }
    if (axis_offset_solver_) {
        if (!(q.size() == 12 && p.size() == 12 && dp.size() == 12)) {
            return axis_offset_solver_->motor_to_joint_velocity(q, p, dp);
        }
        Eigen::VectorXd dq = dp;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D
        dq[kIdxLKnee] = axis_offset_solver_->knee_motor_to_joint_velocity(
            q[kIdxLKnee], p[kIdxLKnee], dp[kIdxLKnee]);
        dq[kIdxRKnee] = axis_offset_solver_->knee_motor_to_joint_velocity(
            q[kIdxRKnee], p[kIdxRKnee], dp[kIdxRKnee]);

        // 业务 B：ankle 4D 双边
        const Eigen::VectorXd q_ankle4  = PickAnkleJoint4(q);
        const Eigen::VectorXd p_ankle4  = PickAnkleMotor4(p);
        const Eigen::VectorXd dp_ankle4 = PickAnkleMotor4(dp);
        const Eigen::VectorXd dq_ankle4 =
            axis_offset_solver_->motor_to_joint_velocity(q_ankle4, p_ankle4, dp_ankle4);
        WriteAnkleJoint4(dq, dq_ankle4);
        return dq;
    }
    if (ankle_solver_type_token_ == "none") {
        return dp;
    }
    throw std::logic_error("AnkleSolver::motor_to_joint_velocity: ankle_solver_type not mapped");
}

Eigen::VectorXd AnkleSolver::joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p,
                                                   const Eigen::VectorXd& tau) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->joint_to_motor_current(q, p, tau);
    }
    if (axis_offset_solver_) {
        if (!(q.size() == 12 && p.size() == 12 && tau.size() == 12)) {
            return axis_offset_solver_->joint_to_motor_current(q, p, tau);
        }
        Eigen::VectorXd i = tau;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D
        i[kIdxLKnee] = axis_offset_solver_->knee_joint_to_motor_current(
            q[kIdxLKnee], p[kIdxLKnee], tau[kIdxLKnee]);
        i[kIdxRKnee] = axis_offset_solver_->knee_joint_to_motor_current(
            q[kIdxRKnee], p[kIdxRKnee], tau[kIdxRKnee]);

        // 业务 B：ankle 4D 双边
        const Eigen::VectorXd q_ankle4   = PickAnkleJoint4(q);
        const Eigen::VectorXd p_ankle4   = PickAnkleMotor4(p);
        const Eigen::VectorXd tau_ankle4 = PickAnkleJoint4(tau);
        const Eigen::VectorXd i_ankle4 =
            axis_offset_solver_->joint_to_motor_current(q_ankle4, p_ankle4, tau_ankle4);
        WriteAnkleMotor4(i, i_ankle4);
        return i;
    }
    if (ankle_solver_type_token_ == "none") {
        return tau;
    }
    throw std::logic_error("AnkleSolver::joint_to_motor_current: ankle_solver_type not mapped");
}

Eigen::VectorXd AnkleSolver::motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p,
                                                  const Eigen::VectorXd& i) {
    if (fixed_axis_solver_) {
        return fixed_axis_solver_->motor_to_joint_torque(q, p, i);
    }
    if (axis_offset_solver_) {
        if (!(q.size() == 12 && p.size() == 12 && i.size() == 12)) {
            return axis_offset_solver_->motor_to_joint_torque(q, p, i);
        }
        Eigen::VectorXd tau = i;  // hip & 预留位 1:1 pass-through

        // 业务 A：knee 1D
        tau[kIdxLKnee] = axis_offset_solver_->knee_motor_to_joint_torque(
            q[kIdxLKnee], p[kIdxLKnee], i[kIdxLKnee]);
        tau[kIdxRKnee] = axis_offset_solver_->knee_motor_to_joint_torque(
            q[kIdxRKnee], p[kIdxRKnee], i[kIdxRKnee]);

        // 业务 B：ankle 4D 双边
        const Eigen::VectorXd q_ankle4 = PickAnkleJoint4(q);
        const Eigen::VectorXd p_ankle4 = PickAnkleMotor4(p);
        const Eigen::VectorXd i_ankle4 = PickAnkleMotor4(i);
        const Eigen::VectorXd tau_ankle4 =
            axis_offset_solver_->motor_to_joint_torque(q_ankle4, p_ankle4, i_ankle4);
        WriteAnkleJoint4(tau, tau_ankle4);
        return tau;
    }
    if (ankle_solver_type_token_ == "none") {
        return i;
    }
    throw std::logic_error("AnkleSolver::motor_to_joint_torque: ankle_solver_type not mapped");
}

// ============================================================================
// AnkleSolver leg12 兼容接口
// ----------------------------------------------------------------------------
// *_leg12 与 joint_to_motor_position / motor_to_joint_position(size-12) 语义一致；
// 索引与 AxisOffsetAnkleSolver 原 leg12_* 私有实现等价。
// ============================================================================

Eigen::VectorXd AnkleSolver::joint_to_motor_position_leg12(const Eigen::VectorXd& q12) {
    if (q12.size() != 12) {
        throw std::invalid_argument("joint_to_motor_position_leg12 expects size-12 vector");
    }
    return joint_to_motor_position(q12);
}

Eigen::VectorXd AnkleSolver::motor_to_joint_position_leg12(const Eigen::VectorXd& p12) {
    if (p12.size() != 12) {
        throw std::invalid_argument("motor_to_joint_position_leg12 expects size-12 vector");
    }
    return motor_to_joint_position(p12);
}

}  // namespace kuavo_solver
