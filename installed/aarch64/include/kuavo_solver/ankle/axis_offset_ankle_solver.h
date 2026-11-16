#ifndef AXIS_OFFSET_ANKLE_SOLVER_H_
#define AXIS_OFFSET_ANKLE_SOLVER_H_

#include <Eigen/Dense>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

struct AxisOffsetAnkleParams {
    // 关节偏移参数（pitch和roll关节不同轴）
    double z_pitch;          // foot_roll 原点相对于 knee 的 z 偏移（= z_pitch_raw + z_roll）
    double z_roll;           // roll 关节相对于 pitch 关节的 z 偏移
    double x_pitch;          // pitch 关节相对于 knee 的 x 偏移

    // 左脚踝参数
    double x_lleq, y_lleq, z_lleq;    // ll 等效力点（相对于 roll 关节）
    double x_lreq, y_lreq, z_lreq;    // lr 等效力点（相对于 roll 关节）
    double x_llbar, z_llbar;          // ll bar 位置（相对于 knee）
    double x_lrbar, z_lrbar;          // lr bar 位置（相对于 knee）
    double x_lltd, y_lltd, z_lltd;    // ll 肌腱附着点（相对于 bar）
    double x_lrtd, y_lrtd, z_lrtd;    // lr 肌腱附着点（相对于 bar）
    double l0_ll_eqtd, l0_lr_eqtd;    // 初始肌腱长度

    // 右脚踝参数
    double x_rleq, y_rleq, z_rleq;    // rl 等效力点（相对于 roll 关节）
    double x_rreq, y_rreq, z_rreq;    // rr 等效力点（相对于 roll 关节）
    double x_rlbar, z_rlbar;          // rl bar 位置（相对于 knee）
    double x_rrbar, z_rrbar;          // rr bar 位置（相对于 knee）
    double x_rltd, y_rltd, z_rltd;    // rl 肌腱附着点（相对于 bar）
    double x_rrtd, y_rrtd, z_rrtd;    // rr 肌腱附着点（相对于 bar）
    double l0_rl_eqtd, l0_rr_eqtd;    // 初始肌腱长度

    // 求解器参数
    double default_tolerance;
    int max_iterations;

    /** 与 AnkleSolver::ANKLE_SOLVER_TYPE_5GEN 中 config[37..42] 一致的膝盖摆杆-腱几何。
     * 为 false 时膝角在关节空间与电机角 1:1 直通（S2GEN_2 等默认）。 */
    bool knee_linkage_enabled{false};
    // 闭环 4-bar 机架：l_knee 轴在 l_knee_bar 轴坐标系下的 xz 偏置。
    // （老数据只保留了 z_bar_knee，忽略 x 偏置；新版支持 x_bar_knee 以对齐真实 URDF/MJCF。
    //  未在 YAML 中给出 x_bar_knee 时默认 0，等价于老行为。）
    double x_bar_knee{0};
    double z_bar_knee{0};
    double l_tendon{0};
    double l_bar_td{0};
    double l_knee_eq{0};
    double q_offset_knee{0};
    double q_offset_bar{0};

    /** 12 维腿 motor_to_joint_position 对踝 pitch/roll 的关节限位（与 YAML ankle_*_limits 一致） */
    double ankle_pitch_lo{-1.57};
    double ankle_pitch_hi{1.57};
    double ankle_roll_lo{-1.57};
    double ankle_roll_hi{1.57};

    // 默认构造函数，初始化新参数为0
    AxisOffsetAnkleParams()
        : z_roll(0.0), x_pitch(0.0),
          x_llbar(0.0), x_lrbar(0.0),
          x_rlbar(0.0), x_rrbar(0.0) {}
};

enum class AnkleSide {
    LEFT,
    RIGHT
};

enum class TendonSide {
    LEFT,
    RIGHT
};

struct TendonParams {
    double x_eq, y_eq, z_eq;         // 等效力点位置（相对于 roll 关节）
    double x_td, y_td, z_td;         // 肌腱附着点位置（相对于 bar 关节）
    double x_bar, z_bar;             // bar 的位置（相对于 knee）
    double l_bar;                    // 辅助长度
    double l0_eqtd;                  // 初始肌腱长度
};

class AxisOffsetAnkleSolver {
public:
    explicit AxisOffsetAnkleSolver(const AxisOffsetAnkleParams& params);

    struct AxisOffsetLoadedParam {
        AxisOffsetAnkleParams params;
        double l_llbar{0};
        double l_lrbar{0};
        double l_rlbar{0};
        double l_rrbar{0};
    };

    /** 从 axisoffsetanklesolver.yaml 读取并解析指定 token 的 variant（禁用 fallback；失败抛异常）。 */
    static AxisOffsetLoadedParam loadOffsetParam(const std::string& ankle_solver_type_token,
                                                 const std::string& ankle_config_dir);

    // -------------------------------------------------------------------------
    // 膝关节 1D 子解算器（slider-crank）
    // -------------------------------------------------------------------------
    double knee_joint_to_motor(double q_knee) const;
    double knee_motor_to_joint(double p_knee) const;
    double knee_joint_to_motor_velocity(double q_knee, double p_knee, double dq_knee) const;
    double knee_motor_to_joint_velocity(double q_knee, double p_knee, double dp_knee) const;
    double knee_joint_to_motor_current(double q_knee, double p_knee, double tau_joint) const;
    double knee_motor_to_joint_torque(double q_knee, double p_knee, double i_motor) const;

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

    using VerificationResult = SolverTools::VerificationResult;

    VerificationResult verify_with_mujoco(
        const Eigen::VectorXd& q_mujoco,
        const Eigen::VectorXd& p_mujoco,
        const Eigen::VectorXd& dq_mujoco = Eigen::VectorXd(),
        const Eigen::VectorXd& tau_mujoco = Eigen::VectorXd(),
        double position_tolerance = 1e-6,
        double velocity_tolerance = 1e-6,
        double torque_tolerance = 1e-6);

    Eigen::Vector3d compute_tendon_vector(
        double pitch, double roll, double actuator_angle,
        AnkleSide ankle_side, TendonSide tendon_side) const;
    Eigen::Matrix<double, 3, 2> compute_jacobian_ankle(
        double pitch, double roll, AnkleSide ankle_side, TendonSide tendon_side) const;
    Eigen::Vector3d compute_jacobian_actuator(
        double pitch, double roll, double actuator_angle,
        AnkleSide ankle_side, TendonSide tendon_side) const;

    /** 统一 JacobianSystem：J_constraint*dq + J_actuator*dp = 0（单侧 pitch/roll vs lbar/rbar） */
    JacobianSystem jacobian_system(
        double pitch, double roll, double lbar, double rbar, AnkleSide ankle_side) const;

    // 关节限位查询（供 AnkleSolver 工厂层 clamp 使用）
    double ankle_pitch_lo() const { return ankle_pitch_lo_; }
    double ankle_pitch_hi() const { return ankle_pitch_hi_; }
    double ankle_roll_lo() const { return ankle_roll_lo_; }
    double ankle_roll_hi() const { return ankle_roll_hi_; }

private:
    std::pair<double, double> inverse_kinematics_(
        double lbar, double rbar, AnkleSide side, double tol = -1.0, int max_iter = -1);
    std::pair<double, double> forward_kinematics_(
        double pitch, double roll, AnkleSide side, double tol = -1.0, int max_iter = -1);
    std::pair<double, double> forward_kinematics_jacobian_(
        double pitch, double roll, AnkleSide side, double tol, int max_iter);
    std::pair<double, double> inverse_kinematics_jacobian_(
        double lbar, double rbar, AnkleSide side, double tol, int max_iter);
    TendonParams get_tendon_params(AnkleSide ankle_side, TendonSide tendon_side) const;

    JacobianSystem compute_ankle_jacobian_system_(
        double pitch, double roll, double lbar, double rbar, AnkleSide ankle_side) const;
    Eigen::Vector2d joint_to_motor_velocity_single_(
        double pitch, double roll, double lbar, double rbar,
        double dpitch, double droll, AnkleSide ankle_side) const;
    Eigen::Vector2d motor_to_joint_velocity_single_(
        double pitch, double roll, double lbar, double rbar,
        double dlbar, double drbar, AnkleSide ankle_side) const;
    Eigen::Vector2d joint_to_motor_current_single_(
        double pitch, double roll, double lbar, double rbar,
        double tau_pitch, double tau_roll, AnkleSide ankle_side) const;
    Eigen::Vector2d motor_to_joint_torque_single_(
        double pitch, double roll, double lbar, double rbar,
        double i_lbar, double i_rbar, AnkleSide ankle_side) const;

    // 关节偏移参数
    double z_pitch_;         // foot_roll 原点相对于 knee 的 z 偏移
    double z_roll_;          // roll 关节相对于 pitch 关节的 z 偏移
    double x_pitch_;         // pitch 关节相对于 knee 的 x 偏移

    // 左脚踝参数
    double x_lleq_, y_lleq_, z_lleq_;
    double x_lreq_, y_lreq_, z_lreq_;
    double x_llbar_, z_llbar_;
    double x_lrbar_, z_lrbar_;
    double x_lltd_, y_lltd_, z_lltd_;
    double x_lrtd_, y_lrtd_, z_lrtd_;
    double l_llbar_, l_lrbar_;
    double l0_ll_eqtd_, l0_lr_eqtd_;

    // 右脚踝参数
    double x_rleq_, y_rleq_, z_rleq_;
    double x_rreq_, y_rreq_, z_rreq_;
    double x_rlbar_, z_rlbar_;
    double x_rrbar_, z_rrbar_;
    double x_rltd_, y_rltd_, z_rltd_;
    double x_rrtd_, y_rrtd_, z_rrtd_;
    double l_rlbar_, l_rrbar_;
    double l0_rl_eqtd_, l0_rr_eqtd_;

    double default_tolerance_;
    int max_iterations_;

    bool knee_linkage_enabled_{false};
    double x_bar_knee_{0};
    double z_bar_knee_{0};
    double l_tendon_{0};
    double l_bar_td_{0};
    double l_knee_eq_{0};
    double q_offset_knee_{0};
    double q_offset_bar_{0};

    struct KneeJacobians {
        double Jknee_bar{1.0};
        double Jbar_knee{1.0};
    };
    KneeJacobians knee_jacobians_(double q_knee, double p_knee) const;

    double ankle_pitch_lo_;
    double ankle_pitch_hi_;
    double ankle_roll_lo_;
    double ankle_roll_hi_;
};

} // namespace kuavo_solver

#endif  // AXIS_OFFSET_ANKLE_SOLVER_H_
