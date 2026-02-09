#ifndef ROBAN_ANKLE_SOLVER_H_
#define ROBAN_ANKLE_SOLVER_H_

#include <Eigen/Dense>
#include <string>
#include <utility>
#include <map>

namespace kuavo_solver {


struct RobanAnkleParams {
    double z_pitch;
    // 左脚踝参数
    double x_lleq, y_lleq, z_lleq;
    double x_lreq, y_lreq, z_lreq;
    double z_llbar, z_lrbar;
    double x_lltd, y_lltd, z_lltd;
    double x_lrtd, y_lrtd, z_lrtd;
    double l0_ll_eqtd, l0_lr_eqtd;
    // 右脚踝参数
    double x_rleq, y_rleq, z_rleq;
    double x_rreq, y_rreq, z_rreq;
    double z_rlbar, z_rrbar;
    double x_rltd, y_rltd, z_rltd;
    double x_rrtd, y_rrtd, z_rrtd;
    double l0_rl_eqtd, l0_rr_eqtd;
    // 求解器参数
    double default_tolerance;
    int max_iterations;
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
    double x_eq, y_eq, z_eq;         // 等效力点位置
    double x_td, y_td, z_td;         // 肌腱附着点位置
    double z_bar;                    // 杆的 z 位置
    double l_bar;                    // 辅助长度
    double l0_eqtd;                  // 初始肌腱长度
};

class Roban_ankle_solver {
public:
    explicit Roban_ankle_solver(const RobanAnkleParams& params);

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

    struct VerificationResult {
        double position_error_norm;
        double velocity_error_norm;
        double torque_error_norm;
        bool is_valid;
        
        VerificationResult() : position_error_norm(0.0), velocity_error_norm(0.0), 
                               torque_error_norm(0.0), is_valid(false) {}
    };

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

    struct AnkleJacobianSystem {
        Eigen::Matrix2d J_constraint;   // 约束矩阵 (2x2)
        Eigen::Matrix2d J_actuator;      // 执行器矩阵 (2x2): 对角矩阵
    };
    
    AnkleJacobianSystem compute_ankle_jacobian_system_(
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

    double z_pitch_;
    double x_lleq_, y_lleq_, z_lleq_;
    double x_lreq_, y_lreq_, z_lreq_;
    double z_llbar_, z_lrbar_;
    double x_lltd_, y_lltd_, z_lltd_;
    double x_lrtd_, y_lrtd_, z_lrtd_;
    double l_llbar_, l_lrbar_;
    double l0_ll_eqtd_, l0_lr_eqtd_;
    double x_rleq_, y_rleq_, z_rleq_;
    double x_rreq_, y_rreq_, z_rreq_;
    double z_rlbar_, z_rrbar_;
    double x_rltd_, y_rltd_, z_rltd_;
    double x_rrtd_, y_rrtd_, z_rrtd_;
    double l_rlbar_, l_rrbar_;
    double l0_rl_eqtd_, l0_rr_eqtd_;
    double default_tolerance_;
    int max_iterations_;
};

} // namespace kuavo_solver

#endif // ROBAN_ANKLE_SOLVER_H_
