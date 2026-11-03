#ifndef KUAVO_SOLVER_PARALLEL_ROTATE_WAIST_SOLVER_H_
#define KUAVO_SOLVER_PARALLEL_ROTATE_WAIST_SOLVER_H_

#include <Eigen/Dense>
#include <string>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

struct ParallelRotateWaistParams {
  // Uses kuavo_solver::Vec3 and kuavo_solver::LimitRange from solver_tools.h

  // Geometry in waist_yaw frame (zero pose)
  Vec3 bar_pivot_left;        // waist_l_bar_joint origin in waist_yaw
  Vec3 bar_pivot_right;       // waist_r_bar_joint origin in waist_yaw
  Vec3 tendon_joint_in_bar;   // waist_*_bar_tendon_joint origin in bar frame

  // Serial waist transforms (for pitch/roll)
  Vec3 pitch_origin_in_yaw;   // waist_pitch_joint origin in waist_yaw
  Vec3 roll_origin_in_pitch;  // waist_roll_joint origin in waist_pitch

  // Ball centers in waist_roll local frame (given)
  Vec3 ball_center_left_in_roll;
  Vec3 ball_center_right_in_roll;

  // Rod length (fixed, meters)
  double rod_length_m = 0.0;

  // IK iteration parameters
  double default_tolerance = 1e-10;
  int max_iterations = 30;

  // Limits (optional)
  LimitRange joint_pitch;
  LimitRange joint_roll;
  LimitRange motor_l_bar;
  LimitRange motor_r_bar;
};

/**
 * @brief ParallelRotateWaistSolver
 *
 * q2 = [pitch, roll]，p2 = [left_bar, right_bar]，单位 rad。
 */
class ParallelRotateWaistSolver {
 public:
  explicit ParallelRotateWaistSolver(const ParallelRotateWaistParams& params);

  struct LoadedParam {
    ParallelRotateWaistParams params;
  };

  static LoadedParam loadParam(const std::string& waist_solver_type_token, const std::string& waist_config_dir);

  // -------------------------------------------------------------------------
  // 单根连杆长度雅可比：joint、actuator、length。
  //   joint    = ∂L/∂(pitch, roll)   (1x2 行向量)
  //   actuator = ∂L/∂bar             (标量)
  //   length   = |A − B|             (当前杆长，IK 残差可复用)
  // -------------------------------------------------------------------------
  struct LinkageLengthJacobian {
    Eigen::RowVector2d joint{Eigen::RowVector2d::Zero()};
    double actuator{0.0};
    double length{0.0};
  };

  // 2D 接口
  Eigen::Vector2d joint_to_motor_position(const Eigen::Vector2d& q2) const;
  Eigen::Vector2d motor_to_joint_position(const Eigen::Vector2d& p2) const;
  /** 由 bars 求 pitch/roll；Gauss–Newton 初值为 q_joint_pitch_roll_hint2；残差过大时用 motor_to_joint_global_search_。 */
  Eigen::Vector2d motor_to_joint_position(const Eigen::Vector2d& bars2,
                                         const Eigen::Vector2d& q_joint_pitch_roll_hint2) const;
  Eigen::Vector2d joint_to_motor_velocity(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dq2) const;
  Eigen::Vector2d motor_to_joint_velocity(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& dp2) const;
  Eigen::Vector2d joint_to_motor_current(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& tau2) const;
  Eigen::Vector2d motor_to_joint_torque(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2, const Eigen::Vector2d& i2) const;

  /** 统一 JacobianSystem：J_constraint*dq + J_actuator*dp = 0。
   *  - J_constraint: 2×2，行 = ∂L_side/∂(pitch, roll)（left/right）
   *  - J_actuator:  2×2，对角 = ∂L_side/∂bar_side（left/right）
   */
  JacobianSystem jacobian_system(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2) const;

 private:
  static Eigen::Vector3d ToEigen(const Vec3& v);

  // =========================================================================
  // 1) 连杆向量（Linkage Vectors）—— 纯几何点位 / 向量
  // =========================================================================
  Eigen::Vector3d bar_end_in_yaw_(double bar_angle, bool is_left) const;          // A = bar 末端
  Eigen::Vector3d ball_center_in_yaw_(double pitch, double roll, bool is_left) const;  // B = 球心
  /** 刚性杆向量 e = A − B（waist_yaw 坐标系）。 */
  Eigen::Vector3d linkage_vector_(double pitch, double roll, double bar_angle, bool is_left) const;

  // =========================================================================
  // 2) 雅可比（Jacobians & JacobianSystem）
  // =========================================================================
  /** 单侧刚性杆的长度雅可比（length form）：
   *  约束 f_side = |A − B| − L_rod = 0。
   *    joint    = [∂L/∂pitch, ∂L/∂roll]
   *    actuator = ∂L/∂bar_side
   *    length   = |A − B|
   */
  LinkageLengthJacobian linkage_length_jacobian_(
      double pitch, double roll, double bar_angle, bool is_left) const;
  JacobianSystem jacobian_system_impl_(const Eigen::Vector2d& q2, const Eigen::Vector2d& p2) const;

  // 3) 正逆运动学：FK = joint→motor；IK = motor→joint。
  //    FK：余弦闭式；IK：Gauss–Newton（无 LM、无 line search）。
  /** 单侧：已知 pitch、roll，求该侧 bar_angle。 */
  double forward_kinematics_impl_side_(double pitch, double roll, bool is_left) const;

  /** IK：给定 bars与初值 init_q 做 Gauss–Newton。 */
  Eigen::Vector2d inverse_kinematics_impl_(const Eigen::Vector2d& bars,
                                           const Eigen::Vector2d& init_q) const;
  /** 两侧杆长约束残差的 L2 范数。 */
  double linkage_residual_l2_(const Eigen::Vector2d& q2, const Eigen::Vector2d& bars2) const;
  /** 多初值尝试求 motor→joint；仍不满足阈值则抛出。 */
  Eigen::Vector2d motor_to_joint_global_search_(const Eigen::Vector2d& p2) const;

  ParallelRotateWaistParams params_;
  Eigen::Vector3d bar_pivot_left_;
  Eigen::Vector3d bar_pivot_right_;
  Eigen::Vector3d tendon_joint_in_bar_;
  Eigen::Vector3d pitch_origin_in_yaw_;
  Eigen::Vector3d roll_origin_in_pitch_;
  Eigen::Vector3d ball_left_in_roll_;
  Eigen::Vector3d ball_right_in_roll_;
  double L_m_ = 0.0;
  double default_tolerance_;
  int max_iterations_;
};

}  // namespace kuavo_solver

#endif  // KUAVO_SOLVER_PARALLEL_ROTATE_WAIST_SOLVER_H_
