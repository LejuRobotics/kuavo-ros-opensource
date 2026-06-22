#ifndef KUAVO_SOLVER_PARALLEL_LINEAR_ARM_SOLVER_H_
#define KUAVO_SOLVER_PARALLEL_LINEAR_ARM_SOLVER_H_

#include <Eigen/Dense>
#include <string>

#include "kuavo_solver/common/solver_tools.h"

namespace kuavo_solver {

struct ParallelLinearArmParams {
  // Uses kuavo_solver::LimitRange and kuavo_solver::Vec3 from solver_tools.h

  struct ElbowSideGeometry {
    double O_f_x = 0.0;
    double O_f_z = 0.0;
    double O_p_x = 0.0;
    double O_p_z = 0.0;
    double a_x = 0.0;
    double a_z = 0.0;
    double t_x = 0.0;
    double t_z = 0.0;
    int sign = 1;
    int theta_sign = -1;
  };

  struct WristSideGeometry {
    Vec3 base_A;
    Vec3 base_B;
    Vec3 roll_center;
    Vec3 roll_axis;
    Vec3 pitch_center_in_roll;
    Vec3 pitch_axis;
    Vec3 anchor_A_pitch;
    Vec3 anchor_B_pitch;
    double len_A_zero = 0.0;
    double len_B_zero = 0.0;
  };

  ElbowSideGeometry elbow_left;
  ElbowSideGeometry elbow_right;
  WristSideGeometry wrist_left;
  WristSideGeometry wrist_right;

  LimitRange joint_elbow_left;
  LimitRange joint_elbow_right;
  LimitRange joint_wrist_roll_left;
  LimitRange joint_wrist_roll_right;
  LimitRange joint_wrist_pitch_left;
  LimitRange joint_wrist_pitch_right;

  LimitRange motor_elbow;
  LimitRange motor_wrist_a;
  LimitRange motor_wrist_b;

  // IK iteration parameters
  double default_tolerance = 1e-8;
  int max_iterations = 20;
};

/**
 * @brief ParallelLinearArmSolver
 *
 * 闭链并联双臂：
 *   - 肘：2D 平面 slider-crank（1 自由度对 1 直线执行器）
 *   - 腕：3D 双线性执行器（2 自由度对 2 直线执行器）
 *
 * 单位约定：
 * - joint: rad
 * - motor(linear): m
 *
 * 14D 接口（双臂拼接 [L7, R7]）：
 * - idx 0-6: 左臂 [shoulder_yaw, shoulder_roll, shoulder_pitch, elbow, elbow_rotation, wrist_roll, wrist_pitch]
 * - idx 7-13: 右臂
 * 其中 elbow=idx3, wrist_roll=idx5, wrist_pitch=idx6
 *
 * 文件按以下统一 5-section 组织：
 *   0) 构造 / 参数加载
 *   1) 连杆向量（Linkage Vectors）
 *   2) 雅可比（Jacobians & JacobianSystem）
 *   3) 正逆运动学（Forward=joint→motor / Inverse=motor→joint）
 *   4) 速度 / 力矩映射（派生自 JacobianSystem）
 *   5) 公共 API（14D 路由）
 */
class ParallelLinearArmSolver {
 public:
  explicit ParallelLinearArmSolver(const ParallelLinearArmParams& params);

  struct LoadedParam {
    ParallelLinearArmParams params;
  };

  static LoadedParam loadParam(const std::string& arm_solver_type_token, const std::string& arm_config_dir);

  // -------------------------------------------------------------------------
  // 单根"连杆"的长度雅可比结构（与踝 / 腰共用设计 pattern）。
  //   joint    = ∂L/∂q           (1×N 行向量；肘 N=1，腕 N=2)
  //   actuator = ∂L/∂p_actuator  (直线执行器恒为 1，即 d_X = L_X − len_X_zero)
  //   length   = 当前连杆长度    (可用于 IK 残差)
  // -------------------------------------------------------------------------
  struct LinkageLengthJacobian {
    Eigen::RowVectorXd joint;  // 尺寸 1 (肘) 或 2 (腕)
    double actuator{1.0};
    double length{0.0};
  };

  // =========================================================================
  // 公共 API：14D 路由
  // =========================================================================
  Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd& q);
  Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd& p);
  Eigen::VectorXd joint_to_motor_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dq);
  Eigen::VectorXd motor_to_joint_velocity(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& dp);
  Eigen::VectorXd joint_to_motor_current(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& tau);
  Eigen::VectorXd motor_to_joint_torque(const Eigen::VectorXd& q, const Eigen::VectorXd& p, const Eigen::VectorXd& i);

  using VerificationResult = SolverTools::VerificationResult;

  /** 使用 Mujoco 数据验证正逆解的一致性 */
  VerificationResult verify_with_mujoco(
      const Eigen::VectorXd& q_mujoco,
      const Eigen::VectorXd& p_mujoco,
      const Eigen::VectorXd& dq_mujoco = Eigen::VectorXd(),
      const Eigen::VectorXd& tau_mujoco = Eigen::VectorXd(),
      double position_tolerance = 1e-6,
      double velocity_tolerance = 1e-6,
      double torque_tolerance = 1e-6);

  /** Vec3 -> Eigen::Vector3d */
  static Eigen::Vector3d ToEigen(const Vec3& v);

  // =========================================================================
  // 公共几何 / JacobianSystem 接口（供严格验证与外部分析）
  // =========================================================================

  /** 肘部 JacobianSystem（1×1 / 1×1）：
   *   Jc * dtheta + Ja * dd = 0
   *   Ja = −1（直线执行器，d = L − len_zero）
   */
  JacobianSystem elbow_jacobian_system(double theta, double d_m, bool is_left) const;

  /** 腕部 JacobianSystem（2×2 / 2×2）：
   *   Jc * d[roll,pitch] + Ja * d[dA,dB] = 0
   *   Ja = −I₂（直线执行器，dA=LA−lenA_zero, dB=LB−lenB_zero）
   */
  JacobianSystem wrist_jacobian_system(double roll, double pitch, bool is_left) const;

 private:
  static constexpr int kDim14 = 14;
  static constexpr int kElbowIdx = 3;
  static constexpr int kWristRollIdx = 5;
  static constexpr int kWristPitchIdx = 6;

  struct ElbowParams {
    double O_f_x = 0.0, O_f_z = 0.0;
    double O_p_x = 0.0, O_p_z = 0.0;
    double a_x = 0.0, a_z = 0.0;
    double t_x = 0.0, t_z = 0.0;
    double L = 0.0, L2 = 0.0;
    int sign = 1;
    int theta_sign = -1;
  };

  struct WristParams {
    Eigen::Vector3d base_A = Eigen::Vector3d::Zero();
    Eigen::Vector3d base_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d roll_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d roll_axis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d pitch_center_in_roll = Eigen::Vector3d::Zero();
    Eigen::Vector3d pitch_axis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d anchor_A_pitch = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchor_B_pitch = Eigen::Vector3d::Zero();
    double len_A_zero = 0.0;
    double len_B_zero = 0.0;
  };

  // =========================================================================
  // 1) 连杆向量（Linkage Vectors）
  // =========================================================================
  //   elbow_linkage_vector_(theta, d, p)  → A_f − P（平面 2D 向量的 x,z 分量）
  //   wrist_linkage_vector_(roll, pitch, side, p) → anchor_cf − base（3D）
  // -------------------------------------------------------------------------
  static void rotate2d_(double x, double z, double cos_t, double sin_t, int sign,
                        double& x_out, double& z_out);
  static Eigen::Matrix3d rotation_axis_angle_(const Eigen::Vector3d& axis, double theta);

  enum class WireSide { A, B };
  static Eigen::Vector3d wrist_linkage_vector_(double roll, double pitch, WireSide side, const WristParams& p);

  // =========================================================================
  // 2) 雅可比（Jacobians & JacobianSystem）
  // =========================================================================
  /** 肘部 2D 闭环约束 f = (A_x−P_x)² + (A_z−P_z)² − L² = 0
   *  joint    = ∂f/∂theta
   *  actuator = ∂f/∂d  （= 2·(A_z−P_z)）
   */
  static LinkageLengthJacobian elbow_linkage_length_jacobian_(double theta, double d, const ElbowParams& p);

  /** 腕部单根拉杆的长度雅可比（A 或 B）：
   *  joint    = [∂L/∂roll, ∂L/∂pitch]
   *  actuator = 1           （直线执行器 d_X = L_X − len_X_zero）
   *  length   = |anchor − base|
   */
  static LinkageLengthJacobian wrist_linkage_length_jacobian_(
      double roll, double pitch, WireSide side, const WristParams& p);

  /** 肘部 JacobianSystem 组装（单侧，已计入 theta_sign） */
  JacobianSystem elbow_jacobian_system_impl_(double theta, double d_m, bool is_left) const;
  /** 腕部 JacobianSystem 组装（单侧，已计入 side_sign） */
  JacobianSystem wrist_jacobian_system_impl_(double roll, double pitch, bool is_left) const;

  // =========================================================================
  // 3) 正逆运动学（FK = joint→motor，IK = motor→joint）
  // =========================================================================
  /** 肘部 FK：关节角 theta → 电机位移 d。闭式（取 L² − dx² 的正平方根）。 */
  static double elbow_forward_kinematics_(double theta_internal, const ElbowParams& p);
  /** 肘部 IK：电机位移 d → 关节角 theta。闭式（余弦定理）。 */
  static double elbow_inverse_kinematics_(double d, const ElbowParams& p);

  /** 腕部 FK：(roll, pitch) → (d_A, d_B)。闭式（|anchor−base|−len_zero）。 */
  static std::pair<double, double> wrist_forward_kinematics_(
      double roll, double pitch, const WristParams& p);
  /** 腕部 IK：(d_A, d_B) → (roll, pitch)。牛顿迭代（残差 = 长度匹配）。 */
  static std::pair<double, double> wrist_inverse_kinematics_(
      double dA, double dB, const WristParams& p, double tol, int max_iter);

  // =========================================================================
  // 4) 速度 / 力矩映射（派生自 JacobianSystem）
  // =========================================================================
  // 肘（1D）：joint_to_motor_velocity_elbow_side_ 等
  double joint_to_motor_velocity_elbow_side_(double theta, double d_m, double dtheta, bool is_left) const;
  double motor_to_joint_velocity_elbow_side_(double theta, double d_m, double dd, bool is_left) const;
  double joint_to_motor_current_elbow_side_(double theta, double d_m, double tau, bool is_left) const;
  double motor_to_joint_torque_elbow_side_(double theta, double d_m, double i_m, bool is_left) const;

  // 腕（2D）：joint_to_motor_velocity_wrist_side_ 等
  Eigen::Vector2d joint_to_motor_velocity_wrist_side_(
      double roll, double pitch, const Eigen::Vector2d& dq_rp, bool is_left) const;
  Eigen::Vector2d motor_to_joint_velocity_wrist_side_(
      double roll, double pitch, const Eigen::Vector2d& dd_ab, bool is_left) const;
  Eigen::Vector2d joint_to_motor_current_wrist_side_(
      double roll, double pitch, const Eigen::Vector2d& tau_rp, bool is_left) const;
  Eigen::Vector2d motor_to_joint_torque_wrist_side_(
      double roll, double pitch, const Eigen::Vector2d& i_ab, bool is_left) const;

  // 公共 API 底层辅助：带 side_sign / theta_sign 的位置映射（单侧）
  double elbow_forward_kinematics_side_(double theta_joint, bool is_left) const;   // joint → motor
  double elbow_inverse_kinematics_side_(double d_m, bool is_left) const;           // motor → joint
  Eigen::Vector2d wrist_forward_kinematics_side_(double roll, double pitch, bool is_left) const;  // joint → motor
  Eigen::Vector2d wrist_inverse_kinematics_side_(double dA, double dB, bool is_left) const;       // motor → joint

  // Clamp helpers
  static double Clamp(double v, double lo, double hi);

  ParallelLinearArmParams params_;
  ElbowParams elbow_left_;
  ElbowParams elbow_right_;
  WristParams wrist_left_;
  WristParams wrist_right_;
  double default_tolerance_;
  int max_iterations_;
};

}  // namespace kuavo_solver

#endif  // KUAVO_SOLVER_PARALLEL_LINEAR_ARM_SOLVER_H_
