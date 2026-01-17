#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_capture_ik
{

/**
 * Header-only analytic / semi-analytic arm IK ported from a Python prototype.
 *
 * Key ideas (mirrors the script):
 * - Closed-form geometry for shoulder/elbow (theta1..theta4)
 * - Wrist orientation solved with a small iterative refinement (replacing "mink" in Python)
 * - Outer loop that compensates residual position error by nudging the input target position
 *
 * NOTE:
 * - This solver assumes the same kinematic convention as the script: joint axes order Y-X-Z-Y-Z-X-Y
 * - The input pose is expressed in a world-aligned frame whose origin is at the arm base (e.g. zarm_l1_link origin).
 *   In practice: p_rel = p_target_world - p_arm_base_world, and R = R_target_world.
 */
class AnalyticArmIk
{
public:
  struct Options
  {
    bool verbose{false};
    int outer_pos_iter{10};         // matches ik_leftarm_loop N_ITER
    int wrist_refine_iter{1};       // matches N_MINK_ITER
    double wrist_orient_gain{1.0};  // step gain inside wrist refinement
    double wrist_posture_weight{1e-2};  // posture regularization towards zero
    double outer_break_eps{1e-6};
  };

  AnalyticArmIk() = default;

  /**
   * Solve left-arm IK for the end-effector pose X_07.
   *
   * @param R_07 rotation (world-aligned)
   * @param p_07 translation relative to arm base origin (world-aligned)
   * @param opts solver options
   * @param q_out 7x1 joint vector [theta1..theta7] in radians
   * @return true if a finite solution is produced
   */
  static bool SolveLeftArm(const Eigen::Matrix3d& R_07,
                           const Eigen::Vector3d& p_07,
                           const Options& opts,
                           Eigen::Matrix<double, 7, 1>& q_out)
  {
    // Outer loop: compensate residual position error by shifting the input target
    Eigen::Vector3d err = Eigen::Vector3d::Zero();
    double err_norm = std::numeric_limits<double>::infinity();

    Eigen::Matrix3d R_tmp = R_07;
    Eigen::Vector3d p_tmp = p_07;

    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();

    for (int i = 0; i < std::max(1, opts.outer_pos_iter); ++i)
    {
      p_tmp = p_07 - err;
      if (!SolveLeftArmOnce(R_tmp, p_tmp, opts, q))
        return false;

      const Eigen::Vector3d p_fk = FKLeftArm(q).translation();
      err = p_fk - p_07;

      const double prev = err_norm;
      err_norm = err.norm();
      if (opts.verbose)
      {
        // keep same spirit as python prints
        // (no i/o here to avoid pulling iostream into header; user can log externally)
      }
      if (std::abs(prev - err_norm) < opts.outer_break_eps)
        break;
    }

    if (!q.allFinite())
      return false;
    q_out = q;
    return true;
  }

  /**
   * Solve right-arm IK for the end-effector pose.
   *
   * The original python prototype only implemented left-arm IK. For the right arm we
   * reuse the same solver via a mirror transform across the sagittal plane (flip Y),
   * then map joint signs for X/Z-axis joints.
   *
   * This matches the joint-axis convention used by the analytic solver: Y-X-Z-Y-Z-X-Y.
   *
   * @param R_07 rotation (world-aligned)
   * @param p_07 translation relative to right arm base origin (world-aligned)
   */
  static bool SolveRightArm(const Eigen::Matrix3d& R_07,
                            const Eigen::Vector3d& p_07,
                            const Options& opts,
                            Eigen::Matrix<double, 7, 1>& q_out)
  {
    // Mirror across Y: S = diag(1, -1, 1). Note: det(S)=-1 but S*R*S keeps det=+1.
    const Eigen::Matrix3d S = (Eigen::Vector3d(1.0, -1.0, 1.0)).asDiagonal();
    const Eigen::Matrix3d R_m = S * R_07 * S;
    const Eigen::Vector3d p_m = S * p_07;

    Eigen::Matrix<double, 7, 1> q_m;
    if (!SolveLeftArm(R_m, p_m, opts, q_m))
      return false;

    // Map mirrored-solution joint signs:
    // For reflection across Y, rotations about X and Z flip sign, rotations about Y keep sign.
    // Joint order is Y-X-Z-Y-Z-X-Y => indices (0,3,6) keep; (1,2,4,5) flip.
    q_out = q_m;
    q_out[1] = -q_out[1];
    q_out[2] = -q_out[2];
    q_out[4] = -q_out[4];
    q_out[5] = -q_out[5];

    return q_out.allFinite();
  }

  /** Forward kinematics used by the python script (world-aligned, origin at arm base). */
  static Eigen::Isometry3d FKLeftArm(const Eigen::Matrix<double, 7, 1>& q)
  {
    const double t1 = q[0], t2 = q[1], t3 = q[2], t4 = q[3], t5 = q[4], t6 = q[5], t7 = q[6];
    const Eigen::Isometry3d X_01 = TransformFromY(t1);
    const Eigen::Isometry3d X_12 = TransformFromX(t2);
    const Eigen::Isometry3d X_23 = TransformFromZ(t3);
    const Eigen::Isometry3d X_34 = TransformFromY(t4, Eigen::Vector3d(0.02, 0.0, -0.2837));
    const Eigen::Isometry3d X_45 = TransformFromZ(t5, Eigen::Vector3d(-0.02, 0.0, -(0.126 + 0.1075)));
    const Eigen::Isometry3d X_56 = TransformFromX(t6);
    const Eigen::Isometry3d X_67 = TransformFromY(t7, Eigen::Vector3d(0.0, 0.0, -0.021));
    return X_01 * X_12 * X_23 * X_34 * X_45 * X_56 * X_67;
  }

private:
  // ======= Ported constants from the script =======
  static constexpr double kA = 0.02;
  static constexpr double kL34 = std::sqrt(kA * kA + 0.2837 * 0.2837);
  static constexpr double kL45 = std::sqrt(kA * kA + (0.126 + 0.1075) * (0.126 + 0.1075));
  static constexpr double kL47 = std::sqrt(kA * kA + (0.126 + 0.1075 + 0.021) * (0.126 + 0.1075 + 0.021));
  static constexpr double kOffsetTheta4 =
      std::atan2(kA, 0.2837) + std::atan2(kA, (0.126 + 0.1075));

  static double SafeAcos(double x) { return std::acos(std::clamp(x, -1.0, 1.0)); }
  static double SafeAsin(double x) { return std::asin(std::clamp(x, -1.0, 1.0)); }
  static double SafeAtan2(double y, double x)
  {
    const double n2 = x * x + y * y;
    return (n2 > 1e-12) ? std::atan2(y, x) : 0.0;
  }

  static Eigen::Matrix3d Rx(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, c, -s,
         0, s, c;
    return R;
  }
  static Eigen::Matrix3d Ry(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, 0, s,
         0, 1, 0,
        -s, 0, c;
    return R;
  }
  static Eigen::Matrix3d Rz(double a)
  {
    const double c = std::cos(a), s = std::sin(a);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
  }

  static Eigen::Isometry3d Transform(const Eigen::Matrix3d& R, const Eigen::Vector3d& p)
  {
    Eigen::Isometry3d X = Eigen::Isometry3d::Identity();
    X.linear() = R;
    X.translation() = p;
    return X;
  }
  static Eigen::Isometry3d TransformFromX(double a, const Eigen::Vector3d& p = Eigen::Vector3d::Zero())
  {
    return Transform(Rx(a), p);
  }
  static Eigen::Isometry3d TransformFromY(double a, const Eigen::Vector3d& p = Eigen::Vector3d::Zero())
  {
    return Transform(Ry(a), p);
  }
  static Eigen::Isometry3d TransformFromZ(double a, const Eigen::Vector3d& p = Eigen::Vector3d::Zero())
  {
    return Transform(Rz(a), p);
  }

  // A lightweight replacement for the python "mink" wrist refinement:
  // do a few LM/gradient steps on wrist angles to match desired R_47.
  static Eigen::Matrix3d WristRotZXY(double t5, double t6, double t7)
  {
    // SciPy uses intrinsic rotations for uppercase "ZXY". We mirror the same composition.
    // R = Rz(t5) * Rx(t6) * Ry(t7)
    return Rz(t5) * Rx(t6) * Ry(t7);
  }

  // Closed-form inverse for the WristRotZXY() convention:
  // R = Rz(a) * Rx(b) * Ry(c)
  // Returns (a,b,c). This matches scipy Rotation.as_euler('ZXY') for the convention used in the script.
  static Eigen::Vector3d EulerZXYFromMatrix(const Eigen::Matrix3d& R)
  {
    // From symbolic derivation:
    // sb = R(2,1)
    // a = atan2(-R(0,1), R(1,1))  when cb != 0
    // c = atan2(-R(2,0), R(2,2))  when cb != 0
    const double sb = std::clamp(R(2, 1), -1.0, 1.0);
    const double b = std::asin(sb);
    const double cb = std::cos(b);

    double a = 0.0;
    double c = 0.0;

    if (std::abs(cb) > 1e-9)
    {
      a = std::atan2(-R(0, 1), R(1, 1));
      c = std::atan2(-R(2, 0), R(2, 2));
    }
    else
    {
      // Gimbal lock (b ~ +/- pi/2). Choose a = 0 and solve c from remaining terms.
      // With cb ~ 0, R(0,1) and R(1,1) ~ 0; use R(0,0), R(1,0).
      a = 0.0;
      c = std::atan2(R(1, 0), R(0, 0));
    }

    return Eigen::Vector3d(a, b, c);
  }

  static Eigen::Vector3d SO3Log(const Eigen::Matrix3d& R)
  {
    // Robust log map using AngleAxis
    Eigen::AngleAxisd aa(R);
    double angle = aa.angle();
    Eigen::Vector3d axis = aa.axis();
    if (!std::isfinite(angle) || !axis.allFinite())
      return Eigen::Vector3d::Zero();
    // Map to [-pi, pi]
    if (angle > M_PI)
      angle -= 2.0 * M_PI;
    return axis * angle;
  }

  static void RefineWrist(const Eigen::Matrix3d& R_47_des,
                          const Options& opts,
                          double& t5, double& t6, double& t7)
  {
    // Posture target is zeros (as in python config.update([0,0,0]))
    const Eigen::Vector3d q_ref = Eigen::Vector3d::Zero();

    Eigen::Vector3d q(t5, t6, t7);
    for (int it = 0; it < std::max(0, opts.wrist_refine_iter); ++it)
    {
      const Eigen::Matrix3d R_cur = WristRotZXY(q[0], q[1], q[2]);
      const Eigen::Matrix3d R_err = R_cur.transpose() * R_47_des;
      Eigen::Vector3d e = SO3Log(R_err);  // in radians

      // Numeric Jacobian: 3x3
      Eigen::Matrix3d J;
      constexpr double eps = 1e-6;
      for (int k = 0; k < 3; ++k)
      {
        Eigen::Vector3d dq = Eigen::Vector3d::Zero();
        dq[k] = eps;
        const Eigen::Matrix3d R_p = WristRotZXY(q[0] + dq[0], q[1] + dq[1], q[2] + dq[2]);
        const Eigen::Matrix3d R_err_p = R_p.transpose() * R_47_des;
        const Eigen::Vector3d e_p = SO3Log(R_err_p);
        J.col(k) = (e_p - e) / eps;
      }

      // Solve damped least squares with posture regularization:
      //   min ||J dq - e||^2 + w ||(q+dq)-q_ref||^2
      const double w = std::max(0.0, opts.wrist_posture_weight);
      Eigen::Matrix3d H = J.transpose() * J + w * Eigen::Matrix3d::Identity();
      Eigen::Vector3d g = J.transpose() * e + w * (q_ref - q);
      Eigen::Vector3d dq = H.ldlt().solve(g);

      q += opts.wrist_orient_gain * dq;
    }

    t5 = q[0];
    t6 = q[1];
    t7 = q[2];
  }

  static bool SolveLeftArmOnce(const Eigen::Matrix3d& R_07,
                               const Eigen::Vector3d& p_07,
                               const Options& opts,
                               Eigen::Matrix<double, 7, 1>& q_out)
  {
    // Port of ik_leftarm_w_err(X_07) from the python script.
    const double l = p_07.norm();

    // x_07,y_07,z_07 = Ry(0*pi/4) * Rz(pi/4) * p_07  (Ry(0) is identity)
    const Eigen::Vector3d p_rot = Rz(M_PI / 4.0) * p_07;
    const double x_07 = p_rot.x();
    const double y_07 = p_rot.y();
    const double z_07 = p_rot.z();

    // theta_4_virtual
    const double cos_v = (kL34 * kL34 + kL47 * kL47 - l * l) / (2.0 * kL34 * kL47);
    const double theta_4_virtual = M_PI - SafeAcos(cos_v);

    const double x_37 = kL47 * std::sin(theta_4_virtual);
    const double z_37 = -(kL34 + kL47 * std::cos(theta_4_virtual));

    const double theta_3_virtual =
        SafeAtan2(x_07, std::sqrt(y_07 * y_07 + z_07 * z_07)) - SafeAtan2(x_37, -z_37);

    const double theta = theta_3_virtual + theta_4_virtual + std::atan2(kA, (0.126 + 0.1075));
    const Eigen::Matrix3d R_24 = TransformFromY(-theta).linear();

    const double theta_2_virtual = SafeAtan2(y_07, -z_07);
    Eigen::Matrix3d R_04 = TransformFromX(theta_2_virtual).linear() * R_24;
    R_04 = TransformFromZ(-M_PI / 4.0).linear() * TransformFromY(-0.0).linear() * R_04;

    const Eigen::Matrix3d R_47 = R_04.inverse() * R_07;

    // Initial wrist angles: match the python script exactly (as_euler('ZXY'))
    // Our WristRotZXY uses R = Rz(t5) * Rx(t6) * Ry(t7), so we invert that here.
    const Eigen::Vector3d zxy = EulerZXYFromMatrix(R_47);
    double theta_5 = zxy[0];
    double theta_6 = zxy[1];
    double theta_7 = zxy[2];

    // Wrist refinement (replaces mink)
    RefineWrist(R_47, opts, theta_5, theta_6, theta_7);

    const double theta_4 = -(theta_4_virtual + kOffsetTheta4);
    const Eigen::Matrix3d R_34 = TransformFromY(theta_4).linear();
    const Eigen::Matrix3d R_03 = R_04 * R_34.inverse();

    double theta_1 = SafeAtan2(R_03(0, 2), R_03(2, 2));
    if (theta_1 > 0.75 * M_PI)
      theta_1 -= 2.0 * M_PI;

    const double theta_2 = SafeAsin(-R_03(1, 2));
    const Eigen::Matrix3d R_01 = TransformFromY(theta_1).linear();
    const Eigen::Matrix3d R_12 = TransformFromX(theta_2).linear();
    const Eigen::Matrix3d R_23 = R_12.inverse() * R_01.inverse() * R_03;
    const double theta_3 = SafeAtan2(R_23(1, 0), R_23(0, 0));

    q_out << theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7;
    return q_out.allFinite();
  }
};

}  // namespace motion_capture_ik


