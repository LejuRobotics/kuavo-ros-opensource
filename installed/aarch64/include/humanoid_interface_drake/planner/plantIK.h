#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Geometry>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/com_position_constraint.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"
#include "drake/common/autodiff.h"
// #include "utils.h"

namespace drake
{
  class CoMIK
  {
  public:
    CoMIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-8);

    bool solve(const std::vector<std::vector<Eigen::Vector3d>> &pose, const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol);

    bool solve(const std::vector<std::vector<Eigen::Vector3d>> &pose, const math::RotationMatrixd &l_arm_rotation, 
                    const math::RotationMatrixd &r_arm_rotation, const Eigen::Vector3d &l_arm_xyz, const Eigen::Vector3d &r_arm_xyz,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol, bool arm_cost);

    bool solveArmConstinues(const std::vector<std::vector<Eigen::Vector3d>> &pose, const math::RotationMatrixd &l_arm_rotation, 
                    const math::RotationMatrixd &r_arm_rotation, const Eigen::Vector3d &l_arm_xyz, const Eigen::Vector3d &r_arm_xyz,
                    const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol, bool arm_cost);
  private:
    solvers::Binding<solvers::Constraint> AddCoMPositionConstraint(multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
    Eigen::VectorXd prev_q_sol;
  };

  class CMIK
  {
  public:
    CMIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-8);

    bool solve(const std::vector<std::vector<Eigen::Vector3d>> &pose, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, double dt,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &v_sol);

  private:
    solvers::Binding<solvers::Constraint> AddCoMPositionConstraint(multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);
    solvers::Binding<solvers::Constraint> AddCMConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                          const Eigen::Vector3d &k_WC_des, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0, double dt);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
  };

  class CoMVIK
  {
  public:
    CoMVIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-7);

    bool solve(std::vector<std::vector<Eigen::Vector3d>> &pose, Eigen::VectorXd &q0, Eigen::VectorXd &v0, double dt,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &v_sol);

  private:
    solvers::Binding<solvers::Constraint> AddCMConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                          Eigen::VectorXd &k_WC_des, Eigen::VectorXd &q0, Eigen::VectorXd &v0, double dt);
    solvers::Binding<solvers::Constraint> AddVelocityConstraint(multibody::InverseKinematics &ik, solvers::VectorXDecisionVariable &v,
                                                                Eigen::VectorXd &q0, Eigen::VectorXd &v0, std::string frame_name,
                                                                const Eigen::Matrix<double, 6, 1> &sv_lower,
                                                                const Eigen::Matrix<double, 6, 1> &sv_upper);

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double tol_;
    double solver_tol_;
  };

  class AnalyticalIK
  {
  public:
    AnalyticalIK(multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name, double tol = 1.0e-8, double solver_tol = 1.0e-7);

    void AnalyticalFK(Eigen::VectorXd &pose_err, Eigen::VectorXd &qv);
    bool solve(const std::vector<Eigen::VectorXd> &pose, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &qd_sol, Eigen::VectorXd &qdd_sol);

    bool solve(const std::vector<Eigen::VectorXd> &pose, const Eigen::VectorXd &q0, const Eigen::VectorXd &v0,
               Eigen::VectorXd &q_sol, Eigen::VectorXd &qd_sol, Eigen::VectorXd &qdd_sol, std::vector<double> &solved_times, 
               bool is_full);

  private:
    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    std::unique_ptr<multibody::MultibodyPlant<AutoDiffXd>> plant_autodiff_;
    std::unique_ptr<systems::Context<AutoDiffXd>> plant_context_autodiff_;
    uint32_t nq_;
    uint32_t nv_;
    double robot_total_mass_;
    std::vector<std::string> frames_name_;
    double alpha_;
    double tol_;
    double solver_tol_;
    // 等式约束LP问题Ax=b，s.t.Cx=0构造为等式约束二次优化问题
    // J*x = e, C*x = constraint
    // 目标函数为f(x) = 0.5*x'*Q*x + g'*x
    // 约束函数为h(x) = Cx-constraint
    // choice为选择变量，1表示选择变量，0表示不选择变量
    // constraint为约束函数值
    // result为解
    void lagrangeMultiplierSolve(const Eigen::MatrixXd &J, const Eigen::VectorXd &e,
                                 const Eigen::VectorXi &choice, const Eigen::VectorXd &constraint, Eigen::VectorXd &result)
    {
      // J(kxn),x(nx1),e(kx1),choice(nx1),constraint(mx1),result(nx1)
      Eigen::MatrixXd Q = J.transpose() * J;
      Eigen::VectorXd g_T = -J.transpose() * e;
      const int n = Q.rows();
      const int m = constraint.rows();

      // choice(i) = 1的个数必须等于m，为提高效率，此处不做校验
      // 构造约束矩阵C
      Eigen::MatrixXd C = Eigen::MatrixXd::Zero(m, n);
      int j = 0;
      for (int i = 0; i < n; i++)
      {
        if (choice(i) == 1)
          C(j++, i) = 1;
      }
      // Eigen::MatrixXd Zeros = Eigen::MatrixXd::Zero(m, m);
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + m, n + m);
      A.block(0, 0, n, n) = Q;
      A.block(0, n, n, m) = C.transpose();
      A.block(n, 0, m, n) = C;
      // A.block(n, n, m, m) = Zeros;
      Eigen::VectorXd b = Eigen::VectorXd::Zero(n + m);
      b.head(n) = -g_T;
      b.tail(m) = constraint; // 为便于使用，使其为关节当前值，而不拘泥于标准的数学形式
      // Eigen::VectorXd res = Eigen::VectorXd::Zero(n + m);
      Eigen::VectorXd x_lambda = A.colPivHouseholderQr().solve(b);
      result = x_lambda.head(n);
    }
    void gaussNewtonSolve(const Eigen::MatrixXd &J, const Eigen::VectorXd &e,
                          const Eigen::VectorXi &choice, const Eigen::VectorXd &constraint, Eigen::VectorXd &result)
    {
      // J(kxn),x(nx1),e(kx1),choice(nx1),constraint(mx1),result(nx1)
      Eigen::MatrixXd Q = J.transpose() * J;
      Eigen::VectorXd g_T = -J.transpose() * e;
      const int n = Q.rows();
      const int m = constraint.rows();

      // choice(i) = 1的个数必须等于m，为提高效率，此处不做校验
      // 构造约束矩阵C
      Eigen::MatrixXd C = Eigen::MatrixXd::Zero(m, n);
      int j = 0;
      for (int i = 0; i < n; i++)
      {
        if (choice(i) == 1)
          C(j++, i) = 1;
      }
      // Eigen::MatrixXd Zeros = Eigen::MatrixXd::Zero(m, m);
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + m, n + m);
      A.block(0, 0, n, n) = Q;
      A.block(0, n, n, m) = C.transpose();
      A.block(n, 0, m, n) = C;
      // A.block(n, n, m, m) = Zeros;
      Eigen::VectorXd b = Eigen::VectorXd::Zero(n + m);
      b.head(n) = -g_T;
      b.tail(m) = constraint; // 为便于使用，使其为关节当前值，而不拘泥于标准的数学形式
      // Eigen::VectorXd res = Eigen::VectorXd::Zero(n + m);
      // Eigen::VectorXd x_lambda = A.colPivHouseholderQr().solve(b);
      // result = x_lambda.head(n);
      // GaussNewton求解
      double lm_lambda = 10.0;
      Eigen::MatrixXd H = A.transpose() * A + lm_lambda * Eigen::MatrixXd::Identity(n + m, n + m);
      Eigen::VectorXd b_new = -A.transpose() * b;
      Eigen::VectorXd x_lambda = H.colPivHouseholderQr().solve(b_new);
      result = x_lambda.head(n);
    }
  };
} // namespace drake
