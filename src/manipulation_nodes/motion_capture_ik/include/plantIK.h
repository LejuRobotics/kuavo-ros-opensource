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

namespace HighlyDynamic
{
  enum class HandSide
  {
    LEFT,
    RIGHT,
    BOTH
  };
  HandSide intToHandSide(int value) {
    if (value < 0 || value > 2) 
      throw std::invalid_argument("Invalid value for HandSide");
    return static_cast<HandSide>(value);
  }
  typedef std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> FramePoseVec;
  struct IKParams
  {
    // snopt params
    double major_optimality_tol{1e-3};
    double major_feasibility_tol{1e-3};
    double minor_feasibility_tol{1e-3};
    double major_iterations_limit{100};
    // constraint and cost params
    double oritation_constraint_tol{1e-3};
    double pos_constraint_tol{1e-3}; // work when pos_cost_weight > 0.0
    double pos_cost_weight{100}; // NOT work if pos_cost_weight <= 0.0
  };
  class CoMIK
  {
  public:
    CoMIK(){};
    CoMIK(drake::multibody::MultibodyPlant<double> *plant, std::vector<std::string> frames_name);

    bool solve(const FramePoseVec &pose, const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol, IKParams params = IKParams());

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q, HandSide side);

  private:
    drake::solvers::Binding<drake::solvers::Constraint> AddCoMPositionConstraint(drake::multibody::InverseKinematics &ik, const Eigen::Vector3d &r_des);

    drake::multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<drake::systems::Context<double>> plant_context_;
    std::vector<std::string> frames_name_;
    Eigen::VectorXd prev_q_sol;
  };

} // namespace drake
