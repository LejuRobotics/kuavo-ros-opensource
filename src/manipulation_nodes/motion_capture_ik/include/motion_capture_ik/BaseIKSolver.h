#pragma once

#include <drake/geometry/scene_graph.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/nlopt_solver.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <Eigen/Dense>
#include <leju_utils/define.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace HighlyDynamic {

using ParameterMap = std::unordered_map<std::string, double>;

enum class SolverType {
  SNOPT,   // Sequential Nonlinear Programming
  IPOPT,   // Interior Point Optimizer
  NLOPT,   // Nonlinear Optimization Library
  OSQP,    // Operator Splitting Quadratic Program
  DEFAULT  // 默认求解器（让Drake自动选择）
};

class BaseIKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit BaseIKSolver(drake::multibody::MultibodyPlant<double>* plant,
                        const std::vector<std::string>& ikConstraintFrameNames,
                        const IKSolverConfig& config);
  virtual ~BaseIKSolver() = default;

  virtual IKSolveResult solveIK(const std::vector<PoseData>& PoseConstraintList,
                                ArmIdx controlArmIndex = ArmIdx::LEFT,
                                const Eigen::VectorXd& jointMidValues = Eigen::VectorXd()) = 0;

 protected:
  virtual void initializeFrames(const std::vector<std::string>& ikConstraintFrameNames);
  virtual void initializeJointIndices() {}
  virtual void initializeJointLimits(){};

  virtual bool setConstraints(const std::vector<drake::multibody::InverseKinematics*>& ikList,
                              const std::vector<const std::vector<PoseData>*>& PoseConstraintLists,
                              const std::vector<ArmIdx>& controlArmIndices) const {
    return false;
  };

  void initInverseKinematicsSolver(drake::multibody::InverseKinematics& ik, SolverType solverType) const;

  // solve interface
  std::pair<bool, Eigen::VectorXd> solveDrakeIK(drake::multibody::InverseKinematics& ik,
                                                const Eigen::VectorXd& initialGuess,
                                                const std::string& stageName = "IK") const;

  // common interface
  Eigen::VectorXd getWarmStartSolution() const;
  virtual Eigen::VectorXd postProcessSolution(const Eigen::VectorXd& rawSolution, const ParameterMap& params) const {
    return rawSolution;
  };
  void updateLatestSolution(const Eigen::VectorXd& solution);

 protected:
  IKSolverConfig config_;
  std::vector<const drake::multibody::Frame<double>*> ConstraintFrames_;

  drake::multibody::MultibodyPlant<double>* plant_;

  int nq_;
  Eigen::VectorXd jointLowerBounds_;
  Eigen::VectorXd jointUpperBounds_;
  bool hasJointLimits_;

  Eigen::VectorXd latestSolution_;
  bool hasLatestSolution_;
};

}  // namespace HighlyDynamic
