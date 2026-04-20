#include "motion_capture_ik/BaseIKSolver.h"

#include <iostream>
#include <stdexcept>

namespace HighlyDynamic {

BaseIKSolver::BaseIKSolver(drake::multibody::MultibodyPlant<double>* plant,
                           const std::vector<std::string>& ikConstraintFrameNames,
                           const IKSolverConfig& config)
    : config_(config), plant_(plant), nq_(0), hasJointLimits_(false), hasLatestSolution_(false) {
  if (!plant_) {
    throw std::invalid_argument("Plant pointer is null - should be valid after CreateTestPlant");
  }

  if (ikConstraintFrameNames.empty()) {
    throw std::invalid_argument("Frame names should not be empty");
  }

  if (!plant_->is_finalized()) {
    plant_->Finalize();
  }

  nq_ = plant_->num_positions();
  std::cout << "[BaseIKSolver] nq: " << nq_ << std::endl;
  latestSolution_ = Eigen::VectorXd::Zero(nq_);
  initializeFrames(ikConstraintFrameNames);
}

void BaseIKSolver::initializeFrames(const std::vector<std::string>& ikConstraintFrameNames) {
  ConstraintFrames_.clear();
  ConstraintFrames_.reserve(ikConstraintFrameNames.size());

  for (const auto& frameName : ikConstraintFrameNames) {
    ConstraintFrames_.push_back(&plant_->GetFrameByName(frameName));
  }
}

std::pair<bool, Eigen::VectorXd> BaseIKSolver::solveDrakeIK(drake::multibody::InverseKinematics& ik,
                                                            const Eigen::VectorXd& initialGuess,
                                                            const std::string& stageName) const {
  drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(ik.prog(), initialGuess);
  if (result.is_success()) {
    auto solution = result.GetSolution(ik.q());
    return {true, solution};
  } else {
    return {false, Eigen::VectorXd::Zero(nq_)};
  }
}

void BaseIKSolver::initInverseKinematicsSolver(drake::multibody::InverseKinematics& ik, SolverType solverType) const {
  switch (solverType) {
    case SolverType::SNOPT: {
      drake::solvers::SnoptSolver snopt;
      auto snoptId = snopt.solver_id();
      ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Optimality Tolerance", config_.solverTolerance);
      ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Iterations Limit", config_.maxIterations);
      break;
    }

    case SolverType::IPOPT: {
      drake::solvers::IpoptSolver ipopt;
      auto ipoptId = ipopt.solver_id();

      // [CZJ]TODO: 后续再配置IPOPT参数
      std::cout << "\033[93m[WARNING] IPOPT solver is not configured yet\033[0m" << std::endl;
      break;
    }

    case SolverType::NLOPT: {
      drake::solvers::NloptSolver nlopt;
      auto nloptId = nlopt.solver_id();

      // [CZJ]TODO: 后续再配置IPOPT参数
      std::cout << "\033[93m[WARNING] NLOPT solver is not configured yet\033[0m" << std::endl;
      break;
    }

    case SolverType::OSQP: {
      drake::solvers::OsqpSolver osqp;
      auto osqpId = osqp.solver_id();

      // [CZJ]TODO: 后续再配置IPOPT参数
      std::cout << "\033[93m[WARNING] OSQP solver is not configured yet\033[0m" << std::endl;
      break;
    }

    case SolverType::DEFAULT:
      // [CZJ]TODO: 后续再配置默认求解器
      std::cout << "\033[93m[WARNING] Default solver is not configured yet\033[0m" << std::endl;
      break;

    default:
      throw std::invalid_argument("Unsupported solver type: " + std::to_string(static_cast<int>(solverType)));
  }
}

void BaseIKSolver::updateLatestSolution(const Eigen::VectorXd& solution) {
  latestSolution_ = solution;
  hasLatestSolution_ = true;
}

Eigen::VectorXd BaseIKSolver::getWarmStartSolution() const {
  if (hasLatestSolution_ && latestSolution_.size() == nq_ && latestSolution_.norm() > 1e-6) {
    return latestSolution_;
  } else {
    return Eigen::VectorXd::Zero(nq_);
  }
}
}  // namespace HighlyDynamic