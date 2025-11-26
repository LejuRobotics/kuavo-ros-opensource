#include "motion_capture_ik/BaseIncrementalIKSolver.h"

#include <iostream>
#include <stdexcept>

namespace HighlyDynamic {

BaseIncrementalIKSolver::BaseIncrementalIKSolver(drake::multibody::MultibodyPlant<double>* plant,
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
  latestSolution_ = Eigen::VectorXd::Zero(nq_);
  initializeFrames(ikConstraintFrameNames);
}

void BaseIncrementalIKSolver::initializeFrames(const std::vector<std::string>& ikConstraintFrameNames) {
  ConstraintFrames_.clear();
  ConstraintFrames_.reserve(ikConstraintFrameNames.size());

  for (const auto& frameName : ikConstraintFrameNames) {
    ConstraintFrames_.push_back(&plant_->GetFrameByName(frameName));
  }
}

bool BaseIncrementalIKSolver::preSolveCheck(const std::vector<PoseData>& PoseConstraintList,
                                            const Eigen::VectorXd& jointMidValues,
                                            int nq,
                                            ArmIdx controlArmIndex) const {
  if (!plant_) {
    std::cerr << "Error: plant_ is null" << std::endl;
    return false;
  }

  if (PoseConstraintList.empty()) {
    std::cerr << "Empty pose data list" << std::endl;
    return false;
  }

  if (PoseConstraintList.size() != ConstraintFrames_.size()) {
    std::cerr << "PoseConstraintList size != ConstraintFrames_.size" << std::endl;
    return false;
  }

  for (size_t i = 0; i < PoseConstraintList.size(); ++i) {
    const auto& pos = PoseConstraintList[i].position;

    bool hasNaN = pos.hasNaN();
    bool hasInf = !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2));
    bool isZero = pos.isZero(1e-10);
    bool isExtreme = (pos.array().abs() > 1e6).any();

    if (hasNaN) std::cerr << "position " << i << " contains NaN value!" << std::endl;
    if (hasInf) std::cerr << "position " << i << " contains infinite value!" << std::endl;
    // if (isZero) std::cout << "position " << i << " is zero vector" << std::endl;
    if (isExtreme) std::cerr << "position " << i << " contains extreme value!" << std::endl;

    if (hasNaN || hasInf || isExtreme) {
      std::cerr << "Invalid position data, cannot solve IK!" << std::endl;
      std::cerr << "Please check the data source of PoseConstraintList is correctly initialized." << std::endl;
      return false;
    }
  }

  // 如果nq为-1（默认值），使用成员变量nq_作为后备（向后兼容）
  int actualNq = (nq == -1) ? nq_ : nq;

  // 根据controlArmIndex检查nq是否足够
  int requiredNq = 0;
  if (controlArmIndex == ArmIdx::LEFT) {
    requiredNq = 7;  // 左臂需要至少7个关节（索引0-6）
  } else if (controlArmIndex == ArmIdx::RIGHT) {
    requiredNq = 14;  // 右臂需要至少14个关节（索引7-13）
  } else if (controlArmIndex == ArmIdx::BOTH) {
    requiredNq = 14;  // 双臂需要至少14个关节
  }

  if (actualNq < requiredNq) {
    std::cerr << "Error: nq (" << actualNq << ") is insufficient for controlArmIndex. Required: " << requiredNq
              << std::endl;
    return false;
  }

  // 如果提供了jointMidValues，检查其有效性
  if (jointMidValues.size() > 0) {
    // 检查jointMidValues的size是否与nq一致
    if (jointMidValues.size() != actualNq) {
      std::cerr << "Error: jointMidValues size (" << jointMidValues.size() << ") does not match nq (" << actualNq << ")"
                << std::endl;
      return false;
    }

    // 检查jointMidValues是否包含无效值（NaN或Inf）
    for (int i = 0; i < jointMidValues.size(); ++i) {
      if (std::isnan(jointMidValues(i))) {
        std::cerr << "Error: jointMidValues[" << i << "] contains NaN" << std::endl;
        return false;
      }
      if (std::isinf(jointMidValues(i))) {
        std::cerr << "Error: jointMidValues[" << i << "] contains Inf" << std::endl;
        return false;
      }
    }

    // 检查jointMidValues的norm是否足够大（避免使用接近零的无效值）
    if (jointMidValues.norm() <= 1e-6) {
      std::cerr << "Error: jointMidValues norm (" << jointMidValues.norm() << ") is too small (<= 1e-6)" << std::endl;
      return false;
    }
  }

  return true;
}

std::pair<bool, Eigen::VectorXd> BaseIncrementalIKSolver::solveDrakeIK(drake::multibody::InverseKinematics& ik,
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

void BaseIncrementalIKSolver::initInverseKinematicsSolver(drake::multibody::InverseKinematics& ik,
                                                          SolverType solverType) const {
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

void BaseIncrementalIKSolver::updateLatestSolution(const Eigen::VectorXd& solution) {
  latestSolution_ = solution;
  hasLatestSolution_ = true;
}

Eigen::VectorXd BaseIncrementalIKSolver::getWarmStartSolution(const Eigen::VectorXd& jointMidValues,
                                                              ArmIdx controlArmIndex) const {
  Eigen::VectorXd referenceSolution;

  // 获取基础warm start solution
  if (hasLatestSolution_ && latestSolution_.size() == nq_ && latestSolution_.norm() > 1e-6) {
    referenceSolution = latestSolution_;
  } else {
    referenceSolution = Eigen::VectorXd::Zero(nq_);
  }

  if (jointMidValues.size() >= nq_) {
    // 左臂关节 (索引0-6: 前4个关节0-3 + 腕关节4-6)
    if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
      for (int i = 0; i <= 6; ++i) {
        referenceSolution(i) = jointMidValues(i);
        if (i > 3) {
          referenceSolution(i) = 0.0;
        }
      }
    }

    // 右臂关节 (索引7-13: 前4个关节7-10 + 腕关节11-13)
    if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
      for (int i = 7; i <= 13; ++i) {
        referenceSolution(i) = jointMidValues(i);
        if (i > 10) {
          referenceSolution(i) = 0.0;
        }
      }
    }
  }

  return referenceSolution;
}
}  // namespace HighlyDynamic