#include "motion_capture_ik/TwoStageIncrementalIK.h"

#include <ros/ros.h>

#include <leju_utils/define.hpp>

namespace HighlyDynamic {

TwoStageIncrementalIK::TwoStageIncrementalIK(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const IKSolverConfig& config)
    : BaseIncrementalIKSolver(plant, ikConstraintFrameNames, config),
      stage1Result_(false, Eigen::VectorXd()),
      stage2Result_(false, Eigen::VectorXd()),
      leftWristIdx_({4, 5, 6}),
      rightWristIdx_({11, 12, 13}) {
  wristFrames_.clear();
  wristFrames_.push_back(&plant_->GetFrameByName("zarm_l6_link"));
  wristFrames_.push_back(&plant_->GetFrameByName("zarm_r6_link"));

  plant_context_ = plant_->CreateDefaultContext();
}

IKSolveResult TwoStageIncrementalIK::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                             ArmIdx controlArmIndex,
                                             const Eigen::VectorXd& jointMidValues) {
  if (!preSolveCheck(PoseConstraintList, jointMidValues, nq_, controlArmIndex)) {
    return IKSolveResult(nq_, "preSolveCheck failed");
  }

  bool useJointLimit = true;  // 使用urdf中定义的joint limit
  drake::multibody::InverseKinematics stage1Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage1Ik, SolverType::SNOPT);

  drake::multibody::InverseKinematics stage2Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage2Ik, SolverType::SNOPT);

  Eigen::VectorXd referenceSolution = getWarmStartSolution(jointMidValues, controlArmIndex);

  // 存储referenceSolution到mutable成员变量，供const函数使用
  currentReferenceSolution_ = referenceSolution;

  std::vector<drake::multibody::InverseKinematics*> stageIkList = {&stage1Ik, &stage2Ik};
  std::vector<const std::vector<PoseData>*> stagePoseDataLists = {&PoseConstraintList, &PoseConstraintList};
  std::vector<ArmIdx> stageControlArmIndices = {controlArmIndex, controlArmIndex};

  if (!setConstraints(stageIkList, stagePoseDataLists, stageControlArmIndices)) {
    return IKSolveResult(nq_, "setConstraints failed");
  }

  auto s1StartTime = std::chrono::high_resolution_clock::now();
  stage1Result_ = solveDrakeIK(stage1Ik, referenceSolution, "SolveStage1");
  if (!stage1Result_.first) {
    ROS_WARN("TwoStageIncrementalIK::solveIK: Stage1 solve failed");
    return IKSolveResult(nq_, "Stage1 solve failed");
  }

  stage2Result_ = solveDrakeIK(stage2Ik, stage1Result_.second, "SolveStage2");
  if (!stage2Result_.first) {
    ROS_WARN("TwoStageIncrementalIK::solveIK: Stage2 solve failed");
    return IKSolveResult(nq_, "Stage2 solve failed");
  }
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - s1StartTime);

  Eigen::VectorXd limitedSolution = postProcessSolution(stage2Result_.second, ikParams_.toParameterMap());

  if (stage1Result_.first && stage1Result_.second.size() == nq_ && limitedSolution.size() == nq_) {
    // 左臂前4个关节 (j1~j4, 索引0-3) 使用stage1结果
    if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
      for (int i = 0; i < 4; ++i) {
        limitedSolution(i) = stage1Result_.second(i);
      }
    }

    // 右臂前4个关节 (j7~j10, 索引7-10) 使用stage1结果
    if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
      for (int i = 7; i <= 10; ++i) {
        limitedSolution(i) = stage1Result_.second(i);
      }
    }
  } else {
    ROS_WARN("TwoStageIncrementalIK::solveIK: Cannot merge stage1 results, using stage2 solution only");
  }

  updateLatestSolution(limitedSolution);

  return IKSolveResult(limitedSolution, duration);
}

Eigen::VectorXd TwoStageIncrementalIK::postProcessSolution(const Eigen::VectorXd& rawSolution,
                                                           const ParameterMap& params) const {
  Eigen::VectorXd limitedSolution = rawSolution;

  if (hasJointLimits_) {
    limitedSolution = ::limitAngle(limitedSolution, jointLowerBounds_, jointUpperBounds_, hasJointLimits_);
  }

  double velocityLimitDeg = 720.0;
  double controllerDt = 0.01;
  double firstJointAngleLimitDeg = 120.0;

  Eigen::VectorXd referenceForVelocity = hasLatestSolution_ ? latestSolution_ : Eigen::VectorXd::Zero(nq_);
  limitedSolution = ::limitJointAngleByVelocity(
      referenceForVelocity, limitedSolution, velocityLimitDeg, controllerDt, firstJointAngleLimitDeg);

  return limitedSolution;
}

bool TwoStageIncrementalIK::setConstraints(const std::vector<drake::multibody::InverseKinematics*>& ikList,
                                           const std::vector<const std::vector<PoseData>*>& PoseConstraintLists,
                                           const std::vector<ArmIdx>& controlArmIndices) const {
  if (ikList.size() != PoseConstraintLists.size() || ikList.size() != controlArmIndices.size()) {
    return false;
  }

  for (size_t i = 0; i < ikList.size(); ++i) {
    if (ikList[i] == nullptr) {
      return false;
    }

    if (i == 0) {
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    } else if (i == 1) {
      setStage2Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    } else {
      ROS_WARN("TwoStageIncrementalIK: Unknown stage %zu, defaulting to Stage1 constraints for batch processing", i);
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    }
  }
  return true;
}

void TwoStageIncrementalIK::setStage1Constraints(drake::multibody::InverseKinematics& ik,
                                                 const std::vector<PoseData>& PoseConstraintList,
                                                 ArmIdx controlArmIndex,
                                                 const Eigen::VectorXd& referenceSolution) const {
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    // Check bounds for left hand and elbow constraints
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND &&
        PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW && wristFrames_.size() > 0 &&
        ConstraintFrames_.size() > 3) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,  // Left hand position
                         *wristFrames_[0],                                             // Use left wrist frame
                         Eigen::Vector3d::Zero(),
                         1e1 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());

      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,  // Left elbow position
                         *ConstraintFrames_[3],
                         Eigen::Vector3d::Zero(),
                         1e-1 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN(
          "TwoStageIncrementalIK::setStage1Constraints: Left hand/elbow constraints skipped due to insufficient data");
    }
  }

  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    // Check bounds for right hand and elbow constraints
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND &&
        PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW && wristFrames_.size() > 1 &&
        ConstraintFrames_.size() > 4) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,  // Right hand position
                         *wristFrames_[1],                                              // Use right wrist frame
                         Eigen::Vector3d::Zero(),
                         1e1 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());

      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,  // Right elbow position
                         *ConstraintFrames_[4],
                         Eigen::Vector3d::Zero(),
                         1e-1 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN(
          "TwoStageIncrementalIK::setStage1Constraints: Right hand/elbow constraints skipped due to insufficient data");
    }
  }

  std::vector<double> stage1Weights(nq_, ikParams_.stage1DefaultWeight);

  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) {
        stage1Weights[idx] = ikParams_.stage1WristWeight;  // Approximate freeze wrist joints
      }
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) {
        stage1Weights[idx] = ikParams_.stage1WristWeight;
      }
    }
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage1Weights.data(), stage1Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, ik.q());
}

// Stage2约束设置方法
void TwoStageIncrementalIK::setStage2Constraints(drake::multibody::InverseKinematics& ik,
                                                 const std::vector<PoseData>& PoseConstraintList,
                                                 ArmIdx controlArmIndex,
                                                 const Eigen::VectorXd& referenceSolution) const {
  // Add hand orientation constraints (hard constraints) - matching Python version
  // Left hand orientation constraint
  if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // Check bounds: ConstraintFrames_[1] is left hand, need size > 1
    if (ConstraintFrames_.size() > 1) {
      drake::math::RotationMatrix<double> R_desired(PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      ik.AddOrientationConstraint(
          plant_->world_frame(), R_desired, *ConstraintFrames_[1], R_ref, config_.constraintTolerance);
    } else {
      ROS_WARN(
          "TwoStageIncrementalIK::setStage2Constraints: Left hand constraint frame not available "
          "(ConstraintFrames_.size()=%zu, need >1), skipping orientation constraint",
          ConstraintFrames_.size());
    }
  }

  // Right hand orientation constraint
  if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // Check bounds: ConstraintFrames_[2] is right hand, need size > 2
    if (ConstraintFrames_.size() > 2) {
      drake::math::RotationMatrix<double> R_desired(
          PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      ik.AddOrientationConstraint(
          plant_->world_frame(), R_desired, *ConstraintFrames_[2], R_ref, config_.constraintTolerance);
    } else {
      ROS_WARN(
          "TwoStageIncrementalIK::setStage2Constraints: Right hand constraint frame not available "
          "(ConstraintFrames_.size()=%zu, need >2), skipping orientation constraint",
          ConstraintFrames_.size());
    }
  }

  // Add joint holding constraints with wrist joint relaxation
  std::vector<double> stage2Weights(nq_, ikParams_.stage2DefaultWeight);  // High weight to hold joints
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = ikParams_.stage2WristWeight;  // Allow wrist joints to change
      }
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = ikParams_.stage2WristWeight;  // Allow wrist joints to change
      }
    }
  }

  // Add quadratic error cost for smoothness
  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage2Weights.data(), stage2Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();

  // 使用Stage1的结果作为参考解，如果不可用则使用传入的referenceSolution
  Eigen::VectorXd stage2ReferenceSolution;
  if (stage1Result_.first && stage1Result_.second.size() > 0) {
    stage2ReferenceSolution = stage1Result_.second;
  } else {
    ROS_WARN(
        "TwoStageIncrementalIK: Stage1 result not available, using provided reference solution for Stage2 constraints");
    stage2ReferenceSolution = referenceSolution;
  }

  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, stage2ReferenceSolution, ik.q());
}

std::pair<bool, Eigen::VectorXd> TwoStageIncrementalIK::getStage1Result() const { return stage1Result_; }
std::pair<bool, Eigen::VectorXd> TwoStageIncrementalIK::getStage2Result() const { return stage2Result_; }

// Forward Kinematics implementation - matching plantIK.cc functionality
std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageIncrementalIK::FK(const Eigen::VectorXd& q,
                                                                         const std::string& frameName,
                                                                         int expectedSize) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageIncrementalIK::FK: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (expectedSize > 0 && q.size() != expectedSize) {
    ROS_ERROR("TwoStageIncrementalIK::FK: Joint vector size mismatch. Expected %d, got %zu", expectedSize, q.size());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  plant_->SetPositions(plant_context_.get(), q);

  try {
    const drake::multibody::Frame<double>& target_frame = plant_->GetFrameByName(frameName);
    const drake::multibody::Frame<double>& reference_frame =
        (ConstraintFrames_.size() > 0) ? *ConstraintFrames_[0] : plant_->world_frame();

    auto pose = target_frame.CalcPose(*plant_context_, reference_frame);
    return std::make_pair(pose.translation(), pose.rotation().ToQuaternion());
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageIncrementalIK::FK: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageIncrementalIK::FKElbow(const Eigen::VectorXd& q,
                                                                              const std::string& frameName,
                                                                              int expectedSize) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageIncrementalIK::FKElbow: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (expectedSize > 0 && q.size() != expectedSize) {
    ROS_ERROR(
        "TwoStageIncrementalIK::FKElbow: Joint vector size mismatch. Expected %d, got %zu", expectedSize, q.size());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  plant_->SetPositions(plant_context_.get(), q);

  try {
    const drake::multibody::Frame<double>& target_frame = plant_->GetFrameByName(frameName);
    const drake::multibody::Frame<double>& reference_frame =
        (ConstraintFrames_.size() > 0) ? *ConstraintFrames_[0] : plant_->world_frame();

    auto pose = target_frame.CalcPose(*plant_context_, reference_frame);
    return std::make_pair(pose.translation(), pose.rotation().ToQuaternion());
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageIncrementalIK::FKElbow: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

Eigen::MatrixXd TwoStageIncrementalIK::getFrameJacobian(const Eigen::VectorXd& q, const std::string& frameName) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageIncrementalIK::getHandJacobian: plant_context_ is null");
    return Eigen::MatrixXd::Zero(3, nq_);
  }

  // Set joint positions
  plant_->SetPositions(plant_context_.get(), q);

  // Get the target frame by name
  const drake::multibody::Frame<double>* target_frame = nullptr;
  try {
    target_frame = &plant_->GetFrameByName(frameName);
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageIncrementalIK::getHandJacobian: Failed to get frame '%s': %s", frameName.c_str(), e.what());
    return Eigen::MatrixXd::Zero(3, nq_);
  }

  try {
    // Compute translational velocity Jacobian
    Eigen::MatrixXd J(3, plant_->num_velocities());
    const drake::multibody::Frame<double>& world_frame = plant_->world_frame();
    plant_->CalcJacobianTranslationalVelocity(*plant_context_,
                                              drake::multibody::JacobianWrtVariable::kV,  // with respect to velocity
                                              *target_frame,
                                              Eigen::Vector3d::Zero(),  // point at frame origin
                                              world_frame,              // expressed in world frame
                                              world_frame,              // Jacobian expressed in world frame
                                              &J);

    return J;
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageIncrementalIK::getHandJacobian: Exception occurred: %s", e.what());
    return Eigen::MatrixXd::Zero(3, nq_);
  }
}

}  // namespace HighlyDynamic