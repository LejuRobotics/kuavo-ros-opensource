#include "motion_capture_ik/TwoStageTorsoIKRec.h"

#include <ros/ros.h>

#include <iostream>
#include <leju_utils/define.hpp>

namespace HighlyDynamic {

TwoStageTorsoIK::TwoStageTorsoIK(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const IKSolverConfig& config)
    : BaseIKSolver(plant, ikConstraintFrameNames, config),
      stage1Result_(false, Eigen::VectorXd()),
      stage2Result_(false, Eigen::VectorXd()) {
  initializeJointIndices();  //[CZJ]TODO: 暂时硬编码，顺序由bone_pose消息决定，后续根据实际情况修改
  initializeJointLimits();  // [CZJ]TODO: 暂时根据nq，将bound设置为正负pi; 后续应该根据配置表来设置bound
  initializeWristFrames();  //[CZJ]TODO: 暂时硬编码为 14-DOF robots，后续根据实际情况修改

  plant_context_ = plant_->CreateDefaultContext();
}

IKSolveResult TwoStageTorsoIK::solveIK(const std::vector<PoseData>& PoseConstraintList, ArmIdx controlArmIndex) {
  if (!preSolveCheck(PoseConstraintList)) {
    return IKSolveResult(nq_, "preSolveCheck failed");
  }

  bool useJointLimit = true;  // 使用urdf中定义的joint limit
  drake::multibody::InverseKinematics stage1Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage1Ik, SolverType::SNOPT);

  drake::multibody::InverseKinematics stage2Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage2Ik, SolverType::SNOPT);

  std::vector<drake::multibody::InverseKinematics*> stageIkList = {&stage1Ik, &stage2Ik};
  std::vector<const std::vector<PoseData>*> stagePoseDataLists = {&PoseConstraintList, &PoseConstraintList};
  std::vector<ArmIdx> stageControlArmIndices = {controlArmIndex, controlArmIndex};

  if (!setConstraints(stageIkList, stagePoseDataLists, stageControlArmIndices)) {
    return IKSolveResult(nq_, "setConstraints failed");
  }

  Eigen::VectorXd referenceSolution = getWarmStartSolution();

  auto s1StartTime = std::chrono::high_resolution_clock::now();
  stage1Result_ = solveDrakeIK(stage1Ik, referenceSolution, "SolveStage1");
  if (!stage1Result_.first) {
    return IKSolveResult(nq_, "Stage1 solve failed");
  }

  stage2Result_ = solveDrakeIK(stage2Ik, stage1Result_.second, "SolveStage2");
  if (!stage2Result_.first) {
    return IKSolveResult(nq_, "Stage2 solve failed");
  }
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - s1StartTime);

  Eigen::VectorXd limitedSolution = postProcessSolution(stage2Result_.second, ikParams_.toParameterMap());
  updateLatestSolution(limitedSolution);

  return IKSolveResult(limitedSolution, duration);
}

void TwoStageTorsoIK::initializeJointIndices() {
  leftWristIdx_.clear();
  rightWristIdx_.clear();

  //[CZJ]TODO: 暂时硬编码，顺序由bone_pose消息决定，后续根据实际情况修改
  if (nq_ >= 14) {
    leftWristIdx_ = {4, 5, 6};
    rightWristIdx_ = {11, 12, 13};
  } else {
    leftWristIdx_ = {};
    rightWristIdx_ = {};
  }
}

void TwoStageTorsoIK::initializeJointLimits() {
  if (!plant_) {
    std::cerr << "Error: Plant is null, cannot initialize joint limits" << std::endl;
    hasJointLimits_ = false;
    return;
  }
  // [CZJ]TODO: 暂时根据nq，将bound设置为正负pi; 后续应该根据配置表来设置bound
  jointLowerBounds_ = Eigen::VectorXd::Constant(nq_, -M_PI);
  jointUpperBounds_ = Eigen::VectorXd::Constant(nq_, M_PI);
  hasJointLimits_ = false;
}

void TwoStageTorsoIK::initializeWristFrames() {
  wristFrames_.clear();
  //[CZJ]TODO: 暂时硬编码为 14-DOF robots，后续根据实际情况修改
  if (nq_ >= 14) {
    // Left wrist frame (zarm_l6_link)
    wristFrames_.push_back(&plant_->GetFrameByName("zarm_l6_link"));

    // Right wrist frame (zarm_r6_link)
    wristFrames_.push_back(&plant_->GetFrameByName("zarm_r6_link"));
  }
}

Eigen::VectorXd TwoStageTorsoIK::postProcessSolution(const Eigen::VectorXd& rawSolution,
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

bool TwoStageTorsoIK::setConstraints(const std::vector<drake::multibody::InverseKinematics*>& ikList,
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
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i]);
    } else if (i == 1) {
      setStage2Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i]);
    } else {
      ROS_WARN("TwoStageTorsoIK: Unknown stage %zu, defaulting to Stage1 constraints for batch processing", i);
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i]);
    }
  }
  return true;
}

void TwoStageTorsoIK::setStage1Constraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex) const {
  // Get warm start solution internally
  Eigen::VectorXd referenceSolution = getWarmStartSolution();
  // Add torso constraints (hard constraints) - both Rotation and Position
  // NOTES: weld模式下，对baselink添加position和orientation约束不生效
  // drake::math::RotationMatrix<double> R_desired(Eigen::Matrix3d::Identity());
  // ik.AddOrientationConstraint(
  //     plant_->world_frame(),
  //     R_desired,
  //     *ConstraintFrames_[0],
  //     drake::math::RotationMatrix<double>::Identity(),
  //     config_.constraintTolerance
  // );

  // ik.AddPositionConstraint(
  //     *ConstraintFrames_[0],
  //     Eigen::Vector3d::Zero(),
  //     plant_->world_frame(),
  //     PoseConstraintList[0].position - config_.constraintTolerance * Eigen::Vector3d::Ones(),
  //     PoseConstraintList[0].position + config_.constraintTolerance * Eigen::Vector3d::Ones()
  // );

  // Add hand and elbow soft constraints (position costs) - matching Python version

  // Left arm constraints - 根据controlArmIndex选择性添加约束
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    // Left hand position cost - use wrist frame for Stage1 (matching Python version)
    if (wristFrames_.size() > 0) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,  // Left hand position
                         *wristFrames_[0],                                             // Use left wrist frame
                         Eigen::Vector3d::Zero(),
                         ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      // Fallback to hand frame if wrist frames not available
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,  // Left hand position
                         *ConstraintFrames_[1],
                         Eigen::Vector3d::Zero(),
                         ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    }

    // Left elbow position cost
    ik.AddPositionCost(plant_->world_frame(),
                       PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,  // Left elbow position
                       *ConstraintFrames_[3],
                       Eigen::Vector3d::Zero(),
                       ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
  }

  // Right arm constraints - 根据controlArmIndex选择性添加约束
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    // Right hand position cost
    if (wristFrames_.size() > 1) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,  // Right hand position
                         *wristFrames_[1],                                              // Use right wrist frame
                         Eigen::Vector3d::Zero(),
                         ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      // Fallback to hand frame if wrist frames not available
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,  // Right hand position
                         *ConstraintFrames_[2],
                         Eigen::Vector3d::Zero(),
                         ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    }

    // Right elbow position cost
    ik.AddPositionCost(plant_->world_frame(),
                       PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,  // Right elbow position
                       *ConstraintFrames_[4],
                       Eigen::Vector3d::Zero(),
                       ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
  }

  // Add smoothness constraints with wrist joint weights - matching Python version
  std::vector<double> stage1Weights(nq_, ikParams_.stage1DefaultWeight);  // Default smoothness weight

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
        stage1Weights[idx] = ikParams_.stage1WristWeight;  // Approximate freeze wrist joints
      }
    }
  }

  // Add quadratic error cost for smoothness
  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage1Weights.data(), stage1Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, ik.q());
}

// Stage2约束设置方法
void TwoStageTorsoIK::setStage2Constraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex) const {
  // Add hand orientation constraints (hard constraints) - matching Python version
  // Left hand orientation constraint
  if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // Check if left hand rotation is valid (not identity matrix) - matching Python version
    bool hasValidLeftHandRotation =
        !PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-6);

    if (hasValidLeftHandRotation && ConstraintFrames_.size() > 1) {
      drake::math::RotationMatrix<double> R_desired(PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      ik.AddOrientationConstraint(
          plant_->world_frame(), R_desired, *ConstraintFrames_[1], R_ref, config_.constraintTolerance);
    }
  }

  // Right hand orientation constraint
  if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // Check if right hand rotation is valid (not identity matrix) - matching Python version
    bool hasValidRightHandRotation = !PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix.isApprox(
        Eigen::Matrix3d::Identity(), 1e-6);

    if (hasValidRightHandRotation && ConstraintFrames_.size() > 2) {
      drake::math::RotationMatrix<double> R_desired(
          PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      ik.AddOrientationConstraint(
          plant_->world_frame(), R_desired, *ConstraintFrames_[2], R_ref, config_.constraintTolerance);
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

  // 使用Stage1的结果作为参考解，如果不可用则使用warm start solution
  Eigen::VectorXd referenceSolution;
  if (stage1Result_.first && stage1Result_.second.size() > 0) {
    referenceSolution = stage1Result_.second;
  } else {
    ROS_WARN("TwoStageTorsoIK: Stage1 result not available, using warm start solution for Stage2 constraints");
    referenceSolution = getWarmStartSolution();
  }

  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, ik.q());
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage1Result() const { return stage1Result_; }
std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage2Result() const { return stage2Result_; }

// Forward Kinematics implementation - matching plantIK.cc functionality
std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageTorsoIK::FK(const Eigen::VectorXd& q, HandSide side) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::FK: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  plant_->SetPositions(plant_context_.get(), q);

  std::string frame_name;
  if (side == HandSide::LEFT) {
    // Use left hand frame - assuming ConstraintFrames_[1] is left hand
    if (ConstraintFrames_.size() > 1) {
      frame_name = ConstraintFrames_[1]->name();
    } else {
      ROS_ERROR("TwoStageTorsoIK::FK: Left hand frame not available in ConstraintFrames_");
      return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    }
  } else if (side == HandSide::RIGHT) {
    // Use right hand frame - assuming ConstraintFrames_[2] is right hand
    if (ConstraintFrames_.size() > 2) {
      frame_name = ConstraintFrames_[2]->name();
    } else {
      ROS_ERROR("TwoStageTorsoIK::FK: Right hand frame not available in ConstraintFrames_");
      return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    }
  } else {
    ROS_ERROR("TwoStageTorsoIK::FK: Invalid HandSide, only LEFT and RIGHT are supported");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  try {
    // Get the frame by name and calculate pose relative to world frame (ConstraintFrames_[0])
    // print the frame name
    ROS_INFO("TwoStageTorsoIK::FK: frame_name: %s", frame_name.c_str());
    const drake::multibody::Frame<double>& target_frame = plant_->GetFrameByName(frame_name);
    const drake::multibody::Frame<double>& reference_frame =
        (ConstraintFrames_.size() > 0) ? *ConstraintFrames_[0] : plant_->world_frame();

    auto pose = target_frame.CalcPose(*plant_context_, reference_frame);
    return std::make_pair(pose.translation(), pose.rotation().ToQuaternion());
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageTorsoIK::FK: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageTorsoIK::FKElbow(const Eigen::VectorXd& q, HandSide side) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::FKElbow: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  plant_->SetPositions(plant_context_.get(), q);

  std::string frame_name;
  if (side == HandSide::LEFT) {
    frame_name = "zarm_l4_link";  // zarm_l4_link (左肘) - matching plantIK.cc
  } else if (side == HandSide::RIGHT) {
    frame_name = "zarm_r4_link";  // zarm_r4_link (右肘) - matching plantIK.cc
  } else {
    ROS_ERROR("TwoStageTorsoIK::FKElbow: Invalid HandSide, only LEFT and RIGHT are supported");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  try {
    // Get the frame by name and calculate pose relative to world frame (ConstraintFrames_[0])
    const drake::multibody::Frame<double>& target_frame = plant_->GetFrameByName(frame_name);
    const drake::multibody::Frame<double>& reference_frame =
        (ConstraintFrames_.size() > 0) ? *ConstraintFrames_[0] : plant_->world_frame();

    auto pose = target_frame.CalcPose(*plant_context_, reference_frame);
    return std::make_pair(pose.translation(), pose.rotation().ToQuaternion());
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageTorsoIK::FKElbow: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

}  // namespace HighlyDynamic