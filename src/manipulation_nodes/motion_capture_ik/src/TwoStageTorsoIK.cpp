
#include "motion_capture_ik/TwoStageTorsoIK.h"

#include <Eigen/src/Core/Matrix.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>

#include <chrono>
#include <iostream>
#include <leju_utils/define.hpp>

namespace HighlyDynamic {
TwoStageTorsoIK::TwoStageTorsoIK(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 double constraintTolerance,
                                 double solverTolerance,
                                 int maxSolverIterations,
                                 ArmIdx controlArmIndex,
                                 bool isWeldBaseLink)
    : plant_(plant),
      constraintTolerance_(constraintTolerance),
      solverTolerance_(solverTolerance),
      maxSolverIterations_(maxSolverIterations),
      controlArmIndex_(controlArmIndex),
      isWeldBaseLink_(isWeldBaseLink),
      stage1Result_(false, Eigen::VectorXd()),
      stage2Result_(false, Eigen::VectorXd()),
      hasLatestSolution_(false),
      hasJointLimits_(false) {
  // [CZJ]TODO: 调通数据流，后续需要针对可读性进行优化
  if (plant_ == nullptr) {
    throw std::invalid_argument("Plant pointer is null - should be valid after CreateTestPlant");
  }

  if (ikConstraintFrameNames.empty()) {
    throw std::invalid_argument("Frame names should not be empty");
  }

  if (!plant_->is_finalized()) {
    plant_->Finalize();
  }

  nq_ = plant_->num_positions();

  frames_.clear();
  for (const auto& frameName : ikConstraintFrameNames) {
    try {
      frames_.push_back(&plant_->GetFrameByName(frameName));
    } catch (const std::exception& e) {
      std::cerr << "Error adding ikConstraintFrame: " << frameName << ", error: " << e.what() << std::endl;
      throw;  // Re-throw to indicate initialization failure
    }
  }

  initializeJointIndices();

  initializeWristFrames();

  latestSolution_ = Eigen::VectorXd::Zero(nq_);
  hasLatestSolution_ = false;

  initializeJointLimits();
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveTwoStageIK(const std::vector<PoseData>& poseDataList,
                                                                  ArmIdx controlArmIndex,
                                                                  const Eigen::VectorXd& q0) {
  if (!plantContext_) {
    std::cerr << "Error: plantContext_ is null" << std::endl;
    return {false, Eigen::VectorXd()};
  }

  Eigen::VectorXd referenceSolution;
  if (q0.size() == nq_ && q0.norm() > 1e-6) {
    referenceSolution = q0;
  } else if (hasLatestSolution_ && latestSolution_.size() == nq_ && latestSolution_.norm() > 1e-6) {
    referenceSolution = latestSolution_;
  } else {
    referenceSolution = Eigen::VectorXd::Zero(nq_);
  }
  stage1Result_ = solveStage1(poseDataList, controlArmIndex, referenceSolution);
  if (!stage1Result_.first) {
    return {false, Eigen::VectorXd()};
  }
  // Solve second stage
  stage2Result_ = solveStage2(poseDataList, controlArmIndex);
  if (!stage2Result_.first) {
    return {false, Eigen::VectorXd()};
  }

  Eigen::VectorXd limitedSolution = stage2Result_.second;

  if (hasJointLimits_) {
    limitedSolution = limitAngle(limitedSolution);
  }

  Eigen::VectorXd referenceForVelocity = hasLatestSolution_ ? latestSolution_ : Eigen::VectorXd::Zero(nq_);
  limitedSolution = limitAngleByVelocity(referenceForVelocity, limitedSolution, 720.0, 0.01);

  latestSolution_ = limitedSolution;
  hasLatestSolution_ = true;

  return {true, limitedSolution};
}

void TwoStageTorsoIK::initializeJointIndices() {
  // Clear existing indices
  leftWristIdx_.clear();
  rightWristIdx_.clear();

  // Initialize wrist joint indices based on robot configuration
  // For 14-DOF robots: left wrist joints are typically at indices 4,5,6; right wrist at 11,12,13
  //[CZJ]TODO: 后续根据实际情况修改
  if (nq_ >= 14) {
    leftWristIdx_ = {4, 5, 6};
    rightWristIdx_ = {11, 12, 13};
  } else {
    // For robots with fewer DOF, use available indices
    leftWristIdx_ = {};
    rightWristIdx_ = {};
  }
}

void TwoStageTorsoIK::initializeWristFrames() {
  // Clear existing wrist frames
  wristFrames_.clear();

  // For 14-DOF robots, use wrist joint frames for Stage1 (matching Python version)
  if (nq_ >= 14) {
    try {
      // Left wrist frame (zarm_l6_link)
      wristFrames_.push_back(&plant_->GetFrameByName("zarm_l6_link"));

      // Right wrist frame (zarm_r6_link)
      wristFrames_.push_back(&plant_->GetFrameByName("zarm_r6_link"));
    } catch (const std::exception& e) {
      std::cerr << "Error adding wrist frames: " << e.what() << std::endl;
      throw;  // Re-throw to indicate initialization failure
    }
  } else {
    // For robots with fewer DOF, use empty list (matching Python version)
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveStage1(const std::vector<PoseData>& poseDataList,
                                                              ArmIdx controlArmIndex,
                                                              const Eigen::VectorXd& q0) {
  // check poseDataList size
  if (poseDataList.size() != 5) {
    std::cerr << "错误: poseDataList 大小无效: " << poseDataList.size() << ", 期望 5" << std::endl;
    return {false, Eigen::VectorXd()};
  }

  // 详细检查每个位置数据
  for (size_t i = 0; i < poseDataList.size(); ++i) {
    const auto& pos = poseDataList[i].position;

    // 检查数据是否异常
    bool hasNaN = pos.hasNaN();
    bool hasInf = !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2));
    bool isZero = pos.isZero(1e-10);
    bool isExtreme = (pos.array().abs() > 1e6).any();

    if (hasNaN) {
      std::cerr << "⚠️  位置 " << i << " 包含 NaN 值!" << std::endl;
    }
    if (hasInf) {
      std::cerr << "⚠️  位置 " << i << " 包含无穷大值!" << std::endl;
    }
    if (isZero) {
      std::cout << "⚠️  位置 " << i << " 为零向量" << std::endl;
    }
    if (isExtreme) {
      std::cerr << "⚠️  位置 " << i << " 包含极值!" << std::endl;
    }
  }

  // 检查关键位置数据是否有效
  bool hasInvalidData = false;
  for (size_t i = 0; i < poseDataList.size(); ++i) {
    const auto& pos = poseDataList[i].position;
    bool hasInf = !std::isfinite(pos(0)) || !std::isfinite(pos(1)) || !std::isfinite(pos(2));
    if (pos.hasNaN() || hasInf || (pos.array().abs() > 1e6).any()) {
      hasInvalidData = true;
      break;
    }
  }

  if (hasInvalidData) {
    std::cerr << "✗ 检测到无效的位置数据，无法进行IK求解!" << std::endl;
    std::cerr << "请检查 poseDataList 的数据源是否正确初始化。" << std::endl;
    return {false, Eigen::VectorXd()};
  }

  Eigen::VectorXd referenceSolution = q0;

  // Create first stage IK solver
  bool useJointLimit = true;
  drake::multibody::InverseKinematics stage1Ik(*plant_, useJointLimit);

  // Configure SNOPT solver
  drake::solvers::SnoptSolver snopt;
  auto snoptId = snopt.solver_id();
  stage1Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Optimality Tolerance", solverTolerance_);
  stage1Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Iterations Limit", maxSolverIterations_);

  // Add torso constraints (hard constraints) - both Rotation and Position
  // NOTES: weld模式下，对baselink添加position和orientation约束不生效
  // drake::math::RotationMatrix<double> R_desired(Eigen::Matrix3d::Identity());
  // stage1Ik.AddOrientationConstraint(
  //     plant_->world_frame(),
  //     R_desired,
  //     *frames_[0],
  //     drake::math::RotationMatrix<double>::Identity(),
  //     constraintTolerance_
  // );

  // stage1Ik.AddPositionConstraint(
  //     *frames_[0],
  //     Eigen::Vector3d::Zero(),
  //     plant_->world_frame(),
  //     poseDataList[0].position - constraintTolerance_ * Eigen::Vector3d::Ones(),
  //     poseDataList[0].position + constraintTolerance_ * Eigen::Vector3d::Ones()
  // );

  // Add hand and elbow soft constraints (position costs) - matching Python version

  // Left arm constraints
  // [CZJ]TODO: 目前直接验证both hand模式，后续再根据ArmIdx来配置不同工况
  // Left hand position cost - use wrist frame for Stage1 (matching Python version)

  if (wristFrames_.size() > 0) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,  // Left hand position
                             *wristFrames_[0],                                       // Use left wrist frame
                             Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  } else {
    // Fallback to hand frame if wrist frames not available
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,  // Left hand position
                             *frames_[1],
                             Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }
  // Right arm constraints

  if (wristFrames_.size() > 1) {
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,  // Right hand position
                             *wristFrames_[1],                                        // Use right wrist frame
                             Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  } else {
    // Fallback to hand frame if wrist frames not available
    stage1Ik.AddPositionCost(plant_->world_frame(),
                             poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,  // Right hand position
                             *frames_[2],
                             Eigen::Vector3d::Zero(),
                             10.0 * Eigen::Matrix3d::Identity());
  }

  // Left elbow position cost
  stage1Ik.AddPositionCost(plant_->world_frame(),
                           poseDataList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,  // Left elbow position
                           *frames_[3],
                           Eigen::Vector3d::Zero(),
                           10.0 * Eigen::Matrix3d::Identity());

  // Right elbow position cost
  stage1Ik.AddPositionCost(plant_->world_frame(),
                           poseDataList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,  // Right elbow position
                           *frames_[4],
                           Eigen::Vector3d::Zero(),
                           10.0 * Eigen::Matrix3d::Identity());

  // Add smoothness constraints with wrist joint weights - matching Python version
  // Use the reference solution determined earlier
  Eigen::VectorXd qRef = referenceSolution;
  std::vector<double> stage1Weights(nq_, 0.2);  // Default smoothness weight

  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) {
        stage1Weights[idx] = 5.0;  // Approximate freeze wrist joints
      }
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) {
        stage1Weights[idx] = 5.0;  // Approximate freeze wrist joints
      }
    }
  }

  // Add quadratic error cost for smoothness
  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage1Weights.data(), stage1Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  stage1Ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, stage1Ik.q());

  // Solve first stage using reference solution as initial guess

  try {
    auto start_time = std::chrono::high_resolution_clock::now();

    drake::solvers::MathematicalProgramResult result1 = drake::solvers::Solve(stage1Ik.prog(), referenceSolution);

    if (result1.is_success()) {
      auto solution = result1.GetSolution(stage1Ik.q());
      return {true, solution};
    } else {
      return {false, Eigen::VectorXd()};
    }
  } catch (const std::exception& e) {
    std::cerr << "✗ SolveStage1 异常: " << e.what() << std::endl;
    return {false, Eigen::VectorXd()};
  } catch (...) {
    std::cerr << "✗ SolveStage1 未知异常" << std::endl;
    return {false, Eigen::VectorXd()};
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::solveStage2(const std::vector<PoseData>& poseDataList,
                                                              ArmIdx controlArmIndex) {
  // Create second stage IK solver
  drake::multibody::InverseKinematics stage2Ik(*plant_, true);

  // Configure SNOPT solver
  drake::solvers::SnoptSolver snopt;
  auto snoptId = snopt.solver_id();
  stage2Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Optimality Tolerance", solverTolerance_);
  stage2Ik.get_mutable_prog()->SetSolverOption(snoptId, "Major Iterations Limit", maxSolverIterations_);

  // Add torso constraints (hard constraints) - matching Python version null checks
  // Check if torso rotation is valid (not identity matrix) and position is valid (not zero vector)
  // bool hasValidTorsoRotation = !poseDataList[0].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(),
  // 1e-6); bool hasValidTorsoPosition = poseDataList[0].position.norm() > 1e-6;

  // [CZJ]TODO: 后续再考虑躯干IK解算问题，目前都按照单位矩阵+零向量处理的位姿处理
  // if (hasValidTorsoRotation) {
  //     drake::math::RotationMatrix<double> R_desired(poseDataList[0].rotation_matrix);
  //     stage2Ik.AddOrientationConstraint(
  //         plant_->world_frame(),
  //         R_desired,
  //         *frames_[0],
  //         drake::math::RotationMatrix<double>::Identity(),
  //         constraintTolerance_
  //     );
  // }

  // if (hasValidTorsoPosition) {
  //     stage2Ik.AddPositionConstraint(
  //         *frames_[0],
  //         Eigen::Vector3d::Zero(),
  //         plant_->world_frame(),
  //         poseDataList[0].position - constraintTolerance_ * Eigen::Vector3d::Ones(),
  //         poseDataList[0].position + constraintTolerance_ * Eigen::Vector3d::Ones()
  //     );
  // }

  // Add hand orientation constraints (hard constraints) - matching Python version
  // Left hand orientation constraint
  if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) &&
      poseDataList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // Check if left hand rotation is valid (not identity matrix) - matching Python version
    bool hasValidLeftHandRotation =
        !poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-6);

    if (hasValidLeftHandRotation && frames_.size() > 1) {
      drake::math::RotationMatrix<double> R_desired(poseDataList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      stage2Ik.AddOrientationConstraint(plant_->world_frame(), R_desired, *frames_[1], R_ref, constraintTolerance_);
    }
  }

  // Right hand orientation constraint
  if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) &&
      poseDataList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // Check if right hand rotation is valid (not identity matrix) - matching Python version
    bool hasValidRightHandRotation =
        !poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-6);

    if (hasValidRightHandRotation && frames_.size() > 2) {
      drake::math::RotationMatrix<double> R_desired(poseDataList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
      drake::math::RotationMatrix<double> R_ref = drake::math::RotationMatrix<double>::Identity();
      stage2Ik.AddOrientationConstraint(plant_->world_frame(), R_desired, *frames_[2], R_ref, constraintTolerance_);
    }
  }

  // Add joint holding constraints with wrist joint relaxation
  std::vector<double> stage2Weights(nq_, 100.0);  // High weight to hold joints
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = 0.05;  // Allow wrist joints to change
      }
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = 0.05;  // Allow wrist joints to change
      }
    }
  }

  // Add quadratic error cost for smoothness
  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage2Weights.data(), stage2Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  stage2Ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, stage1Result_.second, stage2Ik.q());

  // Solve second stage
  try {
    drake::solvers::MathematicalProgramResult result2 = drake::solvers::Solve(stage2Ik.prog(), stage1Result_.second);

    if (result2.is_success()) {
      return {true, result2.GetSolution(stage2Ik.q())};
    } else {
      // std::cout << "[TorsoIK] 第二阶段IK求解失败" << std::endl;
      return {false, Eigen::VectorXd()};
    }
  } catch (const std::exception& e) {
    std::cerr << "Error in SolveStage2: " << e.what() << std::endl;
    return {false, Eigen::VectorXd()};
  }
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage1Result() const { return stage1Result_; }

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage2Result() const { return stage2Result_; }

void TwoStageTorsoIK::setPlantContext(std::unique_ptr<drake::systems::Context<double>> context) {
  if (!context) {
    std::cerr << "Error: Cannot set null context" << std::endl;
    return;
  }

  plantContext_ = std::move(context);
}

void TwoStageTorsoIK::initializeJointLimits() {
  if (!plant_) {
    std::cerr << "Error: Plant is null, cannot initialize joint limits" << std::endl;
    hasJointLimits_ = false;
    return;
  }

  try {
    // Initialize joint limits arrays
    jointLowerBounds_ = Eigen::VectorXd::Zero(nq_);
    jointUpperBounds_ = Eigen::VectorXd::Zero(nq_);

    // Set default joint limits (equivalent to Python's joint limits)
    // These values should match the actual robot joint limits
    for (int i = 0; i < nq_; ++i) {
      jointLowerBounds_(i) = -M_PI;  // -180 degrees
      jointUpperBounds_(i) = M_PI;   // +180 degrees
    }

    // Apply specific joint limits for Kuavo robot if needed
    // This can be customized based on the actual robot specifications

    // IMPORTANT: Following Python version behavior - disable application-level joint limits
    // Python version uses q_limit=None, which means limit_angle() returns original values
    // Drake IK solver still uses with_joint_limits=True for internal constraints
    hasJointLimits_ = false;  // Disable application-level joint angle limiting

  } catch (const std::exception& e) {
    std::cerr << "Error initializing joint limits: " << e.what() << std::endl;
    hasJointLimits_ = false;
  }
}

Eigen::VectorXd TwoStageTorsoIK::limitAngle(const Eigen::VectorXd& q) const {
  if (!hasJointLimits_) {
    return q;  // Return original if no limits available
  }

  Eigen::VectorXd qLimited = q;

  // Apply joint angle limits (equivalent to Python's limit_angle)
  for (int i = 0; i < q.size() && i < jointLowerBounds_.size() && i < jointUpperBounds_.size(); ++i) {
    qLimited(i) = std::max(jointLowerBounds_(i), std::min(q(i), jointUpperBounds_(i)));
  }

  return qLimited;
}

Eigen::VectorXd TwoStageTorsoIK::limitAngleByVelocity(const Eigen::VectorXd& qLast,
                                                      const Eigen::VectorXd& qNow,
                                                      double velLimit,
                                                      double controllerDt) const {
  if (qLast.size() != qNow.size()) {
    std::cerr << "Error: qLast and qNow size mismatch in limitAngleByVelocity" << std::endl;
    return qNow;
  }

  Eigen::VectorXd qLimited = qNow;
  int size = qNow.size();

  // Convert velocity limit from deg/s to rad/s
  double aglLimit = controllerDt * velLimit * M_PI / 180.0;

  // 120-degree limit for first joint of each arm (equivalent to Python's special handling)
  double angleLimit120Deg = controllerDt * 120.0 * M_PI / 180.0;

  // Single arm DOF (assuming 7 DOF per arm)
  int singleArmDof = 7;

  for (int i = 0; i < size; ++i) {
    // Apply velocity limit
    qLimited(i) = std::max(qLast(i) - aglLimit, std::min(qNow(i), qLast(i) + aglLimit));

    // Apply special 120-degree limit to first joint of each arm
    if (i == 0) {  // Left arm first joint (l_arm_pitch)
      qLimited(i) = std::max(qLast(i) - angleLimit120Deg, std::min(qNow(i), qLast(i) + angleLimit120Deg));
    } else if (i == singleArmDof) {  // Right arm first joint (r_arm_pitch), index 7
      qLimited(i) = std::max(qLast(i) - angleLimit120Deg, std::min(qNow(i), qLast(i) + angleLimit120Deg));
    }
  }

  return qLimited;
}

// 新增：参数更新接口实现
void TwoStageTorsoIK::updateBaseHeightOffset(double baseHeightOffset) {
  // TwoStageTorsoIK中可能不需要直接使用这些参数，但为了接口一致性保留
  std::cout << "Updated base_height_offset: " << baseHeightOffset << std::endl;
}

void TwoStageTorsoIK::updateBaseChestOffsetX(double baseChestOffsetX) {
  // TwoStageTorsoIK中可能不需要直接使用这些参数，但为了接口一致性保留
  std::cout << "Updated base_chest_offset_x: " << baseChestOffsetX << std::endl;
}

void TwoStageTorsoIK::updateShoulderWidth(double shoulderWidth) {
  // TwoStageTorsoIK中可能不需要直接使用这些参数，但为了接口一致性保留
  std::cout << "Updated shoulder_width: " << shoulderWidth << std::endl;
}

void TwoStageTorsoIK::updateUpperArmLength(double upperArmLength) {
  // TwoStageTorsoIK中可能不需要直接使用这些参数，但为了接口一致性保留
  std::cout << "Updated upper_arm_length: " << upperArmLength << std::endl;
}

void TwoStageTorsoIK::updateLowerArmLength(double lowerArmLength) {
  // TwoStageTorsoIK中可能不需要直接使用这些参数，但为了接口一致性保留
  std::cout << "Updated lower_arm_length: " << lowerArmLength << std::endl;
}

}  // namespace HighlyDynamic
