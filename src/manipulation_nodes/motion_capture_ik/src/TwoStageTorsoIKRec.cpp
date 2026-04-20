#include "motion_capture_ik/TwoStageTorsoIKRec.h"

#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <leju_utils/define.hpp>

namespace HighlyDynamic {
Eigen::Vector3d kHandRelativeEulerLimitZYX = Eigen::Vector3d(0.9 * 1.57, 0.55, 0.55);  // rad
namespace {

constexpr double kHandRelativeEulerLimit = 0.55;  // rad

double clampToHandEulerLimit(double value) {
  return std::max(-kHandRelativeEulerLimit, std::min(kHandRelativeEulerLimit, value));
}

drake::math::RotationMatrix<double> clampHandRotationByEuler(const drake::math::RotationMatrix<double>& rotation) {
  drake::math::RollPitchYaw<double> rpy(rotation);
  Eigen::Vector3d limited_rpy = rpy.vector();
  for (int i = 0; i < 2; ++i) {  // clip roll pitch only
    limited_rpy[i] = clampToHandEulerLimit(limited_rpy[i]);
  }
  // limited_rpy[2] = clampToHandEulerLimit(0.95 * 1.57);
  limited_rpy[2] = std::max(-0.9 * 1.57, std::min(0.9 * 1.57, limited_rpy[2]));
  // limited_rpy[0] = clampToHandEulerLimit(limited_rpy[0]);
  // limited_rpy[1] = clampToHandEulerLimit(limited_rpy[1]*0.85);
  return drake::math::RollPitchYaw<double>(limited_rpy).ToRotationMatrix();
}

}  // namespace

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

IKSolveResult TwoStageTorsoIK::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                       ArmIdx controlArmIndex,
                                       const Eigen::VectorXd& jointMidValues) {

  bool useJointLimit = true;  // 使用urdf中定义的joint limit
  drake::multibody::InverseKinematics stage1Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage1Ik, SolverType::SNOPT);

  drake::multibody::InverseKinematics stage2Ik(*plant_, useJointLimit);
  initInverseKinematicsSolver(stage2Ik, SolverType::SNOPT);

  // 首先获取warm start solution作为基础
  Eigen::VectorXd referenceSolution = getWarmStartSolution();

  // 如果提供了jointMidValues，根据controlArmIndex只更新对应手臂的关节值
  if (jointMidValues.size() == nq_ && jointMidValues.norm() > 1e-6) {
    // 左臂关节 (索引0-6: 前4个关节0-3 + 腕关节4-6)
    if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
      if (nq_ >= 7) {
        for (int i = 0; i <= 6; ++i) {
          referenceSolution(i) = jointMidValues(i);
          if (i > 3) {
            referenceSolution(i) = 0.0;
          }
        }
      }
    }

    // 右臂关节 (索引7-13: 前4个关节7-10 + 腕关节11-13)
    if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
      if (nq_ >= 14) {
        for (int i = 7; i <= 13; ++i) {
          referenceSolution(i) = jointMidValues(i);
          if (i > 10) {
            referenceSolution(i) = 0.0;
          }
        }
      }
    }
  } else {
    ROS_WARN("TwoStageTorsoIK::solveIK: No joint mid values provided, using warm start solution");
  }

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
    ROS_WARN("TwoStageTorsoIK::solveIK: Stage1 solve failed");
    return IKSolveResult(nq_, "Stage1 solve failed");
  }

  stage2Result_ = solveDrakeIK(stage2Ik, stage1Result_.second, "SolveStage2");
  if (!stage2Result_.first) {
    ROS_WARN("TwoStageTorsoIK::solveIK: Stage2 solve failed");
    return IKSolveResult(nq_, "Stage2 solve failed");
  }
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - s1StartTime);

  Eigen::VectorXd limitedSolution = postProcessSolution(stage2Result_.second, ikParams_.toParameterMap());

  // 将部分关节从stage1结果中提取：j1~j4 (索引0-3) 和 j7~j10 (索引7-10) 使用stage1结果
  // 根据controlArmIndex决定是否替换对应手臂的关节
  if (stage1Result_.first && stage1Result_.second.size() == nq_ && limitedSolution.size() == nq_) {
    // 左臂前4个关节 (j1~j4, 索引0-3) 使用stage1结果
    if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) && nq_ >= 4) {
      for (int i = 0; i < 4; ++i) {
        limitedSolution(i) = stage1Result_.second(i);
      }
    }

    // 右臂前4个关节 (j7~j10, 索引7-10) 使用stage1结果
    if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) && nq_ >= 11) {
      for (int i = 7; i <= 10; ++i) {
        limitedSolution(i) = stage1Result_.second(i);
      }
    }
  } else {
    ROS_WARN("TwoStageTorsoIK::solveIK: Cannot merge stage1 results, using stage2 solution only");
  }

  if (stage2Result_.first && stage2Result_.second.size() == nq_ && limitedSolution.size() == nq_) {
    if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) && nq_ >= 4) {
      limitedSolution(2) = stage2Result_.second(2);
    }

    if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) && nq_ >= 11) {
      limitedSolution(9) = stage2Result_.second(9);
    }
  } else {
    ROS_WARN("TwoStageTorsoIK::solveIK: Cannot merge stage1 results, using stage2 solution only");
  }

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
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    } else if (i == 1) {
      setStage2Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    } else {
      ROS_WARN("TwoStageTorsoIK: Unknown stage %zu, defaulting to Stage1 constraints for batch processing", i);
      setStage1Constraints(*ikList[i], *PoseConstraintLists[i], controlArmIndices[i], currentReferenceSolution_);
    }
  }
  return true;
}

void TwoStageTorsoIK::setStage1Constraints(drake::multibody::InverseKinematics& ik,
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
                         1e-2 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN("TwoStageTorsoIK::setStage1Constraints: Left hand/elbow constraints skipped due to insufficient data");
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
                         1e-2 * ikParams_.stage1PositionWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN("TwoStageTorsoIK::setStage1Constraints: Right hand/elbow constraints skipped due to insufficient data");
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
void TwoStageTorsoIK::setStage2Constraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex,
                                           const Eigen::VectorXd& referenceSolution) const {
  // 使用Stage1的结果作为参考解，如果不可用则使用传入的referenceSolution
  Eigen::VectorXd stage2ReferenceSolution = referenceSolution;
  if (stage1Result_.first && stage1Result_.second.size() > 0) {
    stage2ReferenceSolution = stage1Result_.second;
  } else {
    ROS_WARN("TwoStageTorsoIK: Stage1 result not available, using provided reference solution for Stage2 constraints");
    stage2ReferenceSolution = referenceSolution;
  }

  // ========== 完全固定[1, 2, 4]个关节, 不约束上臂yaw ==========
  // 左臂前4个关节 (索引0-3: j1~j4)
  // if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
  //   if (nq_ >= 4) {
  //     Eigen::VectorXd leftArmFixedJoints = stage2ReferenceSolution.segment(0, 4);

  //     Eigen::VectorXd lb = leftArmFixedJoints - 1e-2 * Eigen::VectorXd::Ones(4);
  //     lb(3) = -M_PI / 3;

  //     Eigen::VectorXd ub = leftArmFixedJoints + 1e-2 * Eigen::VectorXd::Ones(4);
  //     ub(3) = M_PI / 3;

  //     ik.get_mutable_prog()->AddBoundingBoxConstraint(lb, ub, ik.q().segment(0, 4));
  //   }
  // }

  // // ========== 完全固定[8, 9, 11]个关节, 不约束下臂yaw ==========
  // // 右臂前4个关节 (索引7-10: j7~j10)
  // if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
  //   if (nq_ >= 11) {
  //     Eigen::VectorXd rightArmFixedJoints = stage2ReferenceSolution.segment(7, 4);

  //     Eigen::VectorXd lb = rightArmFixedJoints - 1e-2 * Eigen::VectorXd::Ones(4);
  //     lb(3) = -M_PI / 3;

  //     Eigen::VectorXd ub = rightArmFixedJoints + 1e-2 * Eigen::VectorXd::Ones(4);
  //     ub(3) = M_PI / 3;

  //     ik.get_mutable_prog()->AddBoundingBoxConstraint(lb, ub, ik.q().segment(7, 4));
  //   }
  // }

  // ========== 计算相对于j4关节的位姿并设置约束 ==========
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::setStage2Constraints: plant_context_ is null, cannot compute relative pose");
    return;
  }

  plant_->SetPositions(plant_context_.get(), stage2ReferenceSolution);
  // 左臂约束：计算相对于zarm_l4_link的位姿
  if ((controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // Check bounds: ConstraintFrames_[1] is left hand, ConstraintFrames_[3] is zarm_l4_link
    if (ConstraintFrames_.size() > 3) {
      try {
        // 计算j4关节（zarm_l4_link）在世界坐标系中的位姿
        auto j4_pose = ConstraintFrames_[3]->CalcPose(*plant_context_, plant_->world_frame());
        Eigen::Vector3d j4_position = j4_pose.translation();
        drake::math::RotationMatrix<double> j4_rotation = j4_pose.rotation();

        // 目标手部位姿（世界坐标系）
        Eigen::Vector3d hand_position_world = PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
        drake::math::RotationMatrix<double> hand_rotation_world(
            PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);

        // 计算相对于j4关节的位姿
        // 位置：p_hand_j4 = R_j4^T * (p_hand_world - p_j4_world)
        Eigen::Vector3d hand_position_relative = j4_rotation.inverse() * (hand_position_world - j4_position);

        // 旋转：R_hand_j4 = R_j4^T * R_hand_world
        drake::math::RotationMatrix<double> hand_rotation_relative = j4_rotation.inverse() * hand_rotation_world;
        // hand_rotation_relative = clampHandRotationByEuler(hand_rotation_relative);  // Limit RP to ±0.7 rad

        Eigen::Quaterniond hand_rotation_relative_quaternion = Eigen::Quaterniond(hand_rotation_relative.matrix());
        hand_rotation_relative_quaternion =
            limitQuaternionAngleEulerZYX(hand_rotation_relative_quaternion, kHandRelativeEulerLimitZYX);
        hand_rotation_relative = drake::math::RotationMatrix<double>(hand_rotation_relative_quaternion.matrix());

        // 使用相对位姿设置约束（相对于j4关节frame）
        // AddPositionConstraint(frameB, p_BQ, frameA, p_AQ_lower, p_AQ_upper)
        // 约束手部frame的原点在j4 frame中的位置
        // ik.AddPositionConstraint(
        //     *ConstraintFrames_[1],    // frameB: 手部frame
        //     Eigen::Vector3d::Zero(),  // p_BQ: 手部frame的原点
        //     *ConstraintFrames_[3],    // frameA: j4关节frame
        //     hand_position_relative -
        //         config_.constraintTolerance * Eigen::Vector3d::Ones(),  // p_AQ_lower: 在j4 frame中的下界
        //     hand_position_relative +
        //         config_.constraintTolerance * Eigen::Vector3d::Ones()  // p_AQ_upper: 在j4 frame中的上界
        // );

        // 添加姿态约束（相对于j4关节frame）
        ik.AddOrientationConstraint(*ConstraintFrames_[3],   // frameA: j4关节frame
                                    hand_rotation_relative,  // R_A: 期望的相对旋转（限幅后）
                                    *ConstraintFrames_[1],   // frameB: 手部frame
                                    drake::math::RotationMatrix<double>::Identity(),  // R_B: 参考旋转（单位矩阵）
                                    1e4 * config_.constraintTolerance);
        // ik.AddOrientationConstraint(plant_->world_frame(),                            // frameA: j4关节frame
        //                             hand_rotation_world,                              // R_A: 期望的相对旋转
        //                             *ConstraintFrames_[1],                            // frameB: 手部frame
        //                             drake::math::RotationMatrix<double>::Identity(),  // R_B: 参考旋转（单位矩阵）
        //                             config_.constraintTolerance);
      } catch (const std::exception& e) {
        ROS_ERROR("TwoStageTorsoIK::setStage2Constraints: Exception computing left arm relative pose: %s", e.what());
        // // 如果计算相对位姿失败，回退到使用世界坐标系约束
        // drake::math::RotationMatrix<double> R_desired(
        //     PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix);
        // ik.AddOrientationConstraint(plant_->world_frame(),
        //                             R_desired,
        //                             *ConstraintFrames_[1],
        //                             drake::math::RotationMatrix<double>::Identity(),
        //                             config_.constraintTolerance);
      }
    } else {
      ROS_WARN(
          "TwoStageTorsoIK::setStage2Constraints: Left arm constraint frames not available "
          "(ConstraintFrames_.size()=%zu, need >3), skipping relative pose constraint",
          ConstraintFrames_.size());
    }
  }

  // 右臂约束：计算相对于zarm_r4_link的位姿
  if ((controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) &&
      PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // Check bounds: ConstraintFrames_[2] is right hand, ConstraintFrames_[4] is zarm_r4_link
    if (ConstraintFrames_.size() > 4) {
      try {
        // 计算j4关节（zarm_r4_link）在世界坐标系中的位姿
        auto j4_pose = ConstraintFrames_[4]->CalcPose(*plant_context_, plant_->world_frame());
        Eigen::Vector3d j4_position = j4_pose.translation();
        drake::math::RotationMatrix<double> j4_rotation = j4_pose.rotation();

        // 目标手部位姿（世界坐标系）
        Eigen::Vector3d hand_position_world = PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
        drake::math::RotationMatrix<double> hand_rotation_world(
            PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);

        // 计算相对于j4关节的位姿
        // 位置：p_hand_j4 = R_j4^T * (p_hand_world - p_j4_world)
        Eigen::Vector3d hand_position_relative = j4_rotation.inverse() * (hand_position_world - j4_position);

        // 旋转：R_hand_j4 = R_j4^T * R_hand_world
        drake::math::RotationMatrix<double> hand_rotation_relative = j4_rotation.inverse() * hand_rotation_world;
        // 与左臂保持一致：统一使用 limitQuaternionAngleEulerZYX 做相对姿态限幅
        Eigen::Quaterniond hand_rotation_relative_quaternion = Eigen::Quaterniond(hand_rotation_relative.matrix());
        hand_rotation_relative_quaternion =
            limitQuaternionAngleEulerZYX(hand_rotation_relative_quaternion, kHandRelativeEulerLimitZYX);
        hand_rotation_relative = drake::math::RotationMatrix<double>(hand_rotation_relative_quaternion.matrix());

        // 使用相对位姿设置约束（相对于j4关节frame）
        // AddPositionConstraint(frameB, p_BQ, frameA, p_AQ_lower, p_AQ_upper)
        // 约束手部frame的原点在j4 frame中的位置
        // ik.AddPositionConstraint(
        //     *ConstraintFrames_[2],    // frameB: 手部frame
        //     Eigen::Vector3d::Zero(),  // p_BQ: 手部frame的原点
        //     *ConstraintFrames_[4],    // frameA: j4关节frame
        //     hand_position_relative -
        //         config_.constraintTolerance * Eigen::Vector3d::Ones(),  // p_AQ_lower: 在j4 frame中的下界
        //     hand_position_relative +
        //         config_.constraintTolerance * Eigen::Vector3d::Ones()  // p_AQ_upper: 在j4 frame中的上界
        // );

        // 添加姿态约束（相对于j4关节frame）
        ik.AddOrientationConstraint(*ConstraintFrames_[4],   // frameA: j4关节frame
                                    hand_rotation_relative,  // R_A: 期望的相对旋转（限幅后）
                                    *ConstraintFrames_[2],   // frameB: 手部frame
                                    drake::math::RotationMatrix<double>::Identity(),  // R_B: 参考旋转（单位矩阵）
                                    1e4 * config_.constraintTolerance);
        // ik.AddOrientationConstraint(plant_->world_frame(),                            // frameA: j4关节frame
        //                             hand_rotation_world,                              // R_A: 期望的相对旋转
        //                             *ConstraintFrames_[2],                            // frameB: 手部frame
        //                             drake::math::RotationMatrix<double>::Identity(),  // R_B: 参考旋转（单位矩阵）
        //                             config_.constraintTolerance);
      } catch (const std::exception& e) {
        ROS_ERROR("TwoStageTorsoIK::setStage2Constraints: Exception computing right arm relative pose: %s", e.what());
        // 如果计算相对位姿失败，回退到使用世界坐标系约束
        // drake::math::RotationMatrix<double> R_desired(
        //     PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix);
        // ik.AddOrientationConstraint(plant_->world_frame(),
        //                             R_desired,
        //                             *ConstraintFrames_[2],
        //                             drake::math::RotationMatrix<double>::Identity(),
        //                             config_.constraintTolerance);
      }
    } else {
      ROS_WARN(
          "TwoStageTorsoIK::setStage2Constraints: Right arm constraint frames not available "
          "(ConstraintFrames_.size()=%zu, need >4), skipping relative pose constraint",
          ConstraintFrames_.size());
    }
  }
  std::vector<double> stage2Weights(nq_, 5e-5);  // 默认小权重，因为前4个关节已固定
  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : leftWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = ikParams_.stage2WristWeight;  // 腕关节平滑性权重
      }
    }
  }
  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    for (int idx : rightWristIdx_) {
      if (idx < nq_) {
        stage2Weights[idx] = ikParams_.stage2WristWeight;  // 腕关节平滑性权重
      }
    }
  }
  // stage2Weights[2] = ikParams_.stage2WristWeight;
  // stage2Weights[9] = ikParams_.stage2WristWeight;

  // std::vector<double> stage2Weights(nq_, 0);

  // if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
  //   stage2Weights[0] = 100.0;
  //   stage2Weights[1] = 100.0;
  //   stage2Weights[2] = ikParams_.stage2WristWeight;
  //   stage2Weights[3] = 100.0;
  //   stage2Weights[4] = ikParams_.stage2WristWeight;
  //   stage2Weights[5] = ikParams_.stage2WristWeight;
  //   stage2Weights[6] = ikParams_.stage2WristWeight;
  // }
  // if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
  //   stage2Weights[7] = 100.0;
  //   stage2Weights[8] = 100.0;
  //   stage2Weights[9] = ikParams_.stage2WristWeight;
  //   stage2Weights[10] = 100.0;
  //   stage2Weights[11] = ikParams_.stage2WristWeight;
  //   stage2Weights[12] = ikParams_.stage2WristWeight;
  //   stage2Weights[13] = ikParams_.stage2WristWeight;
  // }

  // 为腕关节添加平滑性代价（仅对腕关节）
  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(stage2Weights.data(), stage2Weights.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();

  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, stage2ReferenceSolution, ik.q());
}

std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage1Result() const { return stage1Result_; }
std::pair<bool, Eigen::VectorXd> TwoStageTorsoIK::getStage2Result() const { return stage2Result_; }

// Forward Kinematics implementation - matching plantIK.cc functionality
std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageTorsoIK::FK(const Eigen::VectorXd& q,
                                                                   const std::string& frameName,
                                                                   int expectedSize) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::FK: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (expectedSize > 0 && q.size() != expectedSize) {
    ROS_ERROR("TwoStageTorsoIK::FK: Joint vector size mismatch. Expected %d, got %zu", expectedSize, q.size());
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
    ROS_ERROR("TwoStageTorsoIK::FK: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> TwoStageTorsoIK::FKElbow(const Eigen::VectorXd& q,
                                                                        const std::string& frameName,
                                                                        int expectedSize) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::FKElbow: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (expectedSize > 0 && q.size() != expectedSize) {
    ROS_ERROR("TwoStageTorsoIK::FKElbow: Joint vector size mismatch. Expected %d, got %zu", expectedSize, q.size());
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
    ROS_ERROR("TwoStageTorsoIK::FKElbow: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

Eigen::MatrixXd TwoStageTorsoIK::getFrameJacobian(const Eigen::VectorXd& q, const std::string& frameName) {
  if (!plant_context_) {
    ROS_ERROR("TwoStageTorsoIK::getHandJacobian: plant_context_ is null");
    return Eigen::MatrixXd::Zero(3, nq_);
  }

  // Set joint positions
  plant_->SetPositions(plant_context_.get(), q);

  // Get the target frame by name
  const drake::multibody::Frame<double>* target_frame = nullptr;
  try {
    target_frame = &plant_->GetFrameByName(frameName);
  } catch (const std::exception& e) {
    ROS_ERROR("TwoStageTorsoIK::getHandJacobian: Failed to get frame '%s': %s", frameName.c_str(), e.what());
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
    ROS_ERROR("TwoStageTorsoIK::getHandJacobian: Exception occurred: %s", e.what());
    return Eigen::MatrixXd::Zero(3, nq_);
  }
}

}  // namespace HighlyDynamic
