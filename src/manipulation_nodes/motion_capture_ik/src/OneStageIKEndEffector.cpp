#include "motion_capture_ik/OneStageIKEndEffector.h"

#include <ros/ros.h>

#include <leju_utils/define.hpp>

namespace HighlyDynamic {
Eigen::Vector3d defaultRelativeEulerLimitZYX = Eigen::Vector3d(0.9 * 1.57, 0.55, 0.55);  // rad
namespace {}                                                                             // namespace

OneStageIKEndEffector::OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const IKSolverConfig& config,
                                             size_t historyBufferSize)
    : OneStageIKEndEffector(plant, ikConstraintFrameNames, [&config, historyBufferSize]() {
        PointTrackIKSolverConfig compatibleConfig(config);
        compatibleConfig.historyBufferSize = static_cast<int>(historyBufferSize);
        return compatibleConfig;
      }()) {}

OneStageIKEndEffector::OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const PointTrackIKSolverConfig& config)
    : BaseIKSolver(plant, ikConstraintFrameNames, config),
      historyBuffer_(config.historyBufferSize),
      pointTrackConfig_(std::make_unique<PointTrackIKSolverConfig>(config)) {
  plant_context_ = plant_->CreateDefaultContext();

  if (nq_ == 14) {
    Eigen::VectorXd initialSolution = Eigen::VectorXd::Zero(nq_);
    // 设置初始引导值
    initialSolution(1) = 0.3;
    initialSolution(2) = -0.3;
    initialSolution(3) = 0.3;
    // 0, 1, 2,  3,  4,  5,  6
    // 7, 8, 9, 10, 11, 12, 13
    initialSolution(8) = -0.3;
    initialSolution(9) = 0.3;
    initialSolution(10) = 0.3;

    // 填充满历史步伐
    IKSolveResult initialResult(initialSolution, std::chrono::milliseconds(0));
    for (size_t i = 0; i < config.historyBufferSize; ++i) {
      historyBuffer_.add(initialResult);
    }
  }
  if (nq_ == 18) {
    Eigen::VectorXd initialSolution = Eigen::VectorXd::Zero(nq_);
    // 设置初始引导值
    initialSolution(1 + 4) = 0.3;
    initialSolution(2 + 4) = -0.3;
    initialSolution(3 + 4) = 0.3;
    // 0, 1, 2,  3,  4,  5,  6
    // 7, 8, 9, 10, 11, 12, 13
    initialSolution(8 + 4) = -0.3;
    initialSolution(9 + 4) = 0.3;
    initialSolution(10 + 4) = 0.3;

    // 填充满历史步伐
    IKSolveResult initialResult(initialSolution, std::chrono::milliseconds(0));
    for (size_t i = 0; i < config.historyBufferSize; ++i) {
      historyBuffer_.add(initialResult);
    }
  }
}

IKSolveResult OneStageIKEndEffector::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                             ArmIdx controlArmIndex,
                                             const Eigen::VectorXd& jointMidValues /*未使用，可传空*/) {
  (void)jointMidValues;
  // printConfigTable();
  if (!plant_context_) {
    ROS_ERROR("OneStageIKEndEffector::solveIK: plant_context_ is null");
    return IKSolveResult(nq_, "plant_context_ is null");
  }

  if (nq_ != 14 && nq_ != 18) {
    return IKSolveResult(nq_, "nq should be 14 or 18");
  }
  // ROS_INFO("OneStageIKEndEffector::solveIK: start solve ik");

  // ################ solve ik ###################
  bool useJointLimit = true;
  drake::multibody::InverseKinematics endEffectorIK(*plant_, useJointLimit);
  initInverseKinematicsSolver(endEffectorIK, SolverType::SNOPT);

  Eigen::VectorXd referenceSolution = getWarmStartSolution();
  // print referenceSolution size
  // ROS_INFO("OneStageIKEndEffector::solveIK: referenceSolution size = %zu", referenceSolution.size());
  if (!historyBuffer_.empty()) {
    Eigen::VectorXd meanSolution = getMeanSolution();
    if (meanSolution.size() == nq_ && meanSolution.norm() > 1e-6) {
      referenceSolution = meanSolution;
    }
  }

  currentReferenceSolution_ = referenceSolution;
  setConstraints(endEffectorIK, PoseConstraintList, controlArmIndex, Eigen::VectorXd::Zero(nq_), referenceSolution);

  auto startTime = std::chrono::high_resolution_clock::now();
  auto ikResult = solveDrakeIK(endEffectorIK, referenceSolution, "SolveEndEffectorIK");
  if (!ikResult.first) {
    ROS_WARN("OneStageIKEndEffector::solveIK: EndEffectorIK solve failed");
    return IKSolveResult(nq_, "EndEffectorIK solve failed");
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  updateLatestSolution(ikResult.second);

  IKSolveResult result(ikResult.second, duration);
  historyBuffer_.add(result);

  return result;
}

void OneStageIKEndEffector::setConstraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex,
                                           const Eigen::VectorXd& initialGuess, /*未使用，可传空*/
                                           const Eigen::VectorXd& referenceSolution) const {
  (void)initialGuess;
  // Use PointTrackIKSolverConfig weights if available, otherwise use defaults
  double eeWeight = pointTrackConfig_ ? pointTrackConfig_->eeTrackingWeight : 4e3;
  double elbowWeight = pointTrackConfig_ ? pointTrackConfig_->elbowTrackingWeight : 4e2;
  double link6Weight = pointTrackConfig_ ? pointTrackConfig_->link6TrackingWeight : 4e3;
  double virtualThumbWeight = pointTrackConfig_ ? pointTrackConfig_->virtualThumbTrackingWeight : 4e3;
  double shoulderWeight = pointTrackConfig_ ? pointTrackConfig_->shoulderTrackingWeight : 4e3;
  double chestWeight = pointTrackConfig_ ? pointTrackConfig_->chestTrackingWeight : 4e3;

  if (nq_ == 18) {
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_CHEST) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position,
                         plant_->GetFrameByName("waist_yaw_link"),
                         Eigen::Vector3d::Zero(),
                         chestWeight * Eigen::Matrix3d::Identity());
    }

    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_SHOULDER) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_SHOULDER].position,
                         plant_->GetFrameByName("zarm_l2_joint_parent"),
                         Eigen::Vector3d::Zero(),
                         shoulderWeight * Eigen::Matrix3d::Identity());
    }
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_SHOULDER) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER].position,
                         plant_->GetFrameByName("zarm_r2_joint_parent"),
                         Eigen::Vector3d::Zero(),
                         shoulderWeight * Eigen::Matrix3d::Identity());
    }
  }

  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    // Add LEFT HAND (End Effector) position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position,
                         plant_->GetFrameByName("zarm_l7_end_effector"),
                         Eigen::Vector3d::Zero(),
                         eeWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(
          1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR");
    }

    // add elbow
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,
                         plant_->GetFrameByName("zarm_l4_link"),
                         Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_LEFT_ELBOW");
    }

    // Add LINK6 position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_LINK6].position,
                         plant_->GetFrameByName("zarm_l6_link"),
                         Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_LEFT_LINK6");
    }

    // Add VIRTUAL THUMB position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position,
                         plant_->GetFrameByName("zarm_l7_virtual_thumb_link"),
                         Eigen::Vector3d::Zero(),
                         virtualThumbWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(
          1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB");
    }
  }

  if (controlArmIndex == ArmIdx::RIGHT || controlArmIndex == ArmIdx::BOTH) {
    // Add RIGHT HAND (End Effector) position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].position,
                         plant_->GetFrameByName("zarm_r7_end_effector"),
                         Eigen::Vector3d::Zero(),
                         eeWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(
          1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR");
    }

    // add elbow
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,
                         plant_->GetFrameByName("zarm_r4_link"),
                         Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_RIGHT_ELBOW");
    }

    // Add LINK6 position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position,
                         plant_->GetFrameByName("zarm_r6_link"),
                         Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_RIGHT_LINK6");
    }

    // Add VIRTUAL THUMB position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position,
                         plant_->GetFrameByName("zarm_r7_virtual_thumb_link"),
                         Eigen::Vector3d::Zero(),
                         virtualThumbWeight * Eigen::Matrix3d::Identity());
    } else {
      ROS_WARN_THROTTLE(
          1.0, "OneStageIKEndEffector::setConstraints: missing POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB");
    }
  }
  // Build joint smoothness weights from config (7 joints per arm, symmetric)
  std::vector<double> jointSmoothWeight(nq_, pointTrackConfig_ ? pointTrackConfig_->jointSmoothWeightDefault : 5e1);

  if (pointTrackConfig_) {
    if (nq_ == 14) {
      // Left arm joints [0-6]
      jointSmoothWeight[0] = pointTrackConfig_->jointSmoothWeight0;
      jointSmoothWeight[1] = pointTrackConfig_->jointSmoothWeight1;
      jointSmoothWeight[2] = pointTrackConfig_->jointSmoothWeight2;
      jointSmoothWeight[3] = pointTrackConfig_->jointSmoothWeight3;
      jointSmoothWeight[4] = pointTrackConfig_->jointSmoothWeight4;
      jointSmoothWeight[5] = pointTrackConfig_->jointSmoothWeight5;
      jointSmoothWeight[6] = pointTrackConfig_->jointSmoothWeight6;

      // Right arm joints [7-13] (symmetric to left arm)
      jointSmoothWeight[7] = pointTrackConfig_->jointSmoothWeight0;
      jointSmoothWeight[8] = pointTrackConfig_->jointSmoothWeight1;
      jointSmoothWeight[9] = pointTrackConfig_->jointSmoothWeight2;
      jointSmoothWeight[10] = pointTrackConfig_->jointSmoothWeight3;
      jointSmoothWeight[11] = pointTrackConfig_->jointSmoothWeight4;
      jointSmoothWeight[12] = pointTrackConfig_->jointSmoothWeight5;
      jointSmoothWeight[13] = pointTrackConfig_->jointSmoothWeight6;
    }

    if (nq_ == 18) {
      // [0~3] 腰部关节
      jointSmoothWeight[0] = pointTrackConfig_->waistSmoothWeight0;
      jointSmoothWeight[1] = pointTrackConfig_->waistSmoothWeight1;
      jointSmoothWeight[2] = pointTrackConfig_->waistSmoothWeight2;
      jointSmoothWeight[3] = pointTrackConfig_->waistSmoothWeight3;

      // [7~10]
      // Left arm joints [0-6]
      jointSmoothWeight[0 + 4] = pointTrackConfig_->jointSmoothWeight0;
      jointSmoothWeight[1 + 4] = pointTrackConfig_->jointSmoothWeight1;
      jointSmoothWeight[2 + 4] = pointTrackConfig_->jointSmoothWeight2;
      jointSmoothWeight[3 + 4] = pointTrackConfig_->jointSmoothWeight3;
      jointSmoothWeight[4 + 4] = pointTrackConfig_->jointSmoothWeight4;
      jointSmoothWeight[5 + 4] = pointTrackConfig_->jointSmoothWeight5;
      jointSmoothWeight[6 + 4] = pointTrackConfig_->jointSmoothWeight6;

      // Right arm joints [7-13] (symmetric to left arm)
      jointSmoothWeight[7 + 4] = pointTrackConfig_->jointSmoothWeight0;
      jointSmoothWeight[8 + 4] = pointTrackConfig_->jointSmoothWeight1;
      jointSmoothWeight[9 + 4] = pointTrackConfig_->jointSmoothWeight2;
      jointSmoothWeight[10 + 4] = pointTrackConfig_->jointSmoothWeight3;
      jointSmoothWeight[11 + 4] = pointTrackConfig_->jointSmoothWeight4;
      jointSmoothWeight[12 + 4] = pointTrackConfig_->jointSmoothWeight5;
      jointSmoothWeight[13 + 4] = pointTrackConfig_->jointSmoothWeight6;
    }
  } else {
    // Keep legacy fallback to avoid behavior regression for legacy constructor path.
    if (nq_ == 14) {
      jointSmoothWeight[3] = 1e1;
      jointSmoothWeight[4] = 1e-3;
      jointSmoothWeight[5] = 1e-3;
      jointSmoothWeight[6] = 1e-3;
      jointSmoothWeight[10] = 1e1;
      jointSmoothWeight[11] = 1e-3;
      jointSmoothWeight[12] = 1e-3;
      jointSmoothWeight[13] = 1e-3;
    } else if (nq_ == 18) {
      jointSmoothWeight[7] = 1e1;
      jointSmoothWeight[8] = 1e-3;
      jointSmoothWeight[9] = 1e-3;
      jointSmoothWeight[10] = 1e-3;
      jointSmoothWeight[14] = 1e1;
      jointSmoothWeight[15] = 1e-3;
      jointSmoothWeight[16] = 1e-3;
      jointSmoothWeight[17] = 1e-3;
    }
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(jointSmoothWeight.data(), jointSmoothWeight.size());
  Eigen::VectorXd referenceSolutionVec = referenceSolution;
  if (nq_ == 18) {
    // referenceSolutionVec.segment(0, 4).setZero();
    // referenceSolutionVec[2] = 0.0;
  }
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolutionVec, ik.q());
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> OneStageIKEndEffector::FK(const Eigen::VectorXd& q,
                                                                         const std::string& frameName) {
  return FK(q, frameName, -1);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> OneStageIKEndEffector::FK(const Eigen::VectorXd& q,
                                                                         const std::string& frameName,
                                                                         int expectedSize) {
  if (!plant_context_) {
    ROS_ERROR("OneStageIKEndEffector::FK: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (expectedSize > 0 && q.size() != expectedSize) {
    ROS_ERROR("OneStageIKEndEffector::FK: Joint vector size mismatch. Expected %d, got %zu", expectedSize, q.size());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (q.size() != nq_) {
    ROS_ERROR("OneStageIKEndEffector::FK: Joint vector size mismatch. Expected %d, got %zu", nq_, q.size());
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
    ROS_ERROR("OneStageIKEndEffector::FK: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> OneStageIKEndEffector::FKElbow(const Eigen::VectorXd& q,
                                                                              const std::string& frameName,
                                                                              int expectedSize) {
  return FK(q, frameName, expectedSize);
}

// ONLY FOR DEBUG
void OneStageIKEndEffector::printConfigTable() const {
  char buffer[256];

  ROS_INFO("+==================================================================================+");
  ROS_INFO("|                    OneStageIKEndEffector Configuration Table                    |");
  ROS_INFO("+==================================================================================+");

  // Print base IKSolverConfig fields
  ROS_INFO("| Base IKSolverConfig:");
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "constraintTolerance", config_.constraintTolerance);
  ROS_INFO("%s", buffer);
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "solverTolerance", config_.solverTolerance);
  ROS_INFO("%s", buffer);
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20d", "maxIterations", config_.maxIterations);
  ROS_INFO("%s", buffer);

  // Convert ArmIdx to string
  std::string armIdxStr;
  switch (config_.controlArmIndex) {
    case ArmIdx::LEFT:
      armIdxStr = "LEFT (0)";
      break;
    case ArmIdx::RIGHT:
      armIdxStr = "RIGHT (1)";
      break;
    case ArmIdx::BOTH:
      armIdxStr = "BOTH (2)";
      break;
    default:
      armIdxStr = "UNKNOWN";
      break;
  }
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20s", "controlArmIndex", armIdxStr.c_str());
  ROS_INFO("%s", buffer);
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20s", "isWeldBaseLink", config_.isWeldBaseLink ? "true" : "false");
  ROS_INFO("%s", buffer);
  snprintf(buffer, sizeof(buffer), "|   %-30s | %-20s", "useJointLimits", config_.useJointLimits ? "true" : "false");
  ROS_INFO("%s", buffer);

  // Print PointTrackIKSolverConfig fields if available
  if (pointTrackConfig_) {
    ROS_INFO("+==================================================================================+");
    ROS_INFO("| PointTrackIKSolverConfig:");
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20d", "historyBufferSize", pointTrackConfig_->historyBufferSize);
    ROS_INFO("%s", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "eeTrackingWeight", pointTrackConfig_->eeTrackingWeight);
    ROS_INFO("%s", buffer);
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "elbowTrackingWeight", pointTrackConfig_->elbowTrackingWeight);
    ROS_INFO("%s", buffer);
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "link6TrackingWeight", pointTrackConfig_->link6TrackingWeight);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e",
             "virtualThumbTrackingWeight",
             pointTrackConfig_->virtualThumbTrackingWeight);
    ROS_INFO("%s", buffer);
    ROS_INFO("+==================================================================================+");
    ROS_INFO("| Joint Smoothness Weights (7 joints per arm, symmetric):");
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e",
             "jointSmoothWeightDefault",
             pointTrackConfig_->jointSmoothWeightDefault);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[0] / Right[7])",
             "jointSmoothWeight0",
             pointTrackConfig_->jointSmoothWeight0);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[1] / Right[8])",
             "jointSmoothWeight1",
             pointTrackConfig_->jointSmoothWeight1);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[2] / Right[9])",
             "jointSmoothWeight2",
             pointTrackConfig_->jointSmoothWeight2);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[3] / Right[10])",
             "jointSmoothWeight3",
             pointTrackConfig_->jointSmoothWeight3);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[4] / Right[11])",
             "jointSmoothWeight4",
             pointTrackConfig_->jointSmoothWeight4);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[5] / Right[12])",
             "jointSmoothWeight5",
             pointTrackConfig_->jointSmoothWeight5);
    ROS_INFO("%s", buffer);
    snprintf(buffer,
             sizeof(buffer),
             "|   %-30s | %-20.6e (Left[6] / Right[13])",
             "jointSmoothWeight6",
             pointTrackConfig_->jointSmoothWeight6);
    ROS_INFO("%s", buffer);

    ROS_INFO("+----------------------------------------------------------------------------------+");
    ROS_INFO("| Waist Joint Smoothness Weights (for nq=18):");
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "waistSmoothWeight0", pointTrackConfig_->waistSmoothWeight0);
    ROS_INFO("%s", buffer);
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "waistSmoothWeight1", pointTrackConfig_->waistSmoothWeight1);
    ROS_INFO("%s", buffer);
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "waistSmoothWeight2", pointTrackConfig_->waistSmoothWeight2);
    ROS_INFO("%s", buffer);
    snprintf(
        buffer, sizeof(buffer), "|   %-30s | %-20.6e", "waistSmoothWeight3", pointTrackConfig_->waistSmoothWeight3);
    ROS_INFO("%s", buffer);
  } else {
    ROS_INFO("+==================================================================================+");
    ROS_INFO("| PointTrackIKSolverConfig: Not available (using default/base config)");
  }

  ROS_INFO("+==================================================================================+");
}

}  // namespace HighlyDynamic
