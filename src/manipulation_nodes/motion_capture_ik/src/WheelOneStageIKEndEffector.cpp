#include "motion_capture_ik/WheelOneStageIKEndEffector.h"

#include <algorithm>
#include <ros/ros.h>

#include <leju_utils/define.hpp>

namespace HighlyDynamic {
Eigen::Vector3d defaultRelativeEulerLimitZYX = Eigen::Vector3d(0.9 * 1.57, 0.55, 0.55);  // rad
namespace {}                                                                             // namespace

WheelOneStageIKEndEffector::WheelOneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const WheelPointTrackIKSolverConfig& config)
    : BaseIKSolver(plant, ikConstraintFrameNames, config),
      historyBuffer_(config.historyBufferSize),
      pointTrackConfig_(std::make_unique<WheelPointTrackIKSolverConfig>(config)) {
  plant_context_ = plant_->CreateDefaultContext();
  ROS_INFO("WheelOneStageIKEndEffector::WheelOneStageIKEndEffector: nq_ = %d", nq_);

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
    WheelIKResultHistoryBuffer::IKMotionState initialState(
        initialResult, Eigen::VectorXd::Zero(nq_), Eigen::VectorXd::Zero(nq_), Eigen::VectorXd::Zero(nq_));
    for (size_t i = 0; i < config.historyBufferSize; ++i) {
      historyBuffer_.add(initialState);
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
    WheelIKResultHistoryBuffer::IKMotionState initialState(
        initialResult, Eigen::VectorXd::Zero(nq_), Eigen::VectorXd::Zero(nq_), Eigen::VectorXd::Zero(nq_));
    for (size_t i = 0; i < config.historyBufferSize; ++i) {
      historyBuffer_.add(initialState);
    }
  }
}

IKSolveResult WheelOneStageIKEndEffector::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                             ArmIdx controlArmIndex,
                                             const Eigen::VectorXd& jointMidValues /*未使用，可传空*/) {
  // printConfigTable();
  if (!plant_context_) {
    ROS_ERROR("WheelOneStageIKEndEffector::solveIK: plant_context_ is null");
    return IKSolveResult(nq_, "plant_context_ is null");
  }

  if (nq_ != 14 && nq_ != 18) {
    return IKSolveResult(nq_, "nq should be 14 or 18");
  }
  // ROS_INFO("WheelOneStageIKEndEffector::solveIK: start solve ik");

  // ################ solve ik ###################
  bool useJointLimit = true;
  drake::multibody::InverseKinematics endEffectorIK(*plant_, useJointLimit);
  initInverseKinematicsSolver(endEffectorIK, SolverType::SNOPT);

  Eigen::VectorXd referenceSolution = getWarmStartSolution();
  // print referenceSolution size
  // ROS_INFO("WheelOneStageIKEndEffector::solveIK: referenceSolution size = %zu", referenceSolution.size());
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
    ROS_WARN("WheelOneStageIKEndEffector::solveIK: EndEffectorIK solve failed");
    return IKSolveResult(nq_, "EndEffectorIK solve failed");
  }

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

  updateLatestSolution(ikResult.second);

  IKSolveResult result(ikResult.second, duration);

  const double dt = (pointTrackConfig_ && pointTrackConfig_->dynamicsDt > 1e-9) ? pointTrackConfig_->dynamicsDt : 0.01;
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd jerk = Eigen::VectorXd::Zero(nq_);

  const auto* prevState = historyBuffer_.latest();
  const auto* pprevState = historyBuffer_.prev();
  const auto* ppprevState = historyBuffer_.pprev();

  if (prevState && prevState->result.solution.size() == nq_) {
    const Eigen::VectorXd& qPrev = prevState->result.solution;
    velocity = (ikResult.second - qPrev) / dt;

    if (pprevState && pprevState->result.solution.size() == nq_) {
      const Eigen::VectorXd& qPprev = pprevState->result.solution;
      acceleration = (ikResult.second - 2.0 * qPrev + qPprev) / (dt * dt);

      if (ppprevState && ppprevState->result.solution.size() == nq_) {
        const Eigen::VectorXd& qPpprev = ppprevState->result.solution;
        jerk = (ikResult.second - 3.0 * qPrev + 3.0 * qPprev - qPpprev) / (dt * dt * dt);
      }
    }
  }

  historyBuffer_.add(WheelIKResultHistoryBuffer::IKMotionState(result, velocity, acceleration, jerk));
  ++validIkUpdateCount_;

  return result;
}

void WheelOneStageIKEndEffector::setConstraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex,
                                           const Eigen::VectorXd& initialGuess, /*未使用，可传空*/
                                           const Eigen::VectorXd& referenceSolution) const {
  // Use WheelPointTrackIKSolverConfig weights if available, otherwise use defaults
  double eeWeight = pointTrackConfig_ ? pointTrackConfig_->eeTrackingWeight : 4e3;
  double elbowWeight = pointTrackConfig_ ? pointTrackConfig_->elbowTrackingWeight : 4e2;
  double link6Weight = pointTrackConfig_ ? pointTrackConfig_->link6TrackingWeight : 4e3;
  double virtualThumbWeight = pointTrackConfig_ ? pointTrackConfig_->virtualThumbTrackingWeight : 4e3;
  double shoulderWeight = pointTrackConfig_ ? pointTrackConfig_->shoulderTrackingWeight : 4e3;
  double chestWeight = pointTrackConfig_ ? pointTrackConfig_->chestTrackingWeight : 4e3;

  // add shoulder position constraint
  // add chest position constraint
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

  if (controlArmIndex == ArmIdx::LEFT || controlArmIndex == ArmIdx::BOTH) {
    // Add LEFT HAND (End Effector) position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].position,
                         plant_->GetFrameByName("zarm_l7_end_effector"),
                         Eigen::Vector3d::Zero(),
                         eeWeight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR"
                << std::endl;
    }

    // add elbow
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position,
                         plant_->GetFrameByName("zarm_l4_link"),
                         Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_LEFT_ELBOW
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_LEFT_ELBOW" << std::endl;
    }

    // Add LINK6 position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_LINK6].position,
                         plant_->GetFrameByName("zarm_l6_link"),
                         Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_LEFT_LINK6
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_LEFT_LINK6" << std::endl;
    }

    // Add VIRTUAL THUMB position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB].position,
                         plant_->GetFrameByName("zarm_l7_virtual_thumb_link"),
                         Eigen::Vector3d::Zero(),
                         virtualThumbWeight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB"
                << std::endl;
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
      // print size err POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR"
                << std::endl;
    }

    // add elbow
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position,
                         plant_->GetFrameByName("zarm_r4_link"),
                         Eigen::Vector3d::Zero(),
                         elbowWeight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_RIGHT_ELBOW
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_RIGHT_ELBOW" << std::endl;
    }

    // Add LINK6 position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_LINK6) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position,
                         plant_->GetFrameByName("zarm_r6_link"),
                         Eigen::Vector3d::Zero(),
                         link6Weight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_RIGHT_LINK6
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_RIGHT_LINK6" << std::endl;
    }

    // Add VIRTUAL THUMB position constraint
    if (PoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB) {
      ik.AddPositionCost(plant_->world_frame(),
                         PoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB].position,
                         plant_->GetFrameByName("zarm_r7_virtual_thumb_link"),
                         Eigen::Vector3d::Zero(),
                         virtualThumbWeight * Eigen::Matrix3d::Identity());
    } else {
      // print size err POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB
      std::cout << "WheelOneStageIKEndEffector::setConstraints: size error POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB"
                << std::endl;
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
    // ros error instead of fallback
    ROS_ERROR("WheelOneStageIKEndEffector::setConstraints: pointTrackConfig_ is null");
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(jointSmoothWeight.data(), jointSmoothWeight.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, ik.q());
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> WheelOneStageIKEndEffector::FK(const Eigen::VectorXd& q,
                                                                         const std::string& frameName) {
  if (!plant_context_) {
    ROS_ERROR("WheelOneStageIKEndEffector::FK: plant_context_ is null");
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }

  if (q.size() != nq_) {
    ROS_ERROR("WheelOneStageIKEndEffector::FK: Joint vector size mismatch. Expected %d, got %zu", nq_, q.size());
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
    ROS_ERROR("WheelOneStageIKEndEffector::FK: Exception occurred: %s", e.what());
    return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  }
}

// ONLY FOR DEBUG
void WheelOneStageIKEndEffector::printConfigTable() const {
  char buffer[256];

  ROS_INFO("+==================================================================================+");
  ROS_INFO("|                    WheelOneStageIKEndEffector Configuration Table                    |");
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

  // Print WheelPointTrackIKSolverConfig fields if available
  if (pointTrackConfig_) {
    ROS_INFO("+==================================================================================+");
    ROS_INFO("| WheelPointTrackIKSolverConfig:");
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20d", "historyBufferSize", pointTrackConfig_->historyBufferSize);
    ROS_INFO("%s", buffer);
    snprintf(buffer, sizeof(buffer), "|   %-30s | %-20.6e", "dynamicsDt", pointTrackConfig_->dynamicsDt);
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
    ROS_INFO("| WheelPointTrackIKSolverConfig: Not available (using default/base config)");
  }

  ROS_INFO("+==================================================================================+");
}

}  // namespace HighlyDynamic
