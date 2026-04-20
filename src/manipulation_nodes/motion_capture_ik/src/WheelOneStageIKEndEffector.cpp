#include "motion_capture_ik/WheelOneStageIKEndEffector.h"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <leju_utils/define.hpp>

namespace HighlyDynamic {
namespace {
inline bool isValidSolution(const Eigen::VectorXd& q, int nq) {
  return q.size() == nq && q.allFinite() && q.norm() > 1e-9;
}
}  // namespace

WheelOneStageIKEndEffector::LowpassBiquadCoeff WheelOneStageIKEndEffector::makeSecondOrderLowpassCoeff(
    double cutoffHz,
    double sampleTime) {
  LowpassBiquadCoeff coeff;
  if (cutoffHz <= 0.0 || sampleTime <= 1e-9) {
    return coeff;
  }

  const double fs = 1.0 / sampleTime;
  const double nyquist = 0.5 * fs;
  const double fc = std::min(cutoffHz, 0.95 * nyquist);
  if (fc <= 1e-6) {
    return coeff;
  }

  constexpr double kSqrt2 = 1.4142135623730951;
  constexpr double kPi = 3.14159265358979323846;
  const double k = std::tan(kPi * fc / fs);
  const double k2 = k * k;
  const double norm = 1.0 / (1.0 + kSqrt2 * k + k2);

  coeff.b0 = k2 * norm;
  coeff.b1 = 2.0 * coeff.b0;
  coeff.b2 = coeff.b0;
  coeff.a1 = 2.0 * (k2 - 1.0) * norm;
  coeff.a2 = (1.0 - kSqrt2 * k + k2) * norm;
  coeff.enabled = true;
  return coeff;
}

Eigen::VectorXd WheelOneStageIKEndEffector::applySecondOrderLowpassVec(const Eigen::VectorXd& x,
                                                                       const LowpassBiquadCoeff& coeff,
                                                                       Eigen::VectorXd& x1,
                                                                       Eigen::VectorXd& x2,
                                                                       Eigen::VectorXd& y1,
                                                                       Eigen::VectorXd& y2) {
  if (!coeff.enabled) {
    return x;
  }

  Eigen::VectorXd y = coeff.b0 * x + coeff.b1 * x1 + coeff.b2 * x2 - coeff.a1 * y1 - coeff.a2 * y2;
  x2 = x1;
  x1 = x;
  y2 = y1;
  y1 = y;
  return y;
}

void WheelOneStageIKEndEffector::initializeRefLowpass() {
  const double baseDt = (pointTrackConfig_ && pointTrackConfig_->dynamicsDt > 1e-9) ? pointTrackConfig_->dynamicsDt : 0.01;
  const double lpfDt =
      (pointTrackConfig_ && pointTrackConfig_->refSecondOrderLpfDt > 1e-9) ? pointTrackConfig_->refSecondOrderLpfDt : baseDt;
  const double cutoffHz = pointTrackConfig_ ? pointTrackConfig_->refSecondOrderLpfCutoffHz : 12.0;
  refLowpassCoeff_ = makeSecondOrderLowpassCoeff(cutoffHz, lpfDt);
  if (!refLowpassCoeff_.enabled) {
    ROS_WARN("WheelOneStageIKEndEffector: invalid ref LPF config (cutoff=%.3f, dt=%.6f), fallback to passthrough",
             cutoffHz,
             lpfDt);
  }

  refLpX1_ = Eigen::VectorXd::Zero(nq_);
  refLpX2_ = Eigen::VectorXd::Zero(nq_);
  refLpY1_ = Eigen::VectorXd::Zero(nq_);
  refLpY2_ = Eigen::VectorXd::Zero(nq_);
  refLowpassLatest_ = Eigen::VectorXd::Zero(nq_);
  hasRefLowpassState_ = false;
}

Eigen::VectorXd WheelOneStageIKEndEffector::applyRefLowpass(const Eigen::VectorXd& input) {
  if (!hasRefLowpassState_) {
    refLpX1_ = input;
    refLpX2_ = input;
    refLpY1_ = input;
    refLpY2_ = input;
    refLowpassLatest_ = input;
    hasRefLowpassState_ = true;
    return input;
  }
  refLowpassLatest_ =
      applySecondOrderLowpassVec(input, refLowpassCoeff_, refLpX1_, refLpX2_, refLpY1_, refLpY2_);
  return refLowpassLatest_;
}

WheelOneStageIKEndEffector::WheelOneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                             const std::vector<std::string>& ikConstraintFrameNames,
                                             const WheelPointTrackIKSolverConfig& config)
    : BaseIKSolver(plant, ikConstraintFrameNames, config),
      historyBuffer_(config.historyBufferSize),
      pointTrackConfig_(std::make_unique<WheelPointTrackIKSolverConfig>(config)) {
  plant_context_ = plant_->CreateDefaultContext();
  initializeRefLowpass();
  initialGuessSeed_ = Eigen::VectorXd::Zero(nq_);
  ROS_INFO("WheelOneStageIKEndEffector::WheelOneStageIKEndEffector: nq_ = %d", nq_);

  if (nq_ == 18) {
    // 设置初始引导值
    initialGuessSeed_(1 + 4) = 0.3;
    initialGuessSeed_(2 + 4) = -0.3;
    initialGuessSeed_(3 + 4) = 0.3;
    // 0, 1, 2,  3,  4,  5,  6
    // 7, 8, 9, 10, 11, 12, 13
    initialGuessSeed_(8 + 4) = -0.3;
    initialGuessSeed_(9 + 4) = 0.3;
    initialGuessSeed_(10 + 4) = 0.3;
  }
}

IKSolveResult WheelOneStageIKEndEffector::solveIK(const std::vector<PoseData>& PoseConstraintList,
                                             ArmIdx controlArmIndex,
                                             const Eigen::VectorXd& jointMidValues /*未使用，可传空*/) {
  (void)jointMidValues;
  if (!plant_context_) {
    ROS_ERROR("WheelOneStageIKEndEffector::solveIK: plant_context_ is null");
    return IKSolveResult(nq_, "plant_context_ is null");
  }

  if (nq_ != 18) {
    return IKSolveResult(nq_, "nq should be 18");
  }
  // ROS_INFO("WheelOneStageIKEndEffector::solveIK: start solve ik");

  // ################ solve ik ###################
  bool useJointLimit = true;
  drake::multibody::InverseKinematics endEffectorIK(*plant_, useJointLimit);
  initInverseKinematicsSolver(endEffectorIK, SolverType::SNOPT);

  Eigen::VectorXd referenceSolution = Eigen::VectorXd::Zero(nq_);
  if (hasRefLowpassState_ && isValidSolution(refLowpassLatest_, nq_)) {
    referenceSolution = refLowpassLatest_;
  } else {
    const Eigen::VectorXd warmStart = getWarmStartSolution();
    if (isValidSolution(warmStart, nq_)) {
      referenceSolution = warmStart;
    } else {
      const auto* latestState = historyBuffer_.latest();
      if (latestState && isValidSolution(latestState->result.solution, nq_)) {
        referenceSolution = latestState->result.solution;
      } else if (isValidSolution(initialGuessSeed_, nq_)) {
        referenceSolution = initialGuessSeed_;
      }
    }
  }

  setConstraints(endEffectorIK, PoseConstraintList, controlArmIndex, Eigen::VectorXd::Zero(nq_), referenceSolution);

  auto startTime = std::chrono::high_resolution_clock::now();
  auto ikResult = solveDrakeIK(endEffectorIK, referenceSolution, "SolveEndEffectorIK");
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  ROS_INFO_THROTTLE(
      1.0,
      "WheelOneStageIKEndEffector latest IK solve time: %ld ms (success=%s)",
      static_cast<long>(duration.count()),
      ikResult.first ? "true" : "false");

  if (!ikResult.first) {
    ROS_WARN("WheelOneStageIKEndEffector::solveIK: EndEffectorIK solve failed");
    return IKSolveResult(nq_, "EndEffectorIK solve failed");
  }

  const Eigen::VectorXd filteredSolution = applyRefLowpass(ikResult.second);
  updateLatestSolution(filteredSolution);
  IKSolveResult result(filteredSolution, duration);

  const double dt = (pointTrackConfig_ && pointTrackConfig_->dynamicsDt > 1e-9) ? pointTrackConfig_->dynamicsDt : 0.01;
  Eigen::VectorXd velocity = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(nq_);
  Eigen::VectorXd jerk = Eigen::VectorXd::Zero(nq_);

  const auto* prevState = historyBuffer_.latest();
  const auto* pprevState = historyBuffer_.prev();
  const auto* ppprevState = historyBuffer_.pprev();

  if (prevState && prevState->result.solution.size() == nq_) {
    const Eigen::VectorXd& qPrev = prevState->result.solution;
    velocity = (filteredSolution - qPrev) / dt;

    if (pprevState && pprevState->result.solution.size() == nq_) {
      const Eigen::VectorXd& qPprev = pprevState->result.solution;
      acceleration = (filteredSolution - 2.0 * qPrev + qPprev) / (dt * dt);

      if (ppprevState && ppprevState->result.solution.size() == nq_) {
        const Eigen::VectorXd& qPpprev = ppprevState->result.solution;
        jerk = (filteredSolution - 3.0 * qPrev + 3.0 * qPprev - qPpprev) / (dt * dt * dt);
      }
    }
  }

  historyBuffer_.add(WheelIKResultHistoryBuffer::IKMotionState(result, velocity, acceleration, jerk));

  return result;
}

void WheelOneStageIKEndEffector::setConstraints(drake::multibody::InverseKinematics& ik,
                                           const std::vector<PoseData>& PoseConstraintList,
                                           ArmIdx controlArmIndex,
                                           const Eigen::VectorXd& initialGuess,
                                           const Eigen::VectorXd& referenceSolution) const {
  (void)initialGuess;
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
  // Build joint smoothness weights from config (waist + 7 joints per arm)
  std::vector<double> jointSmoothWeight(nq_, pointTrackConfig_ ? pointTrackConfig_->jointSmoothWeightDefault : 5e1);

  if (pointTrackConfig_) {
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
    } else {
      ROS_ERROR("WheelOneStageIKEndEffector::setConstraints: only nq=18 is supported, but got %d", nq_);
    }
  } else {
    // ros error instead of fallback
    ROS_ERROR("WheelOneStageIKEndEffector::setConstraints: pointTrackConfig_ is null");
  }

  Eigen::VectorXd weightVec = Eigen::VectorXd::Map(jointSmoothWeight.data(), jointSmoothWeight.size());
  Eigen::MatrixXd W_prev_solution = weightVec.asDiagonal();
  ik.get_mutable_prog()->AddQuadraticErrorCost(W_prev_solution, referenceSolution, ik.q());

  const auto* s1 = historyBuffer_.latest();  // q_{k-1}
  const auto* s2 = historyBuffer_.prev();    // q_{k-2}
  const auto* s3 = historyBuffer_.pprev();   // q_{k-3}

  if (pointTrackConfig_ && pointTrackConfig_->accSmoothWeightDefault > 1e-12 && s1 && s2 &&
      s1->result.solution.size() == nq_ && s2->result.solution.size() == nq_) {
    Eigen::VectorXd qAccTarget = 2.0 * s1->result.solution - s2->result.solution;
    Eigen::MatrixXd W_acc = pointTrackConfig_->accSmoothWeightDefault * Eigen::MatrixXd::Identity(nq_, nq_);
    ik.get_mutable_prog()->AddQuadraticErrorCost(W_acc, qAccTarget, ik.q());
  }

  if (pointTrackConfig_ && pointTrackConfig_->jerkSmoothWeightDefault > 1e-12 && s1 && s2 && s3 &&
      s1->result.solution.size() == nq_ && s2->result.solution.size() == nq_ && s3->result.solution.size() == nq_) {
    Eigen::VectorXd qJerkTarget =
        3.0 * s1->result.solution - 3.0 * s2->result.solution + s3->result.solution;
    Eigen::MatrixXd W_jerk = pointTrackConfig_->jerkSmoothWeightDefault * Eigen::MatrixXd::Identity(nq_, nq_);
    ik.get_mutable_prog()->AddQuadraticErrorCost(W_jerk, qJerkTarget, ik.q());
  }
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

}  // namespace HighlyDynamic
