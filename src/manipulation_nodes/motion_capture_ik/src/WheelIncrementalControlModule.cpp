#include "motion_capture_ik/WheelIncrementalControlModule.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace HighlyDynamic {

namespace {
inline double clampToUnit(double x) {
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}

inline double quaternionAngleRad(const Eigen::Quaterniond& qIn) {
  const Eigen::Quaterniond q = qIn.normalized();
  const double wAbs = std::abs(q.w());
  return 2.0 * std::acos(clampToUnit(wAbs));
}

inline Eigen::VectorXd vec3ToXd(const Eigen::Vector3d& p) {
  Eigen::VectorXd x(3);
  x << p.x(), p.y(), p.z();
  return x;
}

void resizeLowpassState3(Eigen::VectorXd& x1,
                         Eigen::VectorXd& x2,
                         Eigen::VectorXd& y1,
                         Eigen::VectorXd& y2) {
  x1.resize(3);
  x2.resize(3);
  y1.resize(3);
  y2.resize(3);
  x1.setZero();
  x2.setZero();
  y1.setZero();
  y2.setZero();
}

void resizeLowpassState1(Eigen::VectorXd& x1,
                         Eigen::VectorXd& x2,
                         Eigen::VectorXd& y1,
                         Eigen::VectorXd& y2) {
  x1.resize(1);
  x2.resize(1);
  y1.resize(1);
  y2.resize(1);
  x1.setZero();
  x2.setZero();
  y1.setZero();
  y2.setZero();
}
}  // namespace

WheelIncrementalControlModule::LowpassBiquadCoeff WheelIncrementalControlModule::makeSecondOrderLowpassCoeff(double cutoffHz,
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

Eigen::VectorXd WheelIncrementalControlModule::applySecondOrderLowpassVec(const Eigen::VectorXd& x,
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

void WheelIncrementalControlModule::filterAssignPosDelta(const Eigen::Vector3d& rawDelta,
                                                    Eigen::Vector3d& outDelta,
                                                    Eigen::Vector3d& outDot,
                                                    Eigen::VectorXd& lpX1,
                                                    Eigen::VectorXd& lpX2,
                                                    Eigen::VectorXd& lpY1,
                                                    Eigen::VectorXd& lpY2,
                                                    Eigen::Vector3d& prevFiltered,
                                                    bool& hasPrev) {
  const double dt = 1.0 / std::max(config_.publishRate, 1.0);
  Eigen::Vector3d out;
  if (posLpCoeff_.enabled) {
    Eigen::VectorXd rawV(3);
    rawV << rawDelta.x(), rawDelta.y(), rawDelta.z();
    const Eigen::VectorXd filt = applySecondOrderLowpassVec(rawV, posLpCoeff_, lpX1, lpX2, lpY1, lpY2);
    out = Eigen::Vector3d(filt[0], filt[1], filt[2]);
  } else {
    out = rawDelta;
  }
  outDelta = out;
  if (hasPrev) {
    outDot = (outDelta - prevFiltered) / dt;
  } else {
    outDot.setZero();
    hasPrev = true;
  }
  prevFiltered = outDelta;
}

void WheelIncrementalControlModule::filterAssignSlerp(double rawT,
                                                 double& outT,
                                                 double& outDt,
                                                 Eigen::VectorXd& lpX1,
                                                 Eigen::VectorXd& lpX2,
                                                 Eigen::VectorXd& lpY1,
                                                 Eigen::VectorXd& lpY2,
                                                 double& prevFiltered,
                                                 bool& hasPrev) {
  const double dt = 1.0 / std::max(config_.publishRate, 1.0);
  double out = rawT;
  if (oriLpCoeff_.enabled) {
    Eigen::VectorXd rawV(1);
    rawV[0] = rawT;
    const Eigen::VectorXd filt = applySecondOrderLowpassVec(rawV, oriLpCoeff_, lpX1, lpX2, lpY1, lpY2);
    out = filt[0];
  }
  outT = clampToUnit(out);
  if (hasPrev) {
    outDt = (outT - prevFiltered) / dt;
  } else {
    outDt = 0.0;
    hasPrev = true;
  }
  prevFiltered = outT;
}

// ==================== WheelIncrementalControlModule Implementation ====================

WheelIncrementalControlModule::WheelIncrementalControlModule(const IncrementalControlConfig& config)
    : result_(),
      config_(config),
      initialized_(true),
      leftHandStatus_(),
      rightHandStatus_(),
      defaultLeftHandPos_(0.05, 0.32, -0.05),
      defaultRightHandPos_(0.05, -0.32, -0.05),
      posAnchorZeroThreshold_(1e-3),
      slerpQuatFactorThreshold_(1e-6),
      zyxLimitsEE_(config.zyxLimitsEE),
      zyxLimitsLink4_(config.zyxLimitsLink4) {
  // Propagate python-compatible orientation config into result_ (stored inside WheelIncrementalPoseResult)
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
  result_.zyxLimitsFinal_ = config_.zyxLimitsFinal;
  initializeLowpassFiltersLocked();
  ROS_INFO("[WheelIncrementalControlModule] Module initialized successfully");
}

WheelIncrementalControlModule::~WheelIncrementalControlModule() = default;

// ==================== 状态机相关方法（原 IncrementalModeStateMachine） ====================

bool WheelIncrementalControlModule::shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (leftHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isLeftGrip;
}

bool WheelIncrementalControlModule::shouldEnterIncrementalModeRightArm(bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  if (rightHandStatus_.activated) return false;  // 已经在增量模式，不需要重复进入
  return isRightGrip;
}

bool WheelIncrementalControlModule::shouldExitIncrementalMode(bool isLeftGrip, bool isRightGrip) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated && !rightHandStatus_.activated) return false;  // 不是增量模式，无需重复执行退出
  return !isLeftGrip && !isRightGrip;
}

bool WheelIncrementalControlModule::isIncrementalMode() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && (leftHandStatus_.activated || rightHandStatus_.activated);
}

bool WheelIncrementalControlModule::isIncrementalModeChest() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && chestActivated_;
}

void WheelIncrementalControlModule::enterIncrementalModeChest(const Eigen::Vector3d& humanChestPos,
                                                         const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }
  if (chestActivated_) {
    // chest 已处于增量激活状态（通常由于另一只手已进入增量模式），不重复激活、不重置 anchor
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via chest");
  chestActivated_ = true;

  // human anchor
  result_.humanChestPosAnchor_ = humanChestPos;

  // robot anchor (waist_yaw_link target in PoseConstraintList)
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_CHEST) {
    result_.robotChestPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position;
  } else {
    result_.robotChestPosAnchor_.setZero();
  }

  // reset chest delta
  result_.robotChestDeltaPos_.setZero();
  result_.dotChestDeltaPos_.setZero();
  resetChestFilterLocked();
}

void WheelIncrementalControlModule::updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                                                       const std::vector<PoseData>& latestPoseConstraintList,
                                                       const Eigen::Vector3d& pEndEffector,
                                                       const Eigen::Quaterniond& qEndEffector,
                                                       const Eigen::Quaterniond& qLink4) {
  (void)pEndEffector;
  // humanAnchor
  result_.humanLeftHandPosAnchor_ = vrLeftPose.position;
  result_.humanLeftHandQuatAnchor_ = vrLeftPose.quaternion.normalized();
  result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  // Python compatible: initialize human anchor used for per-step delta computation
  result_.humanLeftHandQuatAnchorPython_ = result_.humanLeftHandQuatAnchor_;

  // robotAnchor
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    // result_.robotLeftHandQuatAnchor = qEndEffector.normalized();
    // result_.robotLeftHandQuatAnchor = vrLeftPose.quaternion.normalized();

    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    // 1) 相对 end-effector 的限幅（保持现有逻辑）
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;

    // 2) 额外增加：相对 link4 的限幅（zyx = [pi/2, 0.6, 0.6]）
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotLeftHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    // Python compatible: incremental target seed should be the current target (pose constraint), not qEndEffector.
    // This matches quest3_node_incremental.py where the target quaternion is updated incrementally each frame.
    result_.robotLeftHandQuatTarget_ = qTargetQuatAnchor;

    result_.robotLeftHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
    if (result_.robotLeftHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotLeftHandPosAnchor_ = defaultLeftHandPos_;
    }
  }
  result_.robotLeftHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotLeftHandQuatMeasEERealTime_ = qEndEffector.normalized();  // 初始化实时更新的ee fk值
  result_.robotLeftHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetLeftHandDelta();
  resetLeftHandFilterLocked();
  resetLeftSlerpFilterLocked();
}

void WheelIncrementalControlModule::updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                                        const std::vector<PoseData>& latestPoseConstraintList,
                                                        const Eigen::Vector3d& pEndEffector,
                                                        const Eigen::Quaterniond& qEndEffector,
                                                        const Eigen::Quaterniond& qLink4) {
  (void)pEndEffector;
  // humanAnchor
  result_.humanRightHandPosAnchor_ = vrRightPose.position;
  result_.humanRightHandQuatAnchor_ = vrRightPose.quaternion.normalized();
  result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  // Python compatible: initialize human anchor used for per-step delta computation
  result_.humanRightHandQuatAnchorPython_ = result_.humanRightHandQuatAnchor_;

  // robotAnchor
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    // result_.robotRightHandQuatAnchor = qEndEffector.normalized();
    // result_.robotRightHandQuatAnchor = vrRightPose.quaternion.normalized();

    Eigen::Quaterniond qTargetQuatAnchor =
        Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    // 1) 相对 end-effector 的限幅（与左手对称一致）
    Eigen::Quaterniond qTargetQuatAnchorRelEE = qEndEffector.conjugate() * qTargetQuatAnchor;
    qTargetQuatAnchorRelEE = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelEE, zyxLimitsEE_);
    Eigen::Quaterniond qTargetQuatAnchorLimited = qEndEffector * qTargetQuatAnchorRelEE;

    // 2) 额外增加：相对 link4 的限幅（zyx = [pi/2, 0.6, 0.6]），与左手对称一致
    Eigen::Quaterniond qTargetQuatAnchorRelLink4 = qLink4.conjugate() * qTargetQuatAnchorLimited;
    qTargetQuatAnchorRelLink4 = limitQuaternionAngleEulerZYX(qTargetQuatAnchorRelLink4, zyxLimitsLink4_);
    result_.robotRightHandQuatAnchor_ = (qLink4 * qTargetQuatAnchorRelLink4).normalized();
    // Python compatible: incremental target seed should be the current target (pose constraint), not qEndEffector.
    result_.robotRightHandQuatTarget_ = qTargetQuatAnchor;

    result_.robotRightHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
    if (result_.robotRightHandPosAnchor_.norm() < posAnchorZeroThreshold_) {
      result_.robotRightHandPosAnchor_ = defaultRightHandPos_;
    }
  }

  result_.robotRightHandQuatMeasEE_ = qEndEffector.normalized();
  result_.robotRightHandQuatMeasEERealTime_ = qEndEffector.normalized();  // 初始化实时更新的ee fk值
  result_.robotRightHandQuatMeasLink4_ = qLink4.normalized();
  result_.resetRightHandDelta();
  resetRightHandFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
  result_.saveLastOnExit(latestPoseConstraintList);
}

void WheelIncrementalControlModule::updateChestAnchorOnExit(const Eigen::Vector3d& humanChestPos,
                                                       const std::vector<PoseData>& latestPoseConstraintList) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot update chest anchor: module not initialized");
    return;
  }
  result_.humanChestPosAnchor_ = humanChestPos;
  if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_CHEST) {
    result_.robotChestPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_CHEST].position;
  } else {
    result_.robotChestPosAnchor_.setZero();
  }
  result_.robotChestDeltaPos_.setZero();
  result_.dotChestDeltaPos_.setZero();
  resetChestFilterLocked();
}

void WheelIncrementalControlModule::resetDelta() {
  result_.resetDelta();
  resetChestFilterLocked();
  resetLeftHandFilterLocked();
  resetRightHandFilterLocked();
}

void WheelIncrementalControlModule::resetSlerpFactor() {
  result_.resetSlerpFactor();
  resetLeftSlerpFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::initializeLowpassFiltersLocked() {
  const double dt = 1.0 / std::max(config_.publishRate, 1.0);
  posLpCoeff_ = makeSecondOrderLowpassCoeff(config_.posCutoffHz, dt);
  oriLpCoeff_ = makeSecondOrderLowpassCoeff(config_.orientationCutoffHz, dt);

  resizeLowpassState3(leftHandLpX1_, leftHandLpX2_, leftHandLpY1_, leftHandLpY2_);
  resizeLowpassState3(rightHandLpX1_, rightHandLpX2_, rightHandLpY1_, rightHandLpY2_);
  resizeLowpassState3(chestLpX1_, chestLpX2_, chestLpY1_, chestLpY2_);
  resizeLowpassState1(leftSlerpLpX1_, leftSlerpLpX2_, leftSlerpLpY1_, leftSlerpLpY2_);
  resizeLowpassState1(rightSlerpLpX1_, rightSlerpLpX2_, rightSlerpLpY1_, rightSlerpLpY2_);

  resetChestFilterLocked();
  resetLeftHandFilterLocked();
  resetRightHandFilterLocked();
  resetLeftSlerpFilterLocked();
  resetRightSlerpFilterLocked();
}

void WheelIncrementalControlModule::resetChestFilterLocked() {
  const Eigen::VectorXd z = vec3ToXd(result_.robotChestDeltaPos_);
  chestLpX1_ = chestLpX2_ = chestLpY1_ = chestLpY2_ = z;
  hasPrevChestFilteredDelta_ = false;
}

void WheelIncrementalControlModule::resetLeftHandFilterLocked() {
  const Eigen::VectorXd z = vec3ToXd(result_.robotLeftHandDeltaPos_);
  leftHandLpX1_ = leftHandLpX2_ = leftHandLpY1_ = leftHandLpY2_ = z;
  hasPrevLeftHandFilteredDelta_ = false;
}

void WheelIncrementalControlModule::resetRightHandFilterLocked() {
  const Eigen::VectorXd z = vec3ToXd(result_.robotRightHandDeltaPos_);
  rightHandLpX1_ = rightHandLpX2_ = rightHandLpY1_ = rightHandLpY2_ = z;
  hasPrevRightHandFilteredDelta_ = false;
}

void WheelIncrementalControlModule::resetLeftSlerpFilterLocked() {
  leftSlerpLpX1_ = leftSlerpLpX2_ = leftSlerpLpY1_ = leftSlerpLpY2_ = Eigen::VectorXd::Constant(1, result_.leftSlerpQuatT_);
  hasPrevLeftSlerpFiltered_ = false;
}

void WheelIncrementalControlModule::resetRightSlerpFilterLocked() {
  rightSlerpLpX1_ = rightSlerpLpX2_ = rightSlerpLpY1_ = rightSlerpLpY2_ =
      Eigen::VectorXd::Constant(1, result_.rightSlerpQuatT_);
  hasPrevRightSlerpFiltered_ = false;
}

void WheelIncrementalControlModule::computeIncrementalFiltering(const ArmPose& vrLeftPose,
                                                         const ArmPose& vrRightPose,
                                                         bool isLeftActive,
                                                         bool isRightActive,
                                                         const double slerpQuatFactor) {
  if (slerpQuatFactor - 1.0 > slerpQuatFactorThreshold_) {
    ROS_WARN("[WheelIncrementalControlModule] slerpQuatFactor is not 1.0, it is %f", slerpQuatFactor);
    return;
  }
  if (isLeftActive) {
    filterAssignSlerp(slerpQuatFactor,
                      result_.leftSlerpQuatT_,
                      result_.leftSlerpQuatDt_,
                      leftSlerpLpX1_,
                      leftSlerpLpX2_,
                      leftSlerpLpY1_,
                      leftSlerpLpY2_,
                      prevLeftSlerpFiltered_,
                      hasPrevLeftSlerpFiltered_);
  }
  if (isRightActive) {
    filterAssignSlerp(slerpQuatFactor,
                      result_.rightSlerpQuatT_,
                      result_.rightSlerpQuatDt_,
                      rightSlerpLpX1_,
                      rightSlerpLpX2_,
                      rightSlerpLpY1_,
                      rightSlerpLpY2_,
                      prevRightSlerpFiltered_,
                      hasPrevRightSlerpFiltered_);
  }

  // 左臂：仅激活时更新低通状态
  if (isLeftActive) {
    const Eigen::Vector3d rawLeftPosDelta((vrLeftPose.position[0] - result_.humanLeftHandPosAnchor_[0]) * config_.deltaScale[0],
                                          (vrLeftPose.position[1] - result_.humanLeftHandPosAnchor_[1]) * config_.deltaScale[1],
                                          (vrLeftPose.position[2] - result_.humanLeftHandPosAnchor_[2]) * config_.deltaScale[2]);
    filterAssignPosDelta(rawLeftPosDelta,
                         result_.robotLeftHandDeltaPos_,
                         result_.dotLeftHandDeltaPos_,
                         leftHandLpX1_,
                         leftHandLpX2_,
                         leftHandLpY1_,
                         leftHandLpY2_,
                         prevLeftHandFilteredDelta_,
                         hasPrevLeftHandFilteredDelta_);
  }

  if (isRightActive) {
    const Eigen::Vector3d rawRightPosDelta(
        (vrRightPose.position[0] - result_.humanRightHandPosAnchor_[0]) * config_.deltaScale[0],
        (vrRightPose.position[1] - result_.humanRightHandPosAnchor_[1]) * config_.deltaScale[1],
        (vrRightPose.position[2] - result_.humanRightHandPosAnchor_[2]) * config_.deltaScale[2]);
    filterAssignPosDelta(rawRightPosDelta,
                         result_.robotRightHandDeltaPos_,
                         result_.dotRightHandDeltaPos_,
                         rightHandLpX1_,
                         rightHandLpX2_,
                         rightHandLpY1_,
                         rightHandLpY2_,
                         prevRightHandFilteredDelta_,
                         hasPrevRightHandFilteredDelta_);
  }
}

void WheelIncrementalControlModule::setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                                                const Eigen::Quaterniond& rightHandQuatSeed,
                                                bool isIncremetalOrientation) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!isIncremetalOrientation) {
    result_.robotLeftHandQuatAnchor_ = leftHandQuatSeed.normalized();
    result_.robotRightHandQuatAnchor_ = rightHandQuatSeed.normalized();
  }

  result_.robotLeftHandQuatSlerpDes_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatSlerpDes_ = result_.robotRightHandQuatAnchor_;
  result_.robotLeftHandQuatTarget_ = result_.robotLeftHandQuatAnchor_;
  result_.robotRightHandQuatTarget_ = result_.robotRightHandQuatAnchor_;
}

void WheelIncrementalControlModule::enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via left arm");

  leftHandStatus_.ready(vrLeftPose.position, vrLeftPose.quaternion);

  // 只更新左臂的锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void WheelIncrementalControlModule::enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                            const std::vector<PoseData>& latestPoseConstraintList,
                                                            const Eigen::Vector3d& pEndEffector,
                                                            const Eigen::Quaterniond& qEndEffector,
                                                            const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Module not initialized");
    return;
  }

  // ROS_INFO("[WheelIncrementalControlModule] Entering incremental mode via right arm");

  rightHandStatus_.ready(vrRightPose.position, vrRightPose.quaternion);

  // 只更新右臂的锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);
}

void WheelIncrementalControlModule::exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                                          const std::vector<PoseData>& latestPoseConstraintList,
                                                          const Eigen::Vector3d& pEndEffector,
                                                          const Eigen::Quaterniond& qEndEffector,
                                                          const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新左臂锚点
  updateLeftArmPoseAnchor(vrLeftPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记左臂退出增量模式
  leftHandStatus_.unready(vrLeftPose.position, vrLeftPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    chestActivated_ = false;
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

void WheelIncrementalControlModule::exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                                           const std::vector<PoseData>& latestPoseConstraintList,
                                                           const Eigen::Vector3d& pEndEffector,
                                                           const Eigen::Quaterniond& qEndEffector,
                                                           const Eigen::Quaterniond& qLink4) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    ROS_WARN("[WheelIncrementalControlModule] Cannot exit incremental mode: module not initialized");
    return;
  }

  // 更新右臂锚点
  updateRightArmPoseAnchor(vrRightPose, latestPoseConstraintList, pEndEffector, qEndEffector, qLink4);

  // 标记右臂退出增量模式
  rightHandStatus_.unready(vrRightPose.position, vrRightPose.quaternion);

  if (!leftHandStatus_.activated && !rightHandStatus_.activated) {
    chestActivated_ = false;
    updateLastOnExit(latestPoseConstraintList);
    resetDelta();
    resetSlerpFactor();
  }
}

WheelIncrementalPoseResult WheelIncrementalControlModule::computeIncrementalChestPos(const Eigen::Vector3d& humanChestPos,
                                                                           bool isChestActive) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  WheelIncrementalPoseResult latestIncrementalResult;

  if (!initialized_ || !chestActivated_) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Chest not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  if (!isChestActive) {
    // 不更新滤波状态，保持连续性
    result_.isValid_ = true;
    return result_;
  }

  // 平滑插值（与手部位置增量滤波保持一致）
  Eigen::VectorXd rawChestPosDelta(3);
  rawChestPosDelta(0) = (humanChestPos[0] - result_.humanChestPosAnchor_[0]) * config_.deltaScale[0];
  rawChestPosDelta(1) = (humanChestPos[1] - result_.humanChestPosAnchor_[1]) * config_.deltaScale[1];
  rawChestPosDelta(2) = (humanChestPos[2] - result_.humanChestPosAnchor_[2]) * config_.deltaScale[2];
  filterAssignPosDelta(Eigen::Vector3d(rawChestPosDelta[0], rawChestPosDelta[1], rawChestPosDelta[2]),
                       result_.robotChestDeltaPos_,
                       result_.dotChestDeltaPos_,
                       chestLpX1_,
                       chestLpX2_,
                       chestLpY1_,
                       chestLpY2_,
                       prevChestFilteredDelta_,
                       hasPrevChestFilteredDelta_);

  result_.isValid_ = true;
  return result_;
}

WheelIncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPose(const ArmPose& vrLeftPose,
                                                                       const ArmPose& vrRightPose,
                                                                       bool isLeftActive,
                                                                       bool isRightActive,
                                                                       const Eigen::Quaterniond& qLeftEndEffector,
                                                                       const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  WheelIncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || (!leftHandStatus_.activated && !rightHandStatus_.activated)) {
    // print in red
    // std::cout << "\033[91m[WheelIncrementalControlModule] Not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }
  // print once in green color
  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing incremental pose successfully");

  // 实时更新ee的fk值
  if (isLeftActive && qLeftEndEffector.norm() > 1e-6) {
    result_.robotLeftHandQuatMeasEERealTime_ = qLeftEndEffector.normalized();
  }
  if (isRightActive && qRightEndEffector.norm() > 1e-6) {
    result_.robotRightHandQuatMeasEERealTime_ = qRightEndEffector.normalized();
  }

  // 更新当前姿态
  if (isLeftActive) {
    result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  }
  if (isRightActive) {
    result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  }

  computeIncrementalFiltering(vrLeftPose, vrRightPose, isLeftActive, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, isRightActive);

  // ==================== Python compatible incremental orientation ====================
  // quest3_node_incremental.py:
  //   q_delta = q_current * q_anchor^{-1}
  //   if angle(q_delta) < threshold => identity
  //   if significant => anchor = current
  //   q_target = q_delta * q_target
  if (result_.usePythonIncrementalOrientation_) {
    if (isLeftActive) {
      const Eigen::Quaterniond qCur = result_.humanLeftHandQuatMeas_.normalized();
      const Eigen::Quaterniond qAnchor = result_.humanLeftHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      const double angle = quaternionAngleRad(qDelta);
      if (angle < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanLeftHandQuatAnchorPython_ = qCur;
        // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
        result_.robotLeftHandQuatTarget_ = (qDelta * result_.robotLeftHandQuatTarget_).normalized();
      }
      result_.leftHandDeltaQuatLast_ = qDelta;
    }

    if (isRightActive) {
      const Eigen::Quaterniond qCur = result_.humanRightHandQuatMeas_.normalized();
      const Eigen::Quaterniond qAnchor = result_.humanRightHandQuatAnchorPython_.normalized();
      Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
      const double angle = quaternionAngleRad(qDelta);
      if (angle < result_.pythonOrientationThresholdRad_) {
        qDelta.setIdentity();
      } else {
        result_.humanRightHandQuatAnchorPython_ = qCur;
        // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
        result_.robotRightHandQuatTarget_ = (qDelta * result_.robotRightHandQuatTarget_).normalized();
      }
      result_.rightHandDeltaQuatLast_ = qDelta;
    }
  }

  result_.isValid_ = true;

  return result_;
}

WheelIncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPoseLeftArm(
    const ArmPose& vrLeftPose,
    bool isLeftActive,
    const Eigen::Quaterniond& qLeftEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  WheelIncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !leftHandStatus_.activated) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Left arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing left arm incremental pose successfully");

  // 实时更新ee的fk值
  if (isLeftActive && qLeftEndEffector.norm() > 1e-6) {
    result_.robotLeftHandQuatMeasEERealTime_ = qLeftEndEffector.normalized();
  }

  // 更新左手当前姿态
  if (isLeftActive) {
    result_.humanLeftHandQuatMeas_ = vrLeftPose.quaternion.normalized();
  }

  // 创建虚拟右臂姿态（使用当前result_中的右臂数据）
  ArmPose vrRightPose;
  vrRightPose.position = result_.humanRightHandPosAnchor_;
  vrRightPose.quaternion = result_.humanRightHandQuatMeas_;

  // 计算时只激活左臂
  computeIncrementalFiltering(vrLeftPose, vrRightPose, isLeftActive, false);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, isLeftActive, false);

  if (result_.usePythonIncrementalOrientation_ && isLeftActive) {
    const Eigen::Quaterniond qCur = result_.humanLeftHandQuatMeas_.normalized();
    const Eigen::Quaterniond qAnchor = result_.humanLeftHandQuatAnchorPython_.normalized();
    Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
    const double angle = quaternionAngleRad(qDelta);
    if (angle < result_.pythonOrientationThresholdRad_) {
      qDelta.setIdentity();
    } else {
      result_.humanLeftHandQuatAnchorPython_ = qCur;
      // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
      result_.robotLeftHandQuatTarget_ = (qDelta * result_.robotLeftHandQuatTarget_).normalized();
    }
    result_.leftHandDeltaQuatLast_ = qDelta;
  }

  result_.isValid_ = true;

  return result_;
}

WheelIncrementalPoseResult WheelIncrementalControlModule::computeIncrementalPoseRightArm(
    const ArmPose& vrRightPose,
    bool isRightActive,
    const Eigen::Quaterniond& qRightEndEffector) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  WheelIncrementalPoseResult latestIncrementalResult;
  if (!initialized_ || !rightHandStatus_.activated) {
    std::cout << "\033[91m[WheelIncrementalControlModule] Right arm not in incremental mode\033[0m" << std::endl;
    latestIncrementalResult.isValid_ = false;
    return latestIncrementalResult;
  }

  ROS_INFO_ONCE("[WheelIncrementalControlModule] Computing right arm incremental pose successfully");

  // 实时更新ee的fk值
  if (isRightActive && qRightEndEffector.norm() > 1e-6) {
    result_.robotRightHandQuatMeasEERealTime_ = qRightEndEffector.normalized();
  }

  // 更新右手当前姿态
  if (isRightActive) {
    result_.humanRightHandQuatMeas_ = vrRightPose.quaternion.normalized();
  }

  // 创建虚拟左臂姿态（使用当前result_中的左臂数据）
  ArmPose vrLeftPose;
  vrLeftPose.position = result_.humanLeftHandPosAnchor_;
  vrLeftPose.quaternion = result_.humanLeftHandQuatMeas_;

  // 计算时只激活右臂
  computeIncrementalFiltering(vrLeftPose, vrRightPose, false, isRightActive);
  result_.slerpQuat(vrLeftPose.quaternion, vrRightPose.quaternion, false, isRightActive);

  if (result_.usePythonIncrementalOrientation_ && isRightActive) {
    const Eigen::Quaterniond qCur = result_.humanRightHandQuatMeas_.normalized();
    const Eigen::Quaterniond qAnchor = result_.humanRightHandQuatAnchorPython_.normalized();
    Eigen::Quaterniond qDelta = (qCur * qAnchor.conjugate()).normalized();
    const double angle = quaternionAngleRad(qDelta);
    if (angle < result_.pythonOrientationThresholdRad_) {
      qDelta.setIdentity();
    } else {
      result_.humanRightHandQuatAnchorPython_ = qCur;
      // qDelta = Eigen::Quaterniond::Identity().slerp(1.35, qDelta).normalized();
      result_.robotRightHandQuatTarget_ = (qDelta * result_.robotRightHandQuatTarget_).normalized();
    }
    result_.rightHandDeltaQuatLast_ = qDelta;
  }

  result_.isValid_ = true;

  return result_;
}

bool WheelIncrementalControlModule::detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return leftHandStatus_.detectMovement(currentLeftHandPos, config_.armMoveThreshold);
}

bool WheelIncrementalControlModule::detectRightArmMove(const Eigen::Vector3d& currentRightHandPos) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return false;
  return rightHandStatus_.detectMovement(currentRightHandPos, config_.armMoveThreshold);
}

bool WheelIncrementalControlModule::hasLeftArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.moving;
}

bool WheelIncrementalControlModule::hasRightArmMoved() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.moving;
}

bool WheelIncrementalControlModule::shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!leftHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查左手是否应该退出
  return !isLeftArmCtrlModeActive;
}

bool WheelIncrementalControlModule::shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) return true;
  if (!rightHandStatus_.activated) return false;  // 不在增量模式，无需重复执行退出
  // 检查右手是否应该退出
  return !isRightArmCtrlModeActive;
}

bool WheelIncrementalControlModule::isIncrementalModeLeftArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && leftHandStatus_.activated;
}

bool WheelIncrementalControlModule::isIncrementalModeRightArm() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return initialized_ && rightHandStatus_.activated;
}

void WheelIncrementalControlModule::updateConfig(const IncrementalControlConfig& config) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  config_ = config;
  result_.usePythonIncrementalOrientation_ = config_.usePythonIncrementalOrientation;
  result_.pythonOrientationThresholdRad_ = config_.pythonOrientationThresholdRad;
  initializeLowpassFiltersLocked();
}

WheelIncrementalPoseResult WheelIncrementalControlModule::getLatestIncrementalResult() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  if (!initialized_) {
    WheelIncrementalPoseResult emptyResult;
    emptyResult.isValid_ = false;
    return emptyResult;
  }
  return result_;
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotLeftHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotChestAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotChestAnchorPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getLatestRobotChestPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getLatestRobotChestPos();
}

Eigen::Vector3d WheelIncrementalControlModule::getRobotRightHandAnchorPos() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorPos();
}

Eigen::Quaterniond WheelIncrementalControlModule::getRobotLeftHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotLeftHandAnchorQuat();
}

Eigen::Quaterniond WheelIncrementalControlModule::getRobotRightHandAnchorQuat() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return result_.getRobotRightHandAnchorQuat();
}

const IncrementalControlConfig& WheelIncrementalControlModule::getConfig() const {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return config_;
}

void WheelIncrementalControlModule::reset() {
  std::lock_guard<std::mutex> lock(stateMutex_);

  // 重置增量计算结果
  result_ = WheelIncrementalPoseResult();

  leftHandStatus_.reset();
  rightHandStatus_.reset();
  chestActivated_ = false;
  initializeLowpassFiltersLocked();
}

}  // namespace HighlyDynamic
