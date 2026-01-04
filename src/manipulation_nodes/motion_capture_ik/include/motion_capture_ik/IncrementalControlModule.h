#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <tuple>
#include <utility>
#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>

namespace HighlyDynamic {

class IncrementalPoseResult {
 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool isValid_ = false;
  // ##############################################################position##############################################################
  // ##############################################################position##############################################################
  Eigen::Vector3d robotLeftHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanLeftHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d robotLeftHandDeltaPos_ = Eigen::Vector3d::Zero();  //δ
  Eigen::Vector3d dotLeftHandDeltaPos_ = Eigen::Vector3d::Zero();    // dδ/dt

  Eigen::Vector3d robotRightHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanRightHandPosAnchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d robotRightHandDeltaPos_ = Eigen::Vector3d::Zero();  //δ
  Eigen::Vector3d dotRightHandDeltaPos_ = Eigen::Vector3d::Zero();    // dδ/dt
  // ##############################################################position##############################################################
  // ##############################################################position##############################################################

  // ##############################################################quat#################################################################
  // ##############################################################quat#################################################################
  Eigen::Quaterniond robotLeftHandQuatAnchor_ = Eigen::Quaterniond::Identity();    // 退出时目标四元数
  Eigen::Quaterniond robotLeftHandQuatSlerpDes_ = Eigen::Quaterniond::Identity();  // Slerp插值后的四元数
  Eigen::Quaterniond robotLeftHandQuatMeasEE_ = Eigen::Quaterniond::Identity();  // 只在进入/退出增量式时更新
  Eigen::Quaterniond robotLeftHandQuatMeasEERealTime_ = Eigen::Quaterniond::Identity();  // 实时更新的ee fk值
  Eigen::Quaterniond robotLeftHandQuatMeasLink4_ = Eigen::Quaterniond::Identity();

  Eigen::Quaterniond humanLeftHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanLeftHandQuatMeas_ = Eigen::Quaterniond::Identity();

  Eigen::Quaterniond robotRightHandQuatAnchor_ = Eigen::Quaterniond::Identity();    // 退出时目标四元数
  Eigen::Quaterniond robotRightHandQuatSlerpDes_ = Eigen::Quaterniond::Identity();  // Slerp插值后的四元数
  Eigen::Quaterniond robotRightHandQuatMeasEE_ = Eigen::Quaterniond::Identity();  // 只在进入/退出增量式时更新
  Eigen::Quaterniond robotRightHandQuatMeasEERealTime_ = Eigen::Quaterniond::Identity();  // 实时更新的ee fk值
  Eigen::Quaterniond robotRightHandQuatMeasLink4_ = Eigen::Quaterniond::Identity();

  Eigen::Quaterniond humanRightHandQuatAnchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanRightHandQuatMeas_ = Eigen::Quaterniond::Identity();
  // ##############################################################quat#################################################################
  // ##############################################################quat#################################################################
  // ==============================================================
  // Python compatible incremental-orientation (Quest3 python node)
  // - q_delta = q_cur * q_anchor^{-1}
  // - if angle(q_delta) < threshold => identity
  // - if significant => update human anchor to q_cur
  // - q_target = q_delta * q_target
  // ==============================================================
  bool usePythonIncrementalOrientation_ = true;
  double pythonOrientationThresholdRad_ = 0.01;

  Eigen::Quaterniond robotLeftHandQuatTarget_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond robotRightHandQuatTarget_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanLeftHandQuatAnchorPython_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond humanRightHandQuatAnchorPython_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond leftHandDeltaQuatLast_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond rightHandDeltaQuatLast_ = Eigen::Quaterniond::Identity();

  double leftSlerpQuatT_ = 0.0;  // Slerp插值因子
  double leftSlerpQuatDt_ = 0.0;
  double rightSlerpQuatT_ = 0.0;
  double rightSlerpQuatDt_ = 0.0;

  Eigen::Vector3d zyxDeltaQuatLimitsW_ = Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 4.0, M_PI / 4.0);  // World Frame
  Eigen::Vector3d zyxFinalQuatLimitsW_ = Eigen::Vector3d(0.475 * M_PI, 0.475 * M_PI, 0.475 * M_PI);   // World Frame
  Eigen::Vector3d zyxFinalQuatLimitsLink4_ = Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6);                   // Link4 Frame

  Eigen::Vector3d zyxLimitsFinal_ = Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);

  // 旋转到常用工作空间附近 → 裁剪 → 恢复到世界坐标系下
  Eigen::Quaterniond yRotOffset_ = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI * 2.0 / 5.0, Eigen::Vector3d::UnitY()));

  Eigen::Vector3d getLatestRobotLeftHandPos() const { return robotLeftHandPosAnchor_ + robotLeftHandDeltaPos_; }
  Eigen::Vector3d getLatestRobotRightHandPos() const { return robotRightHandPosAnchor_ + robotRightHandDeltaPos_; }

  Eigen::Quaterniond getLatestRobotLeftHandQuatAbs() const { return robotLeftHandQuatSlerpDes_; }
  Eigen::Quaterniond getLatestRobotRightHandQuatAbs() const { return robotRightHandQuatSlerpDes_; }

  void saveLastOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      robotLeftHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      robotLeftHandQuatAnchor_ =
          Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
      robotLeftHandQuatTarget_ = robotLeftHandQuatAnchor_;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      robotRightHandPosAnchor_ = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      robotRightHandQuatAnchor_ =
          Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
      robotRightHandQuatTarget_ = robotRightHandQuatAnchor_;
    }
  }

  void resetDelta() {
    robotLeftHandDeltaPos_.setZero();
    robotRightHandDeltaPos_.setZero();
    dotLeftHandDeltaPos_.setZero();
    dotRightHandDeltaPos_.setZero();
    leftHandDeltaQuatLast_.setIdentity();
    rightHandDeltaQuatLast_.setIdentity();
  }

  void resetSlerpFactor() {
    leftSlerpQuatT_ = 0.0;
    leftSlerpQuatDt_ = 0.0;
    rightSlerpQuatT_ = 0.0;
    rightSlerpQuatDt_ = 0.0;
  }

  void resetLeftHandDelta() {
    robotLeftHandDeltaPos_.setZero();
    dotLeftHandDeltaPos_.setZero();
    leftSlerpQuatT_ = 0.0;
    leftSlerpQuatDt_ = 0.0;
    leftHandDeltaQuatLast_.setIdentity();
  }

  void resetRightHandDelta() {
    robotRightHandDeltaPos_.setZero();
    dotRightHandDeltaPos_.setZero();
    rightSlerpQuatT_ = 0.0;
    rightSlerpQuatDt_ = 0.0;
    rightHandDeltaQuatLast_.setIdentity();
  }

  void slerpQuat(const Eigen::Quaterniond& leftHandQuat,
                 const Eigen::Quaterniond& rightHandQuat,
                 bool isLeftActive,
                 bool isRightActive) {
    if (isLeftActive) {
      const Eigen::Quaterniond anchor = robotLeftHandQuatAnchor_;
      robotLeftHandQuatSlerpDes_ = anchor.slerp(leftSlerpQuatT_, leftHandQuat).normalized();
      if (!usePythonIncrementalOrientation_) {
        robotLeftHandQuatTarget_ = robotLeftHandQuatSlerpDes_;
      }
    }
    if (isRightActive) {
      const Eigen::Quaterniond anchor = robotRightHandQuatAnchor_;
      robotRightHandQuatSlerpDes_ = anchor.slerp(rightSlerpQuatT_, rightHandQuat).normalized();
      if (!usePythonIncrementalOrientation_) {
        robotRightHandQuatTarget_ = robotRightHandQuatSlerpDes_;
      }
    }
  }

 public:
  std::pair<Eigen::Quaterniond, Eigen::Quaterniond> getLatestRobotLeftHandQuatInc(bool smoothRotation = true) const {
    (void)smoothRotation;

    Eigen::Quaterniond qRobotTarget = robotLeftHandQuatTarget_;

    qRobotTarget = robotLeftHandQuatMeasEERealTime_.conjugate() * qRobotTarget;
    qRobotTarget = robotLeftHandQuatMeasEERealTime_ * limitQuaternionAngleEulerZYX(qRobotTarget, zyxLimitsFinal_);

    return std::make_pair(qRobotTarget, leftHandDeltaQuatLast_);
  }

  std::pair<Eigen::Quaterniond, Eigen::Quaterniond> getLatestRobotRightHandQuatInc(bool smoothRotation = true) const {
    Eigen::Quaterniond qRobotTarget = robotRightHandQuatTarget_;

    qRobotTarget = robotRightHandQuatMeasEERealTime_.conjugate() * qRobotTarget;
    // Keep symmetric with left hand in python-incremental mode:
    // clip relative rotation in end-effector frame with the same limits to avoid right-hand under-tracking.
    qRobotTarget = robotRightHandQuatMeasEERealTime_ * limitQuaternionAngleEulerZYX(qRobotTarget, zyxLimitsFinal_);

    return std::make_pair(qRobotTarget, rightHandDeltaQuatLast_);
  }

  std::tuple<Eigen::Quaterniond, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d> getLatestIncrementalHandPose(
      bool incrementalPos = true,
      bool incrementalQuat = false,
      bool smoothRotation = true) const {
    if (incrementalQuat) {
      return std::make_tuple(getLatestRobotLeftHandQuatInc(smoothRotation).first,
                             getLatestRobotRightHandQuatInc(smoothRotation).first,
                             getLatestRobotLeftHandPos(),
                             getLatestRobotRightHandPos());
    } else {
      return std::make_tuple(getLatestRobotLeftHandQuatAbs(),
                             getLatestRobotRightHandQuatAbs(),
                             getLatestRobotLeftHandPos(),
                             getLatestRobotRightHandPos());
    }
  }

  bool isValid() const { return isValid_; }
  Eigen::Vector3d getLeftAnchorPos() const { return humanLeftHandPosAnchor_; }
  Eigen::Vector3d getRightAnchorPos() const { return humanRightHandPosAnchor_; }

  // 获取机器人手部的 anchor 姿态
  Eigen::Vector3d getRobotLeftHandAnchorPos() const { return robotLeftHandPosAnchor_; }
  Eigen::Vector3d getRobotRightHandAnchorPos() const { return robotRightHandPosAnchor_; }
  Eigen::Quaterniond getRobotLeftHandAnchorQuat() const { return robotLeftHandQuatAnchor_; }
  Eigen::Quaterniond getRobotRightHandAnchorQuat() const { return robotRightHandQuatAnchor_; }

  friend class IncrementalControlModule;
};

struct IncrementalControlConfig {
  double fhanR = 900.0;                                         // FHAN跟踪微分器加速度约束参数
  double fhanKh0 = 6.0;                                         // FHAN跟踪微分器平滑系数
  Eigen::Vector3d deltaScale = Eigen::Vector3d(1.0, 1.0, 1.0);  // VR增量缩放参数（x, y, z三轴独立）
  double maxPosDiff = 0.45;                                     // 最大位置差异阈值
  double armMoveThreshold = 0.01;                               // 手臂移动检测阈值
  double publishRate = 100.0;                                   // 发布频率
  bool usePythonIncrementalOrientation = true;                  // 姿态增量使用Quest3 python版本逻辑
  double pythonOrientationThresholdRad = 0.01;                  // Quest3 python: orientation_threshold
  Eigen::Vector3d zyxLimitsFinal =
      Eigen::Vector3d(0.95 * M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);  // 最终四元数角度限制（ZYX欧拉角）
  Eigen::Vector3d zyxLimitsEE =
      Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);  // 相对end-effector的四元数角度限制（ZYX欧拉角）
  Eigen::Vector3d zyxLimitsLink4 = Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6);  // 相对link4的四元数角度限制（ZYX欧拉角）
};

struct HandStatus {
  bool activated = false;                                               // 手臂是否激活
  bool moving = false;                                                  // 手臂是否已移动
  Eigen::Vector3d lastPosition = Eigen::Vector3d::Zero();               // 上一帧手部位置
  Eigen::Quaterniond lastOrientation = Eigen::Quaterniond::Identity();  // 上一帧手部姿态
  bool isFirstFrame = true;           // 是否为第一帧数据（用于移动检测）
  double lastMovementDistance = 0.0;  // 最后一次检测到的移动距离

  void ready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    activated = true;
    moving = false;
    lastPosition = position;
    lastOrientation = orientation;
    isFirstFrame = true;  // 重置为第一帧，准备开始移动检测
  }

  void unready(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    activated = false;
    moving = false;
    lastPosition = position;
    lastOrientation = orientation;
    isFirstFrame = true;  // 重置为第一帧
  }

  bool detectMovement(const Eigen::Vector3d& currentPosition, double threshold = 0.01) {
    if (moving) {
      return true;
    }

    if (isFirstFrame) {
      lastPosition = currentPosition;
      isFirstFrame = false;
      return false;
    }

    // 计算手部位置的norm误差
    double normError = (currentPosition - lastPosition).norm();
    lastMovementDistance = normError;  // 保存移动距离，供外部使用

    // 更新上一帧位置
    lastPosition = currentPosition;

    // 检查是否超过阈值
    if (normError > threshold) {
      moving = true;
      return true;
    }

    return false;
  }

  void reset() {
    moving = false;
    isFirstFrame = true;
    lastPosition.setZero();
    lastOrientation = Eigen::Quaterniond::Identity();
    lastMovementDistance = 0.0;
  }
};

class IncrementalControlModule {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit IncrementalControlModule(const IncrementalControlConfig& config = IncrementalControlConfig{});
  ~IncrementalControlModule() = default;

  IncrementalControlModule(const IncrementalControlModule&) = delete;
  IncrementalControlModule& operator=(const IncrementalControlModule&) = delete;

  // 左右手独立进入增量模式接口
  void enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                   const std::vector<PoseData>& latestPoseConstraintList,
                                   const Eigen::Vector3d& pEndEffector,
                                   const Eigen::Quaterniond& qEndEffector,
                                   const Eigen::Quaterniond& qLink4);

  void enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                    const std::vector<PoseData>& latestPoseConstraintList,
                                    const Eigen::Vector3d& pEndEffector,
                                    const Eigen::Quaterniond& qEndEffector,
                                    const Eigen::Quaterniond& qLink4);

  void exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                  const std::vector<PoseData>& latestPoseConstraintList,
                                  const Eigen::Vector3d& pEndEffector,
                                  const Eigen::Quaterniond& qEndEffector,
                                  const Eigen::Quaterniond& qLink4);

  void exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                   const std::vector<PoseData>& latestPoseConstraintList,
                                   const Eigen::Vector3d& pEndEffector,
                                   const Eigen::Quaterniond& qEndEffector,
                                   const Eigen::Quaterniond& qLink4);

  IncrementalPoseResult computeIncrementalPose(
      const ArmPose& vrLeftPose,
      const ArmPose& vrRightPose,
      bool isLeftActive = true,
      bool isRightActive = true,
      const Eigen::Quaterniond& qLeftEndEffector = Eigen::Quaterniond::Identity(),
      const Eigen::Quaterniond& qRightEndEffector = Eigen::Quaterniond::Identity());

  // 左右手独立计算增量位姿接口
  IncrementalPoseResult computeIncrementalPoseLeftArm(
      const ArmPose& vrLeftPose,
      bool isLeftActive = true,
      const Eigen::Quaterniond& qLeftEndEffector = Eigen::Quaterniond::Identity());

  IncrementalPoseResult computeIncrementalPoseRightArm(
      const ArmPose& vrRightPose,
      bool isRightActive = true,
      const Eigen::Quaterniond& qRightEndEffector = Eigen::Quaterniond::Identity());

  IncrementalPoseResult getLatestIncrementalResult() const;

  // 读取最新 anchor 姿态的接口函数
  Eigen::Vector3d getRobotLeftHandAnchorPos() const;
  Eigen::Vector3d getRobotRightHandAnchorPos() const;
  Eigen::Quaterniond getRobotLeftHandAnchorQuat() const;
  Eigen::Quaterniond getRobotRightHandAnchorQuat() const;

  void updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                               const std::vector<PoseData>& latestPoseConstraintList,
                               const Eigen::Vector3d& pEndEffector = Eigen::Vector3d::Zero(),
                               const Eigen::Quaterniond& qEndEffector = Eigen::Quaterniond::Identity(),
                               const Eigen::Quaterniond& qLink4 = Eigen::Quaterniond::Identity());
  void updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                const std::vector<PoseData>& latestPoseConstraintList,
                                const Eigen::Vector3d& pEndEffector = Eigen::Vector3d::Zero(),
                                const Eigen::Quaterniond& qEndEffector = Eigen::Quaterniond::Identity(),
                                const Eigen::Quaterniond& qLink4 = Eigen::Quaterniond::Identity());

  // 设定手部姿态种子，用于首次进入增量模式的 slerp 起点
  void setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed,
                        const Eigen::Quaterniond& rightHandQuatSeed,
                        bool isIncremetalOrientation = false);

  bool detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos);
  bool detectRightArmMove(const Eigen::Vector3d& currentRightHandPos);

  bool shouldEnterIncrementalModeLeftArm(bool isLeftGrip) const;
  bool shouldEnterIncrementalModeRightArm(bool isRightGrip) const;

  bool shouldExitIncrementalMode(bool isLeftGrip, bool isRightGrip) const;
  bool shouldExitIncrementalModeLeftArm(bool isLeftArmCtrlModeActive) const;
  bool shouldExitIncrementalModeRightArm(bool isRightArmCtrlModeActive) const;

  bool isIncrementalMode() const;
  bool isIncrementalModeLeftArm() const;
  bool isIncrementalModeRightArm() const;

  bool hasLeftArmMoved() const;
  bool hasRightArmMoved() const;

  void updateConfig(const IncrementalControlConfig& config);
  const IncrementalControlConfig& getConfig() const;

  void reset();

 private:
  IncrementalPoseResult result_;

  IncrementalControlConfig config_;
  bool initialized_ = false;

  HandStatus leftHandStatus_;   // 左手状态
  HandStatus rightHandStatus_;  // 右手状态

  mutable std::mutex stateMutex_;

  // 几乎不通过ee限制anchor， 避免打断连续性
  Eigen::Vector3d zyxLimitsEE_ = Eigen::Vector3d(M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);

  // 相对link4设置阈值限制
  Eigen::Vector3d zyxLimitsLink4_ = Eigen::Vector3d(M_PI / 2.0, 0.6, 0.6);

  Eigen::Vector3d defaultLeftHandPos_;   // 左臂默认位置锚点
  Eigen::Vector3d defaultRightHandPos_;  // 右臂默认位置锚点
  double posAnchorZeroThreshold_;        // 位置锚点零值检测阈值
  double slerpQuatFactorThreshold_;      // slerpQuatFactor检查阈值

  void computeFhanFiltering(const ArmPose& vrLeftPose,
                            const ArmPose& vrRightPose,
                            bool isLeftActive,
                            bool isRightActive,
                            const double slerpQuatFactor = 1.0);

  void updateLastOnExit(const std::vector<PoseData>& latestPoseConstraintList);
  void resetDelta();
  void resetSlerpFactor();
};

}  // namespace HighlyDynamic
