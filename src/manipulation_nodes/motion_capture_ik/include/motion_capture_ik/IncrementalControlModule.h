#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <tuple>
#include <leju_utils/define.hpp>
#include <leju_utils/math.hpp>

namespace HighlyDynamic {

// 前向声明
class JoyStickHandler;

/**
 * @brief 增量控制结果结构体
 */
struct IncrementalPoseResult {
  // 手部增量位置和姿态
  Eigen::Vector3d leftHandPosDelta = Eigen::Vector3d::Zero();      //δ
  Eigen::Vector3d rightHandPosDelta = Eigen::Vector3d::Zero();     //δ
  Eigen::Vector3d dotLeftHandPosDelta = Eigen::Vector3d::Zero();   // dδ/dt
  Eigen::Vector3d dotRightHandPosDelta = Eigen::Vector3d::Zero();  // dδ/dt

  Eigen::Vector3d lastTargetLeftHandPosOnExit = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastTargetRightHandPosOnExit = Eigen::Vector3d::Zero();
  Eigen::Vector3d dotLastTargetLeftHandPosOnExit = Eigen::Vector3d::Zero();   // lastTargetLeftHandPosOnExit的速度
  Eigen::Vector3d dotLastTargetRightHandPosOnExit = Eigen::Vector3d::Zero();  // lastTargetRightHandPosOnExit的速度

  // 肘部绝对位置滤波（使用fhan算法平滑绝对位置，而非增量）
  Eigen::Vector3d leftElbowPosFiltered = Eigen::Vector3d::Zero();      // 滤波后的左肘绝对位置
  Eigen::Vector3d rightElbowPosFiltered = Eigen::Vector3d::Zero();     // 滤波后的右肘绝对位置
  Eigen::Vector3d dotLeftElbowPosFiltered = Eigen::Vector3d::Zero();   // 滤波后的左肘速度
  Eigen::Vector3d dotRightElbowPosFiltered = Eigen::Vector3d::Zero();  // 滤波后的右肘速度

  Eigen::Vector3d lastTargetLeftElbowPosOnExit = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastTargetRightElbowPosOnExit = Eigen::Vector3d::Zero();

  //姿态不使用增量，但需要启动时进行平滑插值
  Eigen::Quaterniond lastTargetLeftHandQuatOnExit = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond lastTargetRightHandQuatOnExit = Eigen::Quaterniond::Identity();

  Eigen::Quaterniond latestTargetLeftHandQuatSlerp = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond latestTargetRightHandQuatSlerp = Eigen::Quaterniond::Identity();

  Eigen::Vector3d leftAnchorPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightAnchorPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d leftElbowAnchorPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightElbowAnchorPos = Eigen::Vector3d::Zero();

  double leftSlerpQuat_t_ = 0.0;
  double leftSlerpQuat_dt_ = 0.0;
  double rightSlerpQuat_t_ = 0.0;
  double rightSlerpQuat_dt_ = 0.0;
  bool isValid = false;

  Eigen::Vector3d getLatestIncrementalLeftHandPos() const { return lastTargetLeftHandPosOnExit + leftHandPosDelta; }
  Eigen::Vector3d getLatestIncrementalRightHandPos() const { return lastTargetRightHandPosOnExit + rightHandPosDelta; }
  Eigen::Vector3d getLatestIncrementalLeftElbowPos() const { return leftElbowPosFiltered; }
  Eigen::Vector3d getLatestIncrementalRightElbowPos() const { return rightElbowPosFiltered; }

  Eigen::Quaterniond getLatestIncrementalLeftHandQuat() const { return latestTargetLeftHandQuatSlerp; }
  Eigen::Quaterniond getLatestIncrementalRightHandQuat() const { return latestTargetRightHandQuatSlerp; }

  std::tuple<Eigen::Quaterniond, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d> getLatestIncrementalHandPose()
      const {
    return std::make_tuple(getLatestIncrementalLeftHandQuat(),
                           getLatestIncrementalRightHandQuat(),
                           getLatestIncrementalLeftHandPos(),
                           getLatestIncrementalRightHandPos());
  }

  void saveLastTargetOnExit(const std::vector<PoseData>& latestPoseConstraintList) {
    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
      lastTargetLeftHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position;
      lastTargetLeftHandQuatOnExit =  // 不增量计算，一直持续插值跟踪即可
          Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
    } else {
      std::cout << "\033[91m[IncrementalControlModule] Left hand data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      lastTargetRightHandQuatOnExit =  // 不增量计算，一直持续插值跟踪即可
          Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    } else {
      std::cout << "\033[91m[IncrementalControlModule] Right hand data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      lastTargetLeftElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
    } else {
      std::cout << "\033[91m[IncrementalControlModule] Left elbow data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      lastTargetRightElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
    } else {
      std::cout << "\033[91m[IncrementalControlModule] Right elbow data not available, using default values\033[0m"
                << std::endl;
    }
  }

  void resetDelta() {
    leftHandPosDelta.setZero();
    rightHandPosDelta.setZero();
    dotLeftHandPosDelta.setZero();
    dotRightHandPosDelta.setZero();
    // 重置lastTargetLeftHandPosOnExit和lastTargetRightHandPosOnExit的速度
    dotLastTargetLeftHandPosOnExit.setZero();
    dotLastTargetRightHandPosOnExit.setZero();
    // 手肘使用绝对位置滤波，不需要重置增量，但需要重置滤波状态
    leftElbowPosFiltered = lastTargetLeftElbowPosOnExit;
    rightElbowPosFiltered = lastTargetRightElbowPosOnExit;
    dotLeftElbowPosFiltered.setZero();
    dotRightElbowPosFiltered.setZero();
  }

  void resetSlerpFactor() {
    leftSlerpQuat_t_ = 0.0;
    leftSlerpQuat_dt_ = 0.0;
    rightSlerpQuat_t_ = 0.0;
    rightSlerpQuat_dt_ = 0.0;
  }

  void slerpQuat(const Eigen::Quaterniond& leftHandTargetQuat,
                 const Eigen::Quaterniond& rightHandTargetQuat,
                 bool isLeftActive,
                 bool isRightActive) {
    if (isLeftActive) {
      auto tmpLeftHandQuat = lastTargetLeftHandQuatOnExit;
      latestTargetLeftHandQuatSlerp = tmpLeftHandQuat.slerp(leftSlerpQuat_t_, leftHandTargetQuat).normalized();
    }
    if (isRightActive) {
      auto tmpRightHandQuat = lastTargetRightHandQuatOnExit;
      latestTargetRightHandQuatSlerp = tmpRightHandQuat.slerp(rightSlerpQuat_t_, rightHandTargetQuat).normalized();
    }
  }
};

/**
 * @brief 增量控制参数配置
 */
struct IncrementalControlConfig {
  double fhan_r = 900.0;                                        // FHAN跟踪微分器加速度约束参数
  double fhan_kh0 = 6.0;                                        // FHAN跟踪微分器平滑系数
  Eigen::Vector3d deltaScale = Eigen::Vector3d(1.0, 1.0, 1.0);  // VR增量缩放参数（x, y, z三轴独立）
  double maxPosDiff = 0.45;                                     // 最大位置差异阈值
  double armMoveThreshold = 0.01;                               // 手臂移动检测阈值
  double publishRate = 100.0;                                   // 发布频率

  void print() const {
    // print in green color
    std::cout << "\033[92m"
              << "[IncrementalControlConfig] Incremental mode parameters initialized: fhan_r=" << fhan_r
              << ", fhan_kh0=" << fhan_kh0 << ", deltaScale=[" << deltaScale.x() << ", " << deltaScale.y() << ", "
              << deltaScale.z() << "], maxPosDiff=" << maxPosDiff << ", armMoveThreshold=" << armMoveThreshold
              << "\033[0m" << std::endl;
  }
};

/**
 * @brief 统一的增量控制模块类（取消多层封装）
 */
class IncrementalControlModule {
 public:
  // 构造函数和析构函数
  explicit IncrementalControlModule(std::shared_ptr<JoyStickHandler> joyStickHandler,
                                    const IncrementalControlConfig& config = IncrementalControlConfig{});
  ~IncrementalControlModule() = default;

  // 禁用拷贝和赋值
  IncrementalControlModule(const IncrementalControlModule&) = delete;
  IncrementalControlModule& operator=(const IncrementalControlModule&) = delete;

  // 核心功能接口
  void enterIncrementalMode(const ArmPose& vrLeftPose,
                            const ArmPose& vrRightPose,
                            const ArmPose& vrLeftElbowPose,
                            const ArmPose& vrRightElbowPose,
                            const std::vector<PoseData>& latestPoseConstraintList);

  // 左右手独立进入增量模式接口
  void enterIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                   const ArmPose& vrLeftElbowPose,
                                   const std::vector<PoseData>& latestPoseConstraintList);

  void enterIncrementalModeRightArm(const ArmPose& vrRightPose,
                                    const ArmPose& vrRightElbowPose,
                                    const std::vector<PoseData>& latestPoseConstraintList);

  void exitIncrementalMode(const ArmPose& vrLeftPose,
                           const ArmPose& vrRightPose,
                           const ArmPose& vrLeftElbowPose,
                           const ArmPose& vrRightElbowPose,
                           const std::vector<PoseData>& latestPoseConstraintList);

  // 左右手独立退出增量模式接口
  void exitIncrementalModeLeftArm(const ArmPose& vrLeftPose,
                                  const ArmPose& vrLeftElbowPose,
                                  const std::vector<PoseData>& latestPoseConstraintList);

  void exitIncrementalModeRightArm(const ArmPose& vrRightPose,
                                   const ArmPose& vrRightElbowPose,
                                   const std::vector<PoseData>& latestPoseConstraintList);

  IncrementalPoseResult computeIncrementalPose(const ArmPose& vrLeftPose,
                                               const ArmPose& vrRightPose,
                                               const ArmPose& vrLeftElbowPose,
                                               const ArmPose& vrRightElbowPose,
                                               bool isLeftActive = true,
                                               bool isRightActive = true);

  // 左右手独立计算增量位姿接口
  IncrementalPoseResult computeIncrementalPoseLeftArm(const ArmPose& vrLeftPose,
                                                      const ArmPose& vrLeftElbowPose,
                                                      bool isLeftActive = true);

  IncrementalPoseResult computeIncrementalPoseRightArm(const ArmPose& vrRightPose,
                                                       const ArmPose& vrRightElbowPose,
                                                       bool isRightActive = true);

  IncrementalPoseResult getLatestIncrementalResult() const;

  // 单臂锚点更新（单臂从关闭切换到激活时调用，避免跳变）
  void updateLeftArmAnchor(const ArmPose& vrLeftPose, const Eigen::Vector3d& currentRobotLeftHandPos);
  void updateRightArmAnchor(const ArmPose& vrRightPose, const Eigen::Vector3d& currentRobotRightHandPos);

  // 设定手部姿态种子，用于首次进入增量模式的 slerp 起点
  void setHandQuatSeeds(const Eigen::Quaterniond& leftHandQuatSeed, const Eigen::Quaterniond& rightHandQuatSeed);

  // 人体手臂移动检测
  bool detectHumanArmMove(const Eigen::Vector3d& currentLeftHandPos, const Eigen::Vector3d& currentRightHandPos);
  bool detectLeftArmMove(const Eigen::Vector3d& currentLeftHandPos);
  bool detectRightArmMove(const Eigen::Vector3d& currentRightHandPos);

  // 状态查询接口
  bool shouldEnterIncrementalMode() const;
  bool shouldEnterIncrementalModeLeftArm() const;
  bool shouldEnterIncrementalModeRightArm() const;

  bool shouldExitIncrementalMode() const;
  bool shouldExitIncrementalModeLeftArm() const;
  bool shouldExitIncrementalModeRightArm() const;

  bool isIncrementalMode() const;
  bool isIncrementalModeLeftArm() const;
  bool isIncrementalModeRightArm() const;

  bool hasHumanArmMoved() const;
  bool hasLeftArmMoved() const;
  bool hasRightArmMoved() const;

  void updateConfig(const IncrementalControlConfig& config);
  const IncrementalControlConfig& getConfig() const;

  /**
   * @brief 重置所有内部状态，使模块恢复到初始状态
   */
  void reset();

 private:
  // 原 IncrementalModeStateMachine 的成员
  ControlMode controlMode_ = ControlMode::NONE;
  std::shared_ptr<JoyStickHandler> joyStickHandler_;

  // 原 IncrementalPoseCalculator 的成员
  IncrementalPoseResult result_;

  // 原 IncrementalControlModule 的成员
  HumanArmMoveDetector armMoveDetector_;
  IncrementalControlConfig config_;
  bool initialized_ = false;

  // 左右手独立控制状态
  bool leftArmIncrementalMode_ = false;    // 左臂是否处于增量模式
  bool rightArmIncrementalMode_ = false;   // 右臂是否处于增量模式
  bool leftArmMoved_ = false;              // 左臂是否已移动
  bool rightArmMoved_ = false;             // 右臂是否已移动
  Eigen::Vector3d prevLeftHandPosition_;   // 上一帧左手位置
  Eigen::Vector3d prevRightHandPosition_;  // 上一帧右手位置

  // 互斥锁保护共享状态
  mutable std::mutex stateMutex_;

  // 原 IncrementalPoseCalculator 的私有方法
  void computeFhanFiltering(const ArmPose& vrLeftPose,
                            const ArmPose& vrRightPose,
                            const ArmPose& vrLeftElbowPose,
                            const ArmPose& vrRightElbowPose,
                            bool isLeftActive,
                            bool isRightActive,
                            const double slerpQuatFactor = 1.0);

  void updateHumanPoseAnchor(const ArmPose& vrLeftPose,
                             const ArmPose& vrRightPose,
                             const ArmPose& vrLeftElbowPose,
                             const ArmPose& vrRightElbowPose,
                             const std::vector<PoseData>& latestPoseConstraintList);

  // 左右手独立更新锚点的私有辅助方法
  void updateLeftArmPoseAnchor(const ArmPose& vrLeftPose,
                               const ArmPose& vrLeftElbowPose,
                               const std::vector<PoseData>& latestPoseConstraintList);

  void updateRightArmPoseAnchor(const ArmPose& vrRightPose,
                                const ArmPose& vrRightElbowPose,
                                const std::vector<PoseData>& latestPoseConstraintList);

  void updateLastTargetOnExit(const std::vector<PoseData>& latestPoseConstraintList);
  void resetDelta();
  void resetSlerpFactor();
};

}  // namespace HighlyDynamic
