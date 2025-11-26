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

  // 手部FK位置（基于传感器数据计算）
  Eigen::Vector3d leftHandFkPosition = Eigen::Vector3d::Zero();   // 左手FK位置
  Eigen::Vector3d rightHandFkPosition = Eigen::Vector3d::Zero();  // 右手FK位置

  double slerpQuat_t_ = 0.0;
  double slerpQuat_dt_ = 0.0;
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
      std::cout << "\033[91m[IncrementalPoseCalculator] Left hand data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
      lastTargetRightHandPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position;
      lastTargetRightHandQuatOnExit =  // 不增量计算，一直持续插值跟踪即可
          Eigen::Quaterniond(latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
    } else {
      std::cout << "\033[91m[IncrementalPoseCalculator] Right hand data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
      lastTargetLeftElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
    } else {
      std::cout << "\033[91m[IncrementalPoseCalculator] Left elbow data not available, using default values\033[0m"
                << std::endl;
    }

    if (latestPoseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
      lastTargetRightElbowPosOnExit = latestPoseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
    } else {
      std::cout << "\033[91m[IncrementalPoseCalculator] Right elbow data not available, using default values\033[0m"
                << std::endl;
    }

    std::cout << "\033[92m[IncrementalPoseCalculator] Last target on exit updated successfully\033[0m" << std::endl;
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
    std::cout << "\033[92m[IncrementalPoseCalculator] Delta reset successfully\033[0m" << std::endl;
  }

  void resetSlerpFactor() {
    slerpQuat_t_ = 0.0;
    slerpQuat_dt_ = 0.0;
    std::cout << "\033[92m[IncrementalPoseCalculator] Slerp factor reset successfully\033[0m" << std::endl;
  }

  void slerpQuat(const Eigen::Quaterniond& leftHandTargetQuat, const Eigen::Quaterniond& rightHandTargetQuat) {
    auto tmpLeftHandQuat = lastTargetLeftHandQuatOnExit;
    auto tmpRightHandQuat = lastTargetRightHandQuatOnExit;

    latestTargetLeftHandQuatSlerp = tmpLeftHandQuat.slerp(slerpQuat_t_, leftHandTargetQuat).normalized();
    latestTargetRightHandQuatSlerp = tmpRightHandQuat.slerp(slerpQuat_t_, rightHandTargetQuat).normalized();
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
 * @brief 增量控制模式状态机
 */
class IncrementalModeStateMachine {
 public:
  explicit IncrementalModeStateMachine(std::shared_ptr<JoyStickHandler> joyStickHandler);

  // 状态查询
  bool shouldEnterIncrementalMode() const;
  bool shouldExitIncrementalMode() const;
  bool isIncrementalMode() const;

  // 状态转换
  void enterIncrementalMode();
  void exitIncrementalMode();

 private:
  ControlMode controlMode_ = ControlMode::NONE;
  std::shared_ptr<JoyStickHandler> joyStickHandler_;
};

/**
 * @brief 增量位置计算器
 */
class IncrementalPoseCalculator {
 public:
  explicit IncrementalPoseCalculator(const IncrementalControlConfig& config);

  // 设置锚点位置（进入增量模式时调用）
  void updateHumanPoseAnchor(const ArmPose& vrLeftPose,
                             const ArmPose& vrRightPose,
                             const ArmPose& vrLeftElbowPose,
                             const ArmPose& vrRightElbowPose,
                             const std::vector<PoseData>& latestPoseConstraintList);

  void updateLastTargetOnExit(const std::vector<PoseData>& latestPoseConstraintList);

  // 计算增量位置
  IncrementalPoseResult computeIncrementalPose(const ArmPose& vrLeftPose,
                                               const ArmPose& vrRightPose,
                                               const ArmPose& vrLeftElbowPose,
                                               const ArmPose& vrRightElbowPose);

  // 更新配置
  void updateConfig(const IncrementalControlConfig& config);
  IncrementalPoseResult getLatestIncrementalResult() const { return result_; }

  void resetDelta();
  void resetSlerpFactor();

  // 更新手部FK位置（基于传感器数据计算）
  void updateHandFkPositions(const Eigen::Vector3d& leftHandFkPos, const Eigen::Vector3d& rightHandFkPos);

 private:
  IncrementalControlConfig config_;
  IncrementalPoseResult result_;

  void computeFhanFiltering(const ArmPose& vrLeftPose,
                            const ArmPose& vrRightPose,
                            const ArmPose& vrLeftElbowPose,
                            const ArmPose& vrRightElbowPose,
                            const double slerpQuatFactor = 1.0);
};

/**
 * @brief 主要的增量控制模块类
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

  void exitIncrementalMode(const ArmPose& vrLeftPose,
                           const ArmPose& vrRightPose,
                           const ArmPose& vrLeftElbowPose,
                           const ArmPose& vrRightElbowPose,
                           const std::vector<PoseData>& latestPoseConstraintList);

  IncrementalPoseResult computeIncrementalPose(const ArmPose& vrLeftPose,
                                               const ArmPose& vrRightPose,
                                               const ArmPose& vrLeftElbowPose,
                                               const ArmPose& vrRightElbowPose);

  IncrementalPoseResult getLatestIncrementalResult() const;

  // 更新手部FK位置（基于传感器数据计算）
  void updateHandFkPositions(const Eigen::Vector3d& leftHandFkPos, const Eigen::Vector3d& rightHandFkPos);

  // 人体手臂移动检测
  bool detectHumanArmMove(const Eigen::Vector3d& currentLeftHandPos, const Eigen::Vector3d& currentRightHandPos);

  // 状态查询接口
  bool shouldEnterIncrementalMode() const;
  bool shouldExitIncrementalMode() const;
  bool isIncrementalMode() const;
  bool hasHumanArmMoved() const;

  void updateConfig(const IncrementalControlConfig& config);
  const IncrementalControlConfig& getConfig() const;

 private:
  std::unique_ptr<IncrementalModeStateMachine> stateMachine_;
  std::unique_ptr<IncrementalPoseCalculator> poseCalculator_;
  HumanArmMoveDetector armMoveDetector_;

  IncrementalControlConfig config_;

  bool initialized_ = false;

  // 互斥锁保护共享状态（状态机、计算结果等）
  mutable std::mutex stateMutex_;
};

/**
 * @brief 增量控制模块工厂类
 */
class IncrementalControlFactory {
 public:
  // 创建标准增量控制模块
  static std::unique_ptr<IncrementalControlModule> createStandardModule(
      std::shared_ptr<JoyStickHandler> joyStickHandler);

  // 创建自定义配置的增量控制模块
  static std::unique_ptr<IncrementalControlModule> createCustomModule(std::shared_ptr<JoyStickHandler> joyStickHandler,
                                                                      const IncrementalControlConfig& config);

 private:
  IncrementalControlFactory() = default;
};

}  // namespace HighlyDynamic
