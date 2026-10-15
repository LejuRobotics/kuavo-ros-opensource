#pragma once

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <leju_utils/define.hpp>
#include <mutex>
#include <string>
#include <vector>

#include "motion_capture_ik/ArmLengthMeasurement.h"

namespace HighlyDynamic {

class Quest3ArmInfoTransformer final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quest3ArmInfoTransformer(const std::string &robotModel = "kuavo_45",
                           const Eigen::Vector3d &deltaScale = Eigen::Vector3d(1.0, 1.0, 1.0));

  ~Quest3ArmInfoTransformer() = default;

  bool updateHandPoseAndElbowPosition(const noitom_hi5_hand_udp_python::PoseInfoList &input,
                                      noitom_hi5_hand_udp_python::PoseInfoList &output);

  // Dual VR（input_mode=dual_vr）按侧更新入口：只计算并写入 side 一侧的手部/肘部状态
  // （leftHandPose_ / leftElbowPose_ / leftHandPoseRelativeToChest_ 或对应的 Right 成员），
  // 不触碰另一侧，也不刷新 output 列表。single_vr 不使用本入口，行为完全不变。
  // 实现只是 validateInput + 转调 private computeHandPose，不复制、不修改其算法本体。
  bool updateHandPoseForSide(const noitom_hi5_hand_udp_python::PoseInfoList &input, const std::string &side);

  ArmPose getLeftHandPose() const { return leftHandPose_; }

  ArmPose getRightHandPose() const { return rightHandPose_; }

  // Wheel incremental control needs the hand motion with rigid torso motion removed.
  ArmPose getLeftHandPoseRelativeToChest() const { return leftHandPoseRelativeToChest_; }

  ArmPose getRightHandPoseRelativeToChest() const { return rightHandPoseRelativeToChest_; }

  ArmPose getLeftElbowPose() const { return leftElbowPose_; }

  ArmPose getRightElbowPose() const { return rightElbowPose_; }

  ArmPose getLeftShoulderPose() const { return leftShoulderPose_; }

  ArmPose getRightShoulderPose() const { return rightShoulderPose_; }

  HeadBodyPose getHeadBodyPose() const { return headBodyPose_; }

  // Dual VR 躯干 ownership：按侧暴露**本实例自己那一侧**输入帧里的 Chest 数据。
  // 每个按侧实例只收到自己那一侧 Quest 的骨骼帧（Left 只喂 A、Right 只喂 B），
  // 因此这里的 Chest 天然是"本侧 Chest"，不会串到对侧。
  // 返回值与 /robot_chest_pose 路径完全同构：
  //   position  : (chest.x - root.x, 0, chest.z)，y 恒为 0（轮臂 chest 不会横向移动）
  //   quaternion: transformQuestPoseTORobotPose(输入帧里已由 monitor 左右手转换过的 Chest 四元数)
  // single_vr 下主实例同样持有全部数据，取到的是当前输入帧的 Chest，行为不变。
  Eigen::Vector3d getChestPosition() const;
  Eigen::Quaterniond getChestQuaternion() const;
  bool hasValidChestPose() const;

  // 新增：更新手部和肘部位姿信息列表 - 参考Python版本
  void updateHandElbowPoseInfoList(noitom_hi5_hand_udp_python::PoseInfoList &output);

  // 新增：参数更新接口
  void updateBaseHeightOffset(double baseHeightOffset);
  void updateBaseChestOffsetX(double baseChestOffsetX);
  void updateShoulderWidth(double shoulderWidth);
  void updateUpperArmLength(double upperArmLength);
  void updateLowerArmLength(double lowerArmLength);

  // 新增：动态测量功能接口
  void setMeasureArmLength(bool measureArmLength) { armLengthMeasurement_.setMeasureArmLength(measureArmLength); }
  bool isMeasureArmLength() const { return armLengthMeasurement_.isMeasureArmLength(); }
  void resetArmLengthMeasurement() { armLengthMeasurement_.reset(); }
  void completeArmLengthMeasurement() { armLengthMeasurement_.completeMeasurement(); }
  bool isArmLengthMeasurementComplete() const { return armLengthMeasurement_.isMeasurementComplete(); }

  // 按侧标定样本数（只读）：Dual VR 下每个按侧实例只收到自己那一侧的数据
  // （见 WheelArmControlBaseROS 中 Left/Right 回调分别调用 updateHandPoseForSide），
  // 而 ArmLengthMeasurement 的自动完成条件要求同一实例内左右样本都达标，
  // 因此按侧实例需要按自己那一侧的样本数判定。不改变任何标定/缩放逻辑。
  size_t armLengthSampleCountForSide(const std::string &side) const {
    return side == "Left" ? armLengthMeasurement_.getLeftDataCount()
                          : armLengthMeasurement_.getRightDataCount();
  }

  static constexpr int armLengthSampleTarget() { return ArmLengthMeasurement::sampleTarget(); }
  void setDeltaScale(const Eigen::Vector3d &deltaScale) { deltaScale_ = deltaScale; }

  // 新增：获取平均手臂长度
  double getAvgLeftUpperArmLength() const { return armLengthMeasurement_.getAvgLeftUpperArmLength(); }
  double getAvgLeftLowerArmLength() const { return armLengthMeasurement_.getAvgLeftLowerArmLength(); }
  double getAvgRightUpperArmLength() const { return armLengthMeasurement_.getAvgRightUpperArmLength(); }
  double getAvgRightLowerArmLength() const { return armLengthMeasurement_.getAvgRightLowerArmLength(); }

  // 新增：可视化数据获取接口（不包含ROS特性）
  struct VisualizationData {
    Eigen::Vector3d leftHandPos;
    Eigen::Vector3d rightHandPos;
    Eigen::Vector3d leftElbowPos;
    Eigen::Vector3d rightElbowPos;
    Eigen::Vector3d leftShoulderPos;
    Eigen::Vector3d rightShoulderPos;
    Eigen::Vector3d chestPos;

    Eigen::Quaterniond leftHandQuat;
    Eigen::Quaterniond rightHandQuat;

    // 新增：分别跟踪左右手的可视化状态
    bool leftSideReady;
    bool rightSideReady;
    bool isValid;

    VisualizationData() : leftSideReady(false), rightSideReady(false), isValid(false) {}
  };

  const VisualizationData &getVisualizationData() const { return visualizationData_; }
  bool hasVisualizationData() const { return visualizationData_.isValid; }

  // 新增：可视化回调函数类型（不包含ROS特性）
  // poses 顺序: [handPose, elbowPose, shoulderPose, chestPose]
  // 每个PoseData包含position和rotation_matrix
  using VisualizationCallback = std::function<void(const std::string &side, const std::vector<PoseData> &poses)>;

  void setVisualizationCallback(VisualizationCallback callback) { visualizationCallback_ = callback; }

  bool isRunning() const { return isRunning_; }
  void updateJoystickData(float leftTrigger, float leftGrip, float rightTrigger, float rightGrip);
  void checkRunningChangeByHoldingJoy(const int &holdDurationSteps);

 private:
  ArmPose leftHandPose_;
  ArmPose rightHandPose_;
  ArmPose leftHandPoseRelativeToChest_;
  ArmPose rightHandPoseRelativeToChest_;
  ArmPose leftElbowPose_;
  ArmPose rightElbowPose_;
  ArmPose leftShoulderPose_;
  ArmPose rightShoulderPose_;

  // 新增：左右手四元数数据
  Eigen::Quaterniond qLeftHandW_;
  Eigen::Quaterniond qRightHandW_;

  // 胸部姿态处理相关成员变量
  Eigen::Quaterniond qInitChest_;   // 初始胸部旋转四元数，等效于绕x，绕z分别旋转pi/2
  Eigen::Vector3d chest_axis_agl_;  // 胸部轴角向量
  bool isInitialized_;              // 是否已初始化

  // 新增：用于补偿的胸部姿态信息
  Eigen::Quaterniond yawOnlyQuat_;  // 仅包含偏航角的四元数，用于补偿
  Eigen::Vector3d chestPosition_;   // 胸部位置，用于坐标系变换
  double bodyPitch_;                // 躯干俯仰角

  // Dual VR 躯干 ownership 用的本侧 Chest 快照（在 computeHandPose 内随本侧输入帧更新）：
  // chestPosePosition_   = (chestPosition_.x - rootOffset.x, 0, chestPosition_.z)，
  //                        与 /robot_chest_pose 位置同构（monitor 扣的正是同帧 Root 的 x）
  // chestPoseQuaternion_ = transformQuestPoseTORobotPose(本侧输入帧里已转换过的 Chest 四元数)
  // 只读暴露给 IK 节点，用于 torso_control_side=right 时真正取 B 侧 Chest。
  Eigen::Vector3d chestPosePosition_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond chestPoseQuaternion_{Eigen::Quaterniond::Identity()};
  bool hasChestPose_{false};
  // 保护上面三个本侧 Chest 快照：bone 回调线程写、IK 求解线程读。
  mutable std::mutex chestPoseSnapshotMutex_;

  // 肩部角度信息 - 参考Python版本
  Eigen::Vector3d leftShoulderRpyInRobot_;   // 左肩在机器人坐标系中的RPY角度
  Eigen::Vector3d rightShoulderRpyInRobot_;  // 右肩在机器人坐标系中的RPY角度

  // 肩宽自适应相关成员变量
  double shoulderWidth_;                 // 肩宽参数
  Eigen::Vector3d biasChestToBaseLink_;  // 胸部到机器人基座的偏移量

  // 新增：动态测量功能相关成员变量
  ArmLengthMeasurement armLengthMeasurement_;  // 手臂长度测量类实例

  double robotUpperArmLength_;  // 机器人上臂长度（米）
  double robotLowerArmLength_;  // 机器人下臂长度（米）

  // 新增：可视化数据存储
  VisualizationData visualizationData_;

  // 新增：可视化回调函数
  VisualizationCallback visualizationCallback_;

  // 新增：可视化发布控制（复现Python版本的vis_pub成员变量）
  bool visPub_ = true;  // 默认启用，与Python版本一致

  bool isRunning_ = false;      // 初始状态为false，与Python版本一致
  Eigen::Vector3d deltaScale_;  // 缩放比例（x, y, z三轴独立）

  struct JoystickState {
    float trigger;  // 扳机键状态 [0.0-1.0]
    float grip;     // 握把键状态 [0.0-1.0]
    JoystickState() : trigger(0.0f), grip(0.0f) {}
  };
  JoystickState leftJoystick_;
  JoystickState rightJoystick_;

  int okPressedCounts_;    // OK手势计数（手柄扳机键）
  int stopPressedCounts_;  // Shot手势计数（手柄握把键）

  HeadBodyPose headBodyPose_;

  bool computeHandPose(const noitom_hi5_hand_udp_python::PoseInfoList &input, const std::string &side);

  Eigen::Vector3d extractPosition(const noitom_hi5_hand_udp_python::PoseInfo &poseInfo) const;

  // 对应Python版本的pose_info2_transform函数
  Eigen::Matrix3d poseInfo2Transform(const noitom_hi5_hand_udp_python::PoseInfo &poseInfo) const;

  // 正确的手臂长度比例缩放函数，对应Python版本的scale_arm_positions
  std::pair<Eigen::Vector3d, Eigen::Vector3d> scaleArmPositions(const Eigen::Vector3d &shoulderAdaptivePos,
                                                                const Eigen::Vector3d &elbowPos,
                                                                const Eigen::Vector3d &handPos,
                                                                const Eigen::Vector3d &humanShoulderOriginPos,
                                                                const std::string &side);

  bool validateInput(const noitom_hi5_hand_udp_python::PoseInfoList &input) const;

  // VR四元数转换相关函数
  Eigen::Quaterniond vrQuat2RobotQuat(const Eigen::Quaterniond &vrQuat,
                                      const std::string &side,
                                      double biasAngle = 0.0) const;

  // 工具函数
  bool isOverChest(const Eigen::Vector3d &handPos, const std::string &side) const;

  void adaptShoulderWidthAdvanced(Eigen::Vector3d &shoulderPos,
                                  const Eigen::Vector3d &elbowPos,
                                  const Eigen::Vector3d &handPos,
                                  const Eigen::Vector3d &humanShoulderPos,
                                  const std::string &side,
                                  bool overChest) const;

  // 添加横向位置的额外调整（防止触碰身体）
  void applyLateralPositionAdjustment(Eigen::Vector3d &handPos, const std::string &side) const;

  // 肩部角度计算函数 - 参考Python版本的pub_vis_shoulder_pose
  void computeShoulderAngle(const Eigen::Matrix3d &R_wS, const std::string &side);

  Eigen::Matrix3d extractRotationMatrix(const noitom_hi5_hand_udp_python::PoseInfo &poseInfo) const;

  void quatToAxisAngle(const Eigen::Quaterniond &quat, Eigen::Vector3d &axis, double &angle) const;

  Eigen::Vector3d matrixToRPY(const Eigen::Matrix3d &R) const;

  Eigen::Matrix3d axisAngleToMatrix(const Eigen::Vector3d &axisAngle) const;

  // 新增：为单个手臂更新可视化数据（复现Python版本的分别处理逻辑）
  void updateVisualizationDataForSide(const noitom_hi5_hand_udp_python::PoseInfoList &input,
                                      const std::string &side,
                                      const Eigen::Vector3d &handPos,
                                      const Eigen::Quaterniond &handQuat,
                                      const Eigen::Vector3d &elbowPos,
                                      const Eigen::Vector3d &shoulderPos,
                                      const Eigen::Matrix3d &shoulderRot);

  static bool joyOkPressedCheck(const JoystickState &joystick);
  static bool joyStopPressedCheck(const JoystickState &joystick, float minAngle = 0.5f, float maxAngle = 0.8f);
  bool isJoyRunningPressed() const;
  bool isJoyStopPressed() const;
};

}  // namespace HighlyDynamic
