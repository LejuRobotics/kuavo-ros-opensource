#include "motion_capture_ik/Quest3ArmInfoTransformer.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace HighlyDynamic {

Quest3ArmInfoTransformer::Quest3ArmInfoTransformer(const std::string &robotModel, const Eigen::Vector3d &deltaScale)
    : qInitChest_(0.5, 0.5, 0.5, 0.5),  // 等效于绕x，绕z分别旋转pi/2
      chest_axis_agl_(Eigen::Vector3d::Zero()),
      isInitialized_(true),
      leftHandPose_(robotModel, true),    // 使用带机器人型号参数的构造函数初始化左手位姿
      rightHandPose_(robotModel, false),  // 使用带机器人型号参数的构造函数初始化右手位姿
      leftShoulderPose_(),
      rightShoulderPose_(),
      leftShoulderRpyInRobot_(Eigen::Vector3d::Zero()),
      rightShoulderRpyInRobot_(Eigen::Vector3d::Zero()),
      shoulderWidth_(0.15),  // 肩宽参数，与Python版本一致
      biasChestToBaseLink_(0.0, 0.0, 0.4245),
      armLengthMeasurement_(),
      robotUpperArmLength_(0.2844),
      robotLowerArmLength_(0.45),
      okPressedCounts_(0),
      stopPressedCounts_(0),
      deltaScale_(deltaScale) {
  // [CZJ]TODO:  后续需要从ros param获得所有硬编码数据
  armLengthMeasurement_.setMeasureArmLength(true);
  // print robot model
  ROS_INFO("[Quest3ArmInfoTransformer] Robot model: %s", robotModel.c_str());
}

bool Quest3ArmInfoTransformer::updateHandPoseAndElbowPosition(const noitom_hi5_hand_udp_python::PoseInfoList &input,
                                                              noitom_hi5_hand_udp_python::PoseInfoList &output) {
  if (!validateInput(input)) {
    ROS_WARN("[Quest3ArmInfoTransformer] Invalid input data");
    return false;
  }
  // 核心功能，转换左手手部位姿
  if (!computeHandPose(input, "Left")) {
    ROS_WARN("[Quest3ArmInfoTransformer] Failed to compute left hand pose");
    return false;
  }

  // 核心功能，转换右手手部位姿
  if (!computeHandPose(input, "Right")) {
    ROS_WARN("[Quest3ArmInfoTransformer] Failed to compute right hand pose");
    return false;
  }

  updateHandElbowPoseInfoList(output);
  return true;
}

bool Quest3ArmInfoTransformer::computeHandPose(const noitom_hi5_hand_udp_python::PoseInfoList &input,
                                               const std::string &side) {
  std::map<std::string, ArmData> sideMap = {{"Left",
                                             ArmData(qLeftHandW_,
                                                     leftHandPose_,
                                                     leftElbowPose_,
                                                     POSE_INDEX_LEFT_ELBOW,
                                                     POSE_INDEX_LEFT_ARM_UPPER,
                                                     POSE_INDEX_LEFT_HAND)},
                                            {"Right",
                                             ArmData(qRightHandW_,
                                                     rightHandPose_,
                                                     rightElbowPose_,
                                                     POSE_INDEX_RIGHT_ELBOW,
                                                     POSE_INDEX_RIGHT_ARM_UPPER,
                                                     POSE_INDEX_RIGHT_HAND)}};
  auto it = sideMap.find(side);
  if (it == sideMap.end()) {
    ROS_ERROR(
        "[Quest3ArmInfoTransformer] Invalid side parameter: %s, must be "
        "'Left' or 'Right'",
        side.c_str());
    return false;
  }

  // ########################## armData 现在包含了处理当前"侧"所需的所有信息
  // ##########################
  // ###############################################################################################
  ArmData &armData = it->second;

  // 提取公共位姿
  auto chestPose = input.poses[POSE_INDEX_CHEST];
  auto elbowPose = input.poses[armData.elbowIndex];
  auto shoulderPose = input.poses[armData.shoulderIndex];
  auto handPose = input.poses[armData.handIndex];

  Eigen::Quaterniond vrQuat(
      handPose.orientation.w, handPose.orientation.x, handPose.orientation.y, handPose.orientation.z);

  double biasAngle = 15.0 * M_PI / 180.0;
  Eigen::Quaterniond handQuatInW = vrQuat2RobotQuat(vrQuat, side, biasAngle);
  armData.handQuatInW = handQuatInW;  // 直接通过引用修改成员变量

  // 胸部位置和姿态处理
  chestPosition_ = Eigen::Vector3d(chestPose.position.x, chestPose.position.y, chestPose.position.z);
  const Eigen::Quaterniond qCurrentChest(
      chestPose.orientation.w, chestPose.orientation.x, chestPose.orientation.y, chestPose.orientation.z);
  const Eigen::Quaterniond qRelativeChest = (qInitChest_.inverse() * qCurrentChest).normalized();

  Eigen::Vector3d axis;
  double angle;
  quatToAxisAngle(qRelativeChest, axis, angle);

  // 计算HeadBodyPose数据 - 复现Python版本的compute_hand_pose中的逻辑
  // 对应Python L748-757的计算
  Eigen::Matrix3d initRwC = qInitChest_.toRotationMatrix();
  Eigen::Matrix3d currentChestRotation = qCurrentChest.toRotationMatrix();
  Eigen::Matrix3d relativeChestRotation = initRwC.transpose() * currentChestRotation;

  // 计算chest_rpy - 对应Python L749
  Eigen::Vector3d chestRpy = matrixToRPY(relativeChestRotation);

  // 设置chest_axis_agl和body_yaw - 对应Python L750-751
  chest_axis_agl_ = Eigen::Vector3d(0, 0, axis[1]);
  headBodyPose_.body_yaw = axis[1];

  // 计算去除yaw后的旋转矩阵 - 对应Python L752
  Eigen::Matrix3d RwChestRmYaw = axisAngleToMatrix(chest_axis_agl_).transpose() * currentChestRotation;

  // 计算body_pitch - 对应Python L753
  Eigen::Vector3d bodyRpyAfterYawRemoval = matrixToRPY(initRwC.transpose() * RwChestRmYaw);
  headBodyPose_.body_pitch = bodyRpyAfterYawRemoval[0];

  // 设置body_roll - 对应Python L754
  headBodyPose_.body_roll = chestRpy[2];

  // 设置body位置 - 对应Python L755-757
  headBodyPose_.body_x = chestPose.position.x;
  headBodyPose_.body_y = chestPose.position.y;
  headBodyPose_.body_height = chestPose.position.z;

  yawOnlyQuat_ = Eigen::Quaterniond(Eigen::AngleAxisd(axis[1], Eigen::Vector3d::UnitZ()));
  // CHKED[1]: hand ee 姿态解算完成
  handQuatInW = yawOnlyQuat_.inverse() * handQuatInW;

  // -------------------- handPos 坐标系转换 --------------------
  auto handPos = extractPosition(handPose);
  handPos -= chestPosition_;
  handPos = yawOnlyQuat_.inverse() * handPos;
  handPos += biasChestToBaseLink_;

  // -------------------- elbowPos 坐标系转换 --------------------
  auto elbowPos = extractPosition(elbowPose);
  elbowPos -= chestPosition_;
  elbowPos = yawOnlyQuat_.inverse() * elbowPos;

  // -------------------- shoulderPos 坐标系转换 --------------------
  auto shoulderPos = extractPosition(shoulderPose);
  shoulderPos -= chestPosition_;
  shoulderPos = yawOnlyQuat_.inverse() * shoulderPos;

  // 添加到机器人基坐标系的平移偏置

  elbowPos.x() += biasChestToBaseLink_.x();
  elbowPos.z() += biasChestToBaseLink_.z();
  shoulderPos += biasChestToBaseLink_;

  // 计算肩部角度
  Eigen::Matrix3d R_wS = extractRotationMatrix(shoulderPose);
  computeShoulderAngle(R_wS, side);

  // 固定肩部位置
  shoulderPos.x() = biasChestToBaseLink_.x();
  shoulderPos.z() = biasChestToBaseLink_.z();

  // 保存原始的human shoulder位置，用于后续的手臂长度缩放计算（复现Python L812逻辑）
  Eigen::Vector3d humanShoulderPos = shoulderPos;

  bool overChest = isOverChest(handPos, side);
  adaptShoulderWidthAdvanced(shoulderPos, elbowPos, handPos, humanShoulderPos, side, overChest);

  // 修正：使用原始的humanShoulderPos作为参考点进行缩放，而不是调整后的shoulderPos
  auto scaledPositions = scaleArmPositions(shoulderPos, elbowPos, handPos, humanShoulderPos, side);
  elbowPos = scaledPositions.first;
  handPos = scaledPositions.second;

  applyLateralPositionAdjustment(handPos, side);

  // 位置限制
  if (handPos.x() < 0.1) {
    handPos.x() = 0.1;
  }

  // 更新最终的姿态信息
  armData.handPose = ArmPose(handPos, handQuatInW);
  armData.elbowPose = ArmPose(elbowPos, Eigen::Quaterniond::Identity());

  // 更新肩部位姿信息：将旋转矩阵转换为四元数
  Eigen::Quaterniond shoulderQuat(R_wS);
  shoulderQuat.normalize();
  if (side == "Left") {
    leftShoulderPose_ = ArmPose(shoulderPos, shoulderQuat);
  } else if (side == "Right") {
    rightShoulderPose_ = ArmPose(shoulderPos, shoulderQuat);
  }

  // 验证位姿有效性
  if (!armData.handPose.isValid() || !armData.elbowPose.isValid()) {
    ROS_WARN("[Quest3ArmInfoTransformer] Invalid %s hand poses", side.c_str());
    return false;
  }

  updateVisualizationDataForSide(input, side, handPos, handQuatInW, elbowPos, shoulderPos, R_wS);

  if (side == "Left") {
    leftHandPose_.position.x() = input.poses[armData.handIndex].position.x;
    leftHandPose_.position.y() = input.poses[armData.handIndex].position.y;
    leftHandPose_.position.z() = input.poses[armData.handIndex].position.z;
  }

  if (side == "Right") {
    rightHandPose_.position.x() = input.poses[armData.handIndex].position.x;
    rightHandPose_.position.y() = input.poses[armData.handIndex].position.y;
    rightHandPose_.position.z() = input.poses[armData.handIndex].position.z;
  }

  return true;
}

Eigen::Vector3d Quest3ArmInfoTransformer::extractPosition(const noitom_hi5_hand_udp_python::PoseInfo &poseInfo) const {
  return Eigen::Vector3d(poseInfo.position.x, poseInfo.position.y, poseInfo.position.z);
}

void Quest3ArmInfoTransformer::updateHandElbowPoseInfoList(noitom_hi5_hand_udp_python::PoseInfoList &output) {
  output.timestamp_ms = ros::Time::now().toNSec() / 1000000;  // 转换为毫秒
  output.is_high_confidence = true;
  output.is_hand_tracking = false;  // 参考Python版本设置为false

  // 参考Python的调试版本：[l_hand, l_elbow, r_hand, r_elbow]
  output.poses.resize(4);
  setHandPoseInfo(output.poses[0], leftHandPose_);     // 左手
  setElbowPoseInfo(output.poses[1], leftElbowPose_);   // 左肘
  setHandPoseInfo(output.poses[2], rightHandPose_);    // 右手
  setElbowPoseInfo(output.poses[3], rightElbowPose_);  // 右肘
}

// # [18] CHKED: 已实现scale_arm_positions函数 - 正确的手臂长度比例缩放算法
std::pair<Eigen::Vector3d, Eigen::Vector3d> Quest3ArmInfoTransformer::scaleArmPositions(
    const Eigen::Vector3d &shoulderAdaptivePos,
    const Eigen::Vector3d &elbowPos,
    const Eigen::Vector3d &handPos,
    const Eigen::Vector3d &humanShoulderOriginPos,
    const std::string &side) {
  double humanUpperArmLength = (elbowPos - humanShoulderOriginPos).norm();

  // 计算人体下臂长度（从肘部到手部）
  double humanLowerArmLength = (handPos - elbowPos).norm();

  // 新增：动态测量功能 - 收集手臂长度数据
  if (armLengthMeasurement_.isMeasureArmLength()) {
    armLengthMeasurement_.updateMeasurement(humanUpperArmLength, humanLowerArmLength, side);

    // 自动完成测量：当左右手臂都收集到足够样本时（与Python版本的arm_length_num=30保持一致）
    if (armLengthMeasurement_.getLeftDataCount() >= 30 && armLengthMeasurement_.getRightDataCount() >= 30) {
      armLengthMeasurement_.completeMeasurement();
      ROS_INFO("[Quest3ArmInfoTransformer] Auto-completed arm length measurement after collecting sufficient samples");
    }
  }

  // 计算缩放比例
  double radi1, radi2;

  // 根据测量模式选择不同的缩放算法（与Python版本保持一致）
  if (armLengthMeasurement_.isMeasureArmLength()) {
    // 测量模式：使用当前测量值进行缩放（复现Python L698-699逻辑）
    radi1 = robotUpperArmLength_ / humanUpperArmLength;
    radi2 = (robotLowerArmLength_ + robotUpperArmLength_) / (humanLowerArmLength + humanUpperArmLength);
  } else {
    // 非测量模式：使用历史平均值进行缩放（复现Python L702-713逻辑）
    if (side == "Left") {
      // 左臂：如果有历史数据，使用平均值；否则使用当前值
      if (armLengthMeasurement_.getLeftDataCount() > 0) {
        radi1 = robotUpperArmLength_ / armLengthMeasurement_.getAvgLeftUpperArmLength();
        radi2 = robotLowerArmLength_ / armLengthMeasurement_.getAvgLeftLowerArmLength();
      } else {
        // 没有历史数据时的回退逻辑
        radi1 = robotUpperArmLength_ / humanUpperArmLength;
        radi2 = robotLowerArmLength_ / humanLowerArmLength;
      }
    } else {
      // 右臂：相同逻辑
      if (armLengthMeasurement_.getRightDataCount() > 0) {
        radi1 = robotUpperArmLength_ / armLengthMeasurement_.getAvgRightUpperArmLength();
        radi2 = robotLowerArmLength_ / armLengthMeasurement_.getAvgRightLowerArmLength();
      } else {
        // 没有历史数据时的回退逻辑
        radi1 = robotUpperArmLength_ / humanUpperArmLength;
        radi2 = robotLowerArmLength_ / humanLowerArmLength;
      }
    }
  }

  // 计算缩放后的肘部位置
  Eigen::Vector3d scaledElbowPos = shoulderAdaptivePos + radi1 * (elbowPos - humanShoulderOriginPos);

  // 计算缩放后的手部位置
  Eigen::Vector3d scaledHandPos = scaledElbowPos + radi2 * (handPos - elbowPos);

  return std::make_pair(scaledElbowPos, scaledHandPos);
}

// ################################# end of major function ############################
// ####################################################################################
bool Quest3ArmInfoTransformer::validateInput(const noitom_hi5_hand_udp_python::PoseInfoList &input) const {
  // 检查基本数据
  if (input.poses.empty()) {
    ROS_WARN("[Quest3ArmInfoTransformer] Empty pose data");
    return false;
  }

  if (input.poses.size() < 24) {
    ROS_WARN(
        "[Quest3ArmInfoTransformer] Insufficient pose data: %zu poses, "
        "need at least 24",
        input.poses.size());
    return false;
  }

  // 检查位姿数据的有效性 - 检查关键的4个pose索引
  std::vector<size_t> keyPoseIndices = {
      POSE_INDEX_LEFT_HAND, POSE_INDEX_LEFT_ELBOW, POSE_INDEX_RIGHT_HAND, POSE_INDEX_RIGHT_ELBOW};
  for (size_t i : keyPoseIndices) {
    const auto &pose = input.poses[i];

    // 检查位置数据
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z)) {
      ROS_WARN("[Quest3ArmInfoTransformer] Invalid position data at index %zu", i);
      return false;
    }

    // 检查四元数数据
    if (!std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) ||
        !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)) {
      ROS_WARN("[Quest3ArmInfoTransformer] Invalid orientation data at index %zu", i);
      return false;
    }
  }

  return true;
}

// VR四元数转换相关函数实现
Eigen::Quaterniond Quest3ArmInfoTransformer::vrQuat2RobotQuat(const Eigen::Quaterniond &vrQuat,
                                                              const std::string &side,
                                                              double biasAngle) const {
  Eigen::Quaterniond qHand = vrQuat.normalized();

  double xHandBias = side == "Right" ? biasAngle : -biasAngle;
  double xAxisBias = -M_PI / 2;
  double zAxisBias = side == "Right" ? 0.0 : -M_PI;

  Eigen::Quaterniond qZX =
      Eigen::AngleAxisd(zAxisBias, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(xAxisBias, Eigen::Vector3d::UnitX());

  Eigen::Quaterniond qBias(Eigen::AngleAxisd(xHandBias, Eigen::Vector3d::UnitX()));

  // Eigen的四元数连乘，可参考旋转矩阵连乘，和数学/几何上的计算顺序不太一样
  return qHand * qZX * qBias;
}

// ============================================================================
// 工具函数实现

// # [16] CHKED: cpp中应该封装为isOverChest函数 - 已实现
bool Quest3ArmInfoTransformer::isOverChest(const Eigen::Vector3d &handPos, const std::string &side) const {
  // 检测手部是否跨越胸部
  if (side == "Left" && handPos.y() < 0) {
    return true;  // 左手跨越到右侧
  } else if (side == "Right" && handPos.y() > 0) {
    return true;  // 右手跨越到左侧
  }
  return false;
}

void Quest3ArmInfoTransformer::adaptShoulderWidthAdvanced(Eigen::Vector3d &shoulderPos,
                                                          const Eigen::Vector3d &elbowPos,
                                                          const Eigen::Vector3d &handPos,
                                                          const Eigen::Vector3d &humanShoulderPos,
                                                          const std::string &side,
                                                          bool overChest) const {
  double yDistance = std::abs(handPos.y());

  // 计算肩部旋转角度（基于肘部相对于肩部的位置）
  // 当肘部向前和向内移动时，说明肩部在向前旋转
  Eigen::Vector3d elbowRelativeToShoulder = elbowPos - humanShoulderPos;

  // 计算肘部在水平面上的角度
  double elbowAngleHorizontal = std::atan2(elbowRelativeToShoulder.y(), elbowRelativeToShoulder.x());

  // 根据肘部位置推断肩部旋转
  double shoulderRotationFactor = 0.0;
  if (side == "Right") {
    // 右臂：当肘部角度从-90度（右侧）向0度（前方）移动时，肩部向前旋转
    if (elbowAngleHorizontal > -M_PI / 2 && elbowAngleHorizontal < M_PI / 4) {
      shoulderRotationFactor = (elbowAngleHorizontal + M_PI / 2) / (3 * M_PI / 4);
    }
  } else {  // Left
    // 左臂：当肘部角度从90度（左侧）向0度（前方）移动时，肩部向前旋转
    if (elbowAngleHorizontal < M_PI / 2 && elbowAngleHorizontal > -M_PI / 4) {
      shoulderRotationFactor = (M_PI / 2 - elbowAngleHorizontal) / (3 * M_PI / 4);
    }
  }

  // 限制旋转因子在0-1之间
  shoulderRotationFactor = std::clamp(shoulderRotationFactor, 0.0, 1.0);

  // 根据肩部旋转调整机器人肩部位置
  // 肩部旋转时，肩关节会向前和向内移动
  double shoulderForwardOffset = shoulderRotationFactor * 0.08;  // 最大前移8cm
  double shoulderInwardOffset = shoulderRotationFactor * 0.15;   // 最大内收12cm

  // 如果手臂跨越身体中线，需要额外的调整
  double crossBodyFactor = 0.0;
  if (overChest) {
    // 当手臂跨越身体中线时，肩部需要更大的内收
    crossBodyFactor = std::min(yDistance / shoulderWidth_, 1.0) * 0.08;  // 额外最大8cm内收
  }

  shoulderPos.x() += shoulderForwardOffset;  // 向前移动

  if (side == "Right") {
    shoulderPos.y() = -shoulderWidth_ + shoulderInwardOffset + crossBodyFactor;
  } else if (side == "Left") {
    shoulderPos.y() = shoulderWidth_ - shoulderInwardOffset - crossBodyFactor;
  }
}

void Quest3ArmInfoTransformer::applyLateralPositionAdjustment(Eigen::Vector3d &handPos, const std::string &side) const {
  // 当手向身体中线靠近时，让手能更容易到达中线
  // 检测手是否向身体中线移动
  double handToCenterline = std::abs(handPos.y());  // 手到中线的距离

  // 如果手臂向前且靠近身体中线，进行横向调整
  if (handPos.x() > 0.15 && handToCenterline < 0.2) {  // 手在前方且靠近身体中线
    // 计算横向拉近因子：当手越靠近中线时，向中线拉得越多
    double pullToCenterFactor = (0.2 - handToCenterline) / 0.2;  // 范围：0-1
    double pullAmount = pullToCenterFactor * 0.05;               // 最大向中线拉5cm

    if (side == "Right") {
      handPos.y() = handPos.y() + pullAmount;  // 向右（正方向）移动，靠近中线
    } else {                                   // Left
      handPos.y() = handPos.y() - pullAmount;  // 向左（负方向）移动，靠近中线
    }
  }
}

// 新增：参数更新接口实现
void Quest3ArmInfoTransformer::updateBaseHeightOffset(double baseHeightOffset) {
  biasChestToBaseLink_.z() = baseHeightOffset;
}

void Quest3ArmInfoTransformer::updateBaseChestOffsetX(double baseChestOffsetX) {
  biasChestToBaseLink_.x() = baseChestOffsetX;
}

void Quest3ArmInfoTransformer::updateShoulderWidth(double shoulderWidth) { shoulderWidth_ = shoulderWidth; }

void Quest3ArmInfoTransformer::updateUpperArmLength(double upperArmLength) {
  robotUpperArmLength_ = upperArmLength;
  ROS_INFO("[Quest3ArmInfoTransformer] Updated robot upper arm length to %.4f m", robotUpperArmLength_);
}

void Quest3ArmInfoTransformer::updateLowerArmLength(double lowerArmLength) {
  // 直接使用米单位的参数
  robotLowerArmLength_ = lowerArmLength;
  ROS_INFO("[Quest3ArmInfoTransformer] Updated robot lower arm length to %.4f m", robotLowerArmLength_);
}

void Quest3ArmInfoTransformer::computeShoulderAngle(const Eigen::Matrix3d &R_wS, const std::string &side) {
  // 参考Python版本的compute_shoudler_pose函数 (L647-665)
  // 使用动态初始化而不是硬编码的矩阵值
  static Eigen::Matrix3d initR_wLS = Eigen::Matrix3d::Identity();
  static Eigen::Matrix3d initR_wRS = Eigen::Matrix3d::Identity();
  static bool leftInitialized = false;
  static bool rightInitialized = false;

  // 动态初始化初始肩部旋转矩阵（复现Python L763-766逻辑）
  if (side == "Left" && !leftInitialized) {
    initR_wLS = R_wS;
    leftInitialized = true;
    ROS_INFO("[Quest3ArmInfoTransformer] Initialized left shoulder reference matrix");
    return;  // 第一次调用时仅保存参考矩阵，不计算角度
  }

  if (side == "Right" && !rightInitialized) {
    initR_wRS = R_wS;
    rightInitialized = true;
    ROS_INFO("[Quest3ArmInfoTransformer] Initialized right shoulder reference matrix");
    return;  // 第一次调用时仅保存参考矩阵，不计算角度
  }

  // 计算相对旋转矩阵并转换为RPY角度（复现Python L652-662逻辑）
  Eigen::Matrix3d R_01;
  Eigen::Vector3d rpy;

  if (side == "Left") {
    R_01 = initR_wLS.transpose() * R_wS;
    // 使用标准的旋转矩阵到RPY转换
    rpy = matrixToRPY(R_01);
    // 复现Python版本的角度映射：[2], -[0], -[1]
    leftShoulderRpyInRobot_[0] = rpy[2];   // roll = rpy[2]
    leftShoulderRpyInRobot_[1] = -rpy[0];  // pitch = -rpy[0]
    leftShoulderRpyInRobot_[2] = -rpy[1];  // yaw = -rpy[1]
  } else if (side == "Right") {
    R_01 = initR_wRS.transpose() * R_wS;
    // 使用标准的旋转矩阵到RPY转换
    rpy = matrixToRPY(R_01);
    // 复现Python版本的角度映射：-[2], -[0], [1]
    rightShoulderRpyInRobot_[0] = -rpy[2];  // roll = -rpy[2]
    rightShoulderRpyInRobot_[1] = -rpy[0];  // pitch = -rpy[0]
    rightShoulderRpyInRobot_[2] = rpy[1];   // yaw = rpy[1]
  }
}

Eigen::Matrix3d Quest3ArmInfoTransformer::extractRotationMatrix(
    const noitom_hi5_hand_udp_python::PoseInfo &poseInfo) const {
  // 从PoseInfo中提取四元数并转换为旋转矩阵
  Eigen::Quaterniond quat(
      poseInfo.orientation.w, poseInfo.orientation.x, poseInfo.orientation.y, poseInfo.orientation.z);
  return quat.toRotationMatrix();
}

void Quest3ArmInfoTransformer::quatToAxisAngle(const Eigen::Quaterniond &quat,
                                               Eigen::Vector3d &axis,
                                               double &angle) const {
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();
  double w = quat.w();

  double norm_xyz = std::sqrt(x * x + y * y + z * z);
  angle = 2.0 * std::atan2(norm_xyz, w);

  if (angle <= 1e-3) {
    double angle2 = angle * angle;
    // Python: scale = 2.0 + angle2 / 12.0 + 7.0 * angle2 * angle2 / 2880.0
    double scale = 2.0 + angle2 / 12.0 + 7.0 * angle2 * angle2 / 2880.0;
    // Python: rotvec = quat[:3] * scale
    axis = Eigen::Vector3d(x, y, z) * scale;
  } else {
    // Python: scale = angle / np.sin(angle / 2)
    double scale = angle / std::sin(angle / 2.0);
    axis = Eigen::Vector3d(x, y, z) * scale;
  }
}

Eigen::Vector3d Quest3ArmInfoTransformer::matrixToRPY(const Eigen::Matrix3d &R) const {
  // 标准的旋转矩阵到RPY角度转换（ZYX顺序，与Python的matrix_to_rpy一致）
  double roll, pitch, yaw;

  // 检查gimbal lock情况
  double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-6;

  if (!singular) {
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = std::atan2(-R(1, 2), R(1, 1));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = 0;
  }

  return Eigen::Vector3d(roll, pitch, yaw);
}

// 注意：动态测量功能已封装到ArmLengthMeasurement类中
// 所有相关方法现在通过armLengthMeasurement_成员变量调用

void Quest3ArmInfoTransformer::updateVisualizationDataForSide(const noitom_hi5_hand_udp_python::PoseInfoList &input,
                                                              const std::string &side,
                                                              const Eigen::Vector3d &handPos,
                                                              const Eigen::Quaterniond &handQuat,
                                                              const Eigen::Vector3d &elbowPos,
                                                              const Eigen::Vector3d &shoulderPos,
                                                              const Eigen::Matrix3d &shoulderRot) {
  // 复现Python版本的分别处理逻辑：每个手臂处理时单独更新对应数据
  if (side == "Left") {
    visualizationData_.leftHandPos = handPos;
    visualizationData_.leftElbowPos = elbowPos;
    visualizationData_.leftShoulderPos = shoulderPos;
    visualizationData_.leftHandQuat = qLeftHandW_;
    visualizationData_.leftSideReady = true;
  } else if (side == "Right") {
    visualizationData_.rightHandPos = handPos;
    visualizationData_.rightElbowPos = elbowPos;
    visualizationData_.rightShoulderPos = shoulderPos;
    visualizationData_.rightHandQuat = qRightHandW_;
    visualizationData_.rightSideReady = true;
  }

  Eigen::Vector3d originalChestPos;
  Eigen::Matrix3d chestRot = Eigen::Matrix3d::Identity();
  if (input.poses.size() > POSE_INDEX_CHEST) {
    auto chestPose = input.poses[POSE_INDEX_CHEST];
    originalChestPos = Eigen::Vector3d(chestPose.position.x, chestPose.position.y, chestPose.position.z);
    visualizationData_.chestPos = originalChestPos;
    // 提取胸部旋转矩阵
    Eigen::Quaterniond chestQuat(
        chestPose.orientation.w, chestPose.orientation.x, chestPose.orientation.y, chestPose.orientation.z);
    chestRot = chestQuat.toRotationMatrix();
  }

  visualizationData_.isValid = visualizationData_.leftSideReady || visualizationData_.rightSideReady;
  if (visPub_ && visualizationCallback_) {
    // 构建PoseData向量: [handPose, elbowPose, shoulderPose, chestPose]
    std::vector<PoseData> poses;
    poses.push_back(PoseData(handQuat.toRotationMatrix(), handPos));   // hand
    poses.push_back(PoseData(Eigen::Matrix3d::Identity(), elbowPos));  // elbow
    poses.push_back(PoseData(shoulderRot, shoulderPos));               // shoulder
    poses.push_back(PoseData(chestRot, originalChestPos));             // chest
    visualizationCallback_(side, poses);
  }
}

void Quest3ArmInfoTransformer::updateJoystickData(float leftTrigger,
                                                  float leftGrip,
                                                  float rightTrigger,
                                                  float rightGrip) {
  leftJoystick_.trigger = leftTrigger;
  leftJoystick_.grip = leftGrip;
  rightJoystick_.trigger = rightTrigger;
  rightJoystick_.grip = rightGrip;
}

bool Quest3ArmInfoTransformer::isJoyRunningPressed() const {
  bool isJoyRun = true;
  isJoyRun &= joyOkPressedCheck(leftJoystick_);
  isJoyRun &= joyOkPressedCheck(rightJoystick_);
  return isJoyRun;
}

bool Quest3ArmInfoTransformer::isJoyStopPressed() const {
  bool isJoyStop = true;
  isJoyStop &= joyStopPressedCheck(leftJoystick_);
  isJoyStop &= joyStopPressedCheck(rightJoystick_);
  return isJoyStop;
}

bool Quest3ArmInfoTransformer::joyOkPressedCheck(const JoystickState &joystick) { return joystick.trigger > 0.5f; }

bool Quest3ArmInfoTransformer::joyStopPressedCheck(const JoystickState &joystick, float minAngle, float maxAngle) {
  bool isShot = true;
  isShot &= (joystick.trigger < minAngle);  // 扳机键松开 < 0.5
  isShot &= (joystick.grip > maxAngle);     // 握把键握紧 > 0.8
  return isShot;
}

void Quest3ArmInfoTransformer::checkRunningChangeByHoldingJoy(const int &holdDurationSteps) {
  if (isJoyRunningPressed()) {
    okPressedCounts_++;
    stopPressedCounts_ = 0;

    // 持续50帧后启动
    if (okPressedCounts_ >= holdDurationSteps) {
      if (!isRunning_) {
        isRunning_ = true;
        ROS_INFO("[Quest3ArmInfoTransformer] 🟢 Joystick OK gesture detected - Starting teleoperation");
      }
    }
  } else if (isJoyStopPressed()) {
    stopPressedCounts_++;
    okPressedCounts_ = 0;

    // 持续50帧后停止
    if (stopPressedCounts_ >= holdDurationSteps) {
      if (isRunning_) {
        isRunning_ = false;
        ROS_INFO("[Quest3ArmInfoTransformer] 🔴 Joystick Shot gesture detected - Stopping teleoperation");
      }
    }
  }
}

// 新增：轴角到旋转矩阵转换函数 - 复现Python版本的axis_angle_to_matrix
Eigen::Matrix3d Quest3ArmInfoTransformer::axisAngleToMatrix(const Eigen::Vector3d &axisAngle) const {
  // 计算轴角的模长作为旋转角度
  double angle = axisAngle.norm();

  if (angle < 1e-8) {
    // 如果角度很小，返回单位矩阵
    return Eigen::Matrix3d::Identity();
  }

  // 归一化轴向量
  Eigen::Vector3d axis = axisAngle / angle;

  // 使用Eigen的AngleAxis构造旋转矩阵
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

}  // namespace HighlyDynamic