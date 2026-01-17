#include "motion_capture_ik/Quest3Debugger.h"

namespace HighlyDynamic {

Quest3Debugger::Quest3Debugger(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {}

void Quest3Debugger::initialize() {
  ROS_INFO("[Quest3Debugger] Initializing Quest3 debugging system...");
  initializePublishers();
  ROS_INFO("[Quest3Debugger] Quest3 debugging system initialized successfully");
}

void Quest3Debugger::initializePublishers() {
  ROS_INFO("[Quest3Debugger] Initializing debug publishers...");

  // 初始化骨骼姿态发布器
  bonePoseHandElbowPublisher_ =
      nodeHandle_.advertise<noitom_hi5_hand_udp_python::PoseInfoList>("bone_pose_hand_elbow_cpp", 10);

  // 初始化IK阶段结果发布器
  stage1ResultPublisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/torso_ik/stage1_result_cpp", 10);
  stage2ResultPublisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/torso_ik/stage2_result_cpp", 10);

  ROS_INFO("[Quest3Debugger] All debug publishers initialized successfully");
}

void Quest3Debugger::publishBonePoseHandElbow(const noitom_hi5_hand_udp_python::PoseInfoList& poseListMsg) {
  // 验证消息有效性
  if (poseListMsg.poses.empty()) {
    ROS_WARN("[Quest3Debugger] Bone pose list is empty, skipping publication");
    return;
  }

  // 发布骨骼姿态数据
  bonePoseHandElbowPublisher_.publish(poseListMsg);

  ROS_DEBUG("[Quest3Debugger] Published bone pose hand elbow data with %zu poses", poseListMsg.poses.size());
}

void Quest3Debugger::publishStage1Result(const Eigen::VectorXd& stage1Result) {
  if (!validateEigenVector(stage1Result, "Stage1")) {
    return;
  }

  // 构造并发布消息
  auto msg = constructFloat64MultiArrayMsg(stage1Result);
  stage1ResultPublisher_.publish(msg);

  ROS_DEBUG("[Quest3Debugger] Published Stage1 result with %d elements", static_cast<int>(stage1Result.size()));
}

void Quest3Debugger::publishStage2Result(const Eigen::VectorXd& stage2Result) {
  if (!validateEigenVector(stage2Result, "Stage2")) {
    return;
  }

  // 构造并发布消息
  auto msg = constructFloat64MultiArrayMsg(stage2Result);
  stage2ResultPublisher_.publish(msg);

  ROS_DEBUG("[Quest3Debugger] Published Stage2 result with %d elements", static_cast<int>(stage2Result.size()));
}

void Quest3Debugger::publishDebugDataPackage(const noitom_hi5_hand_udp_python::PoseInfoList& poseListMsg,
                                             const std::shared_ptr<Eigen::VectorXd>& stage1Result,
                                             const std::shared_ptr<Eigen::VectorXd>& stage2Result) {
  ROS_DEBUG("[Quest3Debugger] Publishing complete debug data package...");

  // 发布骨骼姿态数据
  publishBonePoseHandElbow(poseListMsg);

  // 发布第一阶段结果（如果提供）
  if (stage1Result && stage1Result->size() > 0) {
    publishStage1Result(*stage1Result);
  }

  // 发布第二阶段结果（如果提供）
  if (stage2Result && stage2Result->size() > 0) {
    publishStage2Result(*stage2Result);
  }

  ROS_DEBUG("[Quest3Debugger] Debug data package published successfully");
}

std_msgs::Float64MultiArray Quest3Debugger::constructFloat64MultiArrayMsg(const Eigen::VectorXd& eigenVector) {
  std_msgs::Float64MultiArray msg;
  msg.data.resize(eigenVector.size());

  // 转换Eigen::VectorXd到std::vector<double>（保持弧度制以匹配Python版本）
  // 注意：阶段结果现在已经由算法层限制
  for (int i = 0; i < eigenVector.size(); ++i) {
    msg.data[i] = eigenVector(i);
  }

  return msg;
}

bool Quest3Debugger::validateEigenVector(const Eigen::VectorXd& eigenVector, const std::string& stageName) {
  if (eigenVector.size() == 0) {
    ROS_WARN("[Quest3Debugger] %s result is empty, skipping publication", stageName.c_str());
    return false;
  }

  // 检查是否包含无效数值
  for (int i = 0; i < eigenVector.size(); ++i) {
    if (!std::isfinite(eigenVector(i))) {
      ROS_WARN(
          "[Quest3Debugger] %s result contains invalid value at index %d: %f", stageName.c_str(), i, eigenVector(i));
      return false;
    }
  }

  return true;
}

}  // namespace HighlyDynamic
