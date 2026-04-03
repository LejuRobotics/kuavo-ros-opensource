#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "motion_capture_ik/WheelKeyFramesVisualizer.h"
#include <leju_utils/RosMsgConvertor.hpp>

namespace HighlyDynamic {
using namespace leju_utils::ros_msg_convertor;

WheelKeyFramesVisualizer::WheelKeyFramesVisualizer(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), elbowMinDistance_(0.18), elbowMaxDistance_(0.65) {}

void WheelKeyFramesVisualizer::initialize() {
  ROS_INFO("[WheelKeyFramesVisualizer] Initializing Quest3 visualization system...");

  // 从ROS参数读取手到肘距离约束参数
  nodeHandle_.param("/ik_ros_uni_cpp_node/quest3/elbow_min_distance", elbowMinDistance_, 0.18);
  nodeHandle_.param("/ik_ros_uni_cpp_node/quest3/elbow_max_distance", elbowMaxDistance_, 0.65);
  ROS_INFO(
      "[WheelKeyFramesVisualizer] Elbow distance constraints: min=%.2f, max=%.2f", elbowMinDistance_, elbowMaxDistance_);

  initializePublishers();
  ROS_INFO("[WheelKeyFramesVisualizer] Quest3 visualization system initialized successfully");
}

void WheelKeyFramesVisualizer::initializePublishers() {
  ROS_INFO("[WheelKeyFramesVisualizer] Initializing visualization publishers...");

  // 复现Python版本的发布器topic命名
  // 左侧发布器
  markerPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerPubElbow_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker/elbow", 10);
  markerPubShoulder_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker/shoulder", 10);
  markerPubShoulderQuest3_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker/shoulder_quest3", 10);
  markerPubHumanArrayLeft_ =
      nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker/human_array_left", 10);

  // 右侧发布器
  markerPubRight_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker_right", 10);
  markerPubElbowRight_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker_right/elbow", 10);
  markerPubShoulderRight_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker_right/shoulder", 10);
  markerPubShoulderQuest3Right_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker_right/shoulder_quest3", 10);
  markerPubHumanArrayRight_ =
      nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker/human_array_right", 10);

  // 通用发布器
  markerPubChest_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker_chest", 10);

  // 初始化增量模式位置发布器（Point类型）
  leftAnchorPosPublisher_ = nodeHandle_.advertise<geometry_msgs::Point>("/quest3/left_anchor_pos", 1);
  rightAnchorPosPublisher_ = nodeHandle_.advertise<geometry_msgs::Point>("/quest3/right_anchor_pos", 1);
  leftHandMeasuredPosPublisher_ =
      nodeHandle_.advertise<geometry_msgs::Point>("/quest3/left_hand_measured_pos_on_entry", 1);
  rightHandMeasuredPosPublisher_ =
      nodeHandle_.advertise<geometry_msgs::Point>("/quest3/right_hand_measured_pos_on_entry", 1);

  // 初始化手部6D pose可视化发布器（PoseArray类型）
  leftHandTargetPoseVisualPublisher_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("/left_hand_pose_visual", 1);
  rightHandTargetPoseVisualPublisher_ = nodeHandle_.advertise<geometry_msgs::PoseArray>("/right_hand_pose_visual", 1);
  leftHandMeasuredPoseVisualPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseArray>("/left_hand_measured_pose_visual", 1);
  rightHandMeasuredPoseVisualPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseArray>("/right_hand_measured_pose_visual", 1);

  // 初始化增量结果pose发布器（PoseStamped类型）
  leftHandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/incremental_result/left_hand_pose", 1);
  rightHandPosePublisher_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/incremental_result/right_hand_pose", 1);
  leftElbowPoseStampedPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/incremental_result/left_elbow_pose", 1);
  rightElbowPoseStampedPublisher_ =
      nodeHandle_.advertise<geometry_msgs::PoseStamped>("/incremental_result/right_elbow_pose", 1);

  // 初始化手部旋转矩阵X轴向量可视化发布器（Marker类型）
  leftHandXAxisVectorPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_hand_x_axis_vector", 1);
  rightHandXAxisVectorPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_hand_x_axis_vector", 1);

  // 初始化肩部和肘部位置可视化发布器（Marker类型）
  leftShoulderPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/incremental_left_shoulder", 1);
  rightShoulderPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/incremental_right_shoulder", 1);
  leftElbowPosMarkerPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/incremental_left_elbow", 1);
  rightElbowPosMarkerPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/incremental_right_elbow", 1);

  // 初始化手部bounding box可视化发布器（Marker类型）
  leftHandBoundBoxPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_hand_bound_box", 1);
  rightHandBoundBoxPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_hand_bound_box", 1);

  // 初始化缩放后的手部位置可视化发布器（Marker类型）
  scaledLeftHandPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/scaled_left_hand_pos", 1);
  scaledRightHandPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/scaled_right_hand_pos", 1);

  // 初始化六个关键点可视化发布器（Marker类型）
  leftLink4PosPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_link4_pos", 1);
  rightLink4PosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_link4_pos", 1);
  leftEndEffectorPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_end_effector_pos", 1);
  rightEndEffectorPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_end_effector_pos", 1);
  leftVirtualThumbPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_virtual_thumb_pos", 1);
  rightVirtualThumbPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_virtual_thumb_pos", 1);

  // 初始化机器人优化后的肘部位置可视化发布器（Marker类型，墨绿色，不透明）
  robotLeftElbowPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/robot_left_elbow_optimized", 1);
  robotRightElbowPosPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/robot_right_elbow_optimized", 1);

  // 初始化优化前后的手部位置可视化发布器（Marker类型）
  leftHandPosBeforeOptPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_link6_pos_before_opt", 1);
  leftHandPosAfterOptPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/left_link6_pos_after_opt", 1);
  rightHandPosBeforeOptPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_link6_pos_before_opt", 1);
  rightHandPosAfterOptPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker/right_link6_pos_after_opt", 1);

  ROS_INFO("[WheelKeyFramesVisualizer] All visualization publishers initialized successfully");
}

void WheelKeyFramesVisualizer::publishVisualizationMarkersForSide(const std::string& side,
                                                             const Eigen::Vector3d& handPos,
                                                             const Eigen::Vector3d& elbowPos,
                                                             const Eigen::Vector3d& shoulderPos,
                                                             const Eigen::Vector3d& chestPos) {
  // 复现Python版本的颜色和尺寸配置
  if (side == "Left") {
    publishLeftSideMarkers(handPos, elbowPos, shoulderPos);
  } else if (side == "Right") {
    publishRightSideMarkers(handPos, elbowPos, shoulderPos);
  }

  // 胸部marker - 每次都发布（复现Python L871-873逻辑）
  publishChestMarker(chestPos);
}

void WheelKeyFramesVisualizer::publishLeftSideMarkers(const Eigen::Vector3d& handPos,
                                                 const Eigen::Vector3d& elbowPos,
                                                 const Eigen::Vector3d& shoulderPos) {
  // 左手marker - 红色，尺寸0.08，透明度0.9（复现Python L837）
  auto handMarker = constructPointMarker(handPos, 0.08, 0.9, {1, 0, 0}, "base_link");
  markerPub_.publish(handMarker);

  // 左肘marker - 绿色，尺寸0.1，透明度0.3（复现Python L838默认值）
  auto elbowMarker = constructPointMarker(elbowPos, 0.1, 0.3, {0, 1, 0}, "base_link");
  markerPubElbow_.publish(elbowMarker);

  // 左肩marker - 蓝色，尺寸0.1，透明度0.3（复现Python L839默认值）
  auto shoulderMarker = constructPointMarker(shoulderPos, 0.1, 0.3, {0, 0, 1}, "base_link");
  markerPubShoulder_.publish(shoulderMarker);
}

void WheelKeyFramesVisualizer::publishRightSideMarkers(const Eigen::Vector3d& handPos,
                                                  const Eigen::Vector3d& elbowPos,
                                                  const Eigen::Vector3d& shoulderPos) {
  // 右手marker - 红色，尺寸0.08，透明度0.9（复现Python L837）
  auto handMarker = constructPointMarker(handPos, 0.08, 0.9, {1, 0, 0}, "base_link");
  markerPubRight_.publish(handMarker);

  // 右肘marker - 绿色，尺寸0.1，透明度0.3（复现Python L838默认值）
  auto elbowMarker = constructPointMarker(elbowPos, 0.1, 0.3, {0, 1, 0}, "base_link");
  markerPubElbowRight_.publish(elbowMarker);

  // 右肩marker - 蓝色，尺寸0.1，透明度0.3（复现Python L839默认值）
  auto shoulderMarker = constructPointMarker(shoulderPos, 0.1, 0.3, {0, 0, 1}, "base_link");
  markerPubShoulderRight_.publish(shoulderMarker);
}

void WheelKeyFramesVisualizer::publishChestMarker(const Eigen::Vector3d& chestPos) {
  // 胸部marker - 蓝色，尺寸0.1，透明度0.8（每次都发布，复现Python L871-873逻辑）
  // 注意：使用原始胸部位置，不使用变换后的位置
  auto chestMarker = constructPointMarker(chestPos, 0.1, 0.8, {0, 0, 1}, "base_link");
  markerPubChest_.publish(chestMarker);
}

void WheelKeyFramesVisualizer::publishHumanArrayLeft(const std::vector<visualization_msgs::Marker>& markers) {
  auto markerArray = constructMarkerArray(markers, "base_link");
  markerPubHumanArrayLeft_.publish(markerArray);
}

void WheelKeyFramesVisualizer::publishHumanArrayRight(const std::vector<visualization_msgs::Marker>& markers) {
  auto markerArray = constructMarkerArray(markers, "base_link");
  markerPubHumanArrayRight_.publish(markerArray);
}

visualization_msgs::Marker WheelKeyFramesVisualizer::constructPointMarker(const Eigen::Vector3d& point,
                                                                     double scale,
                                                                     double alpha,
                                                                     const std::vector<double>& color,
                                                                     const std::string& frameId) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  // 注意：不设置marker.id，复现Python版本行为

  // 设置尺寸
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  // 设置颜色
  marker.color.a = alpha;
  marker.color.r = color.size() > 0 ? color[0] : 0.0;
  marker.color.g = color.size() > 1 ? color[1] : 0.0;
  marker.color.b = color.size() > 2 ? color[2] : 1.0;

  // 设置位置
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();

  // 设置方向（单位四元数）
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  return marker;
}

visualization_msgs::Marker WheelKeyFramesVisualizer::constructMeshMarker(const Eigen::Vector3d& position,
                                                                    const Eigen::Quaterniond& orientation,
                                                                    const std::vector<double>& rgba,
                                                                    const std::string& side,
                                                                    int markerId,
                                                                    const std::string& frameId) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = markerId;

  // 根据侧别设置不同的网格资源（这里需要根据实际STL文件路径配置）
  // 注意：这里需要根据实际的模型路径进行配置
  if (side == "Left") {
    marker.mesh_resource = "package://kuavo_description/meshes/left_hand.stl";  // 示例路径
  } else if (side == "Right") {
    marker.mesh_resource = "package://kuavo_description/meshes/right_hand.stl";  // 示例路径
  }

  // 设置缩放
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // 设置颜色
  marker.color.a = rgba.size() > 3 ? rgba[3] : 1.0;
  marker.color.r = rgba.size() > 0 ? rgba[0] : 1.0;
  marker.color.g = rgba.size() > 1 ? rgba[1] : 0.0;
  marker.color.b = rgba.size() > 2 ? rgba[2] : 0.0;

  // 设置位置
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();

  // 设置方向
  marker.pose.orientation.w = orientation.w();
  marker.pose.orientation.x = orientation.x();
  marker.pose.orientation.y = orientation.y();
  marker.pose.orientation.z = orientation.z();

  return marker;
}

visualization_msgs::MarkerArray WheelKeyFramesVisualizer::constructMarkerArray(
    const std::vector<visualization_msgs::Marker>& markers,
    const std::string& frameId) {
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers = markers;

  // 确保所有标记都使用相同的坐标系
  for (auto& marker : markerArray.markers) {
    marker.header.frame_id = frameId;
    marker.header.stamp = ros::Time::now();
  }

  return markerArray;
}

void WheelKeyFramesVisualizer::publishIncrementalModePositions(const WheelIncrementalPoseResult& incrementalResult,
                                                          const std::vector<PoseData>& poseConstraintList) {
  if (!incrementalResult.isValid()) return;

  // 发布锚点位置
  leftAnchorPosPublisher_.publish(eigenToPoint(incrementalResult.getLeftAnchorPos()));
  rightAnchorPosPublisher_.publish(eigenToPoint(incrementalResult.getRightAnchorPos()));

  // 发布测量位置
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    leftHandMeasuredPosPublisher_.publish(eigenToPoint(poseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position));
  }
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    rightHandMeasuredPosPublisher_.publish(eigenToPoint(poseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position));
  }
}

void WheelKeyFramesVisualizer::publishIncrementalPoseVisualization(
    const WheelIncrementalPoseResult& incrementalResult,
    const std::vector<PoseData>& poseConstraintList,
    std::function<void(Eigen::Vector3d&, Eigen::Quaterniond&, Eigen::Vector3d&, Eigen::Quaterniond&)> fkCallback) {
  if (!incrementalResult.isValid()) return;

  ros::Time currentTime = ros::Time::now();
  Eigen::Vector3d visLeftPos, visRightPos, incrementalLeftElbowPos, incrementalRightElbowPos;
  Eigen::Quaterniond visLeftQuat, visRightQuat;

  // 通过FK回调计算临时的FK位置用于可视化
  if (fkCallback) {
    Eigen::Quaterniond tempLeftQuat, tempRightQuat;  // 临时四元数变量，这里不使用
    fkCallback(visLeftPos, tempLeftQuat, visRightPos, tempRightQuat);
  } else {
    // 如果没有FK回调，使用零向量作为默认值
    visLeftPos = Eigen::Vector3d::Zero();
    visRightPos = Eigen::Vector3d::Zero();
  }

  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    visLeftQuat = Eigen::Quaterniond(poseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix).normalized();
  } else {
    visLeftQuat = Eigen::Quaterniond::Identity();
  }

  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    visRightQuat = Eigen::Quaterniond(poseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix).normalized();
  } else {
    visRightQuat = Eigen::Quaterniond::Identity();
  }

  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    incrementalLeftElbowPos = poseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  } else {
    incrementalLeftElbowPos = Eigen::Vector3d::Zero();
  }

  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    incrementalRightElbowPos = poseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  } else {
    incrementalRightElbowPos = Eigen::Vector3d::Zero();
  }

  // 发布左手pose (PoseStamped)
  geometry_msgs::PoseStamped leftHandPoseStamped;
  leftHandPoseStamped.header.frame_id = "base_link";
  leftHandPoseStamped.header.stamp = currentTime;
  leftHandPoseStamped.pose.position.x = visLeftPos.x();
  leftHandPoseStamped.pose.position.y = visLeftPos.y();
  leftHandPoseStamped.pose.position.z = visLeftPos.z();
  leftHandPoseStamped.pose.orientation.w = visLeftQuat.w();
  leftHandPoseStamped.pose.orientation.x = visLeftQuat.x();
  leftHandPoseStamped.pose.orientation.y = visLeftQuat.y();
  leftHandPoseStamped.pose.orientation.z = visLeftQuat.z();
  leftHandPosePublisher_.publish(leftHandPoseStamped);

  // 发布右手pose (PoseStamped)
  geometry_msgs::PoseStamped rightHandPoseStamped;
  rightHandPoseStamped.header.frame_id = "base_link";
  rightHandPoseStamped.header.stamp = currentTime;
  rightHandPoseStamped.pose.position.x = visRightPos.x();
  rightHandPoseStamped.pose.position.y = visRightPos.y();
  rightHandPoseStamped.pose.position.z = visRightPos.z();
  rightHandPoseStamped.pose.orientation.w = visRightQuat.w();
  rightHandPoseStamped.pose.orientation.x = visRightQuat.x();
  rightHandPoseStamped.pose.orientation.y = visRightQuat.y();
  rightHandPoseStamped.pose.orientation.z = visRightQuat.z();
  rightHandPosePublisher_.publish(rightHandPoseStamped);

  // 发布左肘pose (PoseStamped，只有位置，姿态设为identity)
  geometry_msgs::PoseStamped leftElbowPoseStamped;
  leftElbowPoseStamped.header.frame_id = "base_link";
  leftElbowPoseStamped.header.stamp = currentTime;
  leftElbowPoseStamped.pose.position.x = incrementalLeftElbowPos.x();
  leftElbowPoseStamped.pose.position.y = incrementalLeftElbowPos.y();
  leftElbowPoseStamped.pose.position.z = incrementalLeftElbowPos.z();
  leftElbowPoseStamped.pose.orientation.w = 1.0;
  leftElbowPoseStamped.pose.orientation.x = 0.0;
  leftElbowPoseStamped.pose.orientation.y = 0.0;
  leftElbowPoseStamped.pose.orientation.z = 0.0;
  leftElbowPoseStampedPublisher_.publish(leftElbowPoseStamped);

  // 发布右肘pose (PoseStamped，只有位置，姿态设为identity)
  geometry_msgs::PoseStamped rightElbowPoseStamped;
  rightElbowPoseStamped.header.frame_id = "base_link";
  rightElbowPoseStamped.header.stamp = currentTime;
  rightElbowPoseStamped.pose.position.x = incrementalRightElbowPos.x();
  rightElbowPoseStamped.pose.position.y = incrementalRightElbowPos.y();
  rightElbowPoseStamped.pose.position.z = incrementalRightElbowPos.z();
  rightElbowPoseStamped.pose.orientation.w = 1.0;
  rightElbowPoseStamped.pose.orientation.x = 0.0;
  rightElbowPoseStamped.pose.orientation.y = 0.0;
  rightElbowPoseStamped.pose.orientation.z = 0.0;
  rightElbowPoseStampedPublisher_.publish(rightElbowPoseStamped);

  // 发布左手6D pose可视化数据 (PoseArray)
  geometry_msgs::PoseArray leftHandPoseArray;
  leftHandPoseArray.header.frame_id = "base_link";
  leftHandPoseArray.header.stamp = currentTime;
  geometry_msgs::Pose leftHandPose;
  leftHandPose.position.x = visLeftPos.x();
  leftHandPose.position.y = visLeftPos.y();
  leftHandPose.position.z = visLeftPos.z();
  leftHandPose.orientation.w = visLeftQuat.w();
  leftHandPose.orientation.x = visLeftQuat.x();
  leftHandPose.orientation.y = visLeftQuat.y();
  leftHandPose.orientation.z = visLeftQuat.z();
  leftHandPoseArray.poses.push_back(leftHandPose);
  leftHandTargetPoseVisualPublisher_.publish(leftHandPoseArray);

  // 发布右手6D pose可视化数据 (PoseArray)
  geometry_msgs::PoseArray rightHandPoseArray;
  rightHandPoseArray.header.frame_id = "base_link";
  rightHandPoseArray.header.stamp = currentTime;
  geometry_msgs::Pose rightHandPose;
  rightHandPose.position.x = visRightPos.x();
  rightHandPose.position.y = visRightPos.y();
  rightHandPose.position.z = visRightPos.z();
  rightHandPose.orientation.w = visRightQuat.w();
  rightHandPose.orientation.x = visRightQuat.x();
  rightHandPose.orientation.y = visRightQuat.y();
  rightHandPose.orientation.z = visRightQuat.z();
  rightHandPoseArray.poses.push_back(rightHandPose);
  rightHandTargetPoseVisualPublisher_.publish(rightHandPoseArray);

  // 发布measured部分的可视化数据（通过sensor data加FK计算手部末端位姿）
  if (fkCallback) {
    Eigen::Vector3d leftMeasuredPosition, rightMeasuredPosition;
    Eigen::Quaterniond leftMeasuredQuaternion, rightMeasuredQuaternion;
    fkCallback(leftMeasuredPosition, leftMeasuredQuaternion, rightMeasuredPosition, rightMeasuredQuaternion);

    // 发布左手measured pose可视化数据 (PoseArray)
    geometry_msgs::PoseArray leftHandMeasuredPoseArray;
    leftHandMeasuredPoseArray.header.frame_id = "base_link";
    leftHandMeasuredPoseArray.header.stamp = currentTime;
    geometry_msgs::Pose leftHandMeasuredPose;
    leftHandMeasuredPose.position.x = leftMeasuredPosition.x();
    leftHandMeasuredPose.position.y = leftMeasuredPosition.y();
    leftHandMeasuredPose.position.z = leftMeasuredPosition.z();
    leftHandMeasuredPose.orientation.w = leftMeasuredQuaternion.w();
    leftHandMeasuredPose.orientation.x = leftMeasuredQuaternion.x();
    leftHandMeasuredPose.orientation.y = leftMeasuredQuaternion.y();
    leftHandMeasuredPose.orientation.z = leftMeasuredQuaternion.z();
    leftHandMeasuredPoseArray.poses.push_back(leftHandMeasuredPose);
    leftHandMeasuredPoseVisualPublisher_.publish(leftHandMeasuredPoseArray);

    // 发布右手measured pose可视化数据 (PoseArray)
    geometry_msgs::PoseArray rightHandMeasuredPoseArray;
    rightHandMeasuredPoseArray.header.frame_id = "base_link";
    rightHandMeasuredPoseArray.header.stamp = currentTime;
    geometry_msgs::Pose rightHandMeasuredPose;
    rightHandMeasuredPose.position.x = rightMeasuredPosition.x();
    rightHandMeasuredPose.position.y = rightMeasuredPosition.y();
    rightHandMeasuredPose.position.z = rightMeasuredPosition.z();
    rightHandMeasuredPose.orientation.w = rightMeasuredQuaternion.w();
    rightHandMeasuredPose.orientation.x = rightMeasuredQuaternion.x();
    rightHandMeasuredPose.orientation.y = rightMeasuredQuaternion.y();
    rightHandMeasuredPose.orientation.z = rightMeasuredQuaternion.z();
    rightHandMeasuredPoseArray.poses.push_back(rightHandMeasuredPose);
    rightHandMeasuredPoseVisualPublisher_.publish(rightHandMeasuredPoseArray);
  }
}

void WheelKeyFramesVisualizer::publishHandXAxisVectorVisualization(const Eigen::Vector3d& leftHandPos,
                                                              const Eigen::Vector3d& leftXAxisVector,
                                                              const Eigen::Vector3d& rightHandPos,
                                                              const Eigen::Vector3d& rightXAxisVector) {
  ros::Time currentTime = ros::Time::now();
  const double vectorLength = 0.2;  // 向量长度（米）

  // 发布左手X轴向量可视化
  visualization_msgs::Marker leftHandXAxisMarker;
  leftHandXAxisMarker.header.frame_id = "base_link";
  leftHandXAxisMarker.header.stamp = currentTime;
  leftHandXAxisMarker.ns = "left_hand_x_axis";
  leftHandXAxisMarker.id = 0;
  leftHandXAxisMarker.type = visualization_msgs::Marker::ARROW;
  leftHandXAxisMarker.action = visualization_msgs::Marker::ADD;

  // 设置起点（手部位置）
  geometry_msgs::Point startPoint;
  startPoint.x = leftHandPos.x();
  startPoint.y = leftHandPos.y();
  startPoint.z = leftHandPos.z();
  leftHandXAxisMarker.points.push_back(startPoint);

  // 设置终点（起点 + 向量方向 * 长度）
  geometry_msgs::Point endPoint;
  endPoint.x = leftHandPos.x() + leftXAxisVector.x() * vectorLength;
  endPoint.y = leftHandPos.y() + leftXAxisVector.y() * vectorLength;
  endPoint.z = leftHandPos.z() + leftXAxisVector.z() * vectorLength;
  leftHandXAxisMarker.points.push_back(endPoint);

  // 设置箭头属性
  leftHandXAxisMarker.scale.x = 0.02;  // 箭头轴直径
  leftHandXAxisMarker.scale.y = 0.04;  // 箭头头直径
  leftHandXAxisMarker.scale.z = 0.0;   // 不使用
  leftHandXAxisMarker.color.a = 1.0;   // 不透明
  leftHandXAxisMarker.color.r = 1.0;   // 红色
  leftHandXAxisMarker.color.g = 0.0;
  leftHandXAxisMarker.color.b = 0.0;

  leftHandXAxisVectorPublisher_.publish(leftHandXAxisMarker);

  // 发布右手X轴向量可视化
  visualization_msgs::Marker rightHandXAxisMarker;
  rightHandXAxisMarker.header.frame_id = "base_link";
  rightHandXAxisMarker.header.stamp = currentTime;
  rightHandXAxisMarker.ns = "right_hand_x_axis";
  rightHandXAxisMarker.id = 0;
  rightHandXAxisMarker.type = visualization_msgs::Marker::ARROW;
  rightHandXAxisMarker.action = visualization_msgs::Marker::ADD;

  // 设置起点（手部位置）
  geometry_msgs::Point rightStartPoint;
  rightStartPoint.x = rightHandPos.x();
  rightStartPoint.y = rightHandPos.y();
  rightStartPoint.z = rightHandPos.z();
  rightHandXAxisMarker.points.push_back(rightStartPoint);

  // 设置终点（起点 + 向量方向 * 长度）
  geometry_msgs::Point rightEndPoint;
  rightEndPoint.x = rightHandPos.x() + rightXAxisVector.x() * vectorLength;
  rightEndPoint.y = rightHandPos.y() + rightXAxisVector.y() * vectorLength;
  rightEndPoint.z = rightHandPos.z() + rightXAxisVector.z() * vectorLength;
  rightHandXAxisMarker.points.push_back(rightEndPoint);

  // 设置箭头属性
  rightHandXAxisMarker.scale.x = 0.02;  // 箭头轴直径
  rightHandXAxisMarker.scale.y = 0.04;  // 箭头头直径
  rightHandXAxisMarker.scale.z = 0.0;   // 不使用
  rightHandXAxisMarker.color.a = 1.0;   // 不透明
  rightHandXAxisMarker.color.r = 0.0;
  rightHandXAxisMarker.color.g = 0.0;
  rightHandXAxisMarker.color.b = 1.0;  // 蓝色

  rightHandXAxisVectorPublisher_.publish(rightHandXAxisMarker);
}

void WheelKeyFramesVisualizer::publishShoulderElbowPosVisualization(const Eigen::Vector3d& leftShoulderPos,
                                                               const Eigen::Vector3d& rightShoulderPos,
                                                               const Eigen::Vector3d& leftElbowPos,
                                                               const Eigen::Vector3d& rightElbowPos) {
  ros::Time currentTime = ros::Time::now();

  // 发布左肩位置可视化（蓝色球体）
  visualization_msgs::Marker leftShoulderMarker;
  leftShoulderMarker.header.frame_id = "base_link";
  leftShoulderMarker.header.stamp = currentTime;
  leftShoulderMarker.ns = "incremental_left_shoulder";
  leftShoulderMarker.id = 0;
  leftShoulderMarker.type = visualization_msgs::Marker::SPHERE;
  leftShoulderMarker.action = visualization_msgs::Marker::ADD;
  leftShoulderMarker.pose.position.x = leftShoulderPos.x();
  leftShoulderMarker.pose.position.y = leftShoulderPos.y();
  leftShoulderMarker.pose.position.z = leftShoulderPos.z();
  leftShoulderMarker.pose.orientation.w = 1.0;
  leftShoulderMarker.scale.x = 0.2845 * 2;  // 球体直径（半径0.2845）
  leftShoulderMarker.scale.y = 0.2845 * 2;
  leftShoulderMarker.scale.z = 0.2845 * 2;
  leftShoulderMarker.color.a = 0.25;  // 25%透明度
  leftShoulderMarker.color.r = 0.0;
  leftShoulderMarker.color.g = 0.0;
  leftShoulderMarker.color.b = 1.0;  // 蓝色
  leftShoulderPosPublisher_.publish(leftShoulderMarker);

  // 发布左肩圆心（不透明小球体）
  visualization_msgs::Marker leftShoulderCenterMarker;
  leftShoulderCenterMarker.header.frame_id = "base_link";
  leftShoulderCenterMarker.header.stamp = currentTime;
  leftShoulderCenterMarker.ns = "incremental_left_shoulder";
  leftShoulderCenterMarker.id = 1;  // 使用不同的id
  leftShoulderCenterMarker.type = visualization_msgs::Marker::SPHERE;
  leftShoulderCenterMarker.action = visualization_msgs::Marker::ADD;
  leftShoulderCenterMarker.pose.position.x = leftShoulderPos.x();
  leftShoulderCenterMarker.pose.position.y = leftShoulderPos.y();
  leftShoulderCenterMarker.pose.position.z = leftShoulderPos.z();
  leftShoulderCenterMarker.pose.orientation.w = 1.0;
  leftShoulderCenterMarker.scale.x = 0.1;  // 小球体直径
  leftShoulderCenterMarker.scale.y = 0.1;
  leftShoulderCenterMarker.scale.z = 0.1;
  leftShoulderCenterMarker.color.a = 1.0;  // 不透明
  leftShoulderCenterMarker.color.r = 0.0;
  leftShoulderCenterMarker.color.g = 0.0;
  leftShoulderCenterMarker.color.b = 1.0;  // 蓝色
  leftShoulderPosPublisher_.publish(leftShoulderCenterMarker);

  // 发布右肩位置可视化（蓝色球体）
  visualization_msgs::Marker rightShoulderMarker;
  rightShoulderMarker.header.frame_id = "base_link";
  rightShoulderMarker.header.stamp = currentTime;
  rightShoulderMarker.ns = "incremental_right_shoulder";
  rightShoulderMarker.id = 0;
  rightShoulderMarker.type = visualization_msgs::Marker::SPHERE;
  rightShoulderMarker.action = visualization_msgs::Marker::ADD;
  rightShoulderMarker.pose.position.x = rightShoulderPos.x();
  rightShoulderMarker.pose.position.y = rightShoulderPos.y();
  rightShoulderMarker.pose.position.z = rightShoulderPos.z();
  rightShoulderMarker.pose.orientation.w = 1.0;
  rightShoulderMarker.scale.x = 0.2845 * 2;  // 球体直径（半径0.2845）
  rightShoulderMarker.scale.y = 0.2845 * 2;
  rightShoulderMarker.scale.z = 0.2845 * 2;
  rightShoulderMarker.color.a = 0.25;  // 25%透明度
  rightShoulderMarker.color.r = 0.0;
  rightShoulderMarker.color.g = 0.0;
  rightShoulderMarker.color.b = 1.0;  // 蓝色
  rightShoulderPosPublisher_.publish(rightShoulderMarker);

  // 发布右肩圆心（不透明小球体）
  visualization_msgs::Marker rightShoulderCenterMarker;
  rightShoulderCenterMarker.header.frame_id = "base_link";
  rightShoulderCenterMarker.header.stamp = currentTime;
  rightShoulderCenterMarker.ns = "incremental_right_shoulder";
  rightShoulderCenterMarker.id = 1;  // 使用不同的id
  rightShoulderCenterMarker.type = visualization_msgs::Marker::SPHERE;
  rightShoulderCenterMarker.action = visualization_msgs::Marker::ADD;
  rightShoulderCenterMarker.pose.position.x = rightShoulderPos.x();
  rightShoulderCenterMarker.pose.position.y = rightShoulderPos.y();
  rightShoulderCenterMarker.pose.position.z = rightShoulderPos.z();
  rightShoulderCenterMarker.pose.orientation.w = 1.0;
  rightShoulderCenterMarker.scale.x = 0.1;  // 小球体直径
  rightShoulderCenterMarker.scale.y = 0.1;
  rightShoulderCenterMarker.scale.z = 0.1;
  rightShoulderCenterMarker.color.a = 1.0;  // 不透明
  rightShoulderCenterMarker.color.r = 0.0;
  rightShoulderCenterMarker.color.g = 0.0;
  rightShoulderCenterMarker.color.b = 1.0;  // 蓝色
  rightShoulderPosPublisher_.publish(rightShoulderCenterMarker);

  // 发布左肘位置可视化（绿色球体）
  visualization_msgs::Marker leftElbowMarker;
  leftElbowMarker.header.frame_id = "base_link";
  leftElbowMarker.header.stamp = currentTime;
  leftElbowMarker.ns = "incremental_left_elbow";
  leftElbowMarker.id = 0;
  leftElbowMarker.type = visualization_msgs::Marker::SPHERE;
  leftElbowMarker.action = visualization_msgs::Marker::ADD;
  leftElbowMarker.pose.position.x = leftElbowPos.x();
  leftElbowMarker.pose.position.y = leftElbowPos.y();
  leftElbowMarker.pose.position.z = leftElbowPos.z();
  leftElbowMarker.pose.orientation.w = 1.0;
  leftElbowMarker.scale.x = 0.08;  // 球体直径
  leftElbowMarker.scale.y = 0.08;
  leftElbowMarker.scale.z = 0.08;
  leftElbowMarker.color.a = 1.0;  // 不透明
  leftElbowMarker.color.r = 0.0;
  leftElbowMarker.color.g = 1.0;  // 绿色
  leftElbowMarker.color.b = 0.0;
  leftElbowPosMarkerPublisher_.publish(leftElbowMarker);

  // 发布右肘位置可视化（绿色球体）
  visualization_msgs::Marker rightElbowMarker;
  rightElbowMarker.header.frame_id = "base_link";
  rightElbowMarker.header.stamp = currentTime;
  rightElbowMarker.ns = "incremental_right_elbow";
  rightElbowMarker.id = 0;
  rightElbowMarker.type = visualization_msgs::Marker::SPHERE;
  rightElbowMarker.action = visualization_msgs::Marker::ADD;
  rightElbowMarker.pose.position.x = rightElbowPos.x();
  rightElbowMarker.pose.position.y = rightElbowPos.y();
  rightElbowMarker.pose.position.z = rightElbowPos.z();
  rightElbowMarker.pose.orientation.w = 1.0;
  rightElbowMarker.scale.x = 0.08;  // 球体直径
  rightElbowMarker.scale.y = 0.08;
  rightElbowMarker.scale.z = 0.08;
  rightElbowMarker.color.a = 1.0;  // 不透明
  rightElbowMarker.color.r = 0.0;
  rightElbowMarker.color.g = 1.0;  // 绿色
  rightElbowMarker.color.b = 0.0;
  rightElbowPosMarkerPublisher_.publish(rightElbowMarker);
}

void WheelKeyFramesVisualizer::publishHandSphereConstraintVisualization(const Eigen::Vector3d& leftShoulderPos,
                                                                   const Eigen::Vector3d& rightShoulderPos,
                                                                   double sphereRadius,
                                                                   double minReachableDistance) {
  ros::Time currentTime = ros::Time::now();

  // ========== 左手外部大球约束可视化（半透明球体） ==========
  visualization_msgs::Marker leftHandSphereMarker;
  leftHandSphereMarker.header.frame_id = "base_link";
  leftHandSphereMarker.header.stamp = currentTime;
  leftHandSphereMarker.ns = "left_hand_sphere_constraint";
  leftHandSphereMarker.id = 0;  // 外部大球
  leftHandSphereMarker.type = visualization_msgs::Marker::SPHERE;
  leftHandSphereMarker.action = visualization_msgs::Marker::ADD;
  leftHandSphereMarker.pose.position.x = leftShoulderPos.x();
  leftHandSphereMarker.pose.position.y = leftShoulderPos.y();
  leftHandSphereMarker.pose.position.z = leftShoulderPos.z();
  leftHandSphereMarker.pose.orientation.w = 1.0;  // 无旋转
  leftHandSphereMarker.pose.orientation.x = 0.0;
  leftHandSphereMarker.pose.orientation.y = 0.0;
  leftHandSphereMarker.pose.orientation.z = 0.0;
  leftHandSphereMarker.scale.x = sphereRadius * 2.0;  // 球体直径
  leftHandSphereMarker.scale.y = sphereRadius * 2.0;
  leftHandSphereMarker.scale.z = sphereRadius * 2.0;
  leftHandSphereMarker.color.a = 0.2;  // 20%透明度
  leftHandSphereMarker.color.r = 1.0;  // 红色
  leftHandSphereMarker.color.g = 0.0;
  leftHandSphereMarker.color.b = 0.0;
  leftHandBoundBoxPublisher_.publish(leftHandSphereMarker);

  // ========== 左手内部小球约束可视化（半透明球体） ==========
  visualization_msgs::Marker leftHandInnerSphereMarker;
  leftHandInnerSphereMarker.header.frame_id = "base_link";
  leftHandInnerSphereMarker.header.stamp = currentTime;
  leftHandInnerSphereMarker.ns = "left_hand_sphere_constraint";
  leftHandInnerSphereMarker.id = 1;  // 内部小球
  leftHandInnerSphereMarker.type = visualization_msgs::Marker::SPHERE;
  leftHandInnerSphereMarker.action = visualization_msgs::Marker::ADD;
  leftHandInnerSphereMarker.pose.position.x = leftShoulderPos.x();
  leftHandInnerSphereMarker.pose.position.y = leftShoulderPos.y();
  leftHandInnerSphereMarker.pose.position.z = leftShoulderPos.z();
  leftHandInnerSphereMarker.pose.orientation.w = 1.0;  // 无旋转
  leftHandInnerSphereMarker.pose.orientation.x = 0.0;
  leftHandInnerSphereMarker.pose.orientation.y = 0.0;
  leftHandInnerSphereMarker.pose.orientation.z = 0.0;
  leftHandInnerSphereMarker.scale.x = minReachableDistance * 2.0;  // 球体直径
  leftHandInnerSphereMarker.scale.y = minReachableDistance * 2.0;
  leftHandInnerSphereMarker.scale.z = minReachableDistance * 2.0;
  leftHandInnerSphereMarker.color.a = 0.3;  // 30%透明度
  leftHandInnerSphereMarker.color.r = 1.0;  // 红色
  leftHandInnerSphereMarker.color.g = 0.5;  // 偏橙色，便于区分
  leftHandInnerSphereMarker.color.b = 0.0;
  leftHandBoundBoxPublisher_.publish(leftHandInnerSphereMarker);

  // ========== 右手外部大球约束可视化（半透明球体） ==========
  visualization_msgs::Marker rightHandSphereMarker;
  rightHandSphereMarker.header.frame_id = "base_link";
  rightHandSphereMarker.header.stamp = currentTime;
  rightHandSphereMarker.ns = "right_hand_sphere_constraint";
  rightHandSphereMarker.id = 0;  // 外部大球
  rightHandSphereMarker.type = visualization_msgs::Marker::SPHERE;
  rightHandSphereMarker.action = visualization_msgs::Marker::ADD;
  rightHandSphereMarker.pose.position.x = rightShoulderPos.x();
  rightHandSphereMarker.pose.position.y = rightShoulderPos.y();
  rightHandSphereMarker.pose.position.z = rightShoulderPos.z();
  rightHandSphereMarker.pose.orientation.w = 1.0;  // 无旋转
  rightHandSphereMarker.pose.orientation.x = 0.0;
  rightHandSphereMarker.pose.orientation.y = 0.0;
  rightHandSphereMarker.pose.orientation.z = 0.0;
  rightHandSphereMarker.scale.x = sphereRadius * 2.0;  // 球体直径
  rightHandSphereMarker.scale.y = sphereRadius * 2.0;
  rightHandSphereMarker.scale.z = sphereRadius * 2.0;
  rightHandSphereMarker.color.a = 0.2;  // 20%透明度
  rightHandSphereMarker.color.r = 0.0;
  rightHandSphereMarker.color.g = 0.0;
  rightHandSphereMarker.color.b = 1.0;  // 蓝色
  rightHandBoundBoxPublisher_.publish(rightHandSphereMarker);

  // ========== 右手内部小球约束可视化（半透明球体） ==========
  visualization_msgs::Marker rightHandInnerSphereMarker;
  rightHandInnerSphereMarker.header.frame_id = "base_link";
  rightHandInnerSphereMarker.header.stamp = currentTime;
  rightHandInnerSphereMarker.ns = "right_hand_sphere_constraint";
  rightHandInnerSphereMarker.id = 1;  // 内部小球
  rightHandInnerSphereMarker.type = visualization_msgs::Marker::SPHERE;
  rightHandInnerSphereMarker.action = visualization_msgs::Marker::ADD;
  rightHandInnerSphereMarker.pose.position.x = rightShoulderPos.x();
  rightHandInnerSphereMarker.pose.position.y = rightShoulderPos.y();
  rightHandInnerSphereMarker.pose.position.z = rightShoulderPos.z();
  rightHandInnerSphereMarker.pose.orientation.w = 1.0;  // 无旋转
  rightHandInnerSphereMarker.pose.orientation.x = 0.0;
  rightHandInnerSphereMarker.pose.orientation.y = 0.0;
  rightHandInnerSphereMarker.pose.orientation.z = 0.0;
  rightHandInnerSphereMarker.scale.x = minReachableDistance * 2.0;  // 球体直径
  rightHandInnerSphereMarker.scale.y = minReachableDistance * 2.0;
  rightHandInnerSphereMarker.scale.z = minReachableDistance * 2.0;
  rightHandInnerSphereMarker.color.a = 0.3;  // 30%透明度
  rightHandInnerSphereMarker.color.r = 0.0;
  rightHandInnerSphereMarker.color.g = 0.5;  // 偏青色，便于区分
  rightHandInnerSphereMarker.color.b = 1.0;  // 蓝色
  rightHandBoundBoxPublisher_.publish(rightHandInnerSphereMarker);
}

void WheelKeyFramesVisualizer::publishHandCylinderConstraintVisualization(const Eigen::Vector3d& leftCenter,
                                                                     const Eigen::Vector3d& rightCenter,
                                                                     double cylinderRadius) {
  ros::Time currentTime = ros::Time::now();

  // 发布左手圆柱体约束可视化（半透明圆柱体，沿z轴方向）
  visualization_msgs::Marker leftHandCylinderMarker;
  leftHandCylinderMarker.header.frame_id = "base_link";
  leftHandCylinderMarker.header.stamp = currentTime;
  leftHandCylinderMarker.ns = "left_hand_cylinder_constraint";
  leftHandCylinderMarker.id = 0;
  leftHandCylinderMarker.type = visualization_msgs::Marker::CYLINDER;
  leftHandCylinderMarker.action = visualization_msgs::Marker::ADD;
  leftHandCylinderMarker.pose.position.x = leftCenter.x();
  leftHandCylinderMarker.pose.position.y = leftCenter.y();
  leftHandCylinderMarker.pose.position.z = leftCenter.z();
  leftHandCylinderMarker.pose.orientation.w = 1.0;  // 无旋转，沿z轴方向
  leftHandCylinderMarker.pose.orientation.x = 0.0;
  leftHandCylinderMarker.pose.orientation.y = 0.0;
  leftHandCylinderMarker.pose.orientation.z = 0.0;
  leftHandCylinderMarker.scale.x = cylinderRadius * 2.0;  // 圆柱体直径（x方向）
  leftHandCylinderMarker.scale.y = cylinderRadius * 2.0;  // 圆柱体直径（y方向）
  leftHandCylinderMarker.scale.z = 2.0;  // 圆柱体高度（z方向），足够大以覆盖工作空间
  leftHandCylinderMarker.color.a = 0.3;  // 30%透明度
  leftHandCylinderMarker.color.r = 0.0;
  leftHandCylinderMarker.color.g = 1.0;  // 绿色
  leftHandCylinderMarker.color.b = 0.0;
  leftHandBoundBoxPublisher_.publish(leftHandCylinderMarker);

  // 发布右手圆柱体约束可视化（半透明圆柱体，沿z轴方向）
  visualization_msgs::Marker rightHandCylinderMarker;
  rightHandCylinderMarker.header.frame_id = "base_link";
  rightHandCylinderMarker.header.stamp = currentTime;
  rightHandCylinderMarker.ns = "right_hand_cylinder_constraint";
  rightHandCylinderMarker.id = 0;
  rightHandCylinderMarker.type = visualization_msgs::Marker::CYLINDER;
  rightHandCylinderMarker.action = visualization_msgs::Marker::ADD;
  rightHandCylinderMarker.pose.position.x = rightCenter.x();
  rightHandCylinderMarker.pose.position.y = rightCenter.y();
  rightHandCylinderMarker.pose.position.z = rightCenter.z();
  rightHandCylinderMarker.pose.orientation.w = 1.0;  // 无旋转，沿z轴方向
  rightHandCylinderMarker.pose.orientation.x = 0.0;
  rightHandCylinderMarker.pose.orientation.y = 0.0;
  rightHandCylinderMarker.pose.orientation.z = 0.0;
  rightHandCylinderMarker.scale.x = cylinderRadius * 2.0;  // 圆柱体直径（x方向）
  rightHandCylinderMarker.scale.y = cylinderRadius * 2.0;  // 圆柱体直径（y方向）
  rightHandCylinderMarker.scale.z = 2.0;  // 圆柱体高度（z方向），足够大以覆盖工作空间
  rightHandCylinderMarker.color.a = 0.3;  // 30%透明度
  rightHandCylinderMarker.color.r = 0.0;
  rightHandCylinderMarker.color.g = 0.0;
  rightHandCylinderMarker.color.b = 1.0;  // 蓝色
  rightHandBoundBoxPublisher_.publish(rightHandCylinderMarker);
}

void WheelKeyFramesVisualizer::publishElbowDistanceConstraintVisualization(const Eigen::Vector3d& leftElbowPos,
                                                                      const Eigen::Vector3d& rightElbowPos) {
  ros::Time currentTime = ros::Time::now();

  // 只有当elbow位置有效（非零）时才发布可视化
  if (leftElbowPos.norm() < 1e-6 && rightElbowPos.norm() < 1e-6) {
    return;  // 两个elbow位置都无效，不发布可视化
  }

  // ========== 左手外部大球约束可视化（最大距离0.55，半透明球体） ==========
  if (leftElbowPos.norm() > 1e-6) {
    visualization_msgs::Marker leftElbowMaxSphereMarker;
    leftElbowMaxSphereMarker.header.frame_id = "base_link";
    leftElbowMaxSphereMarker.header.stamp = currentTime;
    leftElbowMaxSphereMarker.ns = "left_elbow_distance_constraint";
    leftElbowMaxSphereMarker.id = 0;  // 外部大球
    leftElbowMaxSphereMarker.type = visualization_msgs::Marker::SPHERE;
    leftElbowMaxSphereMarker.action = visualization_msgs::Marker::ADD;
    leftElbowMaxSphereMarker.pose.position.x = leftElbowPos.x();
    leftElbowMaxSphereMarker.pose.position.y = leftElbowPos.y();
    leftElbowMaxSphereMarker.pose.position.z = leftElbowPos.z();
    leftElbowMaxSphereMarker.pose.orientation.w = 1.0;  // 无旋转
    leftElbowMaxSphereMarker.pose.orientation.x = 0.0;
    leftElbowMaxSphereMarker.pose.orientation.y = 0.0;
    leftElbowMaxSphereMarker.pose.orientation.z = 0.0;
    leftElbowMaxSphereMarker.scale.x = elbowMaxDistance_ * 2.0;  // 球体直径
    leftElbowMaxSphereMarker.scale.y = elbowMaxDistance_ * 2.0;
    leftElbowMaxSphereMarker.scale.z = elbowMaxDistance_ * 2.0;
    leftElbowMaxSphereMarker.color.a = 0.2;  // 20%透明度
    leftElbowMaxSphereMarker.color.r = 1.0;  // 红色
    leftElbowMaxSphereMarker.color.g = 0.8;  // 偏粉红色，便于区分
    leftElbowMaxSphereMarker.color.b = 0.8;
    leftHandBoundBoxPublisher_.publish(leftElbowMaxSphereMarker);

    // ========== 左手内部小球约束可视化（最小距离0.3，半透明球体） ==========
    visualization_msgs::Marker leftElbowMinSphereMarker;
    leftElbowMinSphereMarker.header.frame_id = "base_link";
    leftElbowMinSphereMarker.header.stamp = currentTime;
    leftElbowMinSphereMarker.ns = "left_elbow_distance_constraint";
    leftElbowMinSphereMarker.id = 1;  // 内部小球
    leftElbowMinSphereMarker.type = visualization_msgs::Marker::SPHERE;
    leftElbowMinSphereMarker.action = visualization_msgs::Marker::ADD;
    leftElbowMinSphereMarker.pose.position.x = leftElbowPos.x();
    leftElbowMinSphereMarker.pose.position.y = leftElbowPos.y();
    leftElbowMinSphereMarker.pose.position.z = leftElbowPos.z();
    leftElbowMinSphereMarker.pose.orientation.w = 1.0;  // 无旋转
    leftElbowMinSphereMarker.pose.orientation.x = 0.0;
    leftElbowMinSphereMarker.pose.orientation.y = 0.0;
    leftElbowMinSphereMarker.pose.orientation.z = 0.0;
    leftElbowMinSphereMarker.scale.x = elbowMinDistance_ * 2.0;  // 球体直径
    leftElbowMinSphereMarker.scale.y = elbowMinDistance_ * 2.0;
    leftElbowMinSphereMarker.scale.z = elbowMinDistance_ * 2.0;
    leftElbowMinSphereMarker.color.a = 0.3;  // 30%透明度
    leftElbowMinSphereMarker.color.r = 1.0;  // 红色偏橙
    leftElbowMinSphereMarker.color.g = 0.6;
    leftElbowMinSphereMarker.color.b = 0.0;
    leftHandBoundBoxPublisher_.publish(leftElbowMinSphereMarker);
  }

  // ========== 右手外部大球约束可视化（最大距离0.55，半透明球体） ==========
  if (rightElbowPos.norm() > 1e-6) {
    visualization_msgs::Marker rightElbowMaxSphereMarker;
    rightElbowMaxSphereMarker.header.frame_id = "base_link";
    rightElbowMaxSphereMarker.header.stamp = currentTime;
    rightElbowMaxSphereMarker.ns = "right_elbow_distance_constraint";
    rightElbowMaxSphereMarker.id = 0;  // 外部大球
    rightElbowMaxSphereMarker.type = visualization_msgs::Marker::SPHERE;
    rightElbowMaxSphereMarker.action = visualization_msgs::Marker::ADD;
    rightElbowMaxSphereMarker.pose.position.x = rightElbowPos.x();
    rightElbowMaxSphereMarker.pose.position.y = rightElbowPos.y();
    rightElbowMaxSphereMarker.pose.position.z = rightElbowPos.z();
    rightElbowMaxSphereMarker.pose.orientation.w = 1.0;  // 无旋转
    rightElbowMaxSphereMarker.pose.orientation.x = 0.0;
    rightElbowMaxSphereMarker.pose.orientation.y = 0.0;
    rightElbowMaxSphereMarker.pose.orientation.z = 0.0;
    rightElbowMaxSphereMarker.scale.x = elbowMaxDistance_ * 2.0;  // 球体直径
    rightElbowMaxSphereMarker.scale.y = elbowMaxDistance_ * 2.0;
    rightElbowMaxSphereMarker.scale.z = elbowMaxDistance_ * 2.0;
    rightElbowMaxSphereMarker.color.a = 0.2;  // 20%透明度
    rightElbowMaxSphereMarker.color.r = 0.8;  // 偏粉蓝色，便于区分
    rightElbowMaxSphereMarker.color.g = 0.8;
    rightElbowMaxSphereMarker.color.b = 1.0;  // 蓝色
    rightHandBoundBoxPublisher_.publish(rightElbowMaxSphereMarker);

    // ========== 右手内部小球约束可视化（最小距离0.3，半透明球体） ==========
    visualization_msgs::Marker rightElbowMinSphereMarker;
    rightElbowMinSphereMarker.header.frame_id = "base_link";
    rightElbowMinSphereMarker.header.stamp = currentTime;
    rightElbowMinSphereMarker.ns = "right_elbow_distance_constraint";
    rightElbowMinSphereMarker.id = 1;  // 内部小球
    rightElbowMinSphereMarker.type = visualization_msgs::Marker::SPHERE;
    rightElbowMinSphereMarker.action = visualization_msgs::Marker::ADD;
    rightElbowMinSphereMarker.pose.position.x = rightElbowPos.x();
    rightElbowMinSphereMarker.pose.position.y = rightElbowPos.y();
    rightElbowMinSphereMarker.pose.position.z = rightElbowPos.z();
    rightElbowMinSphereMarker.pose.orientation.w = 1.0;  // 无旋转
    rightElbowMinSphereMarker.pose.orientation.x = 0.0;
    rightElbowMinSphereMarker.pose.orientation.y = 0.0;
    rightElbowMinSphereMarker.pose.orientation.z = 0.0;
    rightElbowMinSphereMarker.scale.x = elbowMinDistance_ * 2.0;  // 球体直径
    rightElbowMinSphereMarker.scale.y = elbowMinDistance_ * 2.0;
    rightElbowMinSphereMarker.scale.z = elbowMinDistance_ * 2.0;
    rightElbowMinSphereMarker.color.a = 0.3;  // 30%透明度
    rightElbowMinSphereMarker.color.r = 0.0;
    rightElbowMinSphereMarker.color.g = 0.6;  // 偏青色
    rightElbowMinSphereMarker.color.b = 1.0;  // 蓝色
    rightHandBoundBoxPublisher_.publish(rightElbowMinSphereMarker);
  }
}

void WheelKeyFramesVisualizer::publishBoxBoundVisualization(const Eigen::Vector3d& boxMinBound,
                                                       const Eigen::Vector3d& boxMaxBound,
                                                       double chestOffsetY) {
  ros::Time currentTime = ros::Time::now();

  // 计算左手最终生效的 box 边界（考虑胸部中线约束）
  // 左手：y 轴最大值不超过 -chestOffsetY_
  const double leftBoxMinY = boxMinBound.y();
  const double leftBoxMaxY = std::min(boxMaxBound.y(), -chestOffsetY);
  const double leftBoxCenterX = (boxMinBound.x() + boxMaxBound.x()) / 2.0;
  const double leftBoxCenterY = (leftBoxMinY + leftBoxMaxY) / 2.0;
  const double leftBoxCenterZ = (boxMinBound.z() + boxMaxBound.z()) / 2.0;
  const double leftBoxSizeX = boxMaxBound.x() - boxMinBound.x();
  const double leftBoxSizeY = leftBoxMaxY - leftBoxMinY;
  const double leftBoxSizeZ = boxMaxBound.z() - boxMinBound.z();

  // 发布左手 box 可视化（使用 CUBE 类型）
  visualization_msgs::Marker leftBoxMarker;
  leftBoxMarker.header.frame_id = "base_link";
  leftBoxMarker.header.stamp = currentTime;
  leftBoxMarker.ns = "left_hand_box_bound_constraint";
  leftBoxMarker.id = 0;
  leftBoxMarker.type = visualization_msgs::Marker::CUBE;
  leftBoxMarker.action = visualization_msgs::Marker::ADD;
  leftBoxMarker.pose.position.x = leftBoxCenterX;
  leftBoxMarker.pose.position.y = leftBoxCenterY;
  leftBoxMarker.pose.position.z = leftBoxCenterZ;
  leftBoxMarker.pose.orientation.w = 1.0;  // 无旋转
  leftBoxMarker.pose.orientation.x = 0.0;
  leftBoxMarker.pose.orientation.y = 0.0;
  leftBoxMarker.pose.orientation.z = 0.0;
  leftBoxMarker.scale.x = leftBoxSizeX;
  leftBoxMarker.scale.y = leftBoxSizeY;
  leftBoxMarker.scale.z = leftBoxSizeZ;
  leftBoxMarker.color.a = 0.2;  // 20% 透明度
  leftBoxMarker.color.r = 1.0;  // 红色
  leftBoxMarker.color.g = 0.0;
  leftBoxMarker.color.b = 0.0;

  // 通过左手 box 话题发布器发送
  leftHandBoundBoxPublisher_.publish(leftBoxMarker);

  // 计算右手最终生效的 box 边界（考虑胸部中线约束）
  // 右手：y 轴最小值不小于 chestOffsetY_
  const double rightBoxMinY = std::max(boxMinBound.y(), chestOffsetY);
  const double rightBoxMaxY = boxMaxBound.y();
  const double rightBoxCenterX = (boxMinBound.x() + boxMaxBound.x()) / 2.0;
  const double rightBoxCenterY = (rightBoxMinY + rightBoxMaxY) / 2.0;
  const double rightBoxCenterZ = (boxMinBound.z() + boxMaxBound.z()) / 2.0;
  const double rightBoxSizeX = boxMaxBound.x() - boxMinBound.x();
  const double rightBoxSizeY = rightBoxMaxY - rightBoxMinY;
  const double rightBoxSizeZ = boxMaxBound.z() - boxMinBound.z();

  // 发布右手 box 可视化（使用 CUBE 类型）
  visualization_msgs::Marker rightBoxMarker;
  rightBoxMarker.header.frame_id = "base_link";
  rightBoxMarker.header.stamp = currentTime;
  rightBoxMarker.ns = "right_hand_box_bound_constraint";
  rightBoxMarker.id = 0;
  rightBoxMarker.type = visualization_msgs::Marker::CUBE;
  rightBoxMarker.action = visualization_msgs::Marker::ADD;
  rightBoxMarker.pose.position.x = rightBoxCenterX;
  rightBoxMarker.pose.position.y = rightBoxCenterY;
  rightBoxMarker.pose.position.z = rightBoxCenterZ;
  rightBoxMarker.pose.orientation.w = 1.0;  // 无旋转
  rightBoxMarker.pose.orientation.x = 0.0;
  rightBoxMarker.pose.orientation.y = 0.0;
  rightBoxMarker.pose.orientation.z = 0.0;
  rightBoxMarker.scale.x = rightBoxSizeX;
  rightBoxMarker.scale.y = rightBoxSizeY;
  rightBoxMarker.scale.z = rightBoxSizeZ;
  rightBoxMarker.color.a = 0.2;  // 20% 透明度
  rightBoxMarker.color.r = 0.0;
  rightBoxMarker.color.g = 0.0;
  rightBoxMarker.color.b = 1.0;  // 蓝色

  // 通过右手 box 话题发布器发送
  rightHandBoundBoxPublisher_.publish(rightBoxMarker);
}

void WheelKeyFramesVisualizer::publishAllVisualizations(
    const Eigen::Vector3d& leftShoulderPos,
    const Eigen::Vector3d& rightShoulderPos,
    double sphereRadius,
    double minReachableDistance,
    const WheelIncrementalPoseResult& incrementalResult,
    const std::vector<PoseData>& poseConstraintList,
    const Eigen::Vector3d& boxMinBound,
    const Eigen::Vector3d& boxMaxBound,
    double chestOffsetY,
    const Eigen::Vector3d& leftCylinderCenter,
    const Eigen::Vector3d& rightCylinderCenter,
    double cylinderRadius,
    const Eigen::Vector3d& leftElbowPos,
    const Eigen::Vector3d& rightElbowPos,
    std::function<void(Eigen::Vector3d&, Eigen::Quaterniond&, Eigen::Vector3d&, Eigen::Quaterniond&)> fkCallback,
    const Eigen::Vector3d& leftLink6Pos,
    const Eigen::Vector3d& rightLink6Pos,
    const Eigen::Vector3d& leftEndEffectorPos,
    const Eigen::Vector3d& rightEndEffectorPos,
    const Eigen::Vector3d& leftVirtualThumbPos,
    const Eigen::Vector3d& rightVirtualThumbPos) {
  // 从poseConstraintList中获取肘部位置（用于shoulder-elbow可视化）
  Eigen::Vector3d leftElbowPosFromList = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightElbowPosFromList = Eigen::Vector3d::Zero();
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_ELBOW) {
    leftElbowPosFromList = poseConstraintList[POSE_DATA_LIST_INDEX_LEFT_ELBOW].position;
  }
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_ELBOW) {
    rightElbowPosFromList = poseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_ELBOW].position;
  }

  publishShoulderElbowPosVisualization(leftShoulderPos, rightShoulderPos, leftElbowPosFromList, rightElbowPosFromList);
  publishHandSphereConstraintVisualization(leftShoulderPos, rightShoulderPos, sphereRadius, minReachableDistance);
  publishHandCylinderConstraintVisualization(leftCylinderCenter, rightCylinderCenter, cylinderRadius);
  publishBoxBoundVisualization(boxMinBound, boxMaxBound, chestOffsetY);
  // 发布手到肘距离约束可视化（使用实时FK计算的elbow位置）
  publishElbowDistanceConstraintVisualization(leftElbowPos, rightElbowPos);
  publishIncrementalPoseVisualization(incrementalResult, poseConstraintList, fkCallback);
  publishVisualization(poseConstraintList);

  // 发布增量模式位置（锚点和测量位置）
  publishIncrementalModePositions(incrementalResult, poseConstraintList);

  // 发布六个关键点可视化
  publishSixKeyPointsVisualization(
      leftLink6Pos, rightLink6Pos, leftEndEffectorPos, rightEndEffectorPos, leftVirtualThumbPos, rightVirtualThumbPos);
}

void WheelKeyFramesVisualizer::publishVisualization(const std::vector<PoseData>& poseConstraintList) {
  // 发布缩放后的手部位置可视化
  // 从poseConstraintList中读取scaledLeftHandPos和scaledRightHandPos
  // 使用不同的颜色和尺寸来区分scaled位置和FK计算的位置
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_LEFT_HAND) {
    auto scaledLeftHandMarker = constructPointMarker(poseConstraintList[POSE_DATA_LIST_INDEX_LEFT_HAND].position,
                                                     0.1,
                                                     0.8,
                                                     {1, 0, 0},
                                                     "base_link");  // 红色，尺寸0.1
    scaledLeftHandPosPublisher_.publish(scaledLeftHandMarker);
  }
  if (poseConstraintList.size() > POSE_DATA_LIST_INDEX_RIGHT_HAND) {
    auto scaledRightHandMarker = constructPointMarker(poseConstraintList[POSE_DATA_LIST_INDEX_RIGHT_HAND].position,
                                                      0.1,
                                                      0.8,
                                                      {1, 0, 0},
                                                      "base_link");  // 红色，尺寸0.1
    scaledRightHandPosPublisher_.publish(scaledRightHandMarker);
  }
}

void WheelKeyFramesVisualizer::publishSixKeyPointsVisualization(const Eigen::Vector3d& leftLink6Pos,
                                                           const Eigen::Vector3d& rightLink6Pos,
                                                           const Eigen::Vector3d& leftEndEffectorPos,
                                                           const Eigen::Vector3d& rightEndEffectorPos,
                                                           const Eigen::Vector3d& leftVirtualThumbPos,
                                                           const Eigen::Vector3d& rightVirtualThumbPos) {
  // 统一所有点的尺寸为0.06（与thumb一致）
  const double markerSize = 0.06;

  // 发布左臂Link4位置（绿色）
  auto leftLink4Marker = constructPointMarker(leftLink6Pos, markerSize, 0.9, {0, 1, 0}, "base_link");
  leftLink4PosPublisher_.publish(leftLink4Marker);

  // 发布右臂Link4位置（绿色）
  auto rightLink4Marker = constructPointMarker(rightLink6Pos, markerSize, 0.9, {0, 1, 0}, "base_link");
  rightLink4PosPublisher_.publish(rightLink4Marker);

  // 发布左臂末端执行器位置（蓝色）
  auto leftEndEffectorMarker = constructPointMarker(leftEndEffectorPos, markerSize, 0.9, {0, 0, 1}, "base_link");
  leftEndEffectorPosPublisher_.publish(leftEndEffectorMarker);

  // 发布右臂末端执行器位置（蓝色）
  auto rightEndEffectorMarker = constructPointMarker(rightEndEffectorPos, markerSize, 0.9, {0, 0, 1}, "base_link");
  rightEndEffectorPosPublisher_.publish(rightEndEffectorMarker);

  // 发布左虚拟拇指位置（黄色）
  auto leftVirtualThumbMarker = constructPointMarker(leftVirtualThumbPos, markerSize, 0.9, {1, 1, 0}, "base_link");
  leftVirtualThumbPosPublisher_.publish(leftVirtualThumbMarker);

  // 发布右虚拟拇指位置（黄色）
  auto rightVirtualThumbMarker = constructPointMarker(rightVirtualThumbPos, markerSize, 0.9, {1, 1, 0}, "base_link");
  rightVirtualThumbPosPublisher_.publish(rightVirtualThumbMarker);
}

void WheelKeyFramesVisualizer::publishRobotElbowPosVisualization(const Eigen::Vector3d& leftElbowPos,
                                                            const Eigen::Vector3d& rightElbowPos) {
  // 墨绿色 RGB(0, 0.5, 0.5)，不透明 alpha=1.0，尺寸0.08
  const double markerSize = 0.1;
  const double alpha = 1.0;
  const std::vector<double> darkGreenColor = {0.0, 0.5, 0.5};  // 墨绿色

  // 发布机器人左肘优化位置（墨绿色，不透明）
  auto leftElbowMarker = constructPointMarker(leftElbowPos, markerSize, alpha, darkGreenColor, "base_link");
  robotLeftElbowPosPublisher_.publish(leftElbowMarker);

  // 发布机器人右肘优化位置（墨绿色，不透明）
  auto rightElbowMarker = constructPointMarker(rightElbowPos, markerSize, alpha, darkGreenColor, "base_link");
  robotRightElbowPosPublisher_.publish(rightElbowMarker);
}

void WheelKeyFramesVisualizer::publishHandPosOptimizationVisualization(const Eigen::Vector3d& leftHandPosBeforeOpt,
                                                                  const Eigen::Vector3d& leftHandPosAfterOpt,
                                                                  const Eigen::Vector3d& rightHandPosBeforeOpt,
                                                                  const Eigen::Vector3d& rightHandPosAfterOpt) {
  const double markerSize = 0.12;
  const double alpha = 1.0;

  // 优化前的手部位置：橙色 (RGB: 1.0, 0.5, 0.0)
  const std::vector<double> orangeColor = {1.0, 0.5, 0.0};
  // 优化后的手部位置：紫色 (RGB: 0.8, 0.0, 0.8)
  const std::vector<double> purpleColor = {0.8, 0.0, 0.8};

  // 发布优化前的左手位置（橙色）
  auto leftHandBeforeMarker = constructPointMarker(leftHandPosBeforeOpt, markerSize, alpha, orangeColor, "base_link");
  leftHandPosBeforeOptPublisher_.publish(leftHandBeforeMarker);

  // 发布优化后的左手位置（紫色）
  auto leftHandAfterMarker = constructPointMarker(leftHandPosAfterOpt, markerSize, alpha, purpleColor, "base_link");
  leftHandPosAfterOptPublisher_.publish(leftHandAfterMarker);

  // 发布优化前的右手位置（橙色）
  auto rightHandBeforeMarker = constructPointMarker(rightHandPosBeforeOpt, markerSize, alpha, orangeColor, "base_link");
  rightHandPosBeforeOptPublisher_.publish(rightHandBeforeMarker);

  // 发布优化后的右手位置（紫色）
  auto rightHandAfterMarker = constructPointMarker(rightHandPosAfterOpt, markerSize, alpha, purpleColor, "base_link");
  rightHandPosAfterOptPublisher_.publish(rightHandAfterMarker);
}

}  // namespace HighlyDynamic
