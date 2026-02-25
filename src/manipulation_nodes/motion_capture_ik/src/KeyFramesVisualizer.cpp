#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "motion_capture_ik/KeyFramesVisualizer.h"

namespace HighlyDynamic {

KeyFramesVisualizer::KeyFramesVisualizer(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {}

void KeyFramesVisualizer::initialize() {
  ROS_INFO("[KeyFramesVisualizer] Initializing Quest3 visualization system...");
  initializePublishers();
  ROS_INFO("[KeyFramesVisualizer] Quest3 visualization system initialized successfully");
}

void KeyFramesVisualizer::initializePublishers() {
  ROS_INFO("[KeyFramesVisualizer] Initializing visualization publishers...");

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

  ROS_INFO("[KeyFramesVisualizer] All visualization publishers initialized successfully");
}

void KeyFramesVisualizer::publishVisualizationMarkersForSide(const std::string& side,
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

void KeyFramesVisualizer::publishLeftSideMarkers(const Eigen::Vector3d& handPos,
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

void KeyFramesVisualizer::publishRightSideMarkers(const Eigen::Vector3d& handPos,
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

void KeyFramesVisualizer::publishChestMarker(const Eigen::Vector3d& chestPos) {
  // 胸部marker - 蓝色，尺寸0.1，透明度0.8（每次都发布，复现Python L871-873逻辑）
  // 注意：使用原始胸部位置，不使用变换后的位置
  auto chestMarker = constructPointMarker(chestPos, 0.1, 0.8, {0, 0, 1}, "base_link");
  markerPubChest_.publish(chestMarker);
}

void KeyFramesVisualizer::publishHumanArrayLeft(const std::vector<visualization_msgs::Marker>& markers) {
  auto markerArray = constructMarkerArray(markers, "base_link");
  markerPubHumanArrayLeft_.publish(markerArray);
}

void KeyFramesVisualizer::publishHumanArrayRight(const std::vector<visualization_msgs::Marker>& markers) {
  auto markerArray = constructMarkerArray(markers, "base_link");
  markerPubHumanArrayRight_.publish(markerArray);
}

visualization_msgs::Marker KeyFramesVisualizer::constructPointMarker(const Eigen::Vector3d& point,
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

visualization_msgs::Marker KeyFramesVisualizer::constructMeshMarker(const Eigen::Vector3d& position,
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

visualization_msgs::MarkerArray KeyFramesVisualizer::constructMarkerArray(
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

}  // namespace HighlyDynamic
