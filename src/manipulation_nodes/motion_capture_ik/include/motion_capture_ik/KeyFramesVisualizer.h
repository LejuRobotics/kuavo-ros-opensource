#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace HighlyDynamic {

class KeyFramesVisualizer {
 public:
  explicit KeyFramesVisualizer(ros::NodeHandle& nodeHandle);
  ~KeyFramesVisualizer() = default;

  void initialize();

  /**
   * @brief 发布特定侧别的可视化标记
   * @param side 侧别标识 ("Left" 或 "Right")
   * @param handPos 手部位置
   * @param elbowPos 肘部位置
   * @param shoulderPos 肩部位置
   * @param chestPos 胸部位置
   */
  void publishVisualizationMarkersForSide(const std::string& side,
                                          const Eigen::Vector3d& handPos,
                                          const Eigen::Vector3d& elbowPos,
                                          const Eigen::Vector3d& shoulderPos,
                                          const Eigen::Vector3d& chestPos);

  /**
   * @brief 发布左手相关标记
   * @param handPos 手部位置
   * @param elbowPos 肘部位置
   * @param shoulderPos 肩部位置
   */
  void publishLeftSideMarkers(const Eigen::Vector3d& handPos,
                              const Eigen::Vector3d& elbowPos,
                              const Eigen::Vector3d& shoulderPos);

  /**
   * @brief 发布右手相关标记
   * @param handPos 手部位置
   * @param elbowPos 肘部位置
   * @param shoulderPos 肩部位置
   */
  void publishRightSideMarkers(const Eigen::Vector3d& handPos,
                               const Eigen::Vector3d& elbowPos,
                               const Eigen::Vector3d& shoulderPos);

  /**
   * @brief 发布胸部标记
   * @param chestPos 胸部位置
   */
  void publishChestMarker(const Eigen::Vector3d& chestPos);

  /**
   * @brief 发布人体数组标记（左侧）
   * @param markers 标记向量
   */
  void publishHumanArrayLeft(const std::vector<visualization_msgs::Marker>& markers);

  /**
   * @brief 发布人体数组标记（右侧）
   * @param markers 标记向量
   */
  void publishHumanArrayRight(const std::vector<visualization_msgs::Marker>& markers);

 private:
  // 左侧可视化发布器
  ros::Publisher markerPub_;                // 左手marker
  ros::Publisher markerPubElbow_;           // 左肘marker
  ros::Publisher markerPubShoulder_;        // 左肩marker
  ros::Publisher markerPubShoulderQuest3_;  // 左肩Quest3 marker
  ros::Publisher markerPubHumanArrayLeft_;  // 左侧人体MarkerArray

  // 右侧可视化发布器
  ros::Publisher markerPubRight_;                // 右手marker
  ros::Publisher markerPubElbowRight_;           // 右肘marker
  ros::Publisher markerPubShoulderRight_;        // 右肩marker
  ros::Publisher markerPubShoulderQuest3Right_;  // 右肩Quest3 marker
  ros::Publisher markerPubHumanArrayRight_;      // 右侧人体MarkerArray

  // 通用可视化发布器
  ros::Publisher markerPubChest_;  // 胸部marker

  /**
   * @brief 初始化所有可视化发布器
   */
  void initializePublishers();

  /**
   * @brief 构造点类型标记
   * @param point 点位置
   * @param scale 缩放比例，默认0.05
   * @param alpha 透明度，默认0.3
   * @param color RGB颜色数组，默认蓝色 {0, 0, 1}
   * @param frameId 坐标系ID，默认"base_link"
   * @return 构造的标记消息
   */
  visualization_msgs::Marker constructPointMarker(const Eigen::Vector3d& point,
                                                  double scale = 0.05,
                                                  double alpha = 0.3,
                                                  const std::vector<double>& color = {0, 0, 1},
                                                  const std::string& frameId = "base_link");

  /**
   * @brief 构造网格类型标记
   * @param position 位置
   * @param orientation 方向（四元数）
   * @param rgba RGBA颜色数组
   * @param side 侧别标识
   * @param markerId 标记ID
   * @param frameId 坐标系ID，默认"base_link"
   * @return 构造的标记消息
   */
  visualization_msgs::Marker constructMeshMarker(const Eigen::Vector3d& position,
                                                 const Eigen::Quaterniond& orientation,
                                                 const std::vector<double>& rgba,
                                                 const std::string& side,
                                                 int markerId,
                                                 const std::string& frameId = "base_link");

  /**
   * @brief 构造标记数组
   * @param markers 标记向量
   * @param frameId 坐标系ID，默认"base_link"
   * @return 构造的标记数组消息
   */
  visualization_msgs::MarkerArray constructMarkerArray(const std::vector<visualization_msgs::Marker>& markers,
                                                       const std::string& frameId = "base_link");

  // ROS节点句柄引用
  ros::NodeHandle& nodeHandle_;
};

}  // namespace HighlyDynamic
