#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <functional>

#include "motion_capture_ik/IncrementalControlModule.h"
#include <leju_utils/define.hpp>

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

  /**
   * @brief 发布增量模式位置（锚点和测量位置）
   * @param incrementalResult 增量控制结果
   * @param poseConstraintList 位姿约束列表
   */
  void publishIncrementalModePositions(const IncrementalPoseResult& incrementalResult,
                                       const std::vector<PoseData>& poseConstraintList);

  /**
   * @brief 发布增量Pose可视化
   * @param incrementalResult 增量控制结果
   * @param poseConstraintList 位姿约束列表
   * @param fkCallback FK计算回调函数，用于计算measured pose
   */
  void publishIncrementalPoseVisualization(
      const IncrementalPoseResult& incrementalResult,
      const std::vector<PoseData>& poseConstraintList,
      std::function<void(Eigen::Vector3d&, Eigen::Quaterniond&, Eigen::Vector3d&, Eigen::Quaterniond&)> fkCallback);

  /**
   * @brief 发布手部X轴向量可视化
   * @param leftHandPos 左手位置
   * @param leftXAxisVector 左手X轴向量
   * @param rightHandPos 右手位置
   * @param rightXAxisVector 右手X轴向量
   */
  void publishHandXAxisVectorVisualization(const Eigen::Vector3d& leftHandPos,
                                           const Eigen::Vector3d& leftXAxisVector,
                                           const Eigen::Vector3d& rightHandPos,
                                           const Eigen::Vector3d& rightXAxisVector);

  /**
   * @brief 发布肩部和肘部位置可视化
   * @param leftShoulderPos 左肩位置
   * @param rightShoulderPos 右肩位置
   * @param leftElbowPos 左肘位置
   * @param rightElbowPos 右肘位置
   */
  void publishShoulderElbowPosVisualization(const Eigen::Vector3d& leftShoulderPos,
                                            const Eigen::Vector3d& rightShoulderPos,
                                            const Eigen::Vector3d& leftElbowPos,
                                            const Eigen::Vector3d& rightElbowPos);

  /**
   * @brief 发布手部球约束可视化（外部大球和内部小球）
   * @param leftShoulderPos 左肩位置
   * @param rightShoulderPos 右肩位置
   * @param sphereRadius 外部大球半径（sphereRadiusLimit）
   * @param minReachableDistance 内部小球半径（最小可达距离）
   */
  void publishHandSphereConstraintVisualization(const Eigen::Vector3d& leftShoulderPos,
                                                const Eigen::Vector3d& rightShoulderPos,
                                                double sphereRadius,
                                                double minReachableDistance);

  /**
   * @brief 发布手部圆柱体约束可视化
   * @param leftCenter 左手圆柱体中心位置
   * @param rightCenter 右手圆柱体中心位置
   * @param cylinderRadius 圆柱体半径
   */
  void publishHandCylinderConstraintVisualization(const Eigen::Vector3d& leftCenter,
                                                  const Eigen::Vector3d& rightCenter,
                                                  double cylinderRadius);

  /**
   * @brief 发布手到肘距离约束可视化（外部大球和内部小球）
   * @param leftElbowPos 左肘位置
   * @param rightElbowPos 右肘位置
   */
  void publishElbowDistanceConstraintVisualization(const Eigen::Vector3d& leftElbowPos,
                                                   const Eigen::Vector3d& rightElbowPos);

  /**
   * @brief 发布Box边界约束可视化
   * @param boxMinBound Box最小边界
   * @param boxMaxBound Box最大边界
   * @param chestOffsetY 胸部中线偏移量
   */
  void publishBoxBoundVisualization(const Eigen::Vector3d& boxMinBound,
                                    const Eigen::Vector3d& boxMaxBound,
                                    double chestOffsetY);

  /**
   * @brief 发布六个关键点的可视化（Link4和EndEffector位置，以及虚拟拇指位置）
   * @param leftLink6Pos 左臂Link4位置
   * @param rightLink6Pos 右臂Link4位置
   * @param leftEndEffectorPos 左臂末端执行器位置
   * @param rightEndEffectorPos 右臂末端执行器位置
   * @param leftVirtualThumbPos 左虚拟拇指位置
   * @param rightVirtualThumbPos 右虚拟拇指位置
   */
  void publishSixKeyPointsVisualization(const Eigen::Vector3d& leftLink6Pos,
                                        const Eigen::Vector3d& rightLink6Pos,
                                        const Eigen::Vector3d& leftEndEffectorPos,
                                        const Eigen::Vector3d& rightEndEffectorPos,
                                        const Eigen::Vector3d& leftVirtualThumbPos,
                                        const Eigen::Vector3d& rightVirtualThumbPos);

  /**
   * @brief 发布机器人优化后的肘部位置可视化（墨绿色，不透明）
   * @param leftElbowPos 机器人左肘优化位置
   * @param rightElbowPos 机器人右肘优化位置
   */
  void publishRobotElbowPosVisualization(const Eigen::Vector3d& leftElbowPos, const Eigen::Vector3d& rightElbowPos);

  /**
   * @brief 发布优化前后的手部位置可视化对比
   * @param leftHandPosBeforeOpt 优化前的左手位置
   * @param leftHandPosAfterOpt 优化后的左手位置
   * @param rightHandPosBeforeOpt 优化前的右手位置
   * @param rightHandPosAfterOpt 优化后的右手位置
   */
  void publishHandPosOptimizationVisualization(const Eigen::Vector3d& leftHandPosBeforeOpt,
                                               const Eigen::Vector3d& leftHandPosAfterOpt,
                                               const Eigen::Vector3d& rightHandPosBeforeOpt,
                                               const Eigen::Vector3d& rightHandPosAfterOpt);

  /**
   * @brief 统一的可视化发布函数
   * @param leftShoulderPos 左肩位置
   * @param rightShoulderPos 右肩位置
   * @param sphereRadius 外部大球半径（sphereRadiusLimit）
   * @param minReachableDistance 内部小球半径（最小可达距离）
   * @param incrementalResult 增量控制结果
   * @param poseConstraintList 位姿约束列表
   * @param boxMinBound Box最小边界
   * @param boxMaxBound Box最大边界
   * @param chestOffsetY 胸部中线偏移量
   * @param leftCylinderCenter 左手圆柱体中心位置
   * @param rightCylinderCenter 右手圆柱体中心位置
   * @param cylinderRadius 圆柱体半径
   * @param leftElbowPos 左肘位置（通过FK实时计算）
   * @param rightElbowPos 右肘位置（通过FK实时计算）
   * @param fkCallback FK计算回调函数
   */
  void publishAllVisualizations(
      const Eigen::Vector3d& leftShoulderPos,
      const Eigen::Vector3d& rightShoulderPos,
      double sphereRadius,
      double minReachableDistance,
      const IncrementalPoseResult& incrementalResult,
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
      const Eigen::Vector3d& leftLink6Pos = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& rightLink6Pos = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& leftEndEffectorPos = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& rightEndEffectorPos = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& leftVirtualThumbPos = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& rightVirtualThumbPos = Eigen::Vector3d::Zero());

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

  // 增量模式位置发布器（Point类型）
  ros::Publisher leftAnchorPosPublisher_;         // 左手锚点位置发布器
  ros::Publisher rightAnchorPosPublisher_;        // 右手锚点位置发布器
  ros::Publisher leftHandMeasuredPosPublisher_;   // 左手进入时测量位置发布器
  ros::Publisher rightHandMeasuredPosPublisher_;  // 右手进入时测量位置发布器

  // 增量结果pose发布器（PoseStamped类型）
  ros::Publisher leftHandPosePublisher_;           // 左手pose发布器
  ros::Publisher rightHandPosePublisher_;          // 右手pose发布器
  ros::Publisher leftElbowPoseStampedPublisher_;   // 左肘pose发布器
  ros::Publisher rightElbowPoseStampedPublisher_;  // 右肘pose发布器

  // 手部6D pose可视化发布器（PoseArray类型）
  ros::Publisher leftHandTargetPoseVisualPublisher_;     // 左手6D pose可视化发布器
  ros::Publisher rightHandTargetPoseVisualPublisher_;    // 右手6D pose可视化发布器
  ros::Publisher leftHandMeasuredPoseVisualPublisher_;   // 通过sensor data 加fk计算手部末端位姿
  ros::Publisher rightHandMeasuredPoseVisualPublisher_;  // 通过sensor data 加fk计算手部末端位姿

  // 手部旋转矩阵X轴向量可视化发布器（Marker类型）
  ros::Publisher leftHandXAxisVectorPublisher_;   // 左手旋转矩阵X轴向量可视化发布器
  ros::Publisher rightHandXAxisVectorPublisher_;  // 右手旋转矩阵X轴向量可视化发布器

  // 肩部和肘部位置可视化发布器（Marker类型）
  ros::Publisher leftShoulderPosPublisher_;      // 左肩位置可视化发布器
  ros::Publisher rightShoulderPosPublisher_;     // 右肩位置可视化发布器
  ros::Publisher leftElbowPosMarkerPublisher_;   // 左肘位置可视化发布器（Marker类型）
  ros::Publisher rightElbowPosMarkerPublisher_;  // 右肘位置可视化发布器（Marker类型）

  // 手部bounding box可视化发布器（Marker类型）
  ros::Publisher leftHandBoundBoxPublisher_;   // 左手bounding box可视化发布器
  ros::Publisher rightHandBoundBoxPublisher_;  // 右手bounding box可视化发布器

  // 缩放后的手部位置可视化发布器（Marker类型）
  ros::Publisher scaledLeftHandPosPublisher_;   // 缩放后的左手位置可视化发布器
  ros::Publisher scaledRightHandPosPublisher_;  // 缩放后的右手位置可视化发布器

  // 六个关键点可视化发布器（Marker类型）
  ros::Publisher leftLink4PosPublisher_;          // 左臂Link4位置可视化发布器
  ros::Publisher rightLink4PosPublisher_;         // 右臂Link4位置可视化发布器
  ros::Publisher leftEndEffectorPosPublisher_;    // 左臂末端执行器位置可视化发布器
  ros::Publisher rightEndEffectorPosPublisher_;   // 右臂末端执行器位置可视化发布器
  ros::Publisher leftVirtualThumbPosPublisher_;   // 左虚拟拇指位置可视化发布器
  ros::Publisher rightVirtualThumbPosPublisher_;  // 右虚拟拇指位置可视化发布器

  // 机器人优化后的肘部位置可视化发布器（Marker类型，墨绿色，不透明）
  ros::Publisher robotLeftElbowPosPublisher_;   // 机器人左肘优化位置可视化发布器
  ros::Publisher robotRightElbowPosPublisher_;  // 机器人右肘优化位置可视化发布器

  // 优化前后的手部位置可视化发布器（Marker类型）
  ros::Publisher leftHandPosBeforeOptPublisher_;   // 优化前的左手位置可视化发布器
  ros::Publisher leftHandPosAfterOptPublisher_;    // 优化后的左手位置可视化发布器
  ros::Publisher rightHandPosBeforeOptPublisher_;  // 优化前的右手位置可视化发布器
  ros::Publisher rightHandPosAfterOptPublisher_;   // 优化后的右手位置可视化发布器

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

  void publishVisualization(const std::vector<PoseData>& poseConstraintList);

  // ROS节点句柄引用
  ros::NodeHandle& nodeHandle_;

  // 手到肘距离约束参数
  double elbowMinDistance_;  // 最小距离（默认0.18）
  double elbowMaxDistance_;  // 最大距离（默认0.65）
};

}  // namespace HighlyDynamic
