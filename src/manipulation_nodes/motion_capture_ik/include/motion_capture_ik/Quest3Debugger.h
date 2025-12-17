#pragma once

#include <noitom_hi5_hand_udp_python/PoseInfoList.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace HighlyDynamic {

/**
 * @brief Quest3调试器类，负责发布调试相关的ROS消息
 *
 * 该类封装了Quest3IK系统中用于调试的发布功能，包括：
 * - 骨骼姿态数据发布
 * - IK求解阶段结果发布
 *
 * 设计参考KeyFramesVisualizer的模式，提供统一的调试数据发布接口
 */
class Quest3Debugger {
 public:
  explicit Quest3Debugger(ros::NodeHandle& nodeHandle);
  ~Quest3Debugger() = default;

  /**
   * @brief 初始化调试器
   */
  void initialize();

  /**
   * @brief 发布骨骼姿态手肘数据
   * @param poseListMsg 姿态信息列表消息
   */
  void publishBonePoseHandElbow(const noitom_hi5_hand_udp_python::PoseInfoList& poseListMsg);

  /**
   * @brief 发布第一阶段IK求解结果
   * @param stage1Result 第一阶段求解结果向量
   */
  void publishStage1Result(const Eigen::VectorXd& stage1Result);

  /**
   * @brief 发布第二阶段IK求解结果
   * @param stage2Result 第二阶段求解结果向量
   */
  void publishStage2Result(const Eigen::VectorXd& stage2Result);

  /**
   * @brief 发布完整的IK调试数据包
   * @param poseListMsg 姿态信息列表消息
   * @param stage1Result 第一阶段求解结果（可选）
   * @param stage2Result 第二阶段求解结果（可选）
   */
  void publishDebugDataPackage(const noitom_hi5_hand_udp_python::PoseInfoList& poseListMsg,
                               const std::shared_ptr<Eigen::VectorXd>& stage1Result = nullptr,
                               const std::shared_ptr<Eigen::VectorXd>& stage2Result = nullptr);

 private:
  /**
   * @brief 初始化所有调试发布器
   */
  void initializePublishers();

  /**
   * @brief 构造Float64MultiArray消息
   * @param eigenVector Eigen向量数据
   * @return 构造的Float64MultiArray消息
   */
  std_msgs::Float64MultiArray constructFloat64MultiArrayMsg(const Eigen::VectorXd& eigenVector);

  /**
   * @brief 验证Eigen向量数据有效性
   * @param eigenVector 待验证的向量
   * @param stageName 阶段名称（用于日志输出）
   * @return 数据是否有效
   */
  bool validateEigenVector(const Eigen::VectorXd& eigenVector, const std::string& stageName);

  // ROS发布器
  ros::Publisher bonePoseHandElbowPublisher_;  // 骨骼姿态发布器
  ros::Publisher stage1ResultPublisher_;       // 第一阶段结果发布器
  ros::Publisher stage2ResultPublisher_;       // 第二阶段结果发布器

  // ROS节点句柄引用
  ros::NodeHandle& nodeHandle_;
};

}  // namespace HighlyDynamic
