/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include <humanoid_wheel_interface/ManipulatorModelInfo.h>
#include <humanoid_wheel_interface/HumanoidWheelInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <kuavo_msgs/lejuClawCommand.h>
#include <kuavo_msgs/lejuClawState.h>
#include <sensor_msgs/JointState.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorDummyVisualization final : public DummyObserver {
 public:
  MobileManipulatorDummyVisualization(ros::NodeHandle& nodeHandle, const HumanoidWheelInterface& interface)
      : pinocchioInterface_(interface.getPinocchioInterface()), modelInfo_(interface.getManipulatorModelInfo()) {
    launchVisualizerNode(nodeHandle);
  }

  ~MobileManipulatorDummyVisualization() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

  void update_obs(const SystemObservation& observation);

  // 更新头部关节位置
  void updateHeadJointPositions(const Eigen::VectorXd& positions);

 private:
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);
  void publishTargetTrajectories(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);
  void publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy);

  PinocchioInterface pinocchioInterface_;
  const ManipulatorModelInfo modelInfo_;
  std::vector<std::string> removeJointNames_;

  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
  tf::TransformBroadcaster tfBroadcaster_;

  // 可视化发布统一节流：/tf、目标轨迹、优化轨迹、自碰撞距离等都跟随控制环(数百Hz)发布，
  // 跨机会被多节点复制订阅打满百兆链路(尤其 /tf 被底盘节点重复订阅)。
  double lastPublishTime_ = -1e9;            // 上次发布的 observation.time（秒），负初值保证首帧立即发布
  double minPublishTimeDifference_ = 1.0 / 30.0; // 最小发布间隔（默认 30Hz），从 task.info 的 visualization.publishRate 读取；0 表示不节流

  // 缓存 URDF 模型,用于运行期判断关节是否存在,避免向 robot_state_publisher 发布
  // URDF 中不存在的关节(否则会刷出 "Joint state with name: ... was received but not found in URDF")
  urdf::Model urdfModel_;

  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher stateOptimizedPosePublisher_;

  // 每个末端执行器的完整位姿轨迹（PoseArray，用于 rosbag 记录）
  std::vector<ros::Publisher> optimizedEePosePublishers_;
  // 每个末端执行器预测轨迹最终点的 TF frame 名称
  std::vector<std::string> eePredictedFrameNames_;

  std::unique_ptr<GeometryInterfaceVisualization> geometryVisualization_;

  // 头部关节相关
  std::vector<std::string> head_joint_names_;
  std::vector<double> head_joint_positions_;
  bool updateHeadJointPositions_ = false;

  // 夹爪相关
  bool updateClawJointPositions_ = true;
  bool useArticulatedClawKinematics_ = false;
  std::vector<double> claw_joint_positions_ = {0.0, 0.0};
  ros::Subscriber clawCmdSubscriber_;
  ros::Publisher lejuClawStatePub_;
  void lejuClawCmdCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg);
  void updateClawJointPositions(const Eigen::VectorXd& positions);

  // 灵巧手相关
  std::vector<std::string> dexhand_joint_names_;
  std::vector<double> dexhand_joint_positions_;
  bool updateDexhandJointPositions_ = false;
  ros::Subscriber dexhandStateSubscriber_;
  void dexhandStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void updateHandJointPositions(const Eigen::VectorXd& positions);

  // 检测到外部 odom->base_link 时（/external_odom/active 为 true），不发布内部的 odom->base_link
  bool use_external_odom_tf_ = false;
  std::mutex external_odom_mutex_;
  ros::Subscriber external_odom_active_sub_;

  void externalOdomActiveCallback(const std_msgs::BoolConstPtr& msg);
};

}  // namespace mobile_manipulator
}  // namespace ocs2
