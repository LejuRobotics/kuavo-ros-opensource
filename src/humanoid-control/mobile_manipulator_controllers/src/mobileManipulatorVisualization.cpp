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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/package.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <mobile_manipulator_controllers/mobileManipulatorVisualization.h>


namespace mobile_manipulator_controller {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  const std::string urdfName = "robot_description";
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    ROS_ERROR("URDF model load was NOT successful");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);

  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);
  targetTrajectoriesPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/targetTrajectories", 1);
  // Get ROS parameter
  std::string urdfFile, taskFile;
  nodeHandle.getParam("/mm/urdfFile", urdfFile);
  nodeHandle.getParam("/mm/taskFile", taskFile);
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames_, false);
  // read if self-collision checking active
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate", true);
  // create pinocchio interface
  PinocchioInterface pinocchioInterface(mobile_manipulator::createPinocchioInterface(urdfFile, modelType, removeJointNames_));
  // activate markers for self-collision visualization
  if (activateSelfCollision) {
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", collisionObjectPairs, true);
    PinocchioGeometryInterface geomInterface(pinocchioInterface, collisionObjectPairs);
    // set geometry visualization markers
    geometryVisualization_.reset(new GeometryInterfaceVisualization(std::move(pinocchioInterface), geomInterface, nodeHandle));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorVisualization::update(const SystemObservation& observation, const PrimalSolution& policy,
                                                 const CommandData& command) {
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
  publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(timeStamp, policy);
  if (geometryVisualization_ != nullptr) {
    geometryVisualization_->publishDistances(observation.state);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorVisualization::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation) {
  // publish world -> base transform
  const auto r_world_base = getBasePosition(observation.state, modelInfo_);
  const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(observation.state, modelInfo_);

  const std::string prefix_name = "mm";
  geometry_msgs::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "mm/world";
  base_tf.child_frame_id = prefix_name + "/" + modelInfo_.baseFrame;
  base_tf.transform.translation = ros_msg_helpers::getVectorMsg(r_world_base);
  base_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(q_world_base);
  tfBroadcaster_.sendTransform(base_tf);

  // publish joints transforms
  const auto j_arm = getArmJointAngles(observation.state, modelInfo_);
  std::map<std::string, scalar_t> jointPositions;
  for (size_t i = 0; i < modelInfo_.dofNames.size(); i++) {
    jointPositions[modelInfo_.dofNames[i]] = j_arm(i);
  }
  for (const auto& name : removeJointNames_) {
    jointPositions[name] = 0.0;
  }
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp, prefix_name);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorVisualization::publishTargetTrajectories(const ros::Time& timeStamp,
                                                                    const TargetTrajectories& targetTrajectories) {
  // Check if we have target trajectories
  if (targetTrajectories.stateTrajectory.empty()) {
    return;
  }
  
  const scalar_t TRAJECTORYLINEWIDTH = 0.008;  // 目标轨迹线宽，比优化轨迹稍粗
  const std::array<scalar_t, 3> orange{1.0, 0.6, 0.0};  // 橙色用于左手目标
  const std::array<scalar_t, 3> purple{0.8, 0.2, 0.8};  // 紫色用于右手目标
  
  visualization_msgs::MarkerArray targetMarkerArray;
  
  // 双手目标轨迹
  std::vector<geometry_msgs::Point> leftHandTargetTrajectory;
  std::vector<geometry_msgs::Point> rightHandTargetTrajectory;
  leftHandTargetTrajectory.reserve(targetTrajectories.stateTrajectory.size());
  rightHandTargetTrajectory.reserve(targetTrajectories.stateTrajectory.size());
  
  // 处理每个目标状态点
  for (const auto& targetState : targetTrajectories.stateTrajectory) {
    // 假设目标状态结构：[base_pose, left_hand_pose(7), right_hand_pose(7)]
    // 这里需要根据实际的状态结构来调整索引
    const size_t baseDim = modelInfo_.stateDim - modelInfo_.armDim;
    
    if (targetState.size() >= 14) {  // 至少包含基座和双手位置姿态
      // 左手目标位置 (假设在base维度之后)
      const Eigen::Vector3d leftHandTargetPos = targetState.segment<3>(0);
      leftHandTargetTrajectory.push_back(ros_msg_helpers::getPointMsg(leftHandTargetPos));
      
      // 右手目标位置 (假设在左手7维之后)
      const Eigen::Vector3d rightHandTargetPos = targetState.segment<3>(7);
      rightHandTargetTrajectory.push_back(ros_msg_helpers::getPointMsg(rightHandTargetPos));
    }
  }
  
  // 创建左手目标轨迹标记
  if (!leftHandTargetTrajectory.empty()) {
    visualization_msgs::Marker leftHandTargetMarker;
    leftHandTargetMarker.type = visualization_msgs::Marker::LINE_STRIP;
    leftHandTargetMarker.scale.x = TRAJECTORYLINEWIDTH;
    leftHandTargetMarker.points = std::move(leftHandTargetTrajectory);
    leftHandTargetMarker.pose.orientation = ros_msg_helpers::getOrientationMsg({1., 0., 0., 0.});
    leftHandTargetMarker.ns = "Left Hand Target Trajectory";
    leftHandTargetMarker.id = 0;
    leftHandTargetMarker.color.r = orange[0];
    leftHandTargetMarker.color.g = orange[1];
    leftHandTargetMarker.color.b = orange[2];
    leftHandTargetMarker.color.a = 0.8;
    targetMarkerArray.markers.push_back(std::move(leftHandTargetMarker));
  }
  
  // 创建右手目标轨迹标记
  if (!rightHandTargetTrajectory.empty()) {
    visualization_msgs::Marker rightHandTargetMarker;
    rightHandTargetMarker.type = visualization_msgs::Marker::LINE_STRIP;
    rightHandTargetMarker.scale.x = TRAJECTORYLINEWIDTH;
    rightHandTargetMarker.points = std::move(rightHandTargetTrajectory);
    rightHandTargetMarker.pose.orientation = ros_msg_helpers::getOrientationMsg({1., 0., 0., 0.});
    rightHandTargetMarker.ns = "Right Hand Target Trajectory";
    rightHandTargetMarker.id = 1;
    rightHandTargetMarker.color.r = purple[0];
    rightHandTargetMarker.color.g = purple[1];
    rightHandTargetMarker.color.b = purple[2];
    rightHandTargetMarker.color.a = 0.8;
    targetMarkerArray.markers.push_back(std::move(rightHandTargetMarker));
  }
  
  // 发布目标轨迹标记
  if (!targetMarkerArray.markers.empty()) {
    assignHeader(targetMarkerArray.markers.begin(), targetMarkerArray.markers.end(), 
                 ros_msg_helpers::getHeaderMsg("mm/world", timeStamp));
    targetTrajectoriesPublisher_.publish(targetMarkerArray);
  }
  
  // 保留原有的TF框架发布（发布最终目标位置）
  const auto& finalTargetState = targetTrajectories.stateTrajectory.back();
  
  if (finalTargetState.size() >= 7) {
    // 发布左手最终目标TF
    const Eigen::Vector3d leftHandFinalPos = finalTargetState.segment<3>(0);
    Eigen::Quaterniond leftHandFinalOrient;
    leftHandFinalOrient.coeffs() = finalTargetState.segment<4>(3);
    leftHandFinalOrient.normalize();  // 确保四元数是单位四元数
    
    geometry_msgs::TransformStamped leftHandCommand_tf;
    leftHandCommand_tf.header.stamp = timeStamp;
    leftHandCommand_tf.header.frame_id = "mm/world";
    leftHandCommand_tf.child_frame_id = "left_hand_target";
    leftHandCommand_tf.transform.translation = ros_msg_helpers::getVectorMsg(leftHandFinalPos);
    leftHandCommand_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(leftHandFinalOrient);
    tfBroadcaster_.sendTransform(leftHandCommand_tf);
  }
  
  if (finalTargetState.size() >= 14) {
    // 发布右手最终目标TF
    const Eigen::Vector3d rightHandFinalPos = finalTargetState.segment<3>(7);
    Eigen::Quaterniond rightHandFinalOrient;
    rightHandFinalOrient.coeffs() = finalTargetState.segment<4>(10);
    rightHandFinalOrient.normalize();  // 确保四元数是单位四元数
    
    geometry_msgs::TransformStamped rightHandCommand_tf;
    rightHandCommand_tf.header.stamp = timeStamp;
    rightHandCommand_tf.header.frame_id = "mm/world";
    rightHandCommand_tf.child_frame_id = "right_hand_target";
    rightHandCommand_tf.transform.translation = ros_msg_helpers::getVectorMsg(rightHandFinalPos);
    rightHandCommand_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(rightHandFinalOrient);
    tfBroadcaster_.sendTransform(rightHandCommand_tf);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const scalar_t ORIENTATIONARROWLENGTH = 0.05;  // 添加姿态箭头长度
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const std::array<scalar_t, 3> green{0.1, 0.8, 0.1};  // 为右手添加绿色
  const auto& mpcStateTrajectory = policy.stateTrajectory_;

  visualization_msgs::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // 左手和右手轨迹
  std::vector<geometry_msgs::Point> leftHandTrajectory;
  std::vector<geometry_msgs::Point> rightHandTrajectory;
  leftHandTrajectory.reserve(mpcStateTrajectory.size());
  rightHandTrajectory.reserve(mpcStateTrajectory.size());
  
  // 为轨迹线条创建分段显示以实现渐变透明度
  std::vector<visualization_msgs::Marker> trajectorySegments;
  
  // 添加姿态可视化容器
  std::vector<visualization_msgs::Marker> orientationMarkers;
  int markerIdCounter = 0;
  const int totalPoints = mpcStateTrajectory.size();
  
  // 检查是否有多个末端执行器
  const int numEndEffectors = std::min(static_cast<int>(modelInfo_.eeFrames.size()), 2);  // 最多处理两个（左手和右手）
  
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    
    // 处理多个末端执行器
    for (int eeIdx = 0; eeIdx < numEndEffectors; ++eeIdx) {
      const auto eeIndex = model.getBodyId(modelInfo_.eeFrames[eeIdx]);
      const vector_t eePosition = data.oMf[eeIndex].translation();
      const Eigen::Matrix3d eeRotation = data.oMf[eeIndex].rotation();
      
      // 根据末端执行器索引选择轨迹容器
      if (eeIdx == 0) {
        leftHandTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
      } else if (eeIdx == 1) {
        rightHandTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
      }
      
      // 每隔几个点添加姿态可视化（避免过于密集）
      if (markerIdCounter % 5 == 0) {  // 每5个点显示一个姿态
        // 计算透明度：当前时刻透明度低(0.9)，未来时刻透明度高(0.2)
        double alpha = 0.9 - 0.7 * (static_cast<double>(markerIdCounter) / static_cast<double>(totalPoints - 1));
        alpha = std::max(0.2, std::min(0.9, alpha));  // 限制在0.2到0.9之间
        
        // 为每个末端执行器创建姿态可视化
        std::string handName = (eeIdx == 0) ? "Left Hand" : "Right Hand";
        int baseId = markerIdCounter * 6 + eeIdx * 3;  // 为每个末端执行器分配不同的ID范围
        
        // X轴（红色）
        const vector_t xAxis = eeRotation.col(0) * ORIENTATIONARROWLENGTH;
        auto xArrow = ocs2::getArrowAtPointMsg(xAxis, eePosition, ocs2::Color::red);
        xArrow.ns = handName + " Orientation X";
        xArrow.id = baseId;
        xArrow.scale.x = 0.002;  // shaft diameter
        xArrow.scale.y = 0.004;  // arrow-head diameter  
        xArrow.scale.z = 0.008;  // arrow-head length
        xArrow.color.a = alpha;  // 设置透明度
        orientationMarkers.push_back(xArrow);
        
        // Y轴（绿色）
        const vector_t yAxis = eeRotation.col(1) * ORIENTATIONARROWLENGTH;
        auto yArrow = ocs2::getArrowAtPointMsg(yAxis, eePosition, ocs2::Color::green);
        yArrow.ns = handName + " Orientation Y";
        yArrow.id = baseId + 1;
        yArrow.scale.x = 0.002;  // shaft diameter
        yArrow.scale.y = 0.004;  // arrow-head diameter
        yArrow.scale.z = 0.008;  // arrow-head length
        yArrow.color.a = alpha;  // 设置透明度
        orientationMarkers.push_back(yArrow);
        
        // Z轴（蓝色）
        const vector_t zAxis = eeRotation.col(2) * ORIENTATIONARROWLENGTH;
        auto zArrow = ocs2::getArrowAtPointMsg(zAxis, eePosition, ocs2::Color::blue);
        zArrow.ns = handName + " Orientation Z";
        zArrow.id = baseId + 2;
        zArrow.scale.x = 0.002;  // shaft diameter
        zArrow.scale.y = 0.004;  // arrow-head diameter
        zArrow.scale.z = 0.008;  // arrow-head length
        zArrow.color.a = alpha;  // 设置透明度
        orientationMarkers.push_back(zArrow);
      }
    }
    markerIdCounter++;
  });

  // 创建左手分段轨迹线条以实现渐变透明度
  const int segmentSize = 10;  // 每段包含的点数
  if (!leftHandTrajectory.empty()) {
    for (int i = 0; i < static_cast<int>(leftHandTrajectory.size()) - 1; i += segmentSize) {
      std::vector<geometry_msgs::Point> segmentPoints;
      int endIdx = std::min(i + segmentSize + 1, static_cast<int>(leftHandTrajectory.size()));
      
      for (int j = i; j < endIdx; j++) {
        segmentPoints.push_back(leftHandTrajectory[j]);
      }
      
      if (segmentPoints.size() >= 2) {
        // 计算该段的透明度
        double segmentProgress = static_cast<double>(i) / static_cast<double>(leftHandTrajectory.size() - 1);
        double segmentAlpha = 0.9 - 0.7 * segmentProgress;
        segmentAlpha = std::max(0.2, std::min(0.9, segmentAlpha));
        
        // 创建线段标记
        visualization_msgs::Marker segmentMarker;
        segmentMarker.type = visualization_msgs::Marker::LINE_STRIP;
        segmentMarker.scale.x = TRAJECTORYLINEWIDTH;
        segmentMarker.points = std::move(segmentPoints);
        segmentMarker.pose.orientation = ros_msg_helpers::getOrientationMsg({1., 0., 0., 0.});
        segmentMarker.ns = "Left Hand Trajectory";
        segmentMarker.id = i / segmentSize;
        
        // 设置颜色和透明度（蓝色）
        segmentMarker.color.r = blue[0];
        segmentMarker.color.g = blue[1];
        segmentMarker.color.b = blue[2];
        segmentMarker.color.a = segmentAlpha;
        
        markerArray.markers.push_back(std::move(segmentMarker));
      }
    }
  }
  
  // 创建右手分段轨迹线条以实现渐变透明度
  if (!rightHandTrajectory.empty()) {
    for (int i = 0; i < static_cast<int>(rightHandTrajectory.size()) - 1; i += segmentSize) {
      std::vector<geometry_msgs::Point> segmentPoints;
      int endIdx = std::min(i + segmentSize + 1, static_cast<int>(rightHandTrajectory.size()));
      
      for (int j = i; j < endIdx; j++) {
        segmentPoints.push_back(rightHandTrajectory[j]);
      }
      
      if (segmentPoints.size() >= 2) {
        // 计算该段的透明度
        double segmentProgress = static_cast<double>(i) / static_cast<double>(rightHandTrajectory.size() - 1);
        double segmentAlpha = 0.9 - 0.7 * segmentProgress;
        segmentAlpha = std::max(0.2, std::min(0.9, segmentAlpha));
        
        // 创建线段标记
        visualization_msgs::Marker segmentMarker;
        segmentMarker.type = visualization_msgs::Marker::LINE_STRIP;
        segmentMarker.scale.x = TRAJECTORYLINEWIDTH;
        segmentMarker.points = std::move(segmentPoints);
        segmentMarker.pose.orientation = ros_msg_helpers::getOrientationMsg({1., 0., 0., 0.});
        segmentMarker.ns = "Right Hand Trajectory";
        segmentMarker.id = 2000 + i / segmentSize;  // 不同的ID范围避免冲突
        
        // 设置颜色和透明度（绿色）
        segmentMarker.color.r = green[0];
        segmentMarker.color.g = green[1];
        segmentMarker.color.b = green[2];
        segmentMarker.color.a = segmentAlpha;
        
        markerArray.markers.push_back(std::move(segmentMarker));
      }
    }
  }
  
  // 添加姿态标记到markerArray
  for (auto& marker : orientationMarkers) {
    markerArray.markers.push_back(std::move(marker));
  }

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    // extract from observation
    const auto r_world_base = getBasePosition(state, modelInfo_);
    const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(state, modelInfo_);

    // convert to ros message
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(r_world_base);
    pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  // 创建分段基座轨迹线条以实现渐变透明度
  for (int i = 0; i < static_cast<int>(baseTrajectory.size()) - 1; i += segmentSize) {
    std::vector<geometry_msgs::Point> segmentPoints;
    int endIdx = std::min(i + segmentSize + 1, static_cast<int>(baseTrajectory.size()));
    
    for (int j = i; j < endIdx; j++) {
      segmentPoints.push_back(baseTrajectory[j]);
    }
    
    if (segmentPoints.size() >= 2) {
      // 计算该段的透明度
      double segmentProgress = static_cast<double>(i) / static_cast<double>(baseTrajectory.size() - 1);
      double segmentAlpha = 0.9 - 0.7 * segmentProgress;
      segmentAlpha = std::max(0.2, std::min(0.9, segmentAlpha));
      
      // 创建线段标记
      visualization_msgs::Marker segmentMarker;
      segmentMarker.type = visualization_msgs::Marker::LINE_STRIP;
      segmentMarker.scale.x = TRAJECTORYLINEWIDTH;
      segmentMarker.points = std::move(segmentPoints);
      segmentMarker.pose.orientation = ros_msg_helpers::getOrientationMsg({1., 0., 0., 0.});
      segmentMarker.ns = "Base Trajectory";
      segmentMarker.id = 1000 + i / segmentSize;  // 不同的ID范围避免冲突
      
      // 设置颜色和透明度
      segmentMarker.color.r = red[0];
      segmentMarker.color.g = red[1];
      segmentMarker.color.b = red[2];
      segmentMarker.color.a = segmentAlpha;
      
      markerArray.markers.push_back(std::move(segmentMarker));
    }
  }

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("mm/world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("mm/world", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace mobile_manipulator_controller
