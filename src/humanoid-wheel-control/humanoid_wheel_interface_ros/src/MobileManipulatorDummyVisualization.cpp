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

#include <humanoid_wheel_interface/AccessHelperFunctions.h>
#include <humanoid_wheel_interface/FactoryFunctions.h>
#include <humanoid_wheel_interface/ManipulatorModelInfo.h>
#include <humanoid_wheel_interface/HumanoidWheelInterface.h>
#include <humanoid_wheel_interface_ros/MobileManipulatorDummyVisualization.h>

namespace ocs2 {
namespace mobile_manipulator {

static const double baseHeightOffset = 0.0;
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
void MobileManipulatorDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
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

  // 初始化头部关节
  head_joint_names_ = {"zhead_1_joint", "zhead_2_joint"};
  head_joint_positions_ = {0.0, 0.0};

  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);
  // Get ROS parameter
  std::string urdfFile, taskFile;
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/taskFile", taskFile);
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
void MobileManipulatorDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy,
                                                 const CommandData& command) {
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
  publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(timeStamp, policy);
  if (geometryVisualization_ != nullptr) {
    geometryVisualization_->publishDistances(observation.state);
  }
}

void MobileManipulatorDummyVisualization::update_obs(const SystemObservation& observation)
{
  const ros::Time timeStamp = ros::Time::now();

  publishObservation(timeStamp, observation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishObservation(const ros::Time& timeStamp, const SystemObservation& observation) {
  // publish world -> base transform
  const auto r_world_base = getBasePosition(observation.state, modelInfo_);
  const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(observation.state, modelInfo_);

  geometry_msgs::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "odom";
  base_tf.child_frame_id = modelInfo_.baseFrame;
  base_tf.transform.translation = ros_msg_helpers::getVectorMsg(r_world_base);
  base_tf.transform.translation.z += baseHeightOffset;
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
  
  // 添加头部关节
  if (updateHeadJointPositions_) {
    for (size_t i = 0; i < head_joint_names_.size(); i++) {
      jointPositions[head_joint_names_[i]] = head_joint_positions_[i];
    }
  }
  
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishTargetTrajectories(const ros::Time& timeStamp,
                                                                    const TargetTrajectories& targetTrajectories) {
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition = targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() = targetTrajectories.stateTrajectory.back().tail(4);
  geometry_msgs::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "odom";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.translation.z += baseHeightOffset;
  command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
  tfBroadcaster_.sendTransform(command_tf);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishOptimizedTrajectory(const ros::Time& timeStamp, const PrimalSolution& policy) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
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

  std::vector<std::vector<geometry_msgs::Point>> endEffectorTrajectories(modelInfo_.eeFrames.size());
  // 预留内层向量的容量
  for (auto& trajectory : endEffectorTrajectories) {
      trajectory.reserve(mpcStateTrajectory.size());
  }
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd& state) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    // 处理多个末端执行器
    for (int eeIdx = 0; eeIdx < modelInfo_.eeFrames.size(); ++eeIdx) {
      const auto eeIndex = model.getBodyId(modelInfo_.eeFrames[eeIdx]);
      vector_t eePosition = data.oMf[eeIndex].translation();
      eePosition[2] += baseHeightOffset;
      endEffectorTrajectories[eeIdx].push_back(ros_msg_helpers::getPointMsg(eePosition));
    }
  });

  // 处理多个末端执行器
  for (int eeIdx = 0; eeIdx < modelInfo_.eeFrames.size(); ++eeIdx) 
  {
    if (endEffectorTrajectories[eeIdx].empty()) {
        continue; // 跳过空轨迹
    }

    markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectories[eeIdx]), blue, TRAJECTORYLINEWIDTH));
    markerArray.markers.back().ns = "EE Trajectory" + std::to_string(eeIdx);
    markerArray.markers.back().id = eeIdx;
  }

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    // extract from observation
    auto r_world_base = getBasePosition(state, modelInfo_);
    r_world_base[2] += baseHeightOffset;
    const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(state, modelInfo_);

    // convert to ros message
    geometry_msgs::Pose pose;
    pose.position = ros_msg_helpers::getPointMsg(r_world_base);
    pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
    baseTrajectory.push_back(pose.position);
    poseArray.poses.push_back(std::move(pose));
  });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), ros_msg_helpers::getHeaderMsg("odom", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("odom", timeStamp);

  stateOptimizedPublisher_.publish(markerArray);
  stateOptimizedPosePublisher_.publish(poseArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::updateHeadJointPositions(const Eigen::VectorXd& positions) {
  if (positions.size() >= head_joint_names_.size()) {
    for (size_t i = 0; i < head_joint_names_.size(); i++) {
      head_joint_positions_[i] = positions[i];
    }
    updateHeadJointPositions_ = true;
  }
}

}  // namespace mobile_manipulator
}  // namespace ocs2
