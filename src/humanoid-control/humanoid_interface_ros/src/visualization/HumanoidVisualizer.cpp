/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_interface_ros/visualization/HumanoidVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "humanoid_interface/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <kuavo_msgs/endEffectorData.h>

// URDF related
#include <kdl_parser/kdl_parser.hpp>

#include <std_msgs/Float32MultiArray.h>

namespace ocs2
{
  namespace humanoid
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    HumanoidVisualizer::HumanoidVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                           const PinocchioEndEffectorKinematics &endEffectorKinematics,
                                           const PinocchioEndEffectorSpatialKinematics &endEffectorSpatialKinematics,
                                           ros::NodeHandle &nodeHandle, std::string fileName, 
                                           scalar_t maxUpdateFrequency)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          centroidalModelInfo_(std::move(centroidalModelInfo)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          endEffectorSpatialKinematicsPtr_(endEffectorSpatialKinematics.clone()),
          lastTime_(std::numeric_limits<scalar_t>::lowest()),
          minPublishTimeDifference_(1.0 / maxUpdateFrequency)
    {
      endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
      endEffectorSpatialKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
      visualModeSettings_ = loadModelSettings(fileName);
      launchVisualizerNode(nodeHandle);
      head_joint_names_ = {"zhead_1_joint", "zhead_2_joint"};
      head_joint_positions_ = {0.0, 0.0};
      dexhand_joint_names_ = {"l_thumbCMC", "l_thumbMCP", "l_indexMCP", "l_indexPIP", "l_middleMCP", 
                              "l_middlePIP", "l_ringMCP", "l_ringPIP", "l_littleMCP", "l_littlePIP", 
                             "r_thumbCMC", "r_thumbMCP", "r_indexMCP", "r_indexPIP", "r_middleMCP", 
                             "r_middlePIP", "r_ringMCP", "r_ringPIP", "r_littleMCP", "r_littlePIP"};
      dexhand_joint_positions_ = std::vector<double>(dexhand_joint_names_.size(), 0.0);
      simplifiedJointPositions_ = std::vector<double>(visualModeSettings_.simplifiedJointNames.size(), 0.0);
    };

    void HumanoidVisualizer::updateSimplifiedArmPositions(const vector_t &positions)
    {
      if (positions.size() == visualModeSettings_.simplifiedJointNames.size())// 简单判断可视化简化的关节
      {
          for (size_t i = 0; i < positions.size(); i++)
          {
            simplifiedJointPositions_[i] = positions[i];
          }
      }
    }

    void HumanoidVisualizer::updateHeadJointPositions(const vector_t &positions)
    {
      for (size_t i = 0; i < head_joint_names_.size(); i++)
      {
        head_joint_positions_[i] = positions[i];
      }
      updateHeadJointPositions_ = true;
    }

    void HumanoidVisualizer::updateHandJointPositions(const vector_t &positions)
    {
      if(positions.size() != 12) {
        return;
      }
      
      if(!urdfModel_.getJoint("l_thumbCMC")) {
        return;
      }

      auto Curl2Joints = [](double curl, const std::array<double, 2>& j1_range, const std::array<double, 2>& j2_range) {
          // 限制曲度输入范围
          curl = std::clamp(curl, 0.0, 100.0);
          
          // 计算归一化曲度
          double norm = curl / 100.0;
          
          // 生成关节控制指令
          double j1 = j1_range[0] + norm * (j1_range[1] - j1_range[0]);
          double j2 = j2_range[0] + norm * (j2_range[1] - j2_range[0]);
          return std::array<double, 2>{j1, j2};
      };

      // left dexhand(大拇指关节)
      dexhand_joint_positions_[0] = Curl2Joints(positions[0], {0.23, 1.36}, {0.23, 1.36})[0];
      dexhand_joint_positions_[1] = Curl2Joints(positions[0], {0.16, 0.75}, {0.16, 0.75})[0];


      // left dexhand(非大拇指关节)
      for(int i = 0; i < 4; i++) {
        int j0_idx = 2 + i*2, j1_idx = j0_idx + 1;
        auto finger_joint0 = urdfModel_.getJoint(dexhand_joint_names_[j0_idx]);
        auto finger_joint1 = urdfModel_.getJoint(dexhand_joint_names_[j1_idx]);
        if(!finger_joint0 || !finger_joint1) {
          return;
        }

        std::array<double, 2> j0_range = {finger_joint0->limits->lower, finger_joint0->limits->upper};
        std::array<double, 2> j1_range = {finger_joint1->limits->lower, finger_joint1->limits->upper};

        int curl = positions[i+2];
        auto joints_pos = Curl2Joints(curl, j0_range, j1_range);
        dexhand_joint_positions_[j0_idx] = joints_pos[0];
        dexhand_joint_positions_[j1_idx] = joints_pos[1];
      }

      // right dexhand(大拇指关节)
      dexhand_joint_positions_[10] = Curl2Joints(positions[6], {0.23, 1.36}, {0.23, 1.36})[0];
      dexhand_joint_positions_[11] = Curl2Joints(positions[7], {0.16, 0.75}, {0.16, 0.75})[0];

      // right dexhand(非大拇指关节)
      for(int i = 0; i < 4; i++) {
        int j0_idx = 12 + i*2, j1_idx = j0_idx + 1;
        auto finger_joint0 = urdfModel_.getJoint(dexhand_joint_names_[j0_idx]);
        auto finger_joint1 = urdfModel_.getJoint(dexhand_joint_names_[j1_idx]);
        if(!finger_joint0 || !finger_joint1) {
          return;
        }

        std::array<double, 2> j0_range = {finger_joint0->limits->lower, finger_joint0->limits->upper};
        std::array<double, 2> j1_range = {finger_joint1->limits->lower, finger_joint1->limits->upper};

        int curl = positions[i+2+6];
        auto joints_pos = Curl2Joints(curl, j0_range, j1_range);
        dexhand_joint_positions_[j0_idx] = joints_pos[0];
        dexhand_joint_positions_[j1_idx] = joints_pos[1];
      }
      updateDexhandJointPositions_ = true;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::launchVisualizerNode(ros::NodeHandle &nodeHandle)
    {
      costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/humanoid/desiredBaseTrajectory", 1);
      costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        costDesiredFeetPositionPublishers_[i] = nodeHandle.advertise<visualization_msgs::Marker>("/humanoid/desiredFeetTrajectory/contact_" + std::to_string(i), 1);
      }
      stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/humanoid/optimizedStateTrajectory", 1);
      planedFootPositionsPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/humanoid/planedFootPositions", 1);
      currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/humanoid/currentState", 1);
      eePosePub_ = nodeHandle.advertise<kuavo_msgs::endEffectorData>("/humanoid_ee_State", 1);

      // Load URDF model
      if (!urdfModel_.initParam("humanoid_description"))
      {
        std::cerr << "[HumanoidVisualizer] Could not read URDF from: \"humanoid_description\"" << std::endl;
      }
      else
      {
        KDL::Tree kdlTree;
        kdl_parser::treeFromUrdfModel(urdfModel_, kdlTree);

        robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
        robotStatePublisherPtr_->publishFixedTransforms(true);
      }
      auto planedFootPositionsCallback = [this](const std_msgs::Float32MultiArray::ConstPtr &msg)
      {
        feet_array_t<vector3_t> footPositions;
        for(int i=0; i<msg->data.size()/3; i++)
        {
          footPositions[i] = vector3_t(msg->data[3*i], msg->data[3*i+1], msg->data[3*i+2]);
        }
        publishPlanedFootPositions(footPositions);
      };
      planedFootPositionsSubscriber_ = nodeHandle.subscribe<std_msgs::Float32MultiArray>("/humanoid_foot_contact_point", 1, planedFootPositionsCallback);
    }

    bool HumanoidVisualizer::getJointLimits(const std::string &joint_name, std::pair<double, double> &limits) const
    {
      urdf::JointConstSharedPtr joint = urdfModel_.getJoint(joint_name);
      if (!joint)
      {
        std::cout << "[HumanoidVisualizer]Joint " << joint_name << " not found!" << std::endl;
        return false;
      }

      // 检查关节是否有限位
      if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC)
      {
        if (joint->limits)
        {
          limits = std::make_pair(joint->limits->lower, joint->limits->upper);
          return true;
        }
        else
        {
          std::cout << "[HumanoidVisualizer]Joint " << joint_name << " has no limits." << std::endl;
          return false;
        }
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::update(const SystemObservation &observation, const PrimalSolution &primalSolution, const CommandData &command)
    {
      if (observation.time - lastTime_ > minPublishTimeDifference_ || observation.time - lastTime_ < 0.0)
      {
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
        // pinocchio::forwardKinematics(model, data, observation.state.tail(18));
        pinocchio::updateFramePlacements(model, data);

        const auto timeStamp = ros::Time::now();
        publishObservation(timeStamp, observation);
        publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
        publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                        primalSolution.modeSchedule_);
        lastTime_ = observation.time;

        // record feet force input data !!!!!!!!!!!!!!!!!!
        feetInputTraj.push_back(primalSolution.inputTrajectory_[0]);

        // record time stamp data !!!!!!!!!!!!!!!!!!
        timeTraj.push_back(timecount++);

        // record feet position data!!!!!!!!!!!!!!!!!!!
        // pinocchio::forwardKinematics(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), observation.state.tail(18));
        // const auto feetPositions = endEffectorSpatialKinematicsPtr_->getPosition(observation.state);
        quaternion_t quat(1, 0, 0, 0);
        // const auto feetOrientations = endEffectorSpatialKinematicsPtr_->getOrientationError(primalSolution.inputTrajectory_[0], {quat});
        // const auto feetOrientations = endEffectorSpatialKinematicsPtr_->getVelocity(primalSolution.inputTrajectory_[0], primalSolution.inputTrajectory_[0]);

        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
        feetPosTraj.push_back(feetPositions);
        // feetOriTraj.push_back(feetOrientations);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishObservation(ros::Time timeStamp, const SystemObservation &observation)
    {
      // Extract components from state
      const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
      const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

      // Compute cartesian state and inputs
      const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
      const auto handPositions = endEffectorSpatialKinematicsPtr_->getPosition(observation.state);
      const auto handOrientations = endEffectorSpatialKinematicsPtr_->getOrientation(observation.state);
      
      // const auto feetPositions = endEffectorSpatialKinematicsPtr_->getPosition(observation.state);
      std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts); // TODO： change to 4
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
      }

      // record feet force data!!!!!!!!!!!!!!!!!!!
      feetForce.push_back(feetForces);

      // Publish
      vector_t handState(14);
      handState.setZero();
      int cnt_hand = 0;
      for(int i=0; i<handPositions.size(); i++){
        for(int j=0; j<handPositions[i].size(); j++){
          handState[cnt_hand + j] = handPositions[i](j);
        }
        for(int j=0; j<handOrientations[i].coeffs().size(); j++){
          handState[cnt_hand + 3 + j] = handOrientations[i].coeffs()(j);
        }
        cnt_hand += 7;
      }
      publishArmEeStateTrajectory(handState);
      publishJointTransforms(timeStamp, qJoints);
      publishBaseTransform(timeStamp, basePose);
      publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t &jointAngles) const
    {
      if (robotStatePublisherPtr_ != nullptr)
      {
        // std::map<std::string, scalar_t> jointPositions{{"l_leg_roll", jointAngles[0]}, {"l_leg_yaw", jointAngles[1]}, {"l_leg_pitch", jointAngles[2]}, {"l_knee", jointAngles[3]}, {"l_foot_pitch", jointAngles[4]}, {"l_foot_roll", jointAngles[5]}, {"r_leg_roll", jointAngles[6]}, {"r_leg_yaw", jointAngles[7]}, {"r_leg_pitch", jointAngles[8]}, {"r_knee", jointAngles[9]}, {"r_foot_pitch", jointAngles[10]}, {"r_foot_roll", jointAngles[11]}};
        // std::map<std::string, scalar_t> jointPositions{{"leg_l1_joint", jointAngles[0]}, {"leg_l2_joint", jointAngles[1]}, {"leg_l3_joint", jointAngles[2]}, {"leg_l4_joint", jointAngles[3]}, {"leg_l5_joint", jointAngles[4]}, {"leg_l6_joint", jointAngles[5]}, {"leg_r1_joint", jointAngles[6]}, {"leg_r2_joint", jointAngles[7]}, {"leg_r3_joint", jointAngles[8]}, {"leg_r4_joint", jointAngles[9]}, {"leg_r5_joint", jointAngles[10]}, {"leg_r6_joint", jointAngles[11]}};
        std::map<std::string, scalar_t> jointPositions;
        for(size_t i=0; i<jointAngles.rows(); i++){
          jointPositions.insert({visualModeSettings_.jointNames[i], jointAngles[i]});
        }
        for (size_t i = 0; i < visualModeSettings_.simplifiedJointNames.size(); i++)
        {
          jointPositions.insert({visualModeSettings_.simplifiedJointNames[i], simplifiedJointPositions_[i]});
        }
        if (updateHeadJointPositions_)
        {
          for (size_t i = 0; i < head_joint_names_.size(); i++)
          {
            jointPositions.insert({head_joint_names_[i], head_joint_positions_[i]});
          }
        }
        if (updateDexhandJointPositions_)
        {
          for (size_t i = 0; i < dexhand_joint_names_.size(); i++)
          {
            jointPositions.insert({dexhand_joint_names_[i], dexhand_joint_positions_[i]});
          }
        }
        robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t &basePose)
    {
      if (robotStatePublisherPtr_ != nullptr)
      {
        geometry_msgs::TransformStamped baseToWorldTransform;
        baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
        baseToWorldTransform.child_frame_id = "base_link";

        const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
        baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
        baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
                tfBroadcaster_.sendTransform(baseToWorldTransform);

        // record base position & orientation data !!!!!!!!!!!!!!!!!!!!!1
        basePosTraj.push_back(basePose.head<3>());
        baseOriTraj.push_back(basePose.tail<3>());
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishTrajectory(const std::vector<SystemObservation> &system_observation_array, scalar_t speed)
    {
      for (size_t k = 0; k < system_observation_array.size() - 1; k++)
      {
        scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
        scalar_t publishDuration = timedExecutionInSeconds([&]()
                                                           { publishObservation(ros::Time::now(), system_observation_array[k]); });
        if (frameDuration > publishDuration)
        {
          ros::WallDuration(frameDuration - publishDuration).sleep();
        }
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t &contactFlags,
                                                     const std::vector<vector3_t> &feetPositions,
                                                     const std::vector<vector3_t> &feetForces) const
    {
      // Reserve message
      const size_t numberOfCartesianMarkers = 10;
      visualization_msgs::MarkerArray markerArray;
      markerArray.markers.reserve(numberOfCartesianMarkers);

      // Feet positions and Forces
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) // TO-DO:
      {
        markerArray.markers.emplace_back(
            getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
        markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
      }

      // Center of pressure
      markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                                 contactFlags.begin(), Color::green, copMarkerDiameter_));

      // Support polygon
      markerArray.markers.emplace_back(
          getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

      // Give markers an id and a frame
      assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
      assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

      // Publish cartesian markers (minus the CoM Pose)
      currentStatePublisher_.publish(markerArray);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories)
    {
      const auto &stateTrajectory = targetTrajectories.stateTrajectory;
      const auto &inputTrajectory = targetTrajectories.inputTrajectory;

      // Reserve com messages
      std::vector<geometry_msgs::Point> desiredBasePositionMsg;
      desiredBasePositionMsg.reserve(stateTrajectory.size());

      // Reserve feet messages
      feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
      }

      for (size_t j = 0; j < stateTrajectory.size(); j++)
      {
        const auto state = stateTrajectory.at(j);
        vector_t input(centroidalModelInfo_.inputDim);
        if (j < inputTrajectory.size())
        {
          input = inputTrajectory.at(j);
        }
        else
        {
          input.setZero();
        }

        // Construct base pose msg
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());

        // Fill message containers
        desiredBasePositionMsg.push_back(pose.position);

        // Fill feet msgs
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
        // const auto feetPositions = endEffectorSpatialKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
        {
          geometry_msgs::Pose footPose;
          footPose.position = getPointMsg(feetPositions[i]);
          desiredFeetPositionMsgs[i].push_back(footPose.position);
        }
      }

      // Headers
      auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      // Publish
      costDesiredBasePositionPublisher_.publish(comLineMsg);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
        footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
        footLineMsg.id = 0;
        costDesiredFeetPositionPublishers_[i].publish(footLineMsg);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                                             const vector_array_t &mpcStateTrajectory, 
                                                             const ModeSchedule &modeSchedule) {
      if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty())
      {
        return; // Nothing to publish
      }

      // Reserve Feet msg
      feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
      std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point> &v)
                    { v.reserve(mpcStateTrajectory.size()); });

      // Reserve Base Msg
      std::vector<geometry_msgs::Point> baseTrajectory;
      baseTrajectory.reserve(mpcStateTrajectory.size());

      // Reserve Hand Msg
      hand_array_t<std::vector<geometry_msgs::Point>> handMsgs;
      std::for_each(handMsgs.begin(), handMsgs.end(), [&](std::vector<geometry_msgs::Point> &v)
                    { v.reserve(mpcStateTrajectory.size()); });

      // Extract Com, Feet and Hand from state
      std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t &state) {
        const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

        // Fill com position and pose msgs
        geometry_msgs::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());
        baseTrajectory.push_back(pose.position);

        // Fill feet msgs
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            const auto position = getPointMsg(feetPositions[i]);
            feetMsgs[i].push_back(position);
        }

        const auto handPositions = endEffectorSpatialKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numSixDofContacts; i++) {
            const auto position = getPointMsg(handPositions[i]);
            handMsgs[i].push_back(position);
        }
      });

      // Convert feet msgs to Array message
      visualization_msgs::MarkerArray markerArray;
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
        markerArray.markers.back().ns = "Feet Trajectories";
      }
      for (size_t i = 0; i < centroidalModelInfo_.numSixDofContacts; i++)
      {
        markerArray.markers.emplace_back(getLineMsg(std::move(handMsgs[i]), handColorMap_[i], trajectoryLineWidth_));
      }
      markerArray.markers.emplace_back(getLineMsg(std::move(baseTrajectory), Color::red, trajectoryLineWidth_));
      markerArray.markers.back().ns = "CoM Trajectory";

      // Future footholds
      visualization_msgs::Marker sphereList;
      sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
      sphereList.scale.x = footMarkerDiameter_;
      sphereList.scale.y = footMarkerDiameter_;
      sphereList.scale.z = footMarkerDiameter_;
      sphereList.ns = "Future footholds";
      sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
      const auto &eventTimes = modeSchedule.eventTimes;
      const auto &subsystemSequence = modeSchedule.modeSequence;
      const auto tStart = mpcTimeTrajectory.front();
      const auto tEnd = mpcTimeTrajectory.back();
      for (size_t event = 0; event < eventTimes.size(); ++event)
      {
        if (tStart < eventTimes[event] && eventTimes[event] < tEnd)
        { // Only publish future footholds within the optimized horizon
          const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
          const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
          const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

          const auto &model = pinocchioInterface_.getModel();
          auto &data = pinocchioInterface_.getData();
          pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
          pinocchio::updateFramePlacements(model, data);

          const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
          // const auto feetPosition = endEffectorSpatialKinematicsPtr_->getPosition(postEventState);
          for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
          {
            if (!preEventContactFlags[i] && postEventContactFlags[i])
            { // If a foot lands, a marker is added at that location.
              sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
              sphereList.colors.push_back(getColor(feetColorMap_[i]));
            }
          }
        }
      }
      markerArray.markers.push_back(std::move(sphereList));

      // Add headers and Id
      assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
      assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

      stateOptimizedPublisher_.publish(markerArray);
    }

    void HumanoidVisualizer::publishArmEeStateTrajectory(const vector_t& EeState)
    {
      kuavo_msgs::endEffectorData endEffectorState_;
      endEffectorState_.position.resize(EeState.size());
      for(size_t i=0; i<EeState.size(); i++){
        endEffectorState_.position[i] = EeState[i];
      }
      eePosePub_.publish(endEffectorState_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidVisualizer::publishPlanedFootPositions(const feet_array_t<vector3_t> &footPositions)
    {
      visualization_msgs::MarkerArray markerArray;

      // 假设我们将前四个点和后四个点分别划分
      const size_t numPoints = footPositions.size();
      const size_t midPoint = numPoints / 2;
      for(size_t i=0; i<numPoints; i++)
      {
        if (std::isinf(footPositions[i](0)) || std::isinf(footPositions[i](1)) || std::isinf(footPositions[i](2)))
          return;
      }

      const std::vector<int> order_map ={0, 1, 3, 2, 4, 5, 7 ,6};
      // 绘制前四个点的连线
      {
        visualization_msgs::Marker lineStripFront;
        lineStripFront.header.frame_id = "odom"; // 设置坐标系
        lineStripFront.header.stamp = ros::Time::now();
        lineStripFront.ns = "foot_positions_front"; // 指定namespace
        lineStripFront.id = 0; // ID为0
        lineStripFront.type = visualization_msgs::Marker::LINE_STRIP; // 设置为线条
        lineStripFront.action = visualization_msgs::Marker::ADD;

        lineStripFront.scale.x = 0.02; // 设置线条的宽度

        lineStripFront.color.r = 1.0f; // 颜色为红色
        lineStripFront.color.g = 0.0f;
        lineStripFront.color.b = 0.0f;
        lineStripFront.color.a = 1.0f; // 不透明

        // 添加前四个点
        for (size_t i = 0; i < 4 && i < midPoint; i++) {
            geometry_msgs::Point p;
            p.x = footPositions[order_map[i]][0];
            p.y = footPositions[order_map[i]][1];
            p.z = footPositions[order_map[i]][2];
            lineStripFront.points.push_back(p);
        }
        // 连接首尾
        lineStripFront.points.push_back(lineStripFront.points.front());

        markerArray.markers.push_back(lineStripFront); // 将前四个点的线条添加到markerArray
      }

      // 绘制后四个点的连线
      {
        visualization_msgs::Marker lineStripBack;
        lineStripBack.header.frame_id = "odom";
        lineStripBack.header.stamp = ros::Time::now();
        lineStripBack.ns = "foot_positions_back";
        lineStripBack.id = 1; // ID为1
        lineStripBack.type = visualization_msgs::Marker::LINE_STRIP;
        lineStripBack.action = visualization_msgs::Marker::ADD;

        lineStripBack.scale.x = 0.02; // 设置线条的宽度

        lineStripBack.color.r = 0.0f; // 颜色为蓝色
        lineStripBack.color.g = 0.0f;
        lineStripBack.color.b = 1.0f;
        lineStripBack.color.a = 1.0f; // 不透明

        // 添加后四个点
        for (size_t i = midPoint; i < midPoint + 4 && i < numPoints; i++) {
            geometry_msgs::Point p;
            p.x = footPositions[order_map[i]][0];
            p.y = footPositions[order_map[i]][1];
            p.z = footPositions[order_map[i]][2];
            lineStripBack.points.push_back(p);
        }
        // 连接首尾
        lineStripBack.points.push_back(lineStripBack.points.front());

        markerArray.markers.push_back(lineStripBack); // 将后四个点的线条添加到markerArray
      }

      planedFootPositionsPublisher_.publish(markerArray); // 发布markerArray
    }

  } // namespace humanoid
} // namespace ocs2
