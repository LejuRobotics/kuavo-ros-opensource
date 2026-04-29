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
#include <kuavo_msgs/lejuClawState.h>
#include <sensor_msgs/JointState.h>

// URDF related
#include <kdl_parser/kdl_parser.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <algorithm>

namespace ocs2
{
  namespace humanoid
  {

    // 爪机构映射 (200049)
    static const std::pair<const char*, double> kLeftClawJointInsertMap[] = {
      {"l_f_bar-1_joint", +1.0},
      {"l_b_bar-1",      -1.0},
      {"l_f_bar-3",      +1.0},
      {"l_b_bar-3",      -1.0},
    };
    static const std::pair<const char*, double> kRightClawJointInsertMap[] = {
      {"r_f_bar-1_joint", +1.0},
      {"r_b_bar-1",      -1.0},
      {"r_f_bar-3",      +1.0},
      {"r_b_bar-3",      -1.0},
    };
    // 爪机构映射 (200053/200062)
    static const std::pair<const char*, double> kLeftClawJointInsertMap200053[] = {
      {"l_f_bar_1_joint", +1.0},
      {"l_b_bar_1_joint", -1.0},
      {"l_f_bar_3_joint", +1.0},
      {"l_b_bar_3_joint", -1.0},
    };
    static const std::pair<const char*, double> kRightClawJointInsertMap200053[] = {
      {"r_f_bar_1_joint", +1.0},
      {"r_b_bar_1_joint", -1.0},
      {"r_f_bar_3_joint", +1.0},
      {"r_b_bar_3_joint", -1.0},
    };
    static constexpr size_t kLeftClawJointInsertMapSize = sizeof(kLeftClawJointInsertMap) / sizeof(kLeftClawJointInsertMap[0]);
    static constexpr size_t kRightClawJointInsertMapSize = sizeof(kRightClawJointInsertMap) / sizeof(kRightClawJointInsertMap[0]);
    static constexpr size_t kLeftClawJointInsertMap200053Size = sizeof(kLeftClawJointInsertMap200053) / sizeof(kLeftClawJointInsertMap200053[0]);
    static constexpr size_t kRightClawJointInsertMap200053Size = sizeof(kRightClawJointInsertMap200053) / sizeof(kRightClawJointInsertMap200053[0]);

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
      // 原有qiangnao手关节初始化
      dexhand_joint_names_ = {"l_thumbCMC", "l_thumbMCP", "l_indexMCP", "l_indexPIP", "l_middleMCP",
                              "l_middlePIP", "l_ringMCP", "l_ringPIP", "l_littleMCP", "l_littlePIP",
                             "r_thumbCMC", "r_thumbMCP", "r_indexMCP", "r_indexPIP", "r_middleMCP",
                             "r_middlePIP", "r_ringMCP", "r_ringPIP", "r_littleMCP", "r_littlePIP"};
      dexhand_joint_positions_ = std::vector<double>(dexhand_joint_names_.size(), 0.0);

      // 新增Linker L6/O6手关节初始化（22个关节，左右手各11个，和URDF完全一致）
      linker_hand_joint_names_ = {
        // 左手11个关节
        "l_thumb_cmc_yaw", "l_thumb_cmc_pitch", "l_thumb_ip",
        "l_index_mcp_pitch", "l_index_dip",
        "l_middle_mcp_pitch", "l_middle_dip",
        "l_ring_mcp_pitch", "l_ring_dip",
        "l_pinky_mcp_pitch", "l_pinky_dip",
        // 右手11个关节
        "r_thumb_cmc_yaw", "r_thumb_cmc_pitch", "r_thumb_ip",
        "r_index_mcp_pitch", "r_index_dip",
        "r_middle_mcp_pitch", "r_middle_dip",
        "r_ring_mcp_pitch", "r_ring_dip",
        "r_pinky_mcp_pitch", "r_pinky_dip"
      };
      linker_hand_joint_positions_ = std::vector<double>(linker_hand_joint_names_.size(), 0.0);

      simplifiedJointPositions_ = std::vector<double>(visualModeSettings_.simplifiedJointNames.size(), 0.0);
    };
    

    void HumanoidVisualizer::updateClawJointPositions(const vector_t &positions)
    {
      if (positions.size() != 2) return;
      for (size_t i = 0; i < 2; i++)
      {
        claw_joint_positions_[i] = positions[i];
      }
      updateClawJointPositions_ = true;
    }

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

    /******************************************************************************************************/
    // 单独拆分的Linker L6/O6手转换逻辑：12维控制量 → 22个关节角度
    void HumanoidVisualizer::updateLinkerHandJointPositions(const vector_t &positions)
    {
      if(positions.size() != 12) {
        return; // 输入固定为12维：左右手各6个控制值
      }

      // -------------------- Linker L6/O6手转换逻辑（和mujoco中linkerl6_hand.hpp逻辑完全一致） --------------------
      if (isLinkerHand_) {
        // Linker手工具函数：0~255曲度 → 关节角度，耦合比例和真实机械一致
        auto Curl2JointsLinker = [](double curl, const std::array<double, 2>& j1_range, bool is_thumb = false) {
          curl = std::clamp(curl, 0.0, 255.0);
          double norm = curl / 255.0;
          double j1 = j1_range[0] + norm * (j1_range[1] - j1_range[0]);
          // 耦合比例：拇指1.226495，其他手指1.125676（和真实机械mimic比例一致）
          double mimic_ratio = is_thumb ? 1.226495 : 1.125676;
          double j2 = j1 * mimic_ratio;
          return std::array<double, 2>{j1, j2};
        };

        // ==================== 左手（前6维输入：positions[0]~positions[5]） ====================
        // 输入0：拇指pitch + ip（耦合控制）
        auto l_thumb_pitch = urdfModel_.getJoint("l_thumb_cmc_pitch");
        auto l_thumb_ip = urdfModel_.getJoint("l_thumb_ip");
        if (l_thumb_pitch && l_thumb_pitch->limits && l_thumb_ip && l_thumb_ip->limits) {
          std::array<double, 2> pitch_range = {l_thumb_pitch->limits->lower, l_thumb_pitch->limits->upper};
          auto joints = Curl2JointsLinker(positions[0], pitch_range, true);
          linker_hand_joint_positions_[1] = joints[0]; // l_thumb_cmc_pitch
          linker_hand_joint_positions_[2] = std::clamp(joints[1], l_thumb_ip->limits->lower, l_thumb_ip->limits->upper); // l_thumb_ip
        }
        // 输入1：拇指yaw（单关节独立控制）
        auto l_thumb_yaw = urdfModel_.getJoint("l_thumb_cmc_yaw");
        if (l_thumb_yaw && l_thumb_yaw->limits) {
          double norm = std::clamp(positions[1], 0.0, 255.0) / 255.0;
          linker_hand_joint_positions_[0] = l_thumb_yaw->limits->lower + norm * (l_thumb_yaw->limits->upper - l_thumb_yaw->limits->lower);
        }
        // 输入2~5：食指、中指、无名指、小指（每指2关节耦合）
        for (int i = 0; i < 4; i++) {
          int joint_base_idx = 3 + i * 2; // 左手非拇指关节从索引3开始，每指占2个位置
          int input_idx = 2 + i;
          const auto& mcp_name = linker_hand_joint_names_[joint_base_idx];
          const auto& dip_name = linker_hand_joint_names_[joint_base_idx + 1];
          auto mcp_joint = urdfModel_.getJoint(mcp_name);
          auto dip_joint = urdfModel_.getJoint(dip_name);
          if (!mcp_joint || !mcp_joint->limits || !dip_joint || !dip_joint->limits) continue;

          std::array<double, 2> mcp_range = {mcp_joint->limits->lower, mcp_joint->limits->upper};
          auto joints = Curl2JointsLinker(positions[input_idx], mcp_range);
          linker_hand_joint_positions_[joint_base_idx] = joints[0];
          linker_hand_joint_positions_[joint_base_idx + 1] = std::clamp(joints[1], dip_joint->limits->lower, dip_joint->limits->upper);
        }

        // ==================== 右手（后6维输入：positions[6]~positions[11]） ====================
        // 右手拇指顺序和左手完全一致：输入6是拇指pitch + ip，输入7是拇指yaw（与左手控制逻辑对齐）
        // 处理拇指pitch + ip（耦合控制）
        auto r_thumb_pitch = urdfModel_.getJoint("r_thumb_cmc_pitch");
        auto r_thumb_ip = urdfModel_.getJoint("r_thumb_ip");
        if (r_thumb_pitch && r_thumb_pitch->limits && r_thumb_ip && r_thumb_ip->limits) {
          std::array<double, 2> pitch_range = {r_thumb_pitch->limits->lower, r_thumb_pitch->limits->upper};
          auto joints = Curl2JointsLinker(positions[6], pitch_range, true);
          // 右手拇指弯曲方向和四指一致，镜像后自然对称
          linker_hand_joint_positions_[12] = joints[0]; // r_thumb_cmc_pitch
          linker_hand_joint_positions_[13] = std::clamp(joints[1], r_thumb_ip->limits->lower, r_thumb_ip->limits->upper); // r_thumb_ip
        }
        // 处理拇指yaw（单关节独立控制）
        auto r_thumb_yaw = urdfModel_.getJoint("r_thumb_cmc_yaw");
        if (r_thumb_yaw && r_thumb_yaw->limits) {
          double norm = std::clamp(positions[7], 0.0, 255.0) / 255.0;
          // 与mujoco处理逻辑保持一致，URDF已定义关节镜像，无需额外反转
          linker_hand_joint_positions_[11] = r_thumb_yaw->limits->lower + norm * (r_thumb_yaw->limits->upper - r_thumb_yaw->limits->lower);
        }
        // 输入8~11：食指、中指、无名指、小指（每指2关节耦合）
        for (int i = 0; i < 4; i++) {
          int joint_base_idx = 14 + i * 2; // 右手非拇指关节从索引14开始，每指占2个位置
          int input_idx = 8 + i;
          const auto& mcp_name = linker_hand_joint_names_[joint_base_idx];
          const auto& dip_name = linker_hand_joint_names_[joint_base_idx + 1];
          auto mcp_joint = urdfModel_.getJoint(mcp_name);
          auto dip_joint = urdfModel_.getJoint(dip_name);
          if (!mcp_joint || !mcp_joint->limits || !dip_joint || !dip_joint->limits) continue;

          std::array<double, 2> mcp_range = {mcp_joint->limits->lower, mcp_joint->limits->upper};
          auto joints = Curl2JointsLinker(positions[input_idx], mcp_range);
          linker_hand_joint_positions_[joint_base_idx] = joints[0];
          linker_hand_joint_positions_[joint_base_idx + 1] = std::clamp(joints[1], dip_joint->limits->lower, dip_joint->limits->upper);
        }

        updateDexhandJointPositions_ = true;
        return;
      }
    }

    /******************************************************************************************************/
    void HumanoidVisualizer::updateHandJointPositions(const vector_t &positions)
    {
      if(positions.size() != 12 || !urdfModel_.getJoint("l_thumbCMC")) {
        return; // qiangnao手专用：输入固定为12维控制量，左右手各6个（范围0~100）
      }

      // -------------------- qiangnao手转换逻辑 --------------------

      auto Curl2Joints = [](double curl, const std::array<double, 2>& j1_range, const std::array<double, 2>& j2_range) {
          curl = std::clamp(curl, 0.0, 100.0);
          double norm = curl / 100.0;
          double j1 = j1_range[0] + norm * (j1_range[1] - j1_range[0]);
          double j2 = j2_range[0] + norm * (j2_range[1] - j2_range[0]);
          return std::array<double, 2>{j1, j2};
      };

      // left dexhand(大拇指关节)
      dexhand_joint_positions_[0] = Curl2Joints(positions[1], {0.23, 1.36}, {0.23, 1.36})[0];
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
      dexhand_joint_positions_[10] = Curl2Joints(positions[7], {0.23, 1.36}, {0.23, 1.36})[0];
      dexhand_joint_positions_[11] = Curl2Joints(positions[6], {0.16, 0.75}, {0.16, 0.75})[0];

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
      lejuClawStatePub_ = nodeHandle.advertise<kuavo_msgs::lejuClawState>("/leju_claw_state", 10);

      // subscribe claw command (percentage) and map to radians limits from URDF if available
      clawCmdSubscriber_ = nodeHandle.subscribe<kuavo_msgs::lejuClawCommand>(
      "/leju_claw_command", 1, &HumanoidVisualizer::lejuClawCmdCallback, this);

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

        // 自动识别手型
        if (urdfModel_.getJoint("l_thumb_cmc_yaw") != nullptr) {
          isLinkerHand_ = true;
          ROS_INFO("[HumanoidVisualizer] 自动识别到Linker L6/O6灵巧手，使用Linker适配逻辑");
        } else if (urdfModel_.getJoint("l_thumbCMC") != nullptr) {
          isLinkerHand_ = false;
          ROS_INFO("[HumanoidVisualizer] 自动识别到qiangnao灵巧手，使用qiangnao适配逻辑");
          // qiangnao手的订阅逻辑保持原有实现
        } else {
          isLinkerHand_ = false;
          ROS_INFO("[HumanoidVisualizer] 未检测到灵巧手，跳过手部可视化");
        }
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
        // 防护：避免 policy 并发/内存损坏导致 trajectory 元数据异常（如负的 size）而崩溃
        constexpr size_t kMaxReasonableTrajectorySize = 50000;
        if (primalSolution.inputTrajectory_.empty() || primalSolution.inputTrajectory_[0].size() > 2048 ||
            primalSolution.timeTrajectory_.size() == 0 || primalSolution.stateTrajectory_.size() == 0 ||
            primalSolution.timeTrajectory_.size() > kMaxReasonableTrajectorySize ||
            primalSolution.stateTrajectory_.size() > kMaxReasonableTrajectorySize ||
            primalSolution.timeTrajectory_.size() != primalSolution.stateTrajectory_.size())
        {
          lastTime_ = observation.time;
          return;
        }
        publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
        publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                        primalSolution.modeSchedule_);
        lastTime_ = observation.time;
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
          if (isLinkerHand_) {
            // 发布Linker L6/O6手关节
            for (size_t i = 0; i < linker_hand_joint_names_.size(); i++) {
              jointPositions.insert({linker_hand_joint_names_[i], linker_hand_joint_positions_[i]});
            }
          } else {
            // 发布qiangnao手关节
            for (size_t i = 0; i < dexhand_joint_names_.size(); i++) {
              jointPositions.insert({dexhand_joint_names_[i], dexhand_joint_positions_[i]});
            }
          }
        }
        if (updateClawJointPositions_)
        {
          // 从匿名命名空间的静态数组读取映射；仅当 URDF 存在该关节时才插入
          auto tryInsertGroupRaw = [this, &jointPositions](const std::pair<const char*, double>* group, size_t sz, double angle){
            for(size_t i=0; i<sz; ++i){
              const char* name = group[i].first;
              double factor = group[i].second;
              if (urdfModel_.getJoint(name)) {
                jointPositions.insert({name, factor * angle});
              }
            }
          };
          tryInsertGroupRaw(kLeftClawJointInsertMap,  kLeftClawJointInsertMapSize,  claw_joint_positions_[0]);
          tryInsertGroupRaw(kRightClawJointInsertMap, kRightClawJointInsertMapSize, claw_joint_positions_[1]);
          // 200053/200062 使用下划线命名
          tryInsertGroupRaw(kLeftClawJointInsertMap200053,  kLeftClawJointInsertMap200053Size,  claw_joint_positions_[0]);
          tryInsertGroupRaw(kRightClawJointInsertMap200053, kRightClawJointInsertMap200053Size, claw_joint_positions_[1]);
        }
        robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);

        // publish /leju_claw_state for bag recording
        if (updateClawJointPositions_) {
          kuavo_msgs::lejuClawState claw_state_msg;
          claw_state_msg.header.stamp = timeStamp;
          claw_state_msg.header.frame_id = "leju_claw";
          claw_state_msg.data.name = {"left_claw", "right_claw"};
          claw_state_msg.data.position = {claw_joint_positions_[0], claw_joint_positions_[1]};
          claw_state_msg.data.velocity.resize(2, 0.0);
          claw_state_msg.data.effort.resize(2, 0.0);
          claw_state_msg.state = {kuavo_msgs::lejuClawState::kReached, kuavo_msgs::lejuClawState::kReached};
          lejuClawStatePub_.publish(claw_state_msg);
        }
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

    void HumanoidVisualizer::lejuClawCmdCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg)
    {
      if (!msg) return;
      if (msg->data.position.size() < 2) return;

      // 输入为开合度百分比 [0,100]：0 -> 完全打开(-0.698rad)，100 -> 完全闭合(0rad)，线性映射
      auto clamp01 = [](double v){ return std::max(0.0, std::min(100.0, v)); };
      const double l_pct = clamp01(msg->data.position[0]);
      const double r_pct = clamp01(msg->data.position[1]);

      static const double MIN_OPEN = -0.6981317008; // -40°
      static const double MAX_CLOSED = 0.0;         // 0°

      auto pct_to_angle = [&](double pct){
        const double s = pct / 100.0;               // 0.0~1.0
        return MIN_OPEN + (MAX_CLOSED - MIN_OPEN) * s;
      };

      vector_t v(2);
      v.setZero();
      v[0] = pct_to_angle(l_pct);
      v[1] = pct_to_angle(r_pct);
      updateClawJointPositions(v);
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
