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

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "humanoid_interface/common/Types.h"
#include "humanoid_interface/foot_planner/SplineCpg.h"
#include "humanoid_interface/foot_planner/CubicInterpolator.h"
#include "humanoid_interface/foot_planner/SingleStepPlanner.h"
#include "humanoid_interface/gait/GaitSchedule.h"


#include "ocs2_core/thread_support/BufferedValue.h"
#include "humanoid_interface/foot_planner/InverseKinematics.h"

namespace ocs2 {
namespace humanoid {

class SwingTrajectoryPlanner {
 public:
  struct Config {
    scalar_t liftOffVelocity = 0.2;
    scalar_t touchDownVelocity = -0.4;
    scalar_t swingHeight = 0.1;
    scalar_t climbStageSwingHeight = 0.3;
    scalar_t swingTimeScale = 0.15;  // swing phases shorter than this time will be scaled down in height and velocity
    scalar_t toeSwingHeight = 0.08;
    scalar_t heelSwingHeight = 0.08;
    scalar_t deadBandVelocity = 0.1;
    scalar_t heelToeMaxHeightVelocity = 0.5;
    scalar_t next_position_z = 0.0;

    scalar_t swing_shoulder_center = 0.2;
    scalar_t swing_shoulder_scale = 0.2;
    scalar_t swing_elbow_scale = 3.0;
    bool enable_interrupt_with_est_mode = false; // 是否允许根据实际状态终止规划
    bool enable_slope_planner = false; // 使能斜面规划
    double slope_planning_threshold = 5.0;

  };

  SwingTrajectoryPlanner(Config config, size_t numFeet, size_t numHand = 2);
  void setSlopePlanning(bool enable_slope_planning){enable_slope_planning_ = enable_slope_planning;};
  inline void updateConfig(const Config& cfg){config_ = cfg;}
  inline const Config& getConfig() const {return config_;}
  inline const Config& getDefaultConfig() const {return defaultConfig_;}
  inline std::pair<Eigen::Vector3d, Eigen::Vector3d> getFeetNormalVectors() const {return std::make_pair(lf_normal_vector_, rf_normal_vector_);}
  void update(const ModeSchedule& modeSchedule, scalar_t terrainHeight, const TargetTrajectories& targetTrajectories, const scalar_t& initTime);

  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence, const TargetTrajectories& targetTrajectories, const scalar_t& initTime);

  void update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
              const feet_array_t<scalar_array_t>& touchDownHeightSequence,
              const feet_array_t<scalar_array_t>& maxHeightSequence, 
              const TargetTrajectories& targetTrajectories, 
              const scalar_t& initTime);
  std::pair<vector3_t, std::vector<vector3_t>> findFootPosNext(int feet, int current_index, int swingStartIndex, int swingFinalIndex, const ModeSchedule& modeSchedule, const vector3_t &last_stance_position, const feet_array_t<std::vector<bool>> &eesContactFlagStocks);
  
  // 存储规划值到 vector 中
  std::vector<scalar_t> saveTrajectoryToVector() const {
    std::vector<scalar_t> trajectoryData;
    auto node_to_vector = [](const CubicSpline::Node& node) {
         return std::vector<scalar_t>{node.time, node.position, node.velocity};
    };
     // 展开并存储 feetXTrajectories_
    for (const auto& legTrajectories : feetXTrajectories_) {
        for (const auto& spline : legTrajectories) {// 包含两个node
            auto node_vector = node_to_vector(spline.start());
            trajectoryData.insert(trajectoryData.end(), node_vector.begin(), node_vector.end());
            node_vector = node_to_vector(spline.end());
            trajectoryData.insert(trajectoryData.end(), node_vector.begin(), node_vector.end());
        }
    }

    // 展开并存储 feetYTrajectories_
    for (const auto& legTrajectories : feetYTrajectories_) {
        for (const auto& spline : legTrajectories) {
            auto node_vector = node_to_vector(spline.start());
            trajectoryData.insert(trajectoryData.end(), node_vector.begin(), node_vector.end());
            node_vector = node_to_vector(spline.end());
            trajectoryData.insert(trajectoryData.end(), node_vector.begin(), node_vector.end());
        }
    }

    // 展开并存储 feetHeightTrajectories_
    for (const auto& legTrajectories : feetHeightTrajectories_) {
        for (const auto& splineCpg : legTrajectories) {
            // SplineCpg 是由多个 CubicSpline 组成
            auto nodes = splineCpg.getNodes();
            for (const auto& node : nodes) {// 3 nodes per splineCpg
                trajectoryData.push_back(node.time);
                trajectoryData.push_back(node.position);
                trajectoryData.push_back(node.velocity);
            }
        }
    }

    return trajectoryData;
  }

  void loadTrajectoryFromVector(const std::vector<scalar_t> &trajectoryData, const ModeSchedule &modeSchedule )
  {
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &eventTimes = modeSchedule.eventTimes;
    size_t index = 0;
    for (size_t j = 0; j < numFeet_; ++j) {
        feetXTrajectories_[j].clear();
        feetXTrajectories_[j].reserve(modeSequence.size());
        feetYTrajectories_[j].clear();
        feetYTrajectories_[j].reserve(modeSequence.size());
        feetHeightTrajectories_[j].clear();
        feetHeightTrajectories_[j].reserve(modeSequence.size());
        feetHeightTrajectoriesEvents_[j] = eventTimes;
    }

    // 恢复 feetXTrajectories_
    for (size_t j = 0; j < feetXTrajectories_.size(); ++j) {
        for (size_t i = 0; i < modeSequence.size(); ++i) {
            
            // 读取 start node
            CubicSpline::Node startNode;
            startNode.time = trajectoryData[index++];
            startNode.position = trajectoryData[index++];
            startNode.velocity = trajectoryData[index++];
            
            // 读取 end node
            CubicSpline::Node endNode;
            endNode.time = trajectoryData[index++];
            endNode.position = trajectoryData[index++];
            endNode.velocity = trajectoryData[index++];

            feetXTrajectories_[j].emplace_back(startNode, endNode);
        }
    }

    // 恢复 feetYTrajectories_
    for (size_t j = 0; j < feetYTrajectories_.size(); ++j) {
        for (size_t i = 0; i < modeSequence.size(); ++i) {
            
            // 读取 start node
            CubicSpline::Node startNode;
            startNode.time = trajectoryData[index++];
            startNode.position = trajectoryData[index++];
            startNode.velocity = trajectoryData[index++];
            
            // 读取 end node
            CubicSpline::Node endNode;
            endNode.time = trajectoryData[index++];
            endNode.position = trajectoryData[index++];
            endNode.velocity = trajectoryData[index++];

            feetYTrajectories_[j].emplace_back(startNode, endNode);
        }
    }

    // 恢复 feetHeightTrajectories_ 中的 SplineCpg
    for (size_t j = 0; j < feetHeightTrajectories_.size(); ++j) {
        for (size_t i = 0; i < modeSequence.size(); ++i) {
            // SplineCpg 由 3 个 CubicSpline 节点组成
            std::vector<CubicSpline::Node> nodes;
            for (int n = 0; n < 3; ++n) {
                CubicSpline::Node node;
                node.time = trajectoryData[index++];
                node.position = trajectoryData[index++];
                node.velocity = trajectoryData[index++];
                nodes.push_back(node);
            }
            feetHeightTrajectories_[j].emplace_back(nodes[0], nodes[1], nodes[2]);
        }
    }
  }
  vector_t getArmEeWrenchConstraint() const
  {
    return armEeWrenchConstraint_;
  }

  void setArmEeWrenchConstraint(const vector_t& hand_wrench)
  {
    if (hand_wrench.size() == 12)
      armEeWrenchConstraint_ = hand_wrench;
    else
      std::cout << "[SwingTrajectoryPlanner] hand_wrench size is not 12" << std::endl;
  }

  void modifyHeightSequences(const ModeSchedule &modeSchedule, feet_array_t<scalar_array_t> &liftOffHeightSequence, feet_array_t<scalar_array_t> &touchDownHeightSequence);

  scalar_t getXvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getYvelocityConstraint(size_t leg, scalar_t time) const;
  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  feet_array_t<vector3_t> getNextFootPositions() const{ return calc_stance_position_;}

  scalar_t getXpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getYpositionConstraint(size_t leg, scalar_t time) const;
  scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;
  feet_array_t<vector3_t> getFootPositionConstraint(scalar_t time) const;

  // scalar_t getArmJointPositionConstraint(size_t hand, scalar_t time) const;

  // scalar_t getArmJointVelocityConstraint(size_t hand, scalar_t time) const;

  TargetTrajectories getSwingArmTargetTrajectories(scalar_t time) {
    
    armTargetTrajectories_.updateFromBuffer();
    if (armTargetTrajectories_.get().empty()) {
      std::cerr << "Arm target trajectories are empty. Returning Zero trajectories." << std::endl;
      return {{time + 1.0}, {vector_t::Zero(num_arm_joints_)}, {vector_t::Zero(num_arm_joints_)}};
    }
    return armTargetTrajectories_.get();}
  inline void setCurrentFeetPosition(const feet_array_t<vector3_t>& current_feet_position)
  {
    current_feet_position_buf_.setBuffer(current_feet_position);
  }
  inline void setCurrentFeetPosition(const vector_t& current_feet_position)
  {
    feet_array_t<vector3_t> feet_pos;
    for (int i = 0; i < numFeet_; i++)
      feet_pos[i] = current_feet_position.segment<3>(3 * i);
    current_feet_position_buf_.setBuffer(feet_pos);
  }
  inline void setCurrentBodyState(const vector_t& current_body_state)
  {
    current_body_state_buf_.setBuffer(current_body_state);
  }
  inline void setBodyVelCmd(const vector_t& body_vel_cmd)
  {
    body_vel_cmd_buf_.setBuffer(body_vel_cmd);
  }
  inline void setCurrentState(const vector_t& current_state)
  {
    current_state_= current_state;
  }
  inline void interruptSwing()
  {
    interrupt_swing_ = true;
  }
  void printFeetPositions(size_t leg) const;
  inline void setArmDof(size_t num)
  {
    num_arm_joints_ = num;
    std::cout << "[SwingTrajectoryPlanner] num_arm_joints_ = " << num_arm_joints_ << std::endl;
  }
  bool hasFeetTrajectory() const { return has_feet_trajectory_; }
  inline bool isSingleStepPhase() const { return is_single_step_phase_; }
  inline void setSingleStepPhase(bool is_single_step_phase) { is_single_step_phase_ = is_single_step_phase; }
  inline FootPoseSchedule getfootPoseSchedule() const { return footPoseSchedule_; }
  bool planSingleStep(scalar_t initTime, const vector_t& cmd_pose, Eigen::Vector3d xyYawTh);
  inline double getTargetYaw() const { return target_yaw_; }
  inline void setTargetYaw(double target_yaw) { target_yaw_ = target_yaw; }

  inline bool getFinalYawSingleStepMode() const { return final_yaw_single_step_mode_; }
  inline void setFinalYawSingleStepMode(bool mode) { final_yaw_single_step_mode_ = mode; }
  inline double getFinalYawSingleStepTime() const { return final_yaw_single_step_time_; }
  void setInverseKinematics(std::shared_ptr<InverseKinematics> inverseKinematics) { inverseKinematics_ = inverseKinematics; }
 private:
  vector3_t calNextFootPos(int feet, scalar_t current_time, scalar_t stop_time,
                                                         scalar_t next_middle_time, const vector_t &next_middle_body_pos,
                                                         const vector_t &current_body_pos, const vector3_t &current_body_vel,
                                                         const TargetTrajectories& targetTrajectories, vector3_t &last_stance_position, bool verbose = false);
  vector3_t calNextFootPos(int feet, const scalar_t& swingInitTime, const scalar_t& swingFinalTime, 
                           const vector_t& currentBodyState, const TargetTrajectories& targetTrajectories);
  std::pair<bool, vector3_t> calNextFootPos(int feet, const ModeSchedule& modeSchedule, const scalar_t& initTime);
  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  static std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(const std::vector<bool>& contactFlagStock);

  /**
   * Check if event time indices are valid
   * @param leg
   * @param index : phase index
   * @param startIndex : liftoff event time index
   * @param finalIndex : touchdown event time index
   * @param phaseIDsStock : mode sequence
   */
  static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock);

  static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

  Config config_;
  Config defaultConfig_;
  const size_t numFeet_;
  const size_t numHand_;
  size_t num_arm_joints_ = 14;
  BufferedValue<TargetTrajectories> armTargetTrajectories_;
  feet_array_t<std::vector<CubicSpline>> feetXTrajectories_;
  feet_array_t<std::vector<CubicSpline>> feetYTrajectories_;
  feet_array_t<std::vector<SplineCpg>> feetHeightTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetHeightTrajectoriesEvents_;

  feet_array_t<std::vector<DrakeInterpolator>> footXInterpolators_;
  feet_array_t<std::vector<DrakeInterpolator>> footYInterpolators_;
  feet_array_t<std::vector<DrakeInterpolator>> footZInterpolators_;
  
  vector_t armEeWrenchConstraint_ = vector_t::Zero(12);
  hand_array_t<std::vector<SplineCpg>> armJointTrajectories_;
  hand_array_t<std::vector<scalar_t>> armJointTrajectoriesEvents_;
  feet_array_t<vector3_t> feetXYOffset_;
  feet_array_t<vector3_t> feetXYOffsetLocalFrame_;
  feet_array_t<double> feetZOffset_;

  BufferedValue<feet_array_t<vector3_t>> current_feet_position_buf_;
  BufferedValue<vector_t> current_body_state_buf_;
  BufferedValue<vector_t> body_vel_cmd_buf_;
  vector_t current_state_;
  feet_array_t<vector3_t> latestStanceposition_;
  feet_array_t<double> latestStancepositionZ_;
  feet_array_t<vector3_t> stancePositionBeforeStepControl_;
  feet_array_t<vector3_t> latestFullyStanceposition_;
  feet_array_t<vector3_t> des_latestStanceposition_;
  feet_array_t<vector3_t> calc_stance_position_;
  feet_array_t<bool> visualize_stance_position_flag_;
  feet_array_t<vector3_t> last_calc_stance_position_;
  bool has_feet_trajectory_ = false;
  bool interrupt_swing_ = false;
  Eigen::Vector3d lf_normal_vector_, rf_normal_vector_;
  bool is_single_step_phase_ = false;
  FootPoseSchedule footPoseSchedule_;
  double target_yaw_ = 0.0;
  bool final_yaw_single_step_mode_ = false;
  double final_yaw_single_step_time_ = 0.0;
  std::shared_ptr<InverseKinematics> inverseKinematics_;
  bool enable_slope_planning_ = false;
};

SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName,
                                                           const std::string& fieldName = "swing_trajectory_config", bool verbose = true);

}  // namespace humanoid
}  // namespace ocs2
