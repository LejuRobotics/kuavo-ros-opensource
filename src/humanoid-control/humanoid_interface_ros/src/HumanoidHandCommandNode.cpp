/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include "humanoid_interface/command/HumanoidHandTarget.h"
#include <cmath>

#include "humanoid_interface_drake/humanoid_interface_drake.h"

using namespace ocs2;
using namespace humanoid;

namespace
{
  scalar_t targetArmDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t comHeight;
  scalar_t armMode = 1.0;

  vector_t defaultJointState(12);
} // namespace

scalar_t estimateTimeToArmTarget(const vector_t &desiredArmDisplacement)
{
  const scalar_t &dx = desiredArmDisplacement(0);
  const scalar_t &dy = desiredArmDisplacement(1);
  const scalar_t &dz = desiredArmDisplacement(2);
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
  const scalar_t displacementTime = displacement / targetArmDisplacementVelocity;
  return displacementTime;
}

/**
 * Converts the pose of the interactive marker to LeftHandTargetTrajectories.
 */
TargetTrajectories goalLeftHandPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation, const vector_t& lastEeState) {
  
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t currentArmPose = observation.state.segment<14>(12+12);

  const vector_t leftHandTargetPose = [&]()
  {
    vector_t target(7);
     target(0) = position(0);
     target(1) = position(1);
     target(2) = position(2);
     target(3) = orientation.w();
     target(4) = orientation.x();
     target(5) = orientation.y();
     target(6) = orientation.z();

     return target;
  }();

  vector_t targetPose = currentPose;
  targetPose(2) = comHeight;

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToArmTarget(leftHandTargetPose - lastEeState.segment<7>(0));

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size() + lastEeState.size() + 1));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState, currentArmPose, armMode, lastEeState;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState, currentArmPose, armMode, leftHandTargetPose, lastEeState.segment<7>(7);

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * Converts the pose of the interactive marker to RightHandTargetTrajectories.
 */
TargetTrajectories goalRightHandPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation, const vector_t& lastEeState) {

  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t currentArmPose = observation.state.segment<14>(12+12);

  const vector_t rightHandTargetPose = [&]()
  {
    vector_t target(7);
     target(0) = position(0);
     target(1) = position(1);
     target(2) = position(2);
     target(3) = orientation.w();
     target(4) = orientation.x();
     target(5) = orientation.y();
     target(6) = orientation.z();

     return target;
  }();

  vector_t targetPose = currentPose;
  targetPose(2) = comHeight;

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToArmTarget(rightHandTargetPose - lastEeState.segment<7>(7));

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size() + lastEeState.size() + 1));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, defaultJointState, currentArmPose, armMode, lastEeState;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, defaultJointState, currentArmPose, armMode, lastEeState.segment<7>(0), rightHandTargetPose;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "humanoid";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // Get node parameters
  std::string referenceFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  RobotVersion robot_version(3, 4);
  if (nodeHandle.hasParam("/robot_version"))
  {
      int version_num;
      nodeHandle.getParam("/robot_version", version_num);
      int major = version_num / 10;
      int minor = version_num % 10;
      robot_version = RobotVersion(major, minor);
  }
  auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
  defaultJointState = drake_interface_->getDefaultJointState();
  comHeight = drake_interface_->getIntialHeight();
  // loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  // loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetArmDisplacementVelocity", targetArmDisplacementVelocity);

  HandTargetTrajectoriesInteractiveMarker leftTargetPoseCommand(nodeHandle, 0, robotName ,&goalLeftHandPoseToTargetTrajectories);
  HandTargetTrajectoriesInteractiveMarker rightTargetPoseCommand(nodeHandle, 1, robotName ,&goalRightHandPoseToTargetTrajectories);
  ::ros::spin();

  // Successful exit
  return 0;
}
