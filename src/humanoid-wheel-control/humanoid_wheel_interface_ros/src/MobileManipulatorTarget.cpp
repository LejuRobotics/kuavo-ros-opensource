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

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <kuavo_msgs/changeTorsoCtrlMode.h>

using namespace ocs2;

Eigen::Vector3d init_torso_pos = Eigen::Vector3d(0.00766, 0.0, 0.484437);
Eigen::Vector4d init_torso_quat = Eigen::Vector4d(0, 0, 0, 1);
ros::ServiceClient modeClient_;
/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) {
  
  // 准备请求
  kuavo_msgs::changeTorsoCtrlMode srv;
  srv.request.control_mode = 0;  // noControl 模式

  // 调用服务
  if (modeClient_.call(srv)) {
      if (srv.response.result) {
          ROS_INFO("Control mode changed to %d: %s", 
                   srv.response.mode, 
                   srv.response.message.c_str());
      } else {
          ROS_WARN("Failed to change control mode: %s", 
                   srv.response.message.c_str());
      }
  } else {
      ROS_ERROR("Failed to call service mobile_manipulator_mpc_control");
  }

  // time trajectory
  const scalar_array_t timeTrajectory{observation.time};
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  vector_t l_pose = (vector_t(7) << position, orientation.coeffs()).finished();
  vector_t r_pose = l_pose;
  l_pose(1) += 0.1;
  r_pose(1) -= 0.1;
  vector_t torso_pose = vector_t::Zero(7);
  torso_pose.head(3) = init_torso_pos;
  torso_pose.tail(4) = init_torso_quat;
  const vector_t target = (vector_t(14+3+7) << observation.state.head(3), torso_pose, l_pose, r_pose).finished();
  const vector_array_t stateTrajectory{target};
  // input trajectory
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  // 创建服务客户端
  modeClient_ = nodeHandle.serviceClient<kuavo_msgs::changeTorsoCtrlMode>("/mobile_manipulator_mpc_control");

  TargetTrajectoriesInteractiveMarker targetPoseCommand(nodeHandle, robotName, &goalPoseToTargetTrajectories);
  targetPoseCommand.publishInteractiveMarker();

  // Successful exit
  return 0;
}
