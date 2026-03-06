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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <humanoid_wheel_interface/HumanoidWheelInterface.h>

#include <humanoid_wheel_interface_ros/MobileManipulatorDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ros/init.h>
#include <ros/package.h>

using namespace ocs2;
using namespace mobile_manipulator;

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  /********************************初始位置设置************************************/
  vector_t mujoco_q = vector_t::Zero(7 + 4 + 7*2 + 2);
  mujoco_q[3] = 1.0;
  mujoco_q[11] = 0.5236;
  mujoco_q[14] = -1.57;
  mujoco_q[18] = 0.5236;
  mujoco_q[21] = -1.57;
  std::vector<double> robot_init_state_param;
  for (int i = 0; i < mujoco_q.size(); i++)
  {
    robot_init_state_param.push_back(mujoco_q(i));
  }
  // ros::param::set("robot_init_state_param", robot_init_state_param);
  /*******************************************************************************/

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  nodeHandle.setParam("/robot_init_state_param", robot_init_state_param);
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot Interface
  mobile_manipulator::HumanoidWheelInterface interface(taskFile, libFolder, urdfFile);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto dummyVisualization = std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(nodeHandle, interface);

  // Dummy MRT
  MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_, interface.mpcSettings().mpcDesiredFrequency_);
  dummy.subscribeObservers({dummyVisualization});

  auto& info = interface.getManipulatorModelInfo();
  // initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input.setZero(info.inputDim);
  initObservation.time = 0.0;

  // use pinocchio 
  auto model = interface.getPinocchioInterface().getModel();
  auto data = interface.getPinocchioInterface().getData();
  Eigen::VectorXd init_q = initObservation.state;
  pinocchio::framesForwardKinematics(model, data, init_q);

  std::vector<Eigen::Vector3d> init_ee_pos(info.eeFrames.size());
  for(int eef_inx = 0; eef_inx < info.eeFrames.size(); eef_inx++)
  {
    int ee_id = model.getBodyId(info.eeFrames[eef_inx]);
    init_ee_pos[eef_inx] = data.oMf[ee_id].translation();
  }

  Eigen::Vector3d init_torso_pos;
  int torso_id = model.getBodyId(info.torsoFrame);
  init_torso_pos = data.oMf[torso_id].translation();

  // initial command
  int base_nums = info.stateDim - info.armDim;

  vector_t initTarget(base_nums + 7 + info.eeFrames.size() * 7);
  initTarget.head(base_nums) = vector_t::Zero(base_nums);
  initTarget.segment(base_nums, 3) = init_torso_pos;
  initTarget.segment(base_nums+3, 4) = Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  for(int eef_inx = 0; eef_inx < info.eeFrames.size(); eef_inx++)
  {
    initTarget.tail(info.eeFrames.size() * 7).segment(eef_inx*7, 3) = init_ee_pos[eef_inx];
    initTarget.tail(info.eeFrames.size() * 7).segment(eef_inx*7+3, 4) = Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  }
  const vector_t zeroInput = vector_t::Zero(info.inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});

  // Run dummy (loops while ros is ok)
  dummy.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
