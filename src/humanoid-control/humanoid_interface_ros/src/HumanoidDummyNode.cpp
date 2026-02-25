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

#include <humanoid_interface/HumanoidInterface.h>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "humanoid_interface_ros/visualization/HumanoidVisualizer.h"
#include "humanoid_interface_ros/visualization/HumanoidRbdPublisher.h"

using namespace ocs2;
using namespace humanoid;

int main(int argc, char **argv)
{
  const std::string robotName = "humanoid";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, urdfFile, referenceFile,gaitCommandFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);

  RobotVersion rb_version(3, 4);
  if (nodeHandle.hasParam("/robot_version"))
  {
    int rb_version_int;
    nodeHandle.getParam("/robot_version", rb_version_int);
    rb_version = RobotVersion::create(rb_version_int);
  }
  // Robot interface
  HumanoidInterface interface(taskFile, urdfFile, referenceFile, gaitCommandFile, rb_version);

  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                       interface.modelSettings().contactNames3DoF);
  PinocchioEndEffectorSpatialKinematics endEffectorSpatialKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                                     interface.modelSettings().contactNames6DoF);
  auto humanoidVisualizer = std::make_shared<HumanoidVisualizer>(
      interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, endEffectorSpatialKinematics, nodeHandle, taskFile);

  // RBD state publisher
  CentroidalModelRbdConversions rbdConversions(interface.getPinocchioInterface(), interface.getCentroidalModelInfo());
  size_t log_interval = 0.01/(1/interface.mpcSettings().mrtDesiredFrequency_);
  std::cout << "log_interval: " << log_interval << std::endl;
  auto rbdPublisher = std::make_shared<HumanoidRbdPublisher>(rbdConversions, interface.getCentroidalModelInfo(), "rbd_states.csv", log_interval);

  // Dummy huamnoid robot
  MRT_ROS_Dummy_Loop humanoidDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                            interface.mpcSettings().mpcDesiredFrequency_);
  humanoidDummySimulator.subscribeObservers({humanoidVisualizer, rbdPublisher});

  // Initial state
  SystemObservation initObservation;
  initObservation.state = interface.getInitialState();
  initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
  initObservation.mode = ModeNumber::SS;

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});

  // run dummy
  humanoidDummySimulator.run(initObservation, initTargetTrajectories);

  std::cout << "!!!!!!!!!!!!!!!!!!!dummy node end!!!!!!!!!!!!!!" << std::endl;
  std::cout << humanoidVisualizer->timeTraj.size() << std::endl;
  std::cout << humanoidVisualizer->basePosTraj.size() << std::endl;
  std::cout << humanoidVisualizer->baseOriTraj.size() << std::endl;
  std::cout << humanoidVisualizer->feetPosTraj.size() << std::endl;
  std::cout << humanoidVisualizer->feetOriTraj.size() << std::endl;
  std::cout << humanoidVisualizer->feetInputTraj.size() << std::endl;

  // const char *fileName = "~/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_humanoid_ros/test.csv";
  // const char *fileName = "../ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_humanoid_ros/data/test.csv";
  // std::ofstream fp;
  // fp.open(fileName);
  // if (!fp.is_open())
  // {
  //   // ros::Duration(5).sleep();
  //   throw std::runtime_error("Failed to open " + std::string(fileName));
  // }
  // fp << "time, ";
  // // base position & orientation
  // for (int i = 1; i <= 6; i++)
  // {
  //   fp << "base/" << i << ", ";
  // }
  // // left feet position & orientation
  // for (int i = 1; i <= 6; i++)
  // {
  //   fp << "left_feet/" << i << ", ";
  // }
  // // right feet position & orientation
  // for (int i = 1; i <= 6; i++)
  // {
  //   fp << "right_feet/" << i << ", ";
  // }
  // // left feet input
  // for (int i = 1; i <= 6; i++)
  // {
  //   fp << "left_input/" << i << ", ";
  // }
  // // right feet input
  // for (int i = 1; i <= 6 - 1; i++)
  // {
  //   fp << "right_input/" << i << ", ";
  // }
  // fp << "right_input/" << 6 << "\n";

  // std::cout << "Write data ...\n";

  // const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
  // const int m = humanoidVisualizer->timeTraj.size();
  // const int n = 1 + 5 * 6;
  // Eigen::MatrixXd data(m, n);
  // for (int i = 0; i < m; i++)
  // {
  //   data(i, 0) = humanoidVisualizer->timeTraj[i];
  //   data(i, 1) = humanoidVisualizer->basePosTraj[i][0];
  //   data(i, 2) = humanoidVisualizer->basePosTraj[i][1];
  //   data(i, 3) = humanoidVisualizer->basePosTraj[i][2];

  //   data(i, 4) = humanoidVisualizer->baseOriTraj[i][0];
  //   data(i, 5) = humanoidVisualizer->baseOriTraj[i][1];
  //   data(i, 6) = humanoidVisualizer->baseOriTraj[i][2];

  //   // left foot pos
  //   data(i, 7) = humanoidVisualizer->feetPosTraj[i][0][0];
  //   data(i, 8) = humanoidVisualizer->feetPosTraj[i][0][1];
  //   data(i, 9) = humanoidVisualizer->feetPosTraj[i][0][2];
  //   // left foot ori
  //   data(i, 10) = humanoidVisualizer->feetOriTraj[i][0][0];
  //   data(i, 11) = humanoidVisualizer->feetOriTraj[i][0][1];
  //   data(i, 12) = humanoidVisualizer->feetOriTraj[i][0][2];
  //   // right foot pos
  //   data(i, 13) = humanoidVisualizer->feetPosTraj[i][1][0];
  //   data(i, 14) = humanoidVisualizer->feetPosTraj[i][1][1];
  //   data(i, 15) = humanoidVisualizer->feetPosTraj[i][1][2];
  //   // right foot ori
  //   data(i, 16) = humanoidVisualizer->feetOriTraj[i][1][0];
  //   data(i, 17) = humanoidVisualizer->feetOriTraj[i][1][1];
  //   data(i, 18) = humanoidVisualizer->feetOriTraj[i][1][2];
  //   // left input
  //   data(i, 19) = humanoidVisualizer->feetInputTraj[i][0];
  //   data(i, 20) = humanoidVisualizer->feetInputTraj[i][1];
  //   data(i, 21) = humanoidVisualizer->feetInputTraj[i][2];
  //   data(i, 22) = humanoidVisualizer->feetInputTraj[i][3];
  //   data(i, 23) = humanoidVisualizer->feetInputTraj[i][4];
  //   data(i, 24) = humanoidVisualizer->feetInputTraj[i][5];
  //   // data(i, 22) = humanoidVisualizer->feetForce[i][0][0];
  //   // data(i, 23) = humanoidVisualizer->feetForce[i][0][1];
  //   // data(i, 24) = humanoidVisualizer->feetForce[i][0][2];
  //   // right input
  //   data(i, 25) = humanoidVisualizer->feetInputTraj[i][6];
  //   data(i, 26) = humanoidVisualizer->feetInputTraj[i][7];
  //   data(i, 27) = humanoidVisualizer->feetInputTraj[i][8];
  //   data(i, 28) = humanoidVisualizer->feetInputTraj[i][9];
  //   data(i, 29) = humanoidVisualizer->feetInputTraj[i][10];
  //   data(i, 30) = humanoidVisualizer->feetInputTraj[i][11];
  //   // data(i, 28) = humanoidVisualizer->feetForce[i][1][0];
  //   // data(i, 29) = humanoidVisualizer->feetForce[i][1][1];
  //   // data(i, 30) = humanoidVisualizer->feetForce[i][1][2];
  // }
  // // data.col(0) = humanoidVisualizer->timeTraj;
  // // data.col(0) = humanoidVisualizer->basePosTraj(Eigen::all, 0);
  // // data << humanoidVisualizer->timeTraj, humanoidVisualizer->basePosTraj, humanoidVisualizer->baseOriTraj;
  // // data << timeTraj, log.data().transpose();
  // fp << data.format(CSVFormat);
  // std::cout << "Write complete\n";
  // fp.close();

  // ros::Duration(3).sleep();

  // Successful exit
  return 0;
}
