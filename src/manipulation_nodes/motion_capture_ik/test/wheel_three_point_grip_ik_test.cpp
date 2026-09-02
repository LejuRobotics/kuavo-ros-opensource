#include "motion_capture_ik/WheelOneStageIKEndEffector.h"

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

void setPoseFromFk(HighlyDynamic::WheelOneStageIKEndEffector& solver,
                   const Eigen::VectorXd& joints,
                   const char* frameName,
                   PoseData& target) {
  const auto [position, orientation] = solver.FK(joints, frameName);
  target.position = position;
  target.rotation_matrix = orientation.normalized().toRotationMatrix();
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheel_three_point_grip_ik_test",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ros::Time::init();

  const std::string assetsPath = ros::package::getPath("kuavo_assets");
  if (assetsPath.empty()) {
    std::cerr << "cannot locate kuavo_assets" << std::endl;
    return 1;
  }

  drake::multibody::MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser parser(&plant);
  parser.package_map().Add("kuavo_assets", assetsPath);
  parser.AddModelFromFile(
      assetsPath + "/models/biped_s63/urdf/drake/biped_v3_arm.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"));
  plant.Finalize();

  HighlyDynamic::WheelPointTrackIKSolverConfig config;
  config.historyBufferSize = 4;
  config.refSecondOrderLpfCutoffHz = 12.0;
  config.refSecondOrderLpfDt = 0.01;
  config.eeTrackingWeight = 5000.0;
  config.link6TrackingWeight = 5000.0;
  config.virtualThumbTrackingWeight = 5000.0;
  config.elbowTrackingWeight = 1000.0;
  config.shoulderTrackingWeight = 5000.0;
  config.chestTrackingWeight = 5000.0;
  config.enableWaistElbowClearanceConstraint = true;
  config.waistElbowLateralClearance = 0.20;
  config.jointSmoothWeight0 = 1000.0;
  config.jointSmoothWeight1 = 1000.0;
  config.jointSmoothWeight2 = 500.0;
  config.jointSmoothWeight3 = 300.0;
  config.jointSmoothWeight4 = 10.0;
  config.jointSmoothWeight5 = 10.0;
  config.jointSmoothWeight6 = 10.0;
  config.waistSmoothWeight0 = 3000.0;
  config.waistSmoothWeight1 = 3000.0;
  config.waistSmoothWeight2 = 3000.0;
  config.waistSmoothWeight3 = 3000.0;
  config.accSmoothWeightDefault = 50.0;
  config.jerkSmoothWeightDefault = 1.0;

  const std::vector<std::string> frameNames{
      "base_link", "zarm_l7_end_effector", "zarm_r7_end_effector",
      "zarm_l4_link", "zarm_r4_link"};
  HighlyDynamic::WheelOneStageIKEndEffector solver(&plant, frameNames, config);

  if (plant.num_positions() != 18) {
    std::cerr << "expected 18 positions, got " << plant.num_positions() << std::endl;
    return 1;
  }

  Eigen::VectorXd publishedJoints = Eigen::VectorXd::Zero(18);
  publishedJoints(10) = 28.5 * M_PI / 180.0;
  publishedJoints(17) = -24.0 * M_PI / 180.0;

  std::vector<PoseData> constraints(POSE_DATA_LIST_SIZE_PLUS);
  setPoseFromFk(solver, publishedJoints, "waist_yaw_link",
                constraints[POSE_DATA_LIST_INDEX_CHEST]);
  setPoseFromFk(solver, publishedJoints, "zarm_l2_joint_parent",
                constraints[POSE_DATA_LIST_INDEX_LEFT_SHOULDER]);
  setPoseFromFk(solver, publishedJoints, "zarm_r2_joint_parent",
                constraints[POSE_DATA_LIST_INDEX_RIGHT_SHOULDER]);
  setPoseFromFk(solver, publishedJoints, "zarm_l4_link",
                constraints[POSE_DATA_LIST_INDEX_LEFT_ELBOW]);
  setPoseFromFk(solver, publishedJoints, "zarm_r4_link",
                constraints[POSE_DATA_LIST_INDEX_RIGHT_ELBOW]);
  setPoseFromFk(solver, publishedJoints, "zarm_l6_link",
                constraints[POSE_DATA_LIST_INDEX_LEFT_LINK6]);
  setPoseFromFk(solver, publishedJoints, "zarm_r6_link",
                constraints[POSE_DATA_LIST_INDEX_RIGHT_LINK6]);
  setPoseFromFk(solver, publishedJoints, "zarm_l7_end_effector",
                constraints[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR]);
  setPoseFromFk(solver, publishedJoints, "zarm_r7_end_effector",
                constraints[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR]);
  setPoseFromFk(solver, publishedJoints, "zarm_l7_virtual_thumb_link",
                constraints[POSE_DATA_LIST_INDEX_LEFT_VIRTUAL_THUMB]);
  setPoseFromFk(solver, publishedJoints, "zarm_r7_virtual_thumb_link",
                constraints[POSE_DATA_LIST_INDEX_RIGHT_VIRTUAL_THUMB]);
  constraints[POSE_DATA_LIST_INDEX_LEFT_HAND].position =
      constraints[POSE_DATA_LIST_INDEX_LEFT_LINK6].position;
  constraints[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix =
      constraints[POSE_DATA_LIST_INDEX_LEFT_END_EFFECTOR].rotation_matrix;
  constraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].position =
      constraints[POSE_DATA_LIST_INDEX_RIGHT_LINK6].position;
  constraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix =
      constraints[POSE_DATA_LIST_INDEX_RIGHT_END_EFFECTOR].rotation_matrix;

  if (!solver.resyncArmJointState(ArmIdx::LEFT, publishedJoints.segment(4, 7)) ||
      !solver.resyncArmJointState(ArmIdx::RIGHT, publishedJoints.segment(11, 7))) {
    std::cerr << "failed to seed solver from published command" << std::endl;
    return 1;
  }

  const HighlyDynamic::IKSolveResult result =
      solver.solveIK(constraints, ArmIdx::BOTH);
  if (!result.isSuccess || result.solution.size() != publishedJoints.size()) {
    std::cerr << "three-point IK failed: " << result.solverLog << std::endl;
    return 1;
  }

  const double armError =
      (result.solution.tail(14) - publishedJoints.tail(14)).cwiseAbs().maxCoeff();
  constexpr double kJointTolerance = 1.0e-4;
  if (armError > kJointTolerance) {
    std::cerr << "nonzero-q7 grip changed the published arm seed by "
              << armError << " rad\nresult: " << result.solution.tail(14).transpose()
              << "\nseed:   " << publishedJoints.tail(14).transpose() << std::endl;
    return 1;
  }

  std::cout << "wheel_three_point_grip_ik_test: PASS" << std::endl;
  return 0;
}
