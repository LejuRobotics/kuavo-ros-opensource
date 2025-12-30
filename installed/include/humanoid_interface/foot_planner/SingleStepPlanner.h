#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "humanoid_interface/foot_planner/Sat.hpp"
#include "humanoid_interface/gait/GaitSchedule.h"

#pragma once

namespace ocs2 {
namespace humanoid {
  typedef Eigen::Matrix<double, 4, 1> Vector4d;

  // struct SingleStepTrajectory {
  //   std::vector<double> time_traj;
  //   std::vector<int> foot_idx_traj;
  //   std::vector<Vector4d> foot_traj;
  //   std::vector<Vector4d> torso_traj;
  //   SingleStepTrajectory(std::vector<double> time_traj_, std::vector<int> foot_idx_traj_, std::vector<Vector4d> foot_traj_, std::vector<Vector4d> torso_traj_):
  //     time_traj(time_traj_), foot_idx_traj(foot_idx_traj_), foot_traj(foot_traj_), torso_traj(torso_traj_) {}
  // };

  std::pair<Eigen::Vector3d, Eigen::Vector3d> generateSteps(const Eigen::Vector3d& torso_pos, const double torso_yaw, const double foot_bias);
  bool getMultipleStepsTrajectory(FootPoseSchedule &foot_pose_schedule, const std::vector<Vector4d>& body_poses, double dt,
                                     double foot_bias, bool collision_check = true);
}  // namespace humanoid
}  // namespace ocs2