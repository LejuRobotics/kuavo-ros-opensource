#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

// Drake includes
#include <drake/geometry/scene_graph.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>

#include <Eigen/Dense>
#include <leju_utils/define.hpp>

namespace HighlyDynamic {

class TwoStageTorsoIK final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TwoStageTorsoIK(drake::multibody::MultibodyPlant<double>* plant,
                           const std::vector<std::string>& ikConstraintFrameNames,
                           double constraintTolerance = 1.0e-8,
                           double solverTolerance = 1.0e-6,
                           int maxSolverIterations = 100,
                           ArmIdx controlArmIndex = ArmIdx::BOTH,
                           bool isWeldBaseLink = true);

  ~TwoStageTorsoIK() = default;

  std::pair<bool, Eigen::VectorXd> solveTwoStageIK(const std::vector<PoseData>& poseDataList,
                                                   ArmIdx controlArmIndex = ArmIdx::LEFT,
                                                   const Eigen::VectorXd& q0 = Eigen::VectorXd());

  std::pair<bool, Eigen::VectorXd> getStage1Result() const;

  std::pair<bool, Eigen::VectorXd> getStage2Result() const;

  void setPlantContext(std::unique_ptr<drake::systems::Context<double>> context);

  // 新增：参数更新接口
  void updateBaseHeightOffset(double baseHeightOffset);
  void updateBaseChestOffsetX(double baseChestOffsetX);
  void updateShoulderWidth(double shoulderWidth);
  void updateUpperArmLength(double upperArmLength);
  void updateLowerArmLength(double lowerArmLength);

 private:
  // Drake plant and context
  drake::multibody::MultibodyPlant<double>* plant_;
  std::unique_ptr<drake::systems::Context<double>> plantContext_;

  // Configuration parameters
  double constraintTolerance_;
  double solverTolerance_;
  int maxSolverIterations_;
  ArmIdx controlArmIndex_;
  bool isWeldBaseLink_;

  // Robot model information
  int nq_;
  std::vector<const drake::multibody::Frame<double>*> frames_;

  // Wrist joint frames for Stage1 (equivalent to Python's one_stage_ee_frames)
  std::vector<const drake::multibody::Frame<double>*> wristFrames_;

  // Joint indices for wrist joints
  std::vector<int> leftWristIdx_;
  std::vector<int> rightWristIdx_;

  // Stage results storage
  std::pair<bool, Eigen::VectorXd> stage1Result_;
  std::pair<bool, Eigen::VectorXd> stage2Result_;

  // Latest solution tracking (equivalent to Python's __last_solution)
  Eigen::VectorXd latestSolution_;
  bool hasLatestSolution_;

  // Joint limits for limiting
  Eigen::VectorXd jointLowerBounds_;
  Eigen::VectorXd jointUpperBounds_;
  bool hasJointLimits_;

  std::pair<bool, Eigen::VectorXd> solveStage1(const std::vector<PoseData>& poseDataList,
                                               ArmIdx controlArmIndex,
                                               const Eigen::VectorXd& q0 = Eigen::VectorXd());

  std::pair<bool, Eigen::VectorXd> solveStage2(
      const std::vector<std::pair<std::vector<double>, std::vector<double>>>& poseList,
      const Eigen::VectorXd& qStage1,
      ArmIdx controlArmIndex);

  std::pair<bool, Eigen::VectorXd> solveStage2(const std::vector<PoseData>& poseDataList, ArmIdx controlArmIndex);

  void initializeJointIndices();

  void initializeWristFrames();

  Eigen::VectorXd limitAngle(const Eigen::VectorXd& q) const;

  Eigen::VectorXd limitAngleByVelocity(const Eigen::VectorXd& qLast,
                                       const Eigen::VectorXd& qNow,
                                       double velLimit = 50.0,
                                       double controllerDt = 0.01) const;

  void initializeJointLimits();
};

}  // namespace HighlyDynamic