#pragma once

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

#include "motion_capture_ik/BaseIKSolver.h"

namespace HighlyDynamic {

struct TwoStageIKParameters {
  double velocityLimit = 720.0;  // 默认速度限制 (deg/s)
  double controllerDt = 0.01;    // 控制器时间步长 (s)

  double stage1DefaultWeight = 0.2;    // 默认平滑权重
  double stage1WristWeight = 5.0;      // 手腕关节权重（近似冻结）
  double stage1PositionWeight = 10.0;  // 位置代价权重

  double stage2DefaultWeight = 1e4;  // 默认关节保持权重
  double stage2WristWeight = 1e-4;   // 手腕关节权重（允许变化）

  // 平滑性权重参数
  double smoothnessWeight = 0.1;  // 平滑性权重

  // 转换为通用参数映射的方法
  ParameterMap toParameterMap() const {
    return {{"velocityLimit", velocityLimit},
            {"controllerDt", controllerDt},
            {"stage1DefaultWeight", stage1DefaultWeight},
            {"stage1WristWeight", stage1WristWeight},
            {"stage1PositionWeight", stage1PositionWeight},
            {"stage2DefaultWeight", stage2DefaultWeight},
            {"stage2WristWeight", stage2WristWeight},
            {"smoothnessWeight", smoothnessWeight}};
  }
};

class TwoStageTorsoIK : public BaseIKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TwoStageTorsoIK(drake::multibody::MultibodyPlant<double>* plant,
                           const std::vector<std::string>& ikConstraintFrameNames,
                           const IKSolverConfig& config);

  ~TwoStageTorsoIK() = default;

  IKSolveResult solveIK(const std::vector<PoseData>& PoseConstraintList,
                        ArmIdx controlArmIndex = ArmIdx::LEFT,
                        const Eigen::VectorXd& jointMidValues = Eigen::VectorXd()) override;

  std::pair<bool, Eigen::VectorXd> getStage1Result() const;
  std::pair<bool, Eigen::VectorXd> getStage2Result() const;

  // Forward Kinematics methods - matching plantIK.cc functionality
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q,
                                                    const std::string& frameName,
                                                    int expectedSize = -1);
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FKElbow(const Eigen::VectorXd& q,
                                                         const std::string& frameName,
                                                         int expectedSize = -1);

  Eigen::MatrixXd getFrameJacobian(const Eigen::VectorXd& q, const std::string& frameName);

 protected:
  void initializeJointIndices() override;
  void initializeJointLimits() override;

  bool setConstraints(const std::vector<drake::multibody::InverseKinematics*>& ikList,
                      const std::vector<const std::vector<PoseData>*>& PoseConstraintLists,
                      const std::vector<ArmIdx>& controlArmIndices) const override;

  Eigen::VectorXd postProcessSolution(const Eigen::VectorXd& rawSolution, const ParameterMap& params) const override;

 private:
  void initializeWristFrames();

  void setStage1Constraints(drake::multibody::InverseKinematics& ik,
                            const std::vector<PoseData>& PoseConstraintList,
                            ArmIdx controlArmIndex,
                            const Eigen::VectorXd& referenceSolution) const;

  void setStage2Constraints(drake::multibody::InverseKinematics& ik,
                            const std::vector<PoseData>& PoseConstraintList,
                            ArmIdx controlArmIndex,
                            const Eigen::VectorXd& referenceSolution) const;

  std::vector<const drake::multibody::Frame<double>*> wristFrames_;
  std::vector<int> leftWristIdx_;
  std::vector<int> rightWristIdx_;

  std::pair<bool, Eigen::VectorXd> stage1Result_;
  std::pair<bool, Eigen::VectorXd> stage2Result_;

  TwoStageIKParameters ikParams_;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  // Mutable member to store reference solution for use in const constraint functions
  mutable Eigen::VectorXd currentReferenceSolution_;
};

}  // namespace HighlyDynamic