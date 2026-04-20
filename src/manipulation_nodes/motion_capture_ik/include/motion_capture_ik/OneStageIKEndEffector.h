#pragma once

#include <chrono>
#include <deque>
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

// Simple history buffer for IK results
struct PointTrackIKSolverConfig : public IKSolverConfig {
  int historyBufferSize = 10;  // for smoothness

  // tracking weights
  double eeTrackingWeight = 4e3;
  double elbowTrackingWeight = 4e2;
  double link6TrackingWeight = 4e3;
  double virtualThumbTrackingWeight = 4e3;
  double shoulderTrackingWeight = 4e3;
  double chestTrackingWeight = 4e3;

  // joint smoothness weights (7 joints per arm, symmetric for left and right)
  double jointSmoothWeightDefault = 5e1;  // Default weight for all joints
  double jointSmoothWeight0 = 5e1;        // Joint 0 (left) / Joint 7 (right)
  double jointSmoothWeight1 = 5e1;        // Joint 1 (left) / Joint 8 (right)
  double jointSmoothWeight2 = 5e1;        // Joint 2 (left) / Joint 9 (right)
  double jointSmoothWeight3 = 1e1;        // Joint 3 (left) / Joint 10 (right)
  double jointSmoothWeight4 = 1e-3;       // Joint 4 (left) / Joint 11 (right)
  double jointSmoothWeight5 = 1e-3;       // Joint 5 (left) / Joint 12 (right)
  double jointSmoothWeight6 = 1e-3;       // Joint 6 (left) / Joint 13 (right)

  double waistSmoothWeight0 = 0.0;
  double waistSmoothWeight1 = 0.0;
  double waistSmoothWeight2 = 0.0;
  double waistSmoothWeight3 = 0.0;

  // Default constructor - initializes base class with default values
  PointTrackIKSolverConfig() : IKSolverConfig() {
    // Base class defaults are already set:
    // constraintTolerance = 1e-8
    // solverTolerance = 1e-6
    // maxIterations = 3000
    // controlArmIndex = ArmIdx::BOTH
    // isWeldBaseLink = true
  }

  // Constructor from base class - allows conversion from IKSolverConfig
  explicit PointTrackIKSolverConfig(const IKSolverConfig& baseConfig) : IKSolverConfig(baseConfig) {}
};

class IKResultHistoryBuffer {
 public:
  explicit IKResultHistoryBuffer(size_t maxSize = 15) : maxSize_(maxSize) {}

  void add(const IKSolveResult& result) {
    // print result size in cout
    // std::cout << "OneStageIKEndEffector::solveIK: result size = " << result.solution.size() << std::endl;
    if (result.isSuccess) {
      buffer_.push_back(result);
      if (buffer_.size() > maxSize_) {
        buffer_.pop_front();
      }
    }
  }

  void setMaxSize(size_t size) {
    maxSize_ = size;
    while (buffer_.size() > maxSize_) {
      buffer_.pop_front();
    }
  }

  size_t size() const { return buffer_.size(); }
  bool empty() const { return buffer_.empty(); }
  void clear() { buffer_.clear(); }

  const std::deque<IKSolveResult>& getHistory() const { return buffer_; }
  const IKSolveResult* get(size_t index) const { return (index < buffer_.size()) ? &buffer_[index] : nullptr; }

  Eigen::VectorXd getMeanSolution(int nq) const {
    if (buffer_.empty()) {
      return Eigen::VectorXd::Zero(nq);
    }
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(nq);
    for (const auto& result : buffer_) {
      if (result.isSuccess && result.solution.size() == nq) {
        mean += result.solution;
      }
    }
    mean /= static_cast<double>(buffer_.size());
    return mean;
  }

  std::chrono::milliseconds getMeanDuration() const {
    if (buffer_.empty()) {
      return std::chrono::milliseconds(0);
    }
    int64_t total = 0;
    for (const auto& result : buffer_) {
      if (result.isSuccess) {
        total += result.solveDuration.count();
      }
    }
    return std::chrono::milliseconds(total / static_cast<int64_t>(buffer_.size()));
  }

 private:
  std::deque<IKSolveResult> buffer_;
  size_t maxSize_;
};

class OneStageIKEndEffector : public BaseIKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Backward-compatible constructor for legacy callers using base config.
  explicit OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const IKSolverConfig& config,
                                 size_t historyBufferSize = 10);

  // Constructor accepting PointTrackIKSolverConfig for extended configuration
  explicit OneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const PointTrackIKSolverConfig& config);

  ~OneStageIKEndEffector() = default;

  IKSolveResult solveIK(const std::vector<PoseData>& PoseConstraintList,
                        ArmIdx controlArmIndex = ArmIdx::LEFT,
                        const Eigen::VectorXd& jointMidValues = Eigen::VectorXd()) override;

  // Forward Kinematics methods - compatible with legacy and current callers.
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q,
                                                    const std::string& frameName,
                                                    int expectedSize);
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q, const std::string& frameName);
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FKElbow(const Eigen::VectorXd& q,
                                                         const std::string& frameName,
                                                         int expectedSize = -1);

  // Mean calculation
  Eigen::VectorXd getMeanSolution() const { return historyBuffer_.getMeanSolution(nq_); }
  std::chrono::milliseconds getMeanSolveDuration() const { return historyBuffer_.getMeanDuration(); }

 private:
  void initializeWristFrames();

  void setConstraints(drake::multibody::InverseKinematics& ik,
                      const std::vector<PoseData>& PoseConstraintList,
                      ArmIdx controlArmIndex,
                      const Eigen::VectorXd& initialGuess,
                      const Eigen::VectorXd& referenceSolution) const;

  // Debug: Print configuration in table format
  void printConfigTable() const;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  IKSolveResult solveResult_;

  mutable Eigen::VectorXd currentReferenceSolution_;

  IKResultHistoryBuffer historyBuffer_;

  // Optional extended config for tracking weights
  std::unique_ptr<PointTrackIKSolverConfig> pointTrackConfig_;
};

}  // namespace HighlyDynamic