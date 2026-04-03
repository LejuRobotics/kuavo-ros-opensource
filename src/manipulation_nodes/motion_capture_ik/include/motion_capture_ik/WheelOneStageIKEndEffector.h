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
struct WheelPointTrackIKSolverConfig : public IKSolverConfig {
  int historyBufferSize = 10;  // for smoothness
  double dynamicsDt = 0.01;    // second

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
  WheelPointTrackIKSolverConfig() : IKSolverConfig() {
    // Base class defaults are already set:
    // constraintTolerance = 1e-8
    // solverTolerance = 1e-6
    // maxIterations = 3000
    // controlArmIndex = ArmIdx::BOTH
    // isWeldBaseLink = true
  }

  // Constructor from base class - allows conversion from IKSolverConfig
  explicit WheelPointTrackIKSolverConfig(const IKSolverConfig& baseConfig) : IKSolverConfig(baseConfig) {}
};

class WheelIKResultHistoryBuffer {
 public:
  explicit WheelIKResultHistoryBuffer(size_t maxSize = 15) : maxSize_(maxSize) {}

  struct IKMotionState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IKSolveResult result;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd jerk;

    IKMotionState() = default;

    IKMotionState(const IKSolveResult& ikResult,
                  const Eigen::VectorXd& vel,
                  const Eigen::VectorXd& acc,
                  const Eigen::VectorXd& jrk)
        : result(ikResult), velocity(vel), acceleration(acc), jerk(jrk) {}
  };

  void add(const IKMotionState& state) {
    if (!state.result.isSuccess) {
      return;
    }
    buffer_.push_back(state);
    if (buffer_.size() > maxSize_) {
      buffer_.pop_front();
    }
  }

  void add(const IKSolveResult& result) { add(IKMotionState(result, Eigen::VectorXd(), Eigen::VectorXd(), Eigen::VectorXd())); }

  void setMaxSize(size_t size) {
    maxSize_ = size;
    while (buffer_.size() > maxSize_) {
      buffer_.pop_front();
    }
  }

  size_t size() const { return buffer_.size(); }
  bool empty() const { return buffer_.empty(); }
  void clear() { buffer_.clear(); }

  bool hasAtLeast(size_t count) const { return buffer_.size() >= count; }

  const std::deque<IKMotionState>& getHistory() const { return buffer_; }
  const IKMotionState* get(size_t index) const { return (index < buffer_.size()) ? &buffer_[index] : nullptr; }

  const IKMotionState* fromBack(size_t reverseIndex) const {
    if (reverseIndex >= buffer_.size()) {
      return nullptr;
    }
    return &buffer_[buffer_.size() - 1 - reverseIndex];
  }

  const IKMotionState* latest() const { return fromBack(0); }
  const IKMotionState* prev() const { return fromBack(1); }
  const IKMotionState* pprev() const { return fromBack(2); }

  Eigen::VectorXd getMeanSolution(int nq) const {
    if (buffer_.empty()) {
      return Eigen::VectorXd::Zero(nq);
    }
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(nq);
    for (const auto& state : buffer_) {
      if (state.result.isSuccess && state.result.solution.size() == nq) {
        mean += state.result.solution;
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
    for (const auto& state : buffer_) {
      if (state.result.isSuccess) {
        total += state.result.solveDuration.count();
      }
    }
    return std::chrono::milliseconds(total / static_cast<int64_t>(buffer_.size()));
  }

 private:
  std::deque<IKMotionState> buffer_;
  size_t maxSize_;
};

class WheelOneStageIKEndEffector : public BaseIKSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor accepting WheelPointTrackIKSolverConfig for extended configuration
  explicit WheelOneStageIKEndEffector(drake::multibody::MultibodyPlant<double>* plant,
                                 const std::vector<std::string>& ikConstraintFrameNames,
                                 const WheelPointTrackIKSolverConfig& config);

  ~WheelOneStageIKEndEffector() = default;

  IKSolveResult solveIK(const std::vector<PoseData>& PoseConstraintList,
                        ArmIdx controlArmIndex = ArmIdx::LEFT,
                        const Eigen::VectorXd& jointMidValues = Eigen::VectorXd()) override;

  // Forward Kinematics methods - matching plantIK.cc functionality
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> FK(const Eigen::VectorXd& q, const std::string& frameName);

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

  WheelIKResultHistoryBuffer historyBuffer_;
  size_t validIkUpdateCount_ = 0;

  // Optional extended config for tracking weights
  std::unique_ptr<WheelPointTrackIKSolverConfig> pointTrackConfig_;
};

}  // namespace HighlyDynamic