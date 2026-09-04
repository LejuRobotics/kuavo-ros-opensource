#pragma once

#include <algorithm>
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
  double refSecondOrderLpfCutoffHz = 12.0;  // always-on reference LPF cutoff
  double refSecondOrderLpfDt = 0.0;          // <=0 means fallback to dynamicsDt

  // tracking weights
  double eeTrackingWeight = 4e3;
  double elbowTrackingWeight = 4e2;
  double link6TrackingWeight = 4e3;
  double virtualThumbTrackingWeight = 4e3;
  double shoulderTrackingWeight = 4e3;
  double chestTrackingWeight = 4e3;

  // Final-IK safety floor.  Elbow tracking remains a soft posture cost, but
  // the elbow origin may not cross into the waist-side keep-out band.  The
  // bound is expressed laterally in waist_yaw_link, so it follows torso yaw.
  bool enableWaistElbowClearanceConstraint = true;
  double waistElbowLateralClearance = 0.20;  // [m]

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

  // acceleration and jerk smoothness weights
  double accSmoothWeightDefault = 0.0;
  double jerkSmoothWeightDefault = 0.0;

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

  bool empty() const { return buffer_.empty(); }

  const IKMotionState* fromBack(size_t reverseIndex) const {
    if (reverseIndex >= buffer_.size()) {
      return nullptr;
    }
    return &buffer_[buffer_.size() - 1 - reverseIndex];
  }

  const IKMotionState* latest() const { return fromBack(0); }
  const IKMotionState* prev() const { return fromBack(1); }
  const IKMotionState* pprev() const { return fromBack(2); }

  // Reconcile one arm with the command that was actually published without
  // disturbing the other arm or the four lower-body joints.  Replacing the
  // selected segment in every sample also makes the finite-difference
  // acceleration/jerk targets start from rest at the mode boundary.
  void resyncSegment(Eigen::Index offset, const Eigen::VectorXd& values) {
    for (auto& state : buffer_) {
      if (state.result.solution.size() >= offset + values.size()) {
        state.result.solution.segment(offset, values.size()) = values;
      }
      if (state.velocity.size() >= offset + values.size()) {
        state.velocity.segment(offset, values.size()).setZero();
      }
      if (state.acceleration.size() >= offset + values.size()) {
        state.acceleration.segment(offset, values.size()).setZero();
      }
      if (state.jerk.size() >= offset + values.size()) {
        state.jerk.segment(offset, values.size()).setZero();
      }
    }
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

  std::chrono::milliseconds getMeanSolveDuration() const { return historyBuffer_.getMeanDuration(); }

  // Synchronize one arm's complete solver state with the last seven joint
  // positions actually sent to the robot.  Only ArmIdx::LEFT/RIGHT are valid;
  // the other arm and lower-body/chest state are preserved exactly.
  //
  // Calls must be serialized with solveIK() by the owner of this solver.
  bool resyncArmJointState(ArmIdx side, const Eigen::VectorXd& publishedArmJoints);

  void setElbowTrackingActivations(double leftActivation, double rightActivation) {
    leftElbowTrackingActivation_ = std::clamp(leftActivation, 0.0, 1.0);
    rightElbowTrackingActivation_ = std::clamp(rightActivation, 0.0, 1.0);
  }

  // 腰部位置跟随细分关闭时（chestPositionUpdateEnable_=false），chest 位置更改为超高权重软代价近似
  void setFreezeChestPosition(bool freeze) { freezeChestPosition_ = freeze; }

 private:
  struct LowpassBiquadCoeff {
    double b0{0.0};
    double b1{0.0};
    double b2{0.0};
    double a1{0.0};
    double a2{0.0};
    bool enabled{false};
  };

  static LowpassBiquadCoeff makeSecondOrderLowpassCoeff(double cutoffHz, double sampleTime);
  static Eigen::VectorXd applySecondOrderLowpassVec(const Eigen::VectorXd& x,
                                                    const LowpassBiquadCoeff& coeff,
                                                    Eigen::VectorXd& x1,
                                                    Eigen::VectorXd& x2,
                                                    Eigen::VectorXd& y1,
                                                    Eigen::VectorXd& y2);
  void initializeRefLowpass();
  Eigen::VectorXd applyRefLowpass(const Eigen::VectorXd& input);

  void setConstraints(drake::multibody::InverseKinematics& ik,
                      const std::vector<PoseData>& PoseConstraintList,
                      ArmIdx controlArmIndex,
                      const Eigen::VectorXd& initialGuess,
                      const Eigen::VectorXd& referenceSolution) const;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  WheelIKResultHistoryBuffer historyBuffer_;
  LowpassBiquadCoeff refLowpassCoeff_;
  Eigen::VectorXd refLpX1_;
  Eigen::VectorXd refLpX2_;
  Eigen::VectorXd refLpY1_;
  Eigen::VectorXd refLpY2_;
  Eigen::VectorXd refLowpassLatest_;
  Eigen::VectorXd initialGuessSeed_;
  bool hasRefLowpassState_{false};

  // Optional extended config for tracking weights
  std::unique_ptr<WheelPointTrackIKSolverConfig> pointTrackConfig_;
  double leftElbowTrackingActivation_{1.0};
  double rightElbowTrackingActivation_{1.0};
  bool freezeChestPosition_{false};  // true 时 chest 位置用超高权重软代价近似锁定（位置跟随细分关闭）
};

}  // namespace HighlyDynamic
