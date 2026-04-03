#pragma once

#include <cstdint>
#include <memory>

#include <Eigen/Core>
#include <ros/ros.h>

namespace ocs2 {
namespace mobile_manipulator {
class HybridRuckigExtrapolator;
}  // namespace mobile_manipulator
}  // namespace ocs2

namespace humanoidController_wheel_wbc {

class ArmTrajectoryInterpolator {
public:
  struct Config {
    Eigen::VectorXd maxVel;
    Eigen::VectorXd maxAcc;
    Eigen::VectorXd maxJerk;
    double qCutoffHz{30.0};
    double vCutoffHz{20.0};
    double aCutoffHz{15.0};
    double timeoutSec{0.2};
    double controlCycleSec{0.002};
    Eigen::VectorXd dqRemapK;
    Eigen::VectorXd dqRemapOffset;
    // fhan 跟踪微分器参数：r 为加速度阈值，h0Ratio 决定 h0 = ratio * controlCycleSec（建议 2~5）
    double fhanR{50.0};
    double fhanH0Ratio{3.0};
  };

  struct TargetSample {
    Eigen::VectorXd targetQ;
    Eigen::VectorXd targetV;
    bool hasTargetV{false};
    ros::Time msgStamp;
    uint64_t msgSeq{0};
    Eigen::VectorXd measuredDq;
  };

  struct ModeFlags {
    bool useArmTrajectoryControl{false};
    int quickMode{0};
    int lbMpcMode{0};
    int armCtrlMode{0};
  };

  struct Output {
    Eigen::VectorXd smoothQ;
    Eigen::VectorXd smoothV;
    bool valid{false};
    bool timeout{false};
  };

  ArmTrajectoryInterpolator() = default;

  void configure(const Config& config);
  void reset(const Eigen::VectorXd& currentQ, const ros::Time& now);
  void ingestRawTarget(const ros::Time& now, const Eigen::VectorXd& targetQ, const Eigen::VectorXd& targetV,
                       const Eigen::VectorXd& measuredDq);
  void pushTarget(const TargetSample& sample);
  Output compute(const ros::Time& now, const ModeFlags& modeFlags, const Eigen::VectorXd& currentQ);

private:
  static bool isFiniteVector(const Eigen::VectorXd& vec);
  bool isEnabled(const ModeFlags& modeFlags) const;
  void ensureBackend(size_t dof);

  Config config_;
  bool configured_{false};
  bool initialized_{false};
  bool hasTarget_{false};

  ModeFlags prevModeFlags_;
  bool hasPrevModeFlags_{false};
  uint64_t latestTargetSeq_{0};
  bool hasLatestTargetSeq_{false};
  uint64_t lastAppliedTargetSeq_{0};
  bool hasLastAppliedTargetSeq_{false};
  uint64_t localRawSeq_{0};
  TargetSample target_;
  Eigen::VectorXd lastRawTargetQ_;
  Eigen::VectorXd lastRawTargetV_;
  Eigen::VectorXd lastEstimatedTargetV_;
  ros::Time lastRawTargetStamp_;
  bool hasLastRawTarget_{false};

  Eigen::VectorXd smoothQ_;
  Eigen::VectorXd smoothV_;
  // fhan 跟踪微分器状态：持续跟踪 smoothQ_，输出满足动力学约束的 q/dq
  Eigen::VectorXd fhanQ_;
  Eigen::VectorXd fhanV_;
  ros::Time lastTargetStamp_;

  std::shared_ptr<ocs2::mobile_manipulator::HybridRuckigExtrapolator> backend_;
};

}  // namespace humanoidController_wheel_wbc
