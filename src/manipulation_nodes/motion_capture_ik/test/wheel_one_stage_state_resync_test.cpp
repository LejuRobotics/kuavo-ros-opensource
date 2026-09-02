#include "motion_capture_ik/WheelOneStageIKEndEffector.h"

#include <array>
#include <chrono>
#include <iostream>

namespace {

constexpr double kTolerance = 1.0e-12;

bool expectSegmentNear(const Eigen::VectorXd& actual,
                       Eigen::Index offset,
                       const Eigen::VectorXd& expected,
                       const char* label) {
  if (actual.size() < offset + expected.size()) {
    std::cerr << label << " has invalid size " << actual.size() << std::endl;
    return false;
  }
  const double error = (actual.segment(offset, expected.size()) - expected).norm();
  if (error < kTolerance) return true;
  std::cerr << label << " error: " << error << std::endl;
  return false;
}

}  // namespace

int main() {
  using HighlyDynamic::IKSolveResult;
  using HighlyDynamic::WheelIKResultHistoryBuffer;

  constexpr Eigen::Index kWholeBodyDof = 18;
  constexpr Eigen::Index kLowerBodyDof = 4;
  constexpr Eigen::Index kArmDof = 7;
  constexpr Eigen::Index kLeftOffset = kLowerBodyDof;
  constexpr Eigen::Index kRightOffset = kLowerBodyDof + kArmDof;

  WheelIKResultHistoryBuffer history(3);
  std::array<Eigen::VectorXd, 3> originalSolutions;
  std::array<Eigen::VectorXd, 3> originalVelocities;
  std::array<Eigen::VectorXd, 3> originalAccelerations;
  std::array<Eigen::VectorXd, 3> originalJerks;

  for (std::size_t i = 0; i < originalSolutions.size(); ++i) {
    const double base = 10.0 * static_cast<double>(i + 1);
    originalSolutions[i] = Eigen::VectorXd::LinSpaced(kWholeBodyDof, base, base + 1.7);
    originalVelocities[i] = Eigen::VectorXd::Constant(kWholeBodyDof, base + 2.0);
    originalAccelerations[i] = Eigen::VectorXd::Constant(kWholeBodyDof, base + 3.0);
    originalJerks[i] = Eigen::VectorXd::Constant(kWholeBodyDof, base + 4.0);
    history.add(WheelIKResultHistoryBuffer::IKMotionState(
        IKSolveResult(originalSolutions[i], std::chrono::milliseconds(i + 1)),
        originalVelocities[i],
        originalAccelerations[i],
        originalJerks[i]));
  }

  bool success = true;
  const Eigen::VectorXd leftPublished =
      (Eigen::VectorXd(kArmDof) << 0.11, 0.22, 0.33, 0.44, 0.55, 0.66, 0.77).finished();
  history.resyncSegment(kLeftOffset, leftPublished);

  for (std::size_t reverseIndex = 0; reverseIndex < originalSolutions.size(); ++reverseIndex) {
    const auto* state = history.fromBack(reverseIndex);
    if (!state) {
      std::cerr << "missing history state " << reverseIndex << std::endl;
      success = false;
      continue;
    }
    const std::size_t originalIndex = originalSolutions.size() - 1 - reverseIndex;
    success &= expectSegmentNear(state->result.solution, kLeftOffset, leftPublished,
                                 "left solution resync");
    success &= expectSegmentNear(state->velocity, kLeftOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "left velocity reset");
    success &= expectSegmentNear(state->acceleration, kLeftOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "left acceleration reset");
    success &= expectSegmentNear(state->jerk, kLeftOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "left jerk reset");

    // Lower body and the opposite arm retain their own value from each frame,
    // rather than being flattened to a shared fallback state.
    success &= expectSegmentNear(state->result.solution, 0,
                                 originalSolutions[originalIndex].head(kLowerBodyDof),
                                 "lower-body solution preservation");
    success &= expectSegmentNear(state->result.solution, kRightOffset,
                                 originalSolutions[originalIndex].segment(kRightOffset, kArmDof),
                                 "right solution preservation");
    success &= expectSegmentNear(state->velocity, kRightOffset,
                                 originalVelocities[originalIndex].segment(kRightOffset, kArmDof),
                                 "right velocity preservation");
    success &= expectSegmentNear(state->acceleration, 0,
                                 originalAccelerations[originalIndex].head(kLowerBodyDof),
                                 "lower-body acceleration preservation");
  }

  const Eigen::VectorXd rightPublished =
      (Eigen::VectorXd(kArmDof) << -0.17, -0.27, -0.37, -0.47, -0.57, -0.67, -0.77).finished();
  history.resyncSegment(kRightOffset, rightPublished);
  for (std::size_t reverseIndex = 0; reverseIndex < originalSolutions.size(); ++reverseIndex) {
    const auto* state = history.fromBack(reverseIndex);
    if (!state) {
      success = false;
      continue;
    }
    success &= expectSegmentNear(state->result.solution, kLeftOffset, leftPublished,
                                 "left isolation after right resync");
    success &= expectSegmentNear(state->result.solution, kRightOffset, rightPublished,
                                 "right solution resync");
    success &= expectSegmentNear(state->velocity, kRightOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "right velocity reset");
    success &= expectSegmentNear(state->acceleration, kRightOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "right acceleration reset");
    success &= expectSegmentNear(state->jerk, kRightOffset,
                                 Eigen::VectorXd::Zero(kArmDof), "right jerk reset");
  }

  if (!success) return 1;
  std::cout << "wheel_one_stage_state_resync_test: PASS" << std::endl;
  return 0;
}
