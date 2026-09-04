#include "motion_capture_ik/WheelIncrementalControlModule.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

namespace {

constexpr double kToleranceRad = 1.0e-7;

double quaternionDistance(const Eigen::Quaterniond& lhs, const Eigen::Quaterniond& rhs) {
  const double dot = std::abs(lhs.normalized().dot(rhs.normalized()));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

bool expectSameOrientation(const Eigen::Quaterniond& actual,
                           const Eigen::Quaterniond& expected,
                           const char* label) {
  const double error = quaternionDistance(actual, expected);
  if (error < kToleranceRad) return true;
  std::cerr << label << " orientation error: " << error << " rad" << std::endl;
  return false;
}

std::vector<PoseData> makeConstraintList(const Eigen::Quaterniond& leftQuat,
                                         const Eigen::Quaterniond& rightQuat) {
  std::vector<PoseData> constraints(POSE_DATA_LIST_SIZE_PLUS);
  constraints[POSE_DATA_LIST_INDEX_LEFT_HAND].position = Eigen::Vector3d(0.4, 0.3, 0.5);
  constraints[POSE_DATA_LIST_INDEX_LEFT_HAND].rotation_matrix = leftQuat.normalized().toRotationMatrix();
  constraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].position = Eigen::Vector3d(0.4, -0.3, 0.5);
  constraints[POSE_DATA_LIST_INDEX_RIGHT_HAND].rotation_matrix = rightQuat.normalized().toRotationMatrix();
  return constraints;
}

}  // namespace

int main() {
  using HighlyDynamic::IncrementalControlConfig;
  using HighlyDynamic::WheelIncrementalControlModule;

  IncrementalControlConfig config;
  config.posCutoffHz = 0.0;
  config.orientationCutoffHz = 0.0;
  config.usePythonIncrementalOrientation = true;

  WheelIncrementalControlModule controller(config);

  const Eigen::Quaterniond leftRobotAnchor(Eigen::AngleAxisd(40.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond rightRobotAnchor(Eigen::AngleAxisd(-35.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond leftVrAnchor = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond rightVrAnchor(Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
  const auto constraints = makeConstraintList(leftRobotAnchor, rightRobotAnchor);

  const ArmPose leftVrPose(Eigen::Vector3d(0.2, 0.25, 0.1), leftVrAnchor);
  const ArmPose rightVrPose(Eigen::Vector3d(0.2, -0.25, 0.1), rightVrAnchor);

  controller.enterIncrementalModeLeftArm(
      leftVrPose, constraints, Eigen::Vector3d::Zero(), leftRobotAnchor, Eigen::Quaterniond::Identity());
  controller.enterIncrementalModeRightArm(
      rightVrPose, constraints, Eigen::Vector3d::Zero(), rightRobotAnchor, Eigen::Quaterniond::Identity());

  // Grip entry with an unchanged chest-relative VR orientation must preserve
  // each robot hand anchor, even though the quaternion values are different.
  auto result = controller.computeIncrementalPose(
      leftVrPose, rightVrPose, true, true, leftRobotAnchor, rightRobotAnchor);
  auto [leftAbsAtEntry, rightAbsAtEntry, leftPos, rightPos] =
      result.getLatestIncrementalHandPose(true, false, true);
  (void)leftPos;
  (void)rightPos;
  bool success = true;
  success &= expectSameOrientation(
      leftAbsAtEntry, controller.getRobotLeftHandAnchorQuat(), "left absolute at entry");
  success &= expectSameOrientation(
      rightAbsAtEntry, controller.getRobotRightHandAnchorQuat(), "right absolute at entry");

  // Absolute orientation remains a direct mapping of the current VR pose. A
  // relative hand rotation is applied on top of the grip-entry frame mapping.
  const Eigen::Quaterniond leftVrMotion(Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond rightVrMotion(Eigen::AngleAxisd(-25.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
  const ArmPose leftVrMoved(leftVrPose.position, leftVrMotion * leftVrAnchor);
  const ArmPose rightVrMoved(rightVrPose.position, rightVrMotion * rightVrAnchor);

  result = controller.computeIncrementalPose(
      leftVrMoved, rightVrMoved, true, true, leftRobotAnchor, rightRobotAnchor);
  auto [leftAbsMoved, rightAbsMoved, leftMovedPos, rightMovedPos] =
      result.getLatestIncrementalHandPose(true, false, true);
  (void)leftMovedPos;
  (void)rightMovedPos;

  const Eigen::Quaterniond expectedLeftAbs =
      leftRobotAnchor * leftVrAnchor.conjugate() * leftVrMoved.quaternion;
  const Eigen::Quaterniond expectedRightAbs =
      rightRobotAnchor * rightVrAnchor.conjugate() * rightVrMoved.quaternion;
  success &= expectSameOrientation(leftAbsMoved, expectedLeftAbs, "left absolute after motion");
  success &= expectSameOrientation(rightAbsMoved, expectedRightAbs, "right absolute after motion");

  // The existing incremental-orientation output is independent of the new
  // absolute-frame mapping and must keep its original delta-composition rule.
  auto [leftIncremental, rightIncremental, leftIncrementalPos, rightIncrementalPos] =
      result.getLatestIncrementalHandPose(true, true, true);
  (void)leftIncrementalPos;
  (void)rightIncrementalPos;
  success &= expectSameOrientation(
      leftIncremental, leftVrMotion * leftRobotAnchor, "left incremental after motion");
  success &= expectSameOrientation(
      rightIncremental, rightVrMotion * rightRobotAnchor, "right incremental after motion");

  if (!success) return 1;
  std::cout << "wheel_incremental_orientation_frame_test: PASS" << std::endl;
  return 0;
}
