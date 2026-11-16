#include "motion_capture_ik/WheelNaturalElbowGuide.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>

namespace {

constexpr double kTolerance = 1.0e-8;

void assertNear(const Eigen::Vector3d& actual,
                const Eigen::Vector3d& expected,
                double tolerance = kTolerance) {
  assert((actual - expected).norm() < tolerance);
}

void assertCircleGeometry(const HighlyDynamic::WheelNaturalElbowGuideInput& input,
                          const HighlyDynamic::WheelNaturalElbowGuideOutput& output,
                          double upperArmLength,
                          double lowerArmLength,
                          double tolerance = 1.0e-7) {
  assert(std::abs((output.elbowPosition - input.shoulderPosition).norm() - upperArmLength) < tolerance);
  assert(std::abs((output.elbowPosition - output.circleHandPosition).norm() - lowerArmLength) < tolerance);
  assert(std::isfinite(output.elbowPosition.x()));
  assert(std::isfinite(output.elbowPosition.y()));
  assert(std::isfinite(output.elbowPosition.z()));
}

}  // namespace

int main() {
  using HighlyDynamic::WheelNaturalElbowGuide;
  using HighlyDynamic::WheelNaturalElbowGuideConfig;
  using HighlyDynamic::WheelNaturalElbowGuideInput;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double l1 = 0.30;
  constexpr double l2 = 0.25;

  WheelNaturalElbowGuideConfig config;
  config.naturalDirectionBlend = 0.90;
  config.reachMargin = 0.002;
  WheelNaturalElbowGuide guide(l1, l2, config);

  WheelNaturalElbowGuideInput input;
  input.shoulderPosition = Eigen::Vector3d::Zero();
  input.handPosition = Eigen::Vector3d(0.40, 0.0, 0.0);
  input.currentElbowPosition = Eigen::Vector3d(0.20, 0.0, -0.20);
  input.humanShoulderPosition = Eigen::Vector3d::Zero();
  input.humanElbowPosition = Eigen::Vector3d(0.20, 0.20, 0.0);
  input.humanHandPosition = Eigen::Vector3d(0.40, 0.0, 0.0);
  input.humanPoseValid = true;

  const auto blended = guide.update(input);
  const Eigen::Vector3d expectedBlend =
      Eigen::Vector3d(0.0, 0.10, -0.90) / Eigen::Vector3d(0.0, 0.10, -0.90).norm();
  assertCircleGeometry(input, blended, l1, l2);
  assertNear(blended.naturalDirection, Eigen::Vector3d(0.0, 0.0, -1.0));
  assertNear(blended.humanDirection, Eigen::Vector3d(0.0, 1.0, 0.0));
  assertNear(blended.commandDirection, expectedBlend);
  assert(std::abs(blended.humanActivation - 0.10) < kTolerance);
  assert(blended.elbowTrackingActivation > 0.99);

  // Invalid human data must not disable the gravity-guided natural direction.
  guide.reset();
  input.humanPoseValid = false;
  const auto gravityOnly = guide.update(input);
  assertCircleGeometry(input, gravityOnly, l1, l2);
  assertNear(gravityOnly.commandDirection, Eigen::Vector3d(0.0, 0.0, -1.0));
  assert(gravityOnly.humanActivation == 0.0);

  // If gravity is parallel to the arm, preserve the measured/current radial
  // direction. This is the only branch-continuity fallback.
  guide.reset();
  WheelNaturalElbowGuideInput vertical;
  vertical.shoulderPosition = Eigen::Vector3d::Zero();
  vertical.handPosition = Eigen::Vector3d(0.0, 0.0, -0.40);
  vertical.currentElbowPosition = Eigen::Vector3d(0.20, 0.0, -0.20);
  const auto degenerate = guide.update(vertical);
  assertCircleGeometry(vertical, degenerate, l1, l2);
  assert(!degenerate.gravityDirectionValid);
  assertNear(degenerate.commandDirection, Eigen::Vector3d::UnitX());

  // Mirroring the human bend mirrors only the 10% human component; no manual
  // left/right sign convention is present in the geometry.
  guide.reset();
  input.humanPoseValid = true;
  input.humanElbowPosition = Eigen::Vector3d(0.20, -0.20, 0.0);
  const auto mirrored = guide.update(input);
  assertNear(mirrored.commandDirection,
             Eigen::Vector3d(0.0, -0.10, -0.90) /
                 Eigen::Vector3d(0.0, -0.10, -0.90).norm());

  // At full reach the elbow circle is almost unobservable. The elbow cost must
  // fade out while the hand target remains untouched by the guide.
  guide.reset();
  input.humanPoseValid = false;
  input.handPosition = Eigen::Vector3d(l1 + l2, 0.0, 0.0);
  const auto extended = guide.update(input);
  assert(extended.elbowTrackingActivation == 0.0);
  assertNear(input.handPosition, Eigen::Vector3d(l1 + l2, 0.0, 0.0));
  assert(extended.circleHandPosition.x() < input.handPosition.x());
  assertCircleGeometry(input, extended, l1, l2);

  // Retraction releases the elbow soft cost continuously instead of imposing
  // a hard branch transition.
  input.handPosition = Eigen::Vector3d(l1 + l2 - 0.016, 0.0, 0.0);
  const auto retracting = guide.update(input);
  assert(retracting.elbowTrackingActivation > 0.0);
  assert(retracting.elbowTrackingActivation < 1.0);
  assert(retracting.commandDirection.z() < -0.99);
  assertCircleGeometry(input, retracting, l1, l2);

  // The 90/10 blend is unchanged while the elbow is outside the waist safety
  // band. Inside the band, safety locally overrides posture preference and
  // selects the maximum-outward point available on the same elbow circle.
  WheelNaturalElbowGuideConfig safetyConfig = config;
  safetyConfig.waistSoftClearance = 0.26;
  safetyConfig.waistFullActivationClearance = 0.20;
  WheelNaturalElbowGuide safetyGuide(l1, l2, safetyConfig);
  WheelNaturalElbowGuideInput waistInput;
  waistInput.shoulderPosition = Eigen::Vector3d::Zero();
  waistInput.handPosition = Eigen::Vector3d(0.40, 0.0, 0.0);
  waistInput.currentElbowPosition = Eigen::Vector3d(0.20, 0.0, -0.20);
  waistInput.torsoPosition = Eigen::Vector3d::Zero();
  waistInput.torsoOutwardDirection = Eigen::Vector3d::UnitY();
  waistInput.torsoFrameValid = true;
  const auto waistSafe = safetyGuide.update(waistInput);
  assertCircleGeometry(waistInput, waistSafe, l1, l2);
  assert(waistSafe.waistSafetyActivation > 0.99);
  assert(waistSafe.commandDirection.y() > 0.99);

  safetyGuide.reset();
  waistInput.torsoPosition = Eigen::Vector3d(0.0, -1.0, 0.0);
  const auto freeSpace = safetyGuide.update(waistInput);
  assertCircleGeometry(waistInput, freeSpace, l1, l2);
  assert(freeSpace.waistSafetyActivation == 0.0);
  assertNear(freeSpace.commandDirection, Eigen::Vector3d(0.0, 0.0, -1.0));

  // Waist safety must still take a large single-step correction even when
  // free-space rate-limiting is active (MR !4221 review).
  safetyGuide.reset();
  safetyConfig.maxCommandDirectionStepRad = 0.15;  // ~8.6 deg
  WheelNaturalElbowGuide limitedSafetyGuide(l1, l2, safetyConfig);
  waistInput.torsoPosition = Eigen::Vector3d(0.0, -1.0, 0.0);
  const auto primed = limitedSafetyGuide.update(waistInput);
  assert(primed.waistSafetyActivation == 0.0);
  waistInput.torsoPosition = Eigen::Vector3d::Zero();
  waistInput.currentElbowPosition = primed.elbowPosition;
  const auto rescued = limitedSafetyGuide.update(waistInput);
  assertCircleGeometry(waistInput, rescued, l1, l2);
  assert(rescued.waistSafetyActivation > 0.99);
  assert(rescued.commandDirection.y() > 0.99);
  const double rescueStep = std::acos(std::clamp(
      primed.commandDirection.dot(rescued.commandDirection), -1.0, 1.0));
  assert(rescueStep > safetyConfig.maxCommandDirectionStepRad + 0.5);

  // Vertical teleop regression (#4160): cross the gravity-projection
  // singularity (shoulder-hand axis through vertical). Without the soft blend
  // this snaps ~130-180 deg on the elbow circle; with it the step stays near
  // maxCommandDirectionStepRad.
  WheelNaturalElbowGuideConfig verticalConfig = config;
  verticalConfig.waistAvoidanceEnabled = false;
  WheelNaturalElbowGuide verticalGuide(l1, l2, verticalConfig);
  WheelNaturalElbowGuideInput verticalSweep;
  verticalSweep.shoulderPosition = Eigen::Vector3d(0.0, 0.20, 1.20);
  verticalSweep.currentElbowPosition =
      verticalSweep.shoulderPosition + Eigen::Vector3d(0.20, 0.08, -0.18);
  verticalSweep.humanPoseValid = false;

  Eigen::Vector3d prevDir = Eigen::Vector3d::Zero();
  Eigen::Vector3d prevElbow = Eigen::Vector3d::Zero();
  bool hasPrev = false;
  double maxAngleStep = 0.0;
  double maxElbowStep = 0.0;
  double elbowPath = 0.0;
  const int steps = 250;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double epsX = 0.25 - 0.50 * t;  // +0.25 -> -0.25 through vertical
    Eigen::Vector3d axis(epsX, 0.02, -1.0);
    axis.normalize();
    const double reach = 0.42;
    verticalSweep.handPosition = verticalSweep.shoulderPosition + reach * axis;
    if (hasPrev) {
      verticalSweep.currentElbowPosition = prevElbow;
    }
    const auto out = verticalGuide.update(verticalSweep);
    assertCircleGeometry(verticalSweep, out, l1, l2);
    if (hasPrev) {
      const double cosA =
          std::clamp(prevDir.dot(out.commandDirection), -1.0, 1.0);
      const double ang = std::acos(cosA);
      maxAngleStep = std::max(maxAngleStep, ang);
      const double elbowStep = (out.elbowPosition - prevElbow).norm();
      maxElbowStep = std::max(maxElbowStep, elbowStep);
      elbowPath += elbowStep;
    }
    prevDir = out.commandDirection;
    prevElbow = out.elbowPosition;
    hasPrev = true;
  }
  // Allow a little slack: plane normal also changes each sample.
  assert(maxAngleStep < verticalConfig.maxCommandDirectionStepRad + 0.05);
  // Legacy hard-threshold flip jumps ~0.30 m+ on the circle; fixed path stays low.
  assert(maxElbowStep < 0.12);
  std::cout << "vertical_sweep max_angle_step_deg="
            << (maxAngleStep * 180.0 / kPi)
            << " max_elbow_step_m=" << maxElbowStep
            << " elbow_path_m=" << elbowPath << std::endl;

  std::cout << "wheel_natural_elbow_guide_test: PASS" << std::endl;
  return 0;
}
