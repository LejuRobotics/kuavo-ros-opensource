#include "motion_capture_ik/WheelChestReferenceMapping.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {

constexpr double kTolerance = 1.0e-10;

bool expectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const char* label) {
  const double error = (actual - expected).norm();
  if (error < kTolerance) return true;
  std::cerr << label << " position error: " << error << " m" << std::endl;
  return false;
}

bool expectOrientationNear(const Eigen::Quaterniond& actual,
                           const Eigen::Quaterniond& expected,
                           const char* label) {
  const double dot = std::abs(actual.normalized().dot(expected.normalized()));
  const double error = 2.0 * std::acos(std::min(1.0, dot));
  if (error < kTolerance) return true;
  std::cerr << label << " orientation error: " << error << " rad" << std::endl;
  return false;
}

Eigen::Vector3d mirrorAcrossSagittalPlane(const Eigen::Vector3d& point) {
  return Eigen::Vector3d(point.x(), -point.y(), point.z());
}

}  // namespace

int main() {
  using HighlyDynamic::WheelChestReferencePose;
  using HighlyDynamic::WheelElbowActivationTransition;
  using HighlyDynamic::WheelElbowReferenceAnchor;
  using HighlyDynamic::WheelThreePointHandAnchor;
  using HighlyDynamic::anchoredElbowReference;
  using HighlyDynamic::buildThreePointHandTargets;
  using HighlyDynamic::captureElbowReferenceAnchor;
  using HighlyDynamic::captureThreePointHandAnchor;
  using HighlyDynamic::followChestReference;
  using HighlyDynamic::followPublishedPoseWithDesiredReferenceDelta;
  using HighlyDynamic::followPointWithChestReference;
  using HighlyDynamic::incrementalElbowGuideDelta;
  using HighlyDynamic::orientationInChestReference;
  using HighlyDynamic::pointInChestReference;
  using HighlyDynamic::vectorInChestReference;

  bool success = true;
  const Eigen::Vector3d handAtGrip(0.48, -0.31, 0.92);
  const Eigen::Quaterniond handQuatAtGrip(
      Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX()));

  // A persistent measured-vs-commanded chest error is intentionally absent
  // from this API.  Equal reference poses at grip entry must be an identity
  // transform, so repeated grip cycles cannot accumulate that error.
  const WheelChestReferencePose chestRefAtGrip{
      Eigen::Vector3d(0.02, 0.0, 0.79),
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.20, Eigen::Vector3d::UnitY()))};
  Eigen::Vector3d repeatedHand = handAtGrip;
  Eigen::Quaterniond repeatedQuat = handQuatAtGrip;
  for (int gripCycle = 0; gripCycle < 20; ++gripCycle) {
    Eigen::Vector3d command = repeatedHand;
    Eigen::Quaterniond commandQuat = repeatedQuat;
    followChestReference(chestRefAtGrip, chestRefAtGrip, command, commandQuat);
    repeatedHand = command;
    repeatedQuat = commandQuat;
  }
  success &= expectVectorNear(repeatedHand, handAtGrip, "repeated grip identity");
  success &= expectOrientationNear(repeatedQuat, handQuatAtGrip, "repeated grip identity");

  // The desired VR chest and the published command chest are normally not
  // equal.  A grip entry must nevertheless return the published pose exactly;
  // the persistent desired-vs-published tracking error is not motion.
  const WheelChestReferencePose publishedChestAtGrip{
      Eigen::Vector3d(-0.03, 0.01, 0.84),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitY()))};
  const WheelChestReferencePose desiredChestAtGrip{
      Eigen::Vector3d(0.06, -0.02, 0.91),
      (Eigen::Quaterniond(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ())) *
       Eigen::Quaterniond(Eigen::AngleAxisd(-0.18, Eigen::Vector3d::UnitY())))
          .normalized()};
  const WheelChestReferencePose effectiveChestAtEntry =
      followPublishedPoseWithDesiredReferenceDelta(
          desiredChestAtGrip, desiredChestAtGrip, publishedChestAtGrip);
  success &= expectVectorNear(effectiveChestAtEntry.position,
                              publishedChestAtGrip.position,
                              "published chest entry position");
  success &= expectOrientationNear(effectiveChestAtEntry.orientation,
                                   publishedChestAtGrip.orientation,
                                   "published chest entry orientation");

  // A real desired-reference change after entry is applied once to the
  // published pose.  Re-evaluating the same sample does not accumulate it.
  const Eigen::Quaterniond desiredDeltaRotation(
      Eigen::AngleAxisd(-0.12, Eigen::Vector3d::UnitY()));
  const WheelChestReferencePose currentDesiredChest{
      desiredChestAtGrip.position + Eigen::Vector3d(0.03, 0.0, -0.01),
      (desiredDeltaRotation * desiredChestAtGrip.orientation).normalized()};
  const WheelChestReferencePose effectiveMovedChest =
      followPublishedPoseWithDesiredReferenceDelta(
          desiredChestAtGrip, currentDesiredChest, publishedChestAtGrip);
  const Eigen::Vector3d expectedEffectivePosition =
      currentDesiredChest.position +
      desiredDeltaRotation * (publishedChestAtGrip.position - desiredChestAtGrip.position);
  const Eigen::Quaterniond expectedEffectiveOrientation =
      (desiredDeltaRotation * publishedChestAtGrip.orientation).normalized();
  success &= expectVectorNear(effectiveMovedChest.position,
                              expectedEffectivePosition,
                              "desired delta applied to published chest");
  success &= expectOrientationNear(effectiveMovedChest.orientation,
                                   expectedEffectiveOrientation,
                                   "desired delta applied to published orientation");
  const WheelChestReferencePose effectiveMovedChestAgain =
      followPublishedPoseWithDesiredReferenceDelta(
          desiredChestAtGrip, currentDesiredChest, publishedChestAtGrip);
  success &= expectVectorNear(effectiveMovedChestAgain.position,
                              effectiveMovedChest.position,
                              "desired delta no accumulation");
  success &= expectOrientationNear(effectiveMovedChestAgain.orientation,
                                   effectiveMovedChest.orientation,
                                   "desired delta orientation no accumulation");

  // A real chest-reference change is still propagated rigidly to the hand.
  const Eigen::Quaterniond chestRotation(
      Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
  const WheelChestReferencePose currentChestRef{
      chestRefAtGrip.position + Eigen::Vector3d(0.04, -0.02, 0.03),
      (chestRotation * chestRefAtGrip.orientation).normalized()};
  Eigen::Vector3d movedHand = handAtGrip;
  Eigen::Quaterniond movedQuat = handQuatAtGrip;
  followChestReference(chestRefAtGrip, currentChestRef, movedHand, movedQuat);
  const Eigen::Vector3d expectedHand =
      currentChestRef.position + chestRotation * (handAtGrip - chestRefAtGrip.position);
  const Eigen::Quaterniond expectedQuat = (chestRotation * handQuatAtGrip).normalized();
  success &= expectVectorNear(movedHand, expectedHand, "chest reference motion");
  success &= expectOrientationNear(movedQuat, expectedQuat, "chest reference motion");

  // A published command snapshot is transferred as one rigid chain.  Every
  // point is first expressed in the same published chest frame, then mapped
  // through the current command chest frame.  Pairwise link geometry and all
  // chest-relative orientations must therefore remain invariant.
  const std::array<Eigen::Vector3d, 4> publishedChain{{
      Eigen::Vector3d(0.04, 0.24, 0.86),
      Eigen::Vector3d(0.18, 0.39, 0.78),
      Eigen::Vector3d(0.39, 0.43, 0.72),
      Eigen::Vector3d(0.45, 0.44, 0.70),
  }};
  const std::array<Eigen::Quaterniond, 4> publishedOrientations{{
      Eigen::Quaterniond(Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitX())),
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.25, Eigen::Vector3d::UnitY())),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.40, Eigen::Vector3d::UnitZ())),
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitX())),
  }};
  std::array<Eigen::Vector3d, 4> localChain;
  std::array<Eigen::Quaterniond, 4> localOrientations;
  std::array<Eigen::Vector3d, 4> remappedChain;
  std::array<Eigen::Quaterniond, 4> remappedOrientations;
  for (std::size_t i = 0; i < publishedChain.size(); ++i) {
    localChain[i] = pointInChestReference(chestRefAtGrip, publishedChain[i]);
    localOrientations[i] =
        orientationInChestReference(chestRefAtGrip, publishedOrientations[i]);
    remappedChain[i] = currentChestRef.position + currentChestRef.orientation * localChain[i];
    remappedOrientations[i] =
        (currentChestRef.orientation * localOrientations[i]).normalized();

    success &= expectVectorNear(pointInChestReference(currentChestRef, remappedChain[i]),
                                localChain[i],
                                "rigid chain local position");
    success &= expectOrientationNear(
        orientationInChestReference(currentChestRef, remappedOrientations[i]),
        localOrientations[i],
        "rigid chain local orientation");
  }
  for (std::size_t i = 1; i < publishedChain.size(); ++i) {
    const double publishedLength = (publishedChain[i] - publishedChain[i - 1]).norm();
    const double remappedLength = (remappedChain[i] - remappedChain[i - 1]).norm();
    if (std::abs(remappedLength - publishedLength) >= kTolerance) {
      std::cerr << "rigid chain link length error at segment " << i << ": "
                << std::abs(remappedLength - publishedLength) << " m" << std::endl;
      success = false;
    }
  }

  // The three hand points at a grip edge must all describe the same q_pub.
  // Model the s63 distal chain with a non-zero joint 7. Rebuilding from the
  // captured EE-frame geometry must be an identity; the historical zero-joint
  // offsets instead request q7=0 and move the target by several centimetres.
  const Eigen::Vector3d publishedLink6(0.31, 0.27, 0.88);
  const Eigen::Quaterniond publishedLink6Orientation =
      (Eigen::Quaterniond(Eigen::AngleAxisd(0.22, Eigen::Vector3d::UnitX())) *
       Eigen::Quaterniond(Eigen::AngleAxisd(-0.18, Eigen::Vector3d::UnitZ())))
          .normalized();
  const Eigen::Quaterniond joint7Rotation(
      Eigen::AngleAxisd(28.5 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond publishedEeOrientation =
      (publishedLink6Orientation * joint7Rotation).normalized();
  const Eigen::Vector3d publishedJoint7Origin =
      publishedLink6 + publishedLink6Orientation * Eigen::Vector3d(0.0, 0.0, -0.021);
  const Eigen::Vector3d publishedEe =
      publishedJoint7Origin + publishedEeOrientation * Eigen::Vector3d(0.0, 0.0, -0.17);
  const Eigen::Vector3d publishedThumb =
      publishedEe + publishedEeOrientation * Eigen::Vector3d(0.15, 0.0, 0.0);

  WheelThreePointHandAnchor threePointAnchor;
  success &= captureThreePointHandAnchor(threePointAnchor,
                                          publishedLink6,
                                          publishedEe,
                                          publishedThumb,
                                          publishedEeOrientation);
  Eigen::Vector3d rebuiltEe;
  Eigen::Vector3d rebuiltThumb;
  success &= buildThreePointHandTargets(threePointAnchor,
                                         publishedLink6,
                                         publishedEeOrientation,
                                         rebuiltEe,
                                         rebuiltThumb);
  success &= expectVectorNear(rebuiltEe, publishedEe, "nonzero-q7 EE grip identity");
  success &= expectVectorNear(rebuiltThumb, publishedThumb,
                              "nonzero-q7 thumb grip identity");

  const Eigen::Vector3d historicalZeroJointEeTarget =
      publishedLink6 +
      publishedLink6Orientation * Eigen::Vector3d(0.0, 0.0, -0.191);
  if ((historicalZeroJointEeTarget - publishedEe).norm() < 0.05) {
    std::cerr << "nonzero-q7 fixture does not expose the historical target jump"
              << std::endl;
    success = false;
  }

  const Eigen::Quaterniond operatorRotation(
      Eigen::AngleAxisd(0.16, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d movedLink6 =
      publishedLink6 + Eigen::Vector3d(0.025, -0.012, 0.018);
  const Eigen::Quaterniond movedEeOrientation =
      (operatorRotation * publishedEeOrientation).normalized();
  Eigen::Vector3d movedEe;
  Eigen::Vector3d movedThumb;
  success &= buildThreePointHandTargets(threePointAnchor,
                                         movedLink6,
                                         movedEeOrientation,
                                         movedEe,
                                         movedThumb);
  if (std::abs((movedEe - movedLink6).norm() -
               (publishedEe - publishedLink6).norm()) >= kTolerance ||
      std::abs((movedThumb - movedEe).norm() -
               (publishedThumb - publishedEe).norm()) >= kTolerance) {
    std::cerr << "three-point hand geometry changed during commanded motion"
              << std::endl;
    success = false;
  }

  // A large measured-chest mismatch is deliberately irrelevant to a command
  // frame transfer.  Reconstructing at the original command chest is an exact
  // identity even when a hypothetical measured chest is far away.
  const WheelChestReferencePose farMeasuredChest{
      chestRefAtGrip.position + Eigen::Vector3d(0.12, -0.09, 0.04),
      (Eigen::Quaterniond(Eigen::AngleAxisd(0.45, Eigen::Vector3d::UnitZ())) *
       chestRefAtGrip.orientation)
          .normalized()};
  (void)farMeasuredChest;
  for (std::size_t i = 0; i < publishedChain.size(); ++i) {
    const Eigen::Vector3d reconstructed =
        chestRefAtGrip.position + chestRefAtGrip.orientation * localChain[i];
    success &= expectVectorNear(reconstructed, publishedChain[i],
                                "command chest edge identity");
  }

  // The absolute natural-elbow preference and measured FK may both be far from
  // the effective command.  Bumpless entry must preserve the command that was
  // actually consumed by downstream IK, not reset it to measured FK.
  WheelElbowReferenceAnchor leftElbowAnchor;
  WheelElbowReferenceAnchor rightElbowAnchor;
  const Eigen::Vector3d leftPreviousElbowCommand(0.12, 0.34, 0.96);
  const Eigen::Vector3d leftGuideElbow(-0.03, 0.18, 1.05);
  const Eigen::Vector3d leftMeasuredElbow(-0.08, 0.27, 0.91);
  const Eigen::Vector3d rightPreviousElbowCommand =
      mirrorAcrossSagittalPlane(leftPreviousElbowCommand);
  const Eigen::Vector3d rightGuideElbow = mirrorAcrossSagittalPlane(leftGuideElbow);

  success &= captureElbowReferenceAnchor(
      leftElbowAnchor, chestRefAtGrip, leftPreviousElbowCommand, leftGuideElbow);
  success &= captureElbowReferenceAnchor(
      rightElbowAnchor, chestRefAtGrip, rightPreviousElbowCommand, rightGuideElbow);
  success &= expectVectorNear(anchoredElbowReference(
                                  leftElbowAnchor, chestRefAtGrip, leftGuideElbow),
                              leftPreviousElbowCommand,
                              "left elbow at grip entry");
  success &= expectVectorNear(anchoredElbowReference(
                                  rightElbowAnchor, chestRefAtGrip, rightGuideElbow),
                              rightPreviousElbowCommand,
                              "right elbow at grip entry");
  if ((anchoredElbowReference(leftElbowAnchor, chestRefAtGrip, leftGuideElbow) -
       leftMeasuredElbow)
          .norm() < 0.1) {
    std::cerr << "left elbow entry incorrectly reset to measured FK" << std::endl;
    success = false;
  }

  // A rigid chest-reference motion moves the robot elbow anchor rigidly.  It
  // must not be interpreted as a new natural-guide delta.
  const Eigen::Vector3d followedLeftGuide =
      followPointWithChestReference(chestRefAtGrip, currentChestRef, leftGuideElbow);
  const Eigen::Vector3d followedRightGuide =
      followPointWithChestReference(chestRefAtGrip, currentChestRef, rightGuideElbow);
  const Eigen::Vector3d followedLeftRobot =
      followPointWithChestReference(chestRefAtGrip, currentChestRef, leftPreviousElbowCommand);
  const Eigen::Vector3d followedRightRobot =
      followPointWithChestReference(chestRefAtGrip, currentChestRef, rightPreviousElbowCommand);
  success &= expectVectorNear(anchoredElbowReference(
                                  leftElbowAnchor, currentChestRef, followedLeftGuide),
                              followedLeftRobot,
                              "left elbow rigid chest following");
  success &= expectVectorNear(anchoredElbowReference(
                                  rightElbowAnchor, currentChestRef, followedRightGuide),
                              followedRightRobot,
                              "right elbow rigid chest following");

  // A guide increment fixed relative to the chest stays numerically constant
  // in the filter frame while the chest rotates.  This keeps rigid torso
  // motion out of the guide-delta kinematic limiter.
  const Eigen::Vector3d guideIncrementInChest(0.015, 0.025, -0.010);
  const Eigen::Vector3d movedLeftGuide =
      followedLeftGuide + currentChestRef.orientation * guideIncrementInChest;
  const Eigen::Vector3d movedGuideDeltaInChest = vectorInChestReference(
      currentChestRef,
      incrementalElbowGuideDelta(leftElbowAnchor, currentChestRef, movedLeftGuide));
  success &= expectVectorNear(movedGuideDeltaInChest,
                              guideIncrementInChest,
                              "chest-local guide delta invariance");

  // Only guide changes after grip entry are added.  Mirrored guide increments
  // must produce mirrored arm behavior when the chest reference is identical.
  const Eigen::Vector3d leftGuideIncrement(0.015, 0.025, -0.010);
  const Eigen::Vector3d rightGuideIncrement =
      mirrorAcrossSagittalPlane(leftGuideIncrement);
  const Eigen::Vector3d leftElbowCommand = anchoredElbowReference(
      leftElbowAnchor, chestRefAtGrip, leftGuideElbow + leftGuideIncrement);
  const Eigen::Vector3d rightElbowCommand = anchoredElbowReference(
      rightElbowAnchor, chestRefAtGrip, rightGuideElbow + rightGuideIncrement);
  success &= expectVectorNear(leftElbowCommand,
                              leftPreviousElbowCommand + leftGuideIncrement,
                              "left elbow guide increment");
  success &= expectVectorNear(rightElbowCommand,
                              rightPreviousElbowCommand + rightGuideIncrement,
                              "right elbow guide increment");
  success &= expectVectorNear(rightElbowCommand,
                              mirrorAcrossSagittalPlane(leftElbowCommand),
                              "mirrored elbow commands");

  // Resetting one side for a new grip must not alter the other side's anchor.
  leftElbowAnchor.reset();
  success &= !leftElbowAnchor.valid;
  success &= rightElbowAnchor.valid;
  success &= expectVectorNear(anchoredElbowReference(
                                  rightElbowAnchor, chestRefAtGrip, rightGuideElbow),
                              rightPreviousElbowCommand,
                              "right elbow isolated from left reset");

  // Elbow tracking weight must also be bumpless in both directions.  The first
  // sample after a press/release is exactly the previous value, then a
  // zero-slope smoothstep transition proceeds independently per arm.
  WheelElbowActivationTransition leftActivation;
  WheelElbowActivationTransition rightActivation;
  leftActivation.reset(false, 1.0);
  rightActivation.reset(false, 1.0);
  constexpr double activationDt = 0.01;
  constexpr double activationDuration = 0.4;
  const double leftFirstActive =
      leftActivation.update(true, 0.1, activationDt, activationDuration);
  success &= std::abs(leftFirstActive - 1.0) < kTolerance;
  success &= std::abs(rightActivation.update(false, 1.0, activationDt, activationDuration) - 1.0) <
             kTolerance;
  double previousActivation = leftFirstActive;
  for (int i = 0; i < 40; ++i) {
    const double currentActivation =
        leftActivation.update(true, 0.1, activationDt, activationDuration);
    success &= currentActivation <= previousActivation + kTolerance;
    success &= currentActivation >= 0.1 - kTolerance;
    previousActivation = currentActivation;
  }
  success &= std::abs(previousActivation - 0.1) < kTolerance;
  const double leftFirstInactive =
      leftActivation.update(false, 1.0, activationDt, activationDuration);
  success &= std::abs(leftFirstInactive - 0.1) < kTolerance;
  success &= std::abs(rightActivation.currentValue - 1.0) < kTolerance;

  // Rapid short presses reverse the transition from its current value.  Each
  // edge sample is exactly continuous and the untouched arm stays unchanged.
  WheelElbowActivationTransition shortPressActivation;
  WheelElbowActivationTransition untouchedActivation;
  shortPressActivation.reset(false, 1.0);
  untouchedActivation.reset(false, 1.0);
  success &= std::abs(shortPressActivation.update(
                          true, 0.1, activationDt, activationDuration) -
                      1.0) < kTolerance;
  for (int i = 0; i < 10; ++i) {
    shortPressActivation.update(true, 0.1, activationDt, activationDuration);
  }
  const double valueBeforeShortRelease = shortPressActivation.currentValue;
  const double valueAtShortRelease = shortPressActivation.update(
      false, 1.0, activationDt, activationDuration);
  success &= std::abs(valueAtShortRelease - valueBeforeShortRelease) < kTolerance;
  for (int i = 0; i < 5; ++i) {
    shortPressActivation.update(false, 1.0, activationDt, activationDuration);
  }
  const double valueBeforeRepress = shortPressActivation.currentValue;
  const double valueAtRepress = shortPressActivation.update(
      true, 0.1, activationDt, activationDuration);
  success &= std::abs(valueAtRepress - valueBeforeRepress) < kTolerance;
  success &= valueAtRepress >= 0.1 - kTolerance && valueAtRepress <= 1.0 + kTolerance;
  success &= std::abs(untouchedActivation.currentValue - 1.0) < kTolerance;

  if (!success) return 1;
  std::cout << "wheel_chest_reference_mapping_test: PASS" << std::endl;
  return 0;
}
