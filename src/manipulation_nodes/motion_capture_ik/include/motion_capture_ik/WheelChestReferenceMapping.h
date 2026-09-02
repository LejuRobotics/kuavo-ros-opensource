#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>

namespace HighlyDynamic {

struct WheelChestReferencePose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct WheelElbowReferenceAnchor {
  bool valid = false;
  Eigen::Vector3d elbowCommandPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d guideElbowPosition = Eigen::Vector3d::Zero();
  WheelChestReferencePose chestReference;

  void reset() {
    valid = false;
    elbowCommandPosition.setZero();
    guideElbowPosition.setZero();
    chestReference = WheelChestReferencePose{};
  }
};

// Grip-local geometry used by the existing three-point hand IK. The
// orientation argument is always the physical end-effector orientation. By
// expressing both post-Link6 points in that frame, the first target rebuilt
// after a grip edge is exactly the FK(q_pub) geometry even when joint 7 is not
// zero.
struct WheelThreePointHandAnchor {
  bool valid = false;
  Eigen::Vector3d endEffectorOffsetInEndEffector = Eigen::Vector3d::Zero();
  Eigen::Vector3d virtualThumbOffsetInEndEffector = Eigen::Vector3d::Zero();

  void reset() {
    valid = false;
    endEffectorOffsetInEndEffector.setZero();
    virtualThumbOffsetInEndEffector.setZero();
  }
};

inline bool captureThreePointHandAnchor(WheelThreePointHandAnchor& anchor,
                                        const Eigen::Vector3d& link6Position,
                                        const Eigen::Vector3d& endEffectorPosition,
                                        const Eigen::Vector3d& virtualThumbPosition,
                                        const Eigen::Quaterniond& endEffectorOrientation) {
  anchor.reset();
  if (!link6Position.allFinite() || !endEffectorPosition.allFinite() ||
      !virtualThumbPosition.allFinite() ||
      !endEffectorOrientation.coeffs().allFinite() ||
      endEffectorOrientation.norm() <= 1.0e-9) {
    return false;
  }

  const Eigen::Quaterniond orientation = endEffectorOrientation.normalized();
  anchor.endEffectorOffsetInEndEffector =
      orientation.conjugate() * (endEffectorPosition - link6Position);
  anchor.virtualThumbOffsetInEndEffector =
      orientation.conjugate() * (virtualThumbPosition - link6Position);
  anchor.valid = anchor.endEffectorOffsetInEndEffector.allFinite() &&
                 anchor.virtualThumbOffsetInEndEffector.allFinite() &&
                 (anchor.virtualThumbOffsetInEndEffector -
                  anchor.endEffectorOffsetInEndEffector)
                         .norm() > 1.0e-9;
  return anchor.valid;
}

inline bool buildThreePointHandTargets(const WheelThreePointHandAnchor& anchor,
                                       const Eigen::Vector3d& link6Target,
                                       const Eigen::Quaterniond& endEffectorTargetOrientation,
                                       Eigen::Vector3d& endEffectorTarget,
                                       Eigen::Vector3d& virtualThumbTarget) {
  if (!anchor.valid || !link6Target.allFinite() ||
      !endEffectorTargetOrientation.coeffs().allFinite() ||
      endEffectorTargetOrientation.norm() <= 1.0e-9) {
    return false;
  }

  const Eigen::Quaterniond orientation = endEffectorTargetOrientation.normalized();
  endEffectorTarget =
      link6Target + orientation * anchor.endEffectorOffsetInEndEffector;
  virtualThumbTarget =
      link6Target + orientation * anchor.virtualThumbOffsetInEndEffector;
  return endEffectorTarget.allFinite() && virtualThumbTarget.allFinite();
}

struct WheelElbowActivationTransition {
  bool initialized = false;
  bool lastReferenceActive = false;
  bool transitioning = false;
  double startValue = 1.0;
  double currentValue = 1.0;
  double elapsed = 0.0;

  void reset(bool referenceActive = false, double activation = 1.0) {
    initialized = true;
    lastReferenceActive = referenceActive;
    transitioning = false;
    startValue = std::clamp(activation, 0.0, 1.0);
    currentValue = startValue;
    elapsed = 0.0;
  }

  double update(bool referenceActive,
                double targetActivation,
                double dt,
                double transitionDuration) {
    const double target = std::clamp(targetActivation, 0.0, 1.0);
    if (!initialized) {
      reset(referenceActive, target);
      return currentValue;
    }

    if (referenceActive != lastReferenceActive) {
      lastReferenceActive = referenceActive;
      transitioning = true;
      startValue = currentValue;
      elapsed = 0.0;
    }

    if (!transitioning || transitionDuration <= 1.0e-9) {
      currentValue = target;
      transitioning = false;
      return currentValue;
    }

    const double u = std::clamp(elapsed / transitionDuration, 0.0, 1.0);
    const double smooth = u * u * (3.0 - 2.0 * u);
    currentValue = startValue + smooth * (target - startValue);
    if (u >= 1.0) {
      transitioning = false;
    } else {
      elapsed = std::min(transitionDuration, elapsed + std::max(0.0, dt));
    }
    return currentValue;
  }
};

inline Eigen::Vector3d followPointWithChestReference(const WheelChestReferencePose& anchorChestRef,
                                                     const WheelChestReferencePose& currentChestRef,
                                                     const Eigen::Vector3d& pointAtAnchor) {
  const Eigen::Quaterniond chestRefDelta =
      (currentChestRef.orientation.normalized() * anchorChestRef.orientation.normalized().conjugate()).normalized();
  return currentChestRef.position + chestRefDelta * (pointAtAnchor - anchorChestRef.position);
}

/**
 * Applies only the commanded chest-reference motion since grip entry.
 *
 * Both chest poses must come from the same reference stream.  In particular,
 * the anchor must not be replaced with measured chest FK: doing so turns the
 * chest tracking error into an unintended hand displacement on every grip.
 */
inline void followChestReference(const WheelChestReferencePose& anchorChestRef,
                                 const WheelChestReferencePose& currentChestRef,
                                 Eigen::Vector3d& handPosition,
                                 Eigen::Quaterniond& handOrientation) {
  const Eigen::Quaterniond chestRefDelta =
      (currentChestRef.orientation.normalized() * anchorChestRef.orientation.normalized().conjugate()).normalized();
  handPosition = followPointWithChestReference(anchorChestRef, currentChestRef, handPosition);
  handOrientation = (chestRefDelta * handOrientation.normalized()).normalized();
}

/**
 * Applies a desired-reference delta to a pose from the published-command
 * domain.
 *
 * The desired reference at grip entry and the published chest at grip entry
 * are intentionally allowed to differ.  At the entry sample the desired
 * delta is identity, so the returned pose is exactly publishedPoseAtGrip.
 * Only desired-reference motion observed after the entry sample is applied.
 */
inline WheelChestReferencePose followPublishedPoseWithDesiredReferenceDelta(
    const WheelChestReferencePose& desiredRefAtGrip,
    const WheelChestReferencePose& currentDesiredRef,
    const WheelChestReferencePose& publishedPoseAtGrip) {
  WheelChestReferencePose result = publishedPoseAtGrip;
  followChestReference(
      desiredRefAtGrip, currentDesiredRef, result.position, result.orientation);
  return result;
}

inline bool captureElbowReferenceAnchor(WheelElbowReferenceAnchor& anchor,
                                        const WheelChestReferencePose& currentChestRef,
                                        const Eigen::Vector3d& currentElbowCommand,
                                        const Eigen::Vector3d& rawGuideElbow) {
  if (anchor.valid) return false;
  anchor.elbowCommandPosition = currentElbowCommand;
  anchor.guideElbowPosition = rawGuideElbow;
  anchor.chestReference = currentChestRef;
  anchor.valid = true;
  return true;
}

inline Eigen::Vector3d anchoredElbowCommandBaseline(const WheelElbowReferenceAnchor& anchor,
                                                    const WheelChestReferencePose& currentChestRef) {
  return followPointWithChestReference(anchor.chestReference, currentChestRef, anchor.elbowCommandPosition);
}

inline Eigen::Vector3d incrementalElbowGuideDelta(const WheelElbowReferenceAnchor& anchor,
                                                  const WheelChestReferencePose& currentChestRef,
                                                  const Eigen::Vector3d& rawGuideElbow) {
  const Eigen::Vector3d guideBaseline =
      followPointWithChestReference(anchor.chestReference, currentChestRef, anchor.guideElbowPosition);
  return rawGuideElbow - guideBaseline;
}

inline Eigen::Vector3d vectorInChestReference(const WheelChestReferencePose& chestRef,
                                              const Eigen::Vector3d& worldVector) {
  return chestRef.orientation.normalized().conjugate() * worldVector;
}

inline Eigen::Vector3d vectorFromChestReference(const WheelChestReferencePose& chestRef,
                                                const Eigen::Vector3d& chestVector) {
  return chestRef.orientation.normalized() * chestVector;
}

inline Eigen::Vector3d anchoredElbowReference(const WheelElbowReferenceAnchor& anchor,
                                              const WheelChestReferencePose& currentChestRef,
                                              const Eigen::Vector3d& rawGuideElbow) {
  return anchoredElbowCommandBaseline(anchor, currentChestRef) +
         incrementalElbowGuideDelta(anchor, currentChestRef, rawGuideElbow);
}

inline Eigen::Vector3d pointInChestReference(const WheelChestReferencePose& chestRef,
                                             const Eigen::Vector3d& worldPoint) {
  return chestRef.orientation.normalized().conjugate() * (worldPoint - chestRef.position);
}

inline Eigen::Quaterniond orientationInChestReference(const WheelChestReferencePose& chestRef,
                                                       const Eigen::Quaterniond& worldOrientation) {
  return (chestRef.orientation.normalized().conjugate() * worldOrientation.normalized()).normalized();
}

}  // namespace HighlyDynamic
