#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace HighlyDynamic {

struct WheelNaturalElbowGuideConfig {
  // 0.0: follow the mapped human bending plane; 1.0: gravity only.
  double naturalDirectionBlend = 0.90;

  // Gravity is ambiguous when it is nearly parallel to the shoulder-hand axis.
  double downProjectionMinNorm = 0.05;

  // Keep circle construction away from the exact reach boundary.
  double reachMargin = 0.002;

  // Fade only the elbow tracking cost as remaining reach slack approaches
  // zero. The hand target is never changed by this activation.
  double extensionFadeStartDistance = 0.002;
  double extensionFadeFullDistance = 0.030;

  // The 90/10 gravity-human blend is a posture preference, not a collision
  // model.  When the measured or preferred elbow enters this lateral band,
  // smoothly bias the circle direction away from the waist.  This safety bias
  // is inactive outside waistSoftClearance, so normal free-space motion keeps
  // the exact 90/10 blend.
  bool waistAvoidanceEnabled = true;
  double waistSoftClearance = 0.260;
  double waistFullActivationClearance = 0.200;

  double numericalEpsilon = 1.0e-8;
};

struct WheelNaturalElbowGuideInput {
  Eigen::Vector3d shoulderPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d handPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d currentElbowPosition = Eigen::Vector3d::Zero();

  Eigen::Vector3d humanShoulderPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanElbowPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanHandPosition = Eigen::Vector3d::Zero();
  bool humanPoseValid = false;

  // Torso origin and the unit direction pointing away from the torso for this
  // arm (+body-Y for left, -body-Y for right), both expressed in the same
  // frame as the robot points above.
  Eigen::Vector3d torsoPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d torsoOutwardDirection = Eigen::Vector3d::Zero();
  bool torsoFrameValid = false;
};

struct WheelNaturalElbowGuideOutput {
  Eigen::Vector3d elbowPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d circleCenter = Eigen::Vector3d::Zero();
  Eigen::Vector3d circleHandPosition = Eigen::Vector3d::Zero();
  Eigen::Vector3d commandDirection = Eigen::Vector3d::Zero();
  Eigen::Vector3d naturalDirection = Eigen::Vector3d::Zero();
  Eigen::Vector3d humanDirection = Eigen::Vector3d::Zero();

  double requestedShoulderHandDistance = 0.0;
  double circleShoulderHandDistance = 0.0;
  double circleRadius = 0.0;
  double humanActivation = 0.0;
  double elbowTrackingActivation = 0.0;
  double waistSafetyActivation = 0.0;
  double waistSignedClearance = 0.0;

  bool humanDirectionValid = false;
  bool gravityDirectionValid = false;
  bool handTargetReachable = false;
};

/**
 * Computes a soft elbow target on the shoulder-hand elbow circle.
 *
 * The hand target is an input and is never modified. The human elbow contributes
 * only a bending-plane direction; robot link lengths always determine the elbow
 * point. The 90/10 preference has no left/right sign rule.  An optional torso
 * outward direction is used only as a localized safety override near the
 * waist; it does not change the free-space blend.
 */
class WheelNaturalElbowGuide final {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WheelNaturalElbowGuide(double upperArmLength,
                         double lowerArmLength,
                         const WheelNaturalElbowGuideConfig& config = {})
      : upperArmLength_(upperArmLength), lowerArmLength_(lowerArmLength), config_(config) {}

  void reset() {
    hasLastRobotAxis_ = false;
    hasLastCommandDirection_ = false;
    lastRobotAxis_.setZero();
    lastCommandDirection_.setZero();
  }

  const WheelNaturalElbowGuideConfig& config() const { return config_; }

  WheelNaturalElbowGuideOutput update(const WheelNaturalElbowGuideInput& input) {
    WheelNaturalElbowGuideOutput output;
    const double eps = std::max(config_.numericalEpsilon, 1.0e-12);

    if (!(upperArmLength_ > eps) || !(lowerArmLength_ > eps) ||
        !isFiniteVector(input.shoulderPosition) || !isFiniteVector(input.handPosition)) {
      output.elbowPosition = isFiniteVector(input.currentElbowPosition)
                                  ? input.currentElbowPosition
                                  : input.shoulderPosition;
      return output;
    }

    Eigen::Vector3d robotAxisVector = input.handPosition - input.shoulderPosition;
    output.requestedShoulderHandDistance = robotAxisVector.norm();
    Eigen::Vector3d robotAxis = normalizedOrZero(robotAxisVector, eps);
    if (robotAxis.norm() <= eps) {
      robotAxis = hasLastRobotAxis_ ? lastRobotAxis_ : Eigen::Vector3d::UnitX();
    }
    lastRobotAxis_ = robotAxis;
    hasLastRobotAxis_ = true;

    const double minReachRaw = std::abs(upperArmLength_ - lowerArmLength_);
    const double maxReachRaw = upperArmLength_ + lowerArmLength_;
    const double margin = std::max(0.0, config_.reachMargin);
    const double minReach = std::max(
        eps, std::min(minReachRaw + margin, maxReachRaw));
    const double maxReach = std::max(minReach, maxReachRaw - margin);
    const double circleDistance = std::clamp(output.requestedShoulderHandDistance,
                                             minReach,
                                             maxReach);
    output.circleShoulderHandDistance = circleDistance;
    output.handTargetReachable =
        output.requestedShoulderHandDistance >= minReachRaw - margin &&
        output.requestedShoulderHandDistance <= maxReachRaw + margin;
    output.circleHandPosition = input.shoulderPosition + circleDistance * robotAxis;

    const double centerDistance =
        (upperArmLength_ * upperArmLength_ - lowerArmLength_ * lowerArmLength_ +
         circleDistance * circleDistance) /
        (2.0 * circleDistance);
    const double radiusSquared = std::max(
        0.0, upperArmLength_ * upperArmLength_ - centerDistance * centerDistance);
    output.circleRadius = std::sqrt(radiusSquared);
    output.circleCenter = input.shoulderPosition + centerDistance * robotAxis;

    Eigen::Vector3d continuityDirection = radialDirection(
        input.currentElbowPosition, input.shoulderPosition, robotAxis, eps);
    if (continuityDirection.norm() <= eps && hasLastCommandDirection_) {
      continuityDirection = projectAndNormalize(lastCommandDirection_, robotAxis, eps);
    }

    const Eigen::Vector3d gravity(0.0, 0.0, -1.0);
    const Eigen::Vector3d gravityProjection =
        gravity - robotAxis * gravity.dot(robotAxis);
    output.gravityDirectionValid =
        gravityProjection.norm() >= std::max(config_.downProjectionMinNorm, eps);
    output.naturalDirection = output.gravityDirectionValid
                                  ? normalizedOrZero(gravityProjection, eps)
                                  : continuityDirection;
    if (output.naturalDirection.norm() <= eps) {
      output.naturalDirection = deterministicPlaneDirection(robotAxis, eps);
    }

    output.humanDirectionValid = computeMappedHumanDirection(
        input, robotAxis, output.humanDirection, eps);

    const double naturalBlend = std::clamp(config_.naturalDirectionBlend, 0.0, 1.0);
    if (output.humanDirectionValid && naturalBlend < 1.0 - eps) {
      output.commandDirection = projectAndNormalize(
          naturalBlend * output.naturalDirection +
              (1.0 - naturalBlend) * output.humanDirection,
          robotAxis,
          eps);
      output.humanActivation = 1.0 - naturalBlend;
    } else {
      output.commandDirection = output.naturalDirection;
    }
    if (output.commandDirection.norm() <= eps) {
      output.commandDirection = continuityDirection.norm() > eps
                                    ? continuityDirection
                                    : deterministicPlaneDirection(robotAxis, eps);
    }

    applyWaistSafetyBias(input, robotAxis, output, eps);

    output.elbowPosition =
        output.circleCenter + output.circleRadius * output.commandDirection;
    output.elbowTrackingActivation = extensionActivation(
        output.requestedShoulderHandDistance, maxReachRaw);

    lastCommandDirection_ = output.commandDirection;
    hasLastCommandDirection_ = true;
    return output;
  }

  static Eigen::Vector3d projectElbowToCircle(
      const Eigen::Vector3d& shoulderPosition,
      const Eigen::Vector3d& handPosition,
      const Eigen::Vector3d& preferredElbowPosition,
      double upperArmLength,
      double lowerArmLength,
      double reachMargin = 0.002,
      double numericalEpsilon = 1.0e-8) {
    WheelNaturalElbowGuideConfig config;
    config.naturalDirectionBlend = 1.0;
    config.reachMargin = reachMargin;
    config.numericalEpsilon = numericalEpsilon;
    WheelNaturalElbowGuide guide(upperArmLength, lowerArmLength, config);
    WheelNaturalElbowGuideInput input;
    input.shoulderPosition = shoulderPosition;
    input.handPosition = handPosition;
    input.currentElbowPosition = preferredElbowPosition;
    const auto output = guide.update(input);

    const Eigen::Vector3d axis = normalizedOrZero(
        output.circleHandPosition - shoulderPosition,
        std::max(numericalEpsilon, 1.0e-12));
    const Eigen::Vector3d preferredDirection = radialDirection(
        preferredElbowPosition, shoulderPosition, axis,
        std::max(numericalEpsilon, 1.0e-12));
    if (preferredDirection.norm() <= std::max(numericalEpsilon, 1.0e-12)) {
      return output.elbowPosition;
    }
    return output.circleCenter + output.circleRadius * preferredDirection;
  }

 private:
  static Eigen::Vector3d normalizedOrZero(const Eigen::Vector3d& value, double eps) {
    if (!isFiniteVector(value)) return Eigen::Vector3d::Zero();
    const double norm = value.norm();
    if (norm <= eps) return Eigen::Vector3d::Zero();
    return value / norm;
  }

  static Eigen::Vector3d projectAndNormalize(const Eigen::Vector3d& direction,
                                             const Eigen::Vector3d& planeNormal,
                                             double eps) {
    return normalizedOrZero(
        direction - planeNormal * direction.dot(planeNormal), eps);
  }

  static Eigen::Vector3d radialDirection(const Eigen::Vector3d& point,
                                         const Eigen::Vector3d& axisOrigin,
                                         const Eigen::Vector3d& axisDirection,
                                         double eps) {
    if (!isFiniteVector(point)) return Eigen::Vector3d::Zero();
    const Eigen::Vector3d relative = point - axisOrigin;
    return projectAndNormalize(relative, axisDirection, eps);
  }

  static Eigen::Vector3d deterministicPlaneDirection(const Eigen::Vector3d& axis,
                                                       double eps) {
    const Eigen::Vector3d candidates[3] = {
        Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
    int best = 0;
    double smallestAlignment = std::abs(axis.dot(candidates[0]));
    for (int i = 1; i < 3; ++i) {
      const double alignment = std::abs(axis.dot(candidates[i]));
      if (alignment < smallestAlignment) {
        best = i;
        smallestAlignment = alignment;
      }
    }
    return projectAndNormalize(candidates[best], axis, eps);
  }

  static bool computeMappedHumanDirection(
      const WheelNaturalElbowGuideInput& input,
      const Eigen::Vector3d& robotAxis,
      Eigen::Vector3d& mappedDirection,
      double eps) {
    mappedDirection.setZero();
    if (!input.humanPoseValid || !isFiniteVector(input.humanShoulderPosition) ||
        !isFiniteVector(input.humanElbowPosition) || !isFiniteVector(input.humanHandPosition)) {
      return false;
    }

    const Eigen::Vector3d humanAxis = normalizedOrZero(
        input.humanHandPosition - input.humanShoulderPosition, eps);
    if (humanAxis.norm() <= eps) return false;

    const Eigen::Vector3d humanRadial = radialDirection(
        input.humanElbowPosition, input.humanShoulderPosition, humanAxis, eps);
    if (humanRadial.norm() <= eps) return false;

    const Eigen::Quaterniond transport =
        Eigen::Quaterniond::FromTwoVectors(humanAxis, robotAxis);
    mappedDirection = projectAndNormalize(transport * humanRadial, robotAxis, eps);
    return mappedDirection.norm() > eps;
  }

  static bool isFiniteVector(const Eigen::Vector3d& value) {
    return std::isfinite(value.x()) && std::isfinite(value.y()) &&
           std::isfinite(value.z());
  }

  void applyWaistSafetyBias(const WheelNaturalElbowGuideInput& input,
                            const Eigen::Vector3d& robotAxis,
                            WheelNaturalElbowGuideOutput& output,
                            double eps) const {
    if (!config_.waistAvoidanceEnabled || !input.torsoFrameValid ||
        !isFiniteVector(input.torsoPosition) ||
        !isFiniteVector(input.torsoOutwardDirection)) {
      return;
    }

    const Eigen::Vector3d outward =
        normalizedOrZero(input.torsoOutwardDirection, eps);
    const Eigen::Vector3d outwardOnCircle =
        projectAndNormalize(outward, robotAxis, eps);
    if (outward.norm() <= eps || outwardOnCircle.norm() <= eps) return;

    const Eigen::Vector3d preferredElbow =
        output.circleCenter + output.circleRadius * output.commandDirection;
    double signedClearance =
        (preferredElbow - input.torsoPosition).dot(outward);
    if (isFiniteVector(input.currentElbowPosition)) {
      signedClearance = std::min(
          signedClearance,
          (input.currentElbowPosition - input.torsoPosition).dot(outward));
    }
    output.waistSignedClearance = signedClearance;

    const double soft = std::max(0.0, config_.waistSoftClearance);
    const double full = std::clamp(
        config_.waistFullActivationClearance, 0.0, soft);
    if (signedClearance >= soft || soft - full <= eps) return;

    const double normalizedPenetration =
        std::clamp((soft - signedClearance) / (soft - full), 0.0, 1.0);
    output.waistSafetyActivation =
        normalizedPenetration * normalizedPenetration *
        (3.0 - 2.0 * normalizedPenetration);

    Eigen::Vector3d safeDirection = projectAndNormalize(
        (1.0 - output.waistSafetyActivation) * output.commandDirection +
            output.waistSafetyActivation * outwardOnCircle,
        robotAxis,
        eps);
    if (safeDirection.norm() <= eps) safeDirection = outwardOnCircle;
    output.commandDirection = safeDirection;
  }

  double extensionActivation(double requestedDistance, double maximumReach) const {
    const double reachSlack = std::max(0.0, maximumReach - requestedDistance);
    const double start = std::max(0.0, config_.extensionFadeStartDistance);
    const double full = std::max(start + config_.numericalEpsilon,
                                 config_.extensionFadeFullDistance);
    const double t = std::clamp((reachSlack - start) / (full - start), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  double upperArmLength_ = 0.0;
  double lowerArmLength_ = 0.0;
  WheelNaturalElbowGuideConfig config_;

  bool hasLastRobotAxis_ = false;
  bool hasLastCommandDirection_ = false;
  Eigen::Vector3d lastRobotAxis_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastCommandDirection_ = Eigen::Vector3d::Zero();
};

}  // namespace HighlyDynamic
