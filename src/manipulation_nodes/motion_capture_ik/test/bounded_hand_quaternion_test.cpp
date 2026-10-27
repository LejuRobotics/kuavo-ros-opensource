#include "motion_capture_ik/BoundedHandQuaternion.h"
#include <iostream>

int main() {
  using HighlyDynamic::boundHandQuaternion;
  using HighlyDynamic::handCommandInSourceFrame;
  const double pi = std::acos(-1.0), rad = pi / 180.0, radius = 85.5 * rad;
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  const auto x = [&](double degrees) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * rad, Eigen::Vector3d::UnitX()));
  };
  const auto pose = [&](double yaw, double pitch, double roll) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw * rad, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch * rad, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll * rad, Eigen::Vector3d::UnitX()));
  };
  bool ok = true;
  const auto check = [&](bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; ok = false; }
  };
  const auto inRange = pose(15, 25, 30);
  check(boundHandQuaternion(inRange, identity, radius).angularDistance(inRange) < 1e-9,
        "Small reachable orientation changes must retain the original mapping");
  const auto before = boundHandQuaternion(pose(120, 70.1, 20), identity, radius);
  const auto after = boundHandQuaternion(pose(120, 69.9, 20), identity, radius);
  check(before.angularDistance(after) < .3 * rad, "Crossing the old pitch bypass must not create a target jump");
  for (int sign : {-1, 1}) {
    auto source = identity;
    // Fixed feedback with sustained human rotation: bound AND write back the
    // allowed source. Testing a bound without this recurrence misses windup.
    for (int i = 0; i < 540; ++i) {
      const auto previous = source;
      source = boundHandQuaternion(x(sign) * source, identity, radius);
      check(source.angularDistance(identity) <= radius + 1e-9, "Error radius must remain bounded");
      check(source.angularDistance(previous) <= rad + 1e-9, "Crossing +/-180 input must not cause a large step");
    }
    check(source.angularDistance(x(sign * 85.5)) < 1e-8, "Sustained overload must not change the limit side");
    source = boundHandQuaternion(x(-sign) * source, identity, radius);
    check(source.angularDistance(x(sign * 84.5)) < 1e-8, "Reversal after saturation must recover immediately");
    const auto held = source;
    for (int i = 0; i < 500; ++i) source = boundHandQuaternion(source, identity, radius);
    check(source.angularDistance(held) < 1e-8, "A stationary human and stationary feedback must not drift");
  }

  // Closed-loop lagged plant, enough total wrist movement to cross quaternion
  // hemispheres repeatedly. Small human deltas must keep their rotation sign.
  auto source = identity, measured = identity;
  for (int i = 0; i < 1440; ++i) {
    const auto previous = source;
    source = boundHandQuaternion(x(1) * source, measured, radius);
    const Eigen::Quaterniond delta = previous.conjugate() * source;
    const Eigen::AngleAxisd step(delta);
    check(step.angle() * step.axis().x() >= -1e-8, "Lagged feedback must not reverse continuous human wrist motion");
    check(source.angularDistance(measured) <= radius + 1e-8, "Plant lag must not exceed the commanded error radius");
    measured = measured.slerp(.03, source).normalized();
  }

  // Undo chest follow before writeback. Repeated torso motion must not be
  // re-integrated as a human increment on the next cycle.
  const auto anchor = pose(40, -20, 10);
  source = pose(10, 20, 30);
  const auto original = source;
  for (int i = 0; i <= 720; ++i) {
    const auto chest = pose(i * .5, 30 * std::sin(i * .01), 0);
    const Eigen::Quaterniond world = chest * anchor.conjugate() * source;
    const auto command = boundHandQuaternion(world, world, radius);
    source = handCommandInSourceFrame(command, chest, anchor);
    check(source.angularDistance(original) < 1e-7, "Chest-follow writeback must not wind up the source frame");
  }
  for (int pitch = -89; pitch <= 89; ++pitch) {
    const auto output = boundHandQuaternion(pose(120, pitch, 150), identity, radius);
    const auto r = output.toRotationMatrix();
    check(std::abs(std::atan2(r(1, 0), r(0, 0))) <= radius + 1e-8 &&
          std::abs(std::asin(std::clamp(-r(2, 0), -1.0, 1.0))) <= radius + 1e-8 &&
          std::abs(std::atan2(r(2, 1), r(2, 2))) <= radius + 1e-8,
          "The conservative error ball must not enlarge any Euler safety bound");
  }
  auto flipped = inRange; flipped.coeffs() *= -1;
  check(boundHandQuaternion(flipped, identity, radius).angularDistance(inRange) < 1e-9,
        "Quaternion signs must not change the mapping");
  return ok ? 0 : 1;
}
