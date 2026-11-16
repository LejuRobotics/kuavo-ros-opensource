#include "motion_capture_ik/LowerBodyGripHandoff.h"
#include <cmath>
#include <iostream>

int main() {
  HighlyDynamic::LowerBodyGripHandoff handoff;
  const double rad = std::acos(-1.0) / 180.0;
  // The actual last published pose from the release regression bag, with a
  // live IK goal roughly 20 degrees away in waist pitch on re-entry.
  const Eigen::Vector3d published = Eigen::Vector3d(27.40, -33.92, 9.49) * rad;
  const Eigen::Vector3d target = Eigen::Vector3d(18.62, -39.85, 28.85) * rad;
  bool ok = true;
  auto check = [&](bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; ok = false; }
  };
  handoff.freeze(published);
  handoff.resume(published, 100.0);
  check((handoff.apply(target, 100.0, 1.0) - published).norm() < 1e-12,
        "Resume must start at the final published position");
  Eigen::Vector3d previous = published;
  for (int i = 1; i <= 1000; ++i) {
    const auto next = handoff.apply(target, 100.0 + i * .001, 1.0);
    check((next - previous).cwiseAbs().maxCoeff() < .001,
          "Recorded target gap must be spread over the transition, not emitted as a step");
    check((next - published).dot(target - published) >= -1e-12,
          "Transition must move toward the live target");
    previous = next;
  }
  check((previous - target).norm() < 1e-12 && !handoff.active,
        "Normal target following must resume after the bridge");
  handoff.resume(published, 200.0);
  const auto interrupted = handoff.apply(target, 200.4, 1.0);
  handoff.freeze(interrupted);
  handoff.resume(interrupted, 200.5);
  check((handoff.apply(published, 200.5, 1.0) - interrupted).norm() < 1e-12,
        "Release and re-grip mid-transition must preserve the latest published position");
  handoff.resume(published, 300.0);
  check((handoff.apply(target, 299.0, 1.0) - published).norm() < 1e-12,
        "A backwards clock must not extrapolate the target");
  return ok ? 0 : 1;
}
