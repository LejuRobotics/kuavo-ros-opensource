#include "motion_capture_ik/GripQuaternionRecovery.h"
#include <iostream>

int main() {
  using HighlyDynamic::GripQuaternionRecovery;
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  const auto rotation = [](double angle) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
  };
  const auto check = [](bool ok, const char* message) {
    if (!ok) std::cerr << message << '\n';
    return ok;
  };
  bool ok = true;
  GripQuaternionRecovery recovery;
  auto previous = identity;
  ok &= check(recovery.update(previous, rotation(0.1)).angularDistance(rotation(0.1)) < 1e-9,
              "Small rotations must pass without translation");
  ok &= check(recovery.update(previous, rotation(1.5)).angularDistance(identity) < 1e-9,
              "An isolated spike must be rejected");
  previous = recovery.update(previous, identity);
  ok &= check(previous.angularDistance(identity) < 1e-9, "Returning input must preserve output");
  for (int frame = 0; frame < 10; ++frame) {
    const auto output = recovery.update(previous, rotation(1.5));
    ok &= check(previous.angularDistance(output) <= 0.5 + 1e-9, "Recovery step must stay bounded");
    previous = output;
  }
  ok &= check(previous.angularDistance(rotation(1.5)) < 1e-9,
              "A sustained valid target must recover without translation");
  recovery.reset();
  previous = identity;
  for (int frame = 0; frame < 8; ++frame) {
    const auto output = recovery.update(previous, rotation((frame % 2) ? -1.5 : 1.5));
    ok &= check(output.angularDistance(identity) < 1e-9, "Discontinuous spikes must stay rejected");
  }
  recovery.reset();
  auto negative = rotation(1.5);
  negative.coeffs() *= -1.0;
  recovery.update(identity, rotation(1.5));
  recovery.update(identity, negative);
  ok &= check(recovery.update(identity, rotation(1.5)).angularDistance(identity) > 0.1,
              "Quaternion sign changes must not interrupt recovery");
  return ok ? 0 : 1;
}
