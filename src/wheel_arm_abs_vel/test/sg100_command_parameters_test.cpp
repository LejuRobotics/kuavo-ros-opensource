#include "hardware_node/sg100_command_encoding.h"
#include "wheel_arm_abs_vel/sg100/SG100GestureLibrary.h"
#include <cmath>
#include <cstdio>
#include <limits>

int main() {
  using namespace HighlyDynamic;
  int failed = 0;
  auto check = [&](bool ok, const char* message) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", message);
    if (!ok) ++failed;
  };
  check(encodeSg100OutputLimit(0.6f) == 1966, "0.6 uses deployment wire code 1966, not 19660");
  check(encodeSg100OutputLimit(0.5f) == 1638, "0.5 is not scaled to 5.0");
  check(encodeSg100OutputLimit(0.0f) == 0, "zero output remains zero");
  check(encodeSg100OutputLimit(1.0f) == 3277, "upper allowed value uses SDK units");
  SG100CommandParameters defaults;
  check(defaults.valid() && defaults.kp == 10 && defaults.kd == 0.2f &&
        defaults.output_limit == 0.6f, "deployment defaults");
  CurveGesture g;
  g.name = "partial_position_override";
  g.joints.resize(11);
  for (auto& j : g.joints) j.position = {{0, 0}, {100, 1}};
  g.joints[0].kp = {{0, 2}, {100, 4}};
  g.joints[0].kd = {{0, 0}, {100, 0}};
  g.joints[0].output_limit = {{0, 0.2f}, {100, 0.4f}};
  SG100GestureLibrary library;
  SG100HandPose pose;
  check(library.buildCurveTarget(g, 0.5f, pose, &defaults), "sample position-only gesture");
  check(!pose.has_force_control && pose.kp.size() == 11 && pose.kd.size() == 11 &&
        pose.output_limit.size() == 11, "position mode carries complete parameter arrays");
  check(pose.kp[0] == 3 && pose.kd[0] == 0 && std::abs(pose.output_limit[0]-0.3f)<1e-6f,
        "joint overrides interpolate and explicit zero is preserved");
  check(pose.kp[1] == 10 && pose.kd[1] == 0.2f && pose.output_limit[1] == 0.6f,
        "unspecified joints inherit defaults instead of zero");
  defaults.kp = 1.5f; defaults.kd = 0.1f; defaults.output_limit = 0.25f;
  library.buildCurveTarget(g, 0.5f, pose, &defaults);
  check(pose.kp[0] == 3 && pose.kp[1] == 1.5f && pose.output_limit[1] == 0.25f,
        "launch values supply only missing curves");
  g.joints[2].mode = {{0, 9}, {100, 9}};
  library.buildCurveTarget(g, 0.5f, pose, &defaults);
  check(pose.has_force_control && pose.modes[0] == 7 && pose.modes[2] == 9,
        "mixed modes preserved");
  defaults.output_limit = 5;
  check(!library.buildCurveTarget(g, 0, pose, &defaults), "reject obsolete scaled output limit");
  defaults.output_limit = std::numeric_limits<float>::quiet_NaN();
  check(!defaults.valid(), "reject NaN output limit");
  return failed ? 1 : 0;
}
