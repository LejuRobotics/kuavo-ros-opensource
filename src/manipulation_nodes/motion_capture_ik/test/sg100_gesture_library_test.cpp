/**
 * @brief SG100 手势库单测:折线校验/采样、增删改查、越界语义、左右库独立、YAML 往返
 *
 * 覆盖要点
 *  - 折线手势的合法性与采样语义(必须覆盖 0→100、trigger 非降序、线性插值、阶跃取后值)
 *  - 增删改查的返回语义(重名/非法折线/越界 id 的处理)
 *  - **左右手是两个独立的库实例**(生产代码里 left_lib_ / right_lib_ 各加载一份 yaml):
 *    互不影响、各自取自己的曲线、各自保存/加载各自的文件
 *
 * 无需 ROS master / 硬件;直接跑可执行文件,退出码 0 = 全部通过
 */
#include <cstdio>
#include <string>
#include <vector>

#include "motion_capture_ik/SG100GestureLibrary.h"

namespace {

int failures = 0;

void expect(bool cond, const char* what) {
  if (cond) {
    std::printf("ok  : %s\n", what);
  } else {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

using HighlyDynamic::CurvePoint;
using HighlyDynamic::JointCurves;

constexpr int kDof = HighlyDynamic::SG100_HAND_CMD_DOF;  // 11

/// 合法折线:0% → 0.0,100% → end_value
std::vector<CurvePoint> rampCurve(float end_value = 1.0f) {
  return {{0.0f, 0.0f}, {100.0f, end_value}};
}

/// 构造 11 关节手势(每关节一条合法 position 折线)
std::vector<JointCurves> makeJoints(float end_value = 1.0f) {
  std::vector<JointCurves> joints(kDof);
  for (JointCurves& jc : joints) {
    jc.position = rampCurve(end_value);
  }
  return joints;
}

}  // namespace

int main() {
  using HighlyDynamic::CurveGesture;
  using HighlyDynamic::SG100GestureLibrary;
  using HighlyDynamic::sampleCurve;
  using HighlyDynamic::validateCurve;

  // ===== 一、折线校验规则 =====
  expect(!validateCurve({}), "empty curve rejected");
  expect(!validateCurve({{0.0f, 0.0f}}), "single-point curve rejected");
  expect(!validateCurve({{0.0f, 0.0f}, {80.0f, 0.0f}}), "curve not reaching 100 rejected");
  expect(!validateCurve({{10.0f, 0.0f}, {100.0f, 1.0f}}), "curve not starting at 0 rejected");
  expect(!validateCurve({{0.0f, 0.0f}, {100.0f, 1.0f}, {50.0f, 0.5f}}),
         "non-monotonic trigger rejected");
  expect(validateCurve({{0.0f, 0.0f}, {100.0f, 1.0f}}), "ramp curve accepted");
  expect(validateCurve({{0.0f, 0.0f}, {50.0f, 0.5f}, {50.0f, 0.7f}, {100.0f, 1.0f}}),
         "duplicate trigger (step) accepted");

  // ===== 二、折线采样(trig_pct 与 CurvePoint.trigger 同为 0~100)=====
  const std::vector<CurvePoint> ramp = rampCurve(1.0f);
  expect(sampleCurve(ramp, 0.0f) == 0.0f, "sample at 0% == 0.0");
  expect(sampleCurve(ramp, 100.0f) == 1.0f, "sample at 100% == 1.0");
  const float mid = sampleCurve(ramp, 50.0f);
  expect(mid > 0.49f && mid < 0.51f, "sample at 50% == 0.5 (linear)");
  expect(sampleCurve(ramp, -5.0f) == 0.0f, "sample below range clamps to front");
  expect(sampleCurve(ramp, 500.0f) == 1.0f, "sample above range clamps to back");
  expect(sampleCurve({}, 50.0f) == 0.0f, "sample empty curve == 0.0");

  // ===== 三、空库语义 =====
  SG100GestureLibrary lib;
  expect(lib.modeCount() == 0, "empty library modeCount == 0");
  HighlyDynamic::SG100HandPose pose;
  expect(!lib.buildTarget(0, 0.5f, pose), "buildTarget on empty library == false");

  // ===== 四、增删改查 =====
  expect(!lib.addGesture("", makeJoints()), "addGesture empty name rejected");
  expect(!lib.addGesture("g", {}), "addGesture empty joints rejected");
  std::vector<JointCurves> bad = makeJoints();
  bad[3].position = {{0.0f, 0.0f}, {80.0f, 0.0f}};  // 非法折线(未覆盖 0-100)
  expect(!lib.addGesture("bad", bad), "addGesture invalid curve rejected");

  expect(lib.addGesture("first", makeJoints()), "addGesture ok");
  expect(lib.modeCount() == 1, "modeCount == 1 after add");
  expect(!lib.addGesture("first", makeJoints()), "duplicate name rejected");
  expect(lib.addGesture("second", makeJoints()), "add second gesture");
  expect(lib.modeCount() == 2, "modeCount == 2");

  expect(lib.buildTarget(0, 0.0f, pose), "buildTarget at trigger 0.0 ok");
  expect(pose.positions.size() == static_cast<std::size_t>(kDof),
         "buildTarget output has 11 positions");
  expect(pose.positions[0] == 0.0f, "trigger 0.0 → position 0.0");
  expect(lib.buildTarget(0, 1.0f, pose), "buildTarget at trigger 1.0 ok");
  expect(pose.positions[0] == 1.0f, "trigger 1.0 → position 1.0");
  expect(pose.modes.size() == static_cast<std::size_t>(kDof), "buildTarget fills per-joint modes");
  expect(lib.buildTarget(999, 1.0f, pose), "out-of-range key_mode clamps to 0");
  expect(pose.positions[0] == 1.0f, "clamped key_mode uses gesture 0");

  CurveGesture g;
  expect(lib.getGesture(1, g) && g.name == "second", "getGesture(1) == second");
  expect(!lib.getGesture(5, g), "getGesture out of range == false");
  expect(lib.updateGesture(1, "second_v2", makeJoints()), "updateGesture ok");
  expect(lib.getGesture(1, g) && g.name == "second_v2", "updateGesture applied");
  expect(!lib.updateGesture(7, "x", makeJoints()), "updateGesture out of range == false");

  expect(!lib.deleteGesture(9), "deleteGesture out of range == false");
  expect(lib.deleteGesture(0), "deleteGesture ok");
  expect(lib.modeCount() == 1, "modeCount == 1 after delete");

  // 单库 YAML 往返
  const std::string path = "/tmp/sg100_gesture_library_test.yaml";
  expect(lib.save(path), "save yaml");
  SG100GestureLibrary reloaded;
  expect(reloaded.load(path), "load saved yaml");
  expect(reloaded.modeCount() == lib.modeCount(), "round-trip modeCount preserved");
  CurveGesture r;
  expect(reloaded.getGesture(0, r) && r.name == "second_v2", "round-trip gesture name preserved");

  // ===== 五、左右手是两个独立的库实例(生产: left_lib_ / right_lib_)=====
  SG100GestureLibrary left_lib;
  SG100GestureLibrary right_lib;

  expect(left_lib.addGesture("left_g", makeJoints(1.0f)), "left lib: addGesture ok");
  expect(right_lib.addGesture("right_g", makeJoints(0.5f)), "right lib: addGesture ok");
  expect(left_lib.modeCount() == 1 && right_lib.modeCount() == 1,
         "left and right libs keep independent modeCount");

  HighlyDynamic::SG100HandPose pose_l;
  HighlyDynamic::SG100HandPose pose_r;
  expect(left_lib.buildTarget(0, 1.0f, pose_l), "left lib buildTarget ok");
  expect(right_lib.buildTarget(0, 1.0f, pose_r), "right lib buildTarget ok");
  expect(pose_l.positions[0] == 1.0f, "left lib yields its own curve (1.0)");
  expect(pose_r.positions[0] == 0.5f, "right lib yields its own curve (0.5)");
  expect(pose_l.positions[0] != pose_r.positions[0], "left/right targets differ (no cross-talk)");

  // 左右各自保存/加载各自的 yaml 文件(先存后删:此时左右各持有 1 个 gesture;
  // save() 对空库返回 false,故本段必须在 deleteGesture 之前执行)
  const std::string left_yaml = "/tmp/sg100_left_gesture_test.yaml";
  const std::string right_yaml = "/tmp/sg100_right_gesture_test.yaml";
  expect(left_lib.save(left_yaml), "left lib save its own yaml");
  expect(right_lib.save(right_yaml), "right lib save its own yaml");

  SG100GestureLibrary left_reload;
  SG100GestureLibrary right_reload;
  expect(left_reload.load(left_yaml), "left yaml reload ok");
  expect(right_reload.load(right_yaml), "right yaml reload ok");
  expect(left_reload.modeCount() == 1, "left reload keeps its gesture");
  expect(right_reload.modeCount() == 1, "right reload keeps its gesture");
  CurveGesture l_g;
  expect(left_reload.getGesture(0, l_g) && l_g.name == "left_g",
         "left reload gesture name preserved");
  CurveGesture r_g;
  expect(right_reload.getGesture(0, r_g) && r_g.name == "right_g",
         "right reload gesture name preserved");
  expect(left_reload.buildTarget(0, 1.0f, pose_l) && pose_l.positions[0] == 1.0f,
         "left reload still yields its own curve (1.0)");
  expect(right_reload.buildTarget(0, 1.0f, pose_r) && pose_r.positions[0] == 0.5f,
         "right reload still yields its own curve (0.5)");

  // 删除验证(排在保存之后,避免清空后 save 失败)
  expect(left_lib.deleteGesture(0), "left lib: deleteGesture ok");
  expect(left_lib.modeCount() == 0, "left lib empty after delete");
  expect(right_lib.modeCount() == 1, "right lib unaffected by left deletion");

  std::printf(failures ? "FAILED (%d checks)\n" : "PASSED\n", failures);
  return failures ? 1 : 0;
}
