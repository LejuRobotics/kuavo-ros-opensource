#include "wheel_arm_abs_vel/sg100/SG100GestureLibrary.h"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>
using namespace HighlyDynamic;
namespace {
using Curve = std::vector<CurvePoint>;
using Member = Curve JointCurves::*;
const Member fields[] = {&JointCurves::position, &JointCurves::torque_ff,
    &JointCurves::kp, &JointCurves::kd, &JointCurves::output_limit, &JointCurves::mode};
const char* names[] = {"position", "torque_ff", "kp", "kd", "output_limit", "mode"};
CurveGesture good() {
  CurveGesture g{"valid", std::vector<JointCurves>(11)};
  for (auto& j : g.joints) j.position = {{0, 0}, {100, 1}};
  return g;
}
YAML::Node yamlGesture(const CurveGesture& g) {
  YAML::Node n; n["name"] = g.name;
  for (int f = 0; f < 6; ++f) {
    n[names[f]] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& j : g.joints) {
      YAML::Node c(YAML::NodeType::Sequence);
      for (const auto& p : j.*fields[f]) {
        YAML::Node point; point.push_back(p.trigger); point.push_back(p.value); c.push_back(point);
      }
      n[names[f]].push_back(c);
    }
  }
  return n;
}
class CurveValidation : public ::testing::Test {
 protected:
  std::string path;
  void SetUp() override {
    char name[] = "/tmp/sg100-curves-XXXXXX";
    int fd = mkstemp(name); ASSERT_GE(fd, 0); close(fd); path = name;
  }
  void TearDown() override { std::filesystem::remove(path); }
  void write(const YAML::Node& n) { std::ofstream out(path); out << n; }
  void rejectEverywhere(const CurveGesture& bad) {
    SG100GestureLibrary lib; const auto original = good();
    ASSERT_TRUE(lib.addGesture(original.name, original.joints));
    EXPECT_FALSE(lib.addGesture("bad", bad.joints));
    EXPECT_FALSE(lib.updateGesture(0, "bad", bad.joints));
    CurveGesture retained; ASSERT_TRUE(lib.getGesture(0, retained));
    EXPECT_EQ(retained.name, "valid"); EXPECT_EQ(lib.modeCount(), 1u);
    SG100HandPose pose; EXPECT_FALSE(lib.buildCurveTarget(bad, .5f, pose));
    YAML::Node root; root.push_back(yamlGesture(original)); root.push_back(yamlGesture(bad));
    write(root); EXPECT_FALSE(lib.load(path)); EXPECT_EQ(lib.modeCount(), 0u);
  }
};
class RejectNonfiniteValuesAndTriggersAtAllEntrances : public CurveValidation {
 public:
  void TestBody() override {
  for (int f = 0; f < 6; ++f) {
    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
          std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()}) {
      auto g = good(); g.joints[0].*fields[f] = {{0, f == 5 ? 7.f : 0.f}, {50, invalid}, {100, f == 5 ? 7.f : 0.f}};
      rejectEverywhere(g);
      g.joints[0].*fields[f] = {{0, f == 5 ? 7.f : 0.f}, {invalid, 0}, {100, f == 5 ? 7.f : 0.f}};
      rejectEverywhere(g);
    }
  }

  }
};
class RejectRangesAndMalformedCurves : public CurveValidation {
 public:
  void TestBody() override {
  for (int f = 0; f < 6; ++f) {
    auto g = good(); g.joints[0].*fields[f] = {{0, 0}, {101, 0}, {100, 0}}; rejectEverywhere(g);
    g.joints[0].*fields[f] = {{0, 0}, {-1, 0}, {100, 0}}; rejectEverywhere(g);
    g.joints[0].*fields[f] = {{0, 0}}; rejectEverywhere(g);
    if (f == 0) continue;
    for (float v : {-0.01f, f == 2 || f == 3 ? 10.01f : 1.01f}) {
      g.joints[0].*fields[f] = {{0, v}, {100, v}}; rejectEverywhere(g);
    }
  }
  for (float v : {0.f, 8.f, 7.5f, 256.f}) {
    auto g = good(); g.joints[0].mode = {{0, v}, {100, v}}; rejectEverywhere(g);
  }
  auto g = good(); g.joints.pop_back(); rejectEverywhere(g);

  }
};
class DiscreteModesAndBoundaries : public CurveValidation {
 public:
  void TestBody() override {
  auto g = good(); auto& j = g.joints[0];
  j.kp = {{0, 0}, {100, 10}}; j.kd = j.kp;
  j.torque_ff = {{0, 0}, {100, 1}}; j.output_limit = j.torque_ff;
  j.mode = {{0, 7}, {50, 7}, {50, 9}, {100, 7}};
  SG100GestureLibrary lib; ASSERT_TRUE(lib.addGesture(g.name, g.joints));
  SG100HandPose pose;
  for (const auto& sample : std::vector<std::pair<float,int>>{{0,7},{.49f,7},{.5f,9},{.75f,9},{1,7}}) {
    ASSERT_TRUE(lib.buildTarget(0, sample.first, pose)); EXPECT_EQ(pose.modes[0], sample.second);
  }
  for (float t : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()})
    EXPECT_FALSE(lib.buildTarget(0, t, pose));
  EXPECT_TRUE(lib.buildTarget(0, -1, pose)); EXPECT_TRUE(lib.buildTarget(0, 2, pose));

  }
};
class MalformedYamlFailsWithoutRenumbering : public CurveValidation {
 public:
  void TestBody() override {
  YAML::Node root; root.push_back(yamlGesture(good())); root.push_back(yamlGesture(good()));
  root[1]["kp"][0].push_back("bad"); write(root);
  SG100GestureLibrary lib; EXPECT_FALSE(lib.load(path)); EXPECT_EQ(lib.modeCount(), 0u);
  root[1] = yamlGesture(good()); root[1]["position"][0][0][1] = "not-a-number";
  write(root); EXPECT_FALSE(lib.load(path)); EXPECT_EQ(lib.modeCount(), 0u);

  }
};
class ExtremeFinitePositionsInterpolateWithoutOverflow : public CurveValidation {
 public:
  void TestBody() override {
  auto g = good(); float m = std::numeric_limits<float>::max();
  g.joints[0].position = {{0, -m}, {100, m}};
  SG100GestureLibrary lib; SG100HandPose pose;
  ASSERT_TRUE(lib.buildCurveTarget(g, .5f, pose)); EXPECT_TRUE(std::isfinite(pose.positions[0]));
  EXPECT_EQ(pose.positions[0], 0);

  }
};
class ProductionGesturesKeepPositionsAndDefaults : public CurveValidation {
 public:
  void TestBody() override {
  const auto config = std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
  for (const char* side : {"left", "right"}) {
    const std::string file = (config / (std::string("sg100_") + side + "_gesture.yaml")).string();
    const auto input = YAML::LoadFile(file);
    SG100GestureLibrary lib; ASSERT_TRUE(lib.load(file)); ASSERT_EQ(lib.modeCount(), 5u);
    SG100CommandParameters defaults;
    for (int id = 0; id < 5; ++id) for (float t : {0.f, .25f, .5f, .75f, 1.f}) {
      SG100HandPose pose; ASSERT_TRUE(lib.buildTarget(id, t, pose, &defaults));
      ASSERT_EQ(pose.positions.size(), 11u);
      for (int j = 0; j < 11; ++j) {
        const auto curve = input[id]["position"][j];
        ASSERT_EQ(curve.size(), 2u);
        const float start = curve[0][1].as<float>(), end = curve[1][1].as<float>();
        EXPECT_NEAR(pose.positions[j], start + t * (end - start), 1e-6);
        EXPECT_EQ(pose.modes[j], 7); EXPECT_EQ(pose.kp[j], 10);
        EXPECT_EQ(pose.kd[j], .2f); EXPECT_EQ(pose.output_limit[j], .6f);
      }
    }
  }

  }
};

}

// Explicit registration keeps the fixture semantics visible to older cppcheck.
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::RegisterTest("CurveValidation", "RejectNonfiniteValuesAndTriggersAtAllEntrances",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new RejectNonfiniteValuesAndTriggersAtAllEntrances; });
  ::testing::RegisterTest("CurveValidation", "RejectRangesAndMalformedCurves",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new RejectRangesAndMalformedCurves; });
  ::testing::RegisterTest("CurveValidation", "DiscreteModesAndBoundaries",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new DiscreteModesAndBoundaries; });
  ::testing::RegisterTest("CurveValidation", "MalformedYamlFailsWithoutRenumbering",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new MalformedYamlFailsWithoutRenumbering; });
  ::testing::RegisterTest("CurveValidation", "ExtremeFinitePositionsInterpolateWithoutOverflow",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new ExtremeFinitePositionsInterpolateWithoutOverflow; });
  ::testing::RegisterTest("CurveValidation", "ProductionGesturesKeepPositionsAndDefaults",
      nullptr, nullptr, __FILE__, __LINE__, []() -> CurveValidation* { return new ProductionGesturesKeepPositionsAndDefaults; });
  return RUN_ALL_TESTS();
}
