/**
 * @brief SG100 关节限位单测:URDF 限位读取、越界裁剪、左右手各自独立、缺 <limit> 即失败
 *
 * 覆盖要点
 *  - 左右手使用同一个 URDF、不同前缀("l_"/"r_"),必须各读各的那一套
 *    (真机 `biped_s400055.urdf` 左右限位并非完全镜像:`thumb_j2` 左手 -2.62 / 右手 -2.618;
 *     厂商 SG100V3.0 的左右手 URDF 同样如此,故保留该不对称)
 *  - 不再套用 [0, 1.57] 默认区间:任一关节缺 <limit> → load() 返回 false,调用方据此拒绝启动
 *
 * 无需 ROS master / 硬件;直接跑可执行文件,退出码 0 = 全部通过
 */
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "motion_capture_ik/SG100JointLimits.h"

using HighlyDynamic::SG100JointLimits;

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

/// 11 个手关节的左右限位(取自 biped_s400055;thumb_j2 左右不同,其余相同)
struct JointSpec {
  const char* name;
  const char* lower_l;
  const char* upper_l;
  const char* lower_r;
  const char* upper_r;
};

const JointSpec kJoints[] = {
    {"thumb_j1", "0.0", "2.182", "0.0", "2.182"},
    {"thumb_j2", "-2.620", "0.0", "-2.618", "0.0"},  // ← 左右不对称(与真机 / 厂商 URDF 一致)
    {"thumb_j3", "-1.050", "1.570", "-1.050", "1.570"},
    {"index_j1", "-1.570", "0.0", "-1.570", "0.0"},
    {"index_j2", "0.0", "2.180", "0.0", "2.180"},
    {"index_j3", "-1.050", "1.570", "-1.050", "1.570"},
    {"middle_j1", "0.0", "2.180", "0.0", "2.180"},
    {"middle_j2", "-1.050", "1.570", "-1.050", "1.570"},
    {"little_j1", "0.0", "3.140", "0.0", "3.140"},
    {"little_j2", "0.0", "2.180", "0.0", "2.180"},
    {"little_j3", "-1.050", "1.570", "-1.050", "1.570"},
};
constexpr std::size_t kDof = sizeof(kJoints) / sizeof(kJoints[0]);

/// 生成含左右两套手关节的 URDF;drop_joint 形如 "r_little_j3" 时整条关节(link + joint)都不输出
/// 注意:不可用"revolute 关节缺 <limit>"构造缺失场景 —— urdf 库会在解析阶段拒绝整个文件
/// (Joint [x] is of type REVOLUTE but it does not specify limits),导致左右都加载不了
std::string makeUrdf(const char* drop_joint) {
  std::string s = "<?xml version=\"1.0\"?>\n<robot name=\"sg100_limits_test\">\n";
  s += "  <link name=\"base\"/>\n";
  const char* kSides[2] = {"l_", "r_"};
  for (const JointSpec& j : kJoints) {
    for (const char* side : kSides) {
      const std::string link_name = std::string(side) + j.name;
      if (drop_joint != nullptr && link_name == drop_joint) {
        continue;  // 该 link 不输出,避免孤儿 link(第二个 root)
      }
      s += std::string("  <link name=\"") + link_name + "\"/>\n";
    }
  }
  for (const JointSpec& j : kJoints) {
    const char* sides[2] = {"l_", "r_"};
    for (int k = 0; k < 2; ++k) {
      const std::string joint_name = std::string(sides[k]) + j.name;
      if (drop_joint != nullptr && joint_name == drop_joint) {
        continue;  // 该关节同样不输出
      }
      const char* lo = (k == 0) ? j.lower_l : j.lower_r;
      const char* up = (k == 0) ? j.upper_l : j.upper_r;
      s += std::string("  <joint name=\"") + joint_name + "\" type=\"revolute\">"
           "<parent link=\"base\"/><child link=\"" + joint_name + "\"/>"
           "<limit lower=\"" + lo + "\" upper=\"" + up +
           "\" effort=\"1.0\" velocity=\"1.0\"/></joint>\n";
    }
  }
  s += "</robot>\n";
  return s;
}

const char* kPathFull = "/tmp/sg100_limits_full.urdf";
const char* kPathMissingRight = "/tmp/sg100_limits_missing_right.urdf";

void writeFile(const char* path, const std::string& content) {
  std::ofstream f(path);
  f << content;
}

/// clamp 后逐项断言区间端点(左右手共用,端点由参数给出)
void expectClamped(SG100JointLimits& limits, float expect_lower_j2,
                   float expect_upper_thumb1, const char* side_label) {
  const std::string tag = side_label;
  std::vector<float> lo(kDof, -10.0f);
  std::vector<float> hi(kDof, 10.0f);

  const std::string m_clamp_lo = tag + ": clamp lower bound";
  const std::string m_clamp_hi = tag + ": clamp upper bound";
  expect(limits.clamp(lo), m_clamp_lo.c_str());
  expect(limits.clamp(hi), m_clamp_hi.c_str());

  const std::string m_j2_lo = tag + ": thumb_j2 lower == expected";
  expect(lo[1] >= expect_lower_j2 - 1e-4f && lo[1] <= expect_lower_j2 + 1e-4f, m_j2_lo.c_str());

  const std::string m_j2_hi = tag + ": thumb_j2 upper == 0";
  expect(hi[1] >= -1e-4f && hi[1] <= 1e-4f, m_j2_hi.c_str());

  const std::string m_j1_hi = tag + ": thumb_j1 upper == expected";
  expect(hi[0] >= expect_upper_thumb1 - 1e-4f && hi[0] <= expect_upper_thumb1 + 1e-4f,
         m_j1_hi.c_str());

  const std::string m_little = tag + ": little_j1 upper == 3.140";
  expect(hi[8] >= 3.140f - 1e-3f && hi[8] <= 3.140f + 1e-3f, m_little.c_str());

  const std::string m_index = tag + ": index_j1 lower == -1.570";
  expect(lo[3] >= -1.570f - 1e-4f && lo[3] <= -1.570f + 1e-4f, m_index.c_str());
}

}  // namespace

int main() {
  using HighlyDynamic::SG100JointLimits;

  std::vector<float> v(kDof, 0.5f);

  // 1) 未加载:clamp 返回 false
  SG100JointLimits limits;
  expect(!limits.clamp(v), "clamp before load returns false");

  // 2) 完整 URDF(左右两套手关节都在)→ 左右都能加载
  writeFile(kPathFull, makeUrdf(nullptr));

  SG100JointLimits left_limits;
  expect(left_limits.load(kPathFull, "l_"), "load complete urdf with l_ prefix (left hand)");
  SG100JointLimits right_limits;
  expect(right_limits.load(kPathFull, "r_"), "load complete urdf with r_ prefix (right hand)");

  // 3) 左右各读各的限位:thumb_j2 左手 -2.620 / 右手 -2.618 → 证明两套没串
  expectClamped(left_limits, -2.620f, 2.182f, "left");
  expectClamped(right_limits, -2.618f, 2.182f, "right");

  // 4) 右手整条关节缺失 → 只有右手失败,左手仍可加载(左右独立判定)
  //    注:用"整关节缺失"而非"缺 <limit>" —— urdf 库拒绝解析含无 limit revolute 关节的文件
  writeFile(kPathMissingRight, makeUrdf("r_little_j3"));
  SG100JointLimits indep_left;
  SG100JointLimits indep_right;
  expect(indep_left.load(kPathMissingRight, "l_"),
         "left hand still loads when one of right-hand joints is missing");
  expect(!indep_right.load(kPathMissingRight, "r_"),
         "right hand load fails when one of its joints is missing");

  // 5) 前缀写错(样本里没有 x_ 关节)→ 失败
  SG100JointLimits wrong_prefix;
  expect(!wrong_prefix.load(kPathFull, "x_"), "unknown joint prefix makes load fail");

  // 6) 文件不存在 → 失败
  SG100JointLimits nonexistent;
  expect(!nonexistent.load("/tmp/sg100_no_such_file.urdf", "l_"),
         "load non-existent file returns false");

  // 7) 维度不符 → clamp 失败
  std::vector<float> wrong_dim(kDof - 1, 0.0f);
  expect(!left_limits.clamp(wrong_dim), "clamp rejects wrong dimension");

  std::printf(failures ? "FAILED (%d checks)\n" : "PASSED\n", failures);
  return failures ? 1 : 0;
}
