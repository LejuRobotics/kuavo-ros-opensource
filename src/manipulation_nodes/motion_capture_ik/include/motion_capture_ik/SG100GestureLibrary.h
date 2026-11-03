#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace HighlyDynamic {

// SG100 dexterous hand command DOF (per hand): 11 joints.
constexpr int SG100_HAND_CMD_DOF = 11;

// 单手控制目标:11 维关节位置 + 可选逐关节控制元数据。
// 逐关节数组为空时,由组装方回落到全局 control_mode / launch 默认值。
struct SG100HandPose {
  std::vector<float> positions;      // size SG100_HAND_CMD_DOF, rad
  std::vector<int8_t> modes;         // 逐关节模式(SG100HandCommand::MODE_*)
  std::vector<float> kp;             // 逐关节刚度(空=默认)
  std::vector<float> kd;             // 逐关节阻尼(空=默认)
  std::vector<float> torque_ff;      // 逐关节前馈力矩(有符号)
  std::vector<float> output_limit;   // 逐关节输出限制(空=默认)
  bool has_force_control = false;
};

// === 折线图格式:每关节每参数一条 trigger 折线 ===

// 折线关键点:trigger(0~100) + value
struct CurvePoint {
  float trigger = 0.0f;
  float value = 0.0f;
};

// 单关节 6 条折线(空 vector = 用默认值)
struct JointCurves {
  std::vector<CurvePoint> position;      // 必须非空,覆盖 0-100
  std::vector<CurvePoint> torque_ff;     // 空=默认 0
  std::vector<CurvePoint> kp;            // 空=默认 launch kp
  std::vector<CurvePoint> kd;            // 空=默认 launch kd
  std::vector<CurvePoint> output_limit;  // 空=默认 launch limit
  std::vector<CurvePoint> mode;          // 空=默认 7,值仅 7/9
};

// 折线图手势:11 关节,每关节 6 条折线
struct CurveGesture {
  std::string name;
  std::vector<JointCurves> joints;  // size == SG100_HAND_CMD_DOF
};

// 折线校验:非空、trigger 升序(允许相等=阶跃)、首点=0、尾点=100
bool validateCurve(const std::vector<CurvePoint>& curve);

// 折线采样:在 trig_pct 处取值(线性插值,同 trigger 双点取后值)
float sampleCurve(const std::vector<CurvePoint>& curve, float trig_pct);

// 手势库(折线图格式)
class SG100GestureLibrary {
 public:
  // 加载折线图 YAML 文件。失败返回 false(并清空状态)。
  bool load(const std::string& yaml_path);

  // 手势数量。
  std::size_t modeCount() const;

  // 按 key_mode(clamp 到 [0, modeCount)) 和 trigger(0~1) 解算单手目标。
  bool buildTarget(int key_mode, float trigger, SG100HandPose& out) const;

  // 折线图解算:根据 trigger 采样 gesture 的 6 条折线,输出 SG100HandPose。
  bool buildCurveTarget(const CurveGesture& gesture, float trigger,
                        SG100HandPose& out) const;

  // 新增单个手势。返回 false 当名称重复或关节数非法。
  bool addGesture(const std::string& name,
                  const std::vector<JointCurves>& joints);
  // 删除指定 id(下标)。返回 false 当 id 越界。
  bool deleteGesture(int id);
  // 重定义指定 id 的手势内容。返回 false 当 id 越界或关节数非法。
  bool updateGesture(int id, const std::string& name,
                     const std::vector<JointCurves>& joints);
  // 返回指定 id 手势(供 service 查询)。返回 false 当 id 越界。
  bool getGesture(int id, CurveGesture& out) const;
  // 把所有手势序列化回 YAML(覆盖写)。返回 false 当写失败。
  bool save(const std::string& yaml_path) const;

 private:
  std::vector<CurveGesture> gestures_;
  std::string loaded_path_;  // load 时的原路径,供 save 复用
};

}  // namespace HighlyDynamic
