#include "motion_capture_ik/SG100GestureLibrary.h"

#include <kuavo_msgs/SG100HandCommand.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace HighlyDynamic {

namespace {

double clampToRange(double value, double lo, double hi) {
  return std::max(lo, std::min(hi, value));
}

// 解析一条折线:YAML 序列 -> 关键点列表。空序列返回 true(空折线)。
bool parseCurve(const YAML::Node& node, std::vector<CurvePoint>& out) {
  out.clear();
  if (!node || !node.IsSequence()) {
    return false;
  }
  for (const YAML::Node& kp : node) {
    if (!kp.IsSequence() || kp.size() < 2) {
      return false;
    }
    CurvePoint p;
    p.trigger = kp[0].as<float>();
    p.value = kp[1].as<float>();
    out.push_back(p);
  }
  return true;
}

}  // namespace

bool validateCurve(const std::vector<CurvePoint>& curve) {
  if (curve.size() < 2) {
    return false;
  }
  if (curve.front().trigger != 0.0f || curve.back().trigger != 100.0f) {
    return false;
  }
  for (std::size_t i = 1; i < curve.size(); ++i) {
    if (curve[i].trigger < curve[i - 1].trigger) {
      return false;
    }
  }
  return true;
}

float sampleCurve(const std::vector<CurvePoint>& curve, float trig_pct) {
  if (curve.empty()) {
    return 0.0f;
  }
  if (trig_pct <= curve.front().trigger) {
    return curve.front().value;
  }
  if (trig_pct >= curve.back().trigger) {
    return curve.back().value;
  }
  for (std::size_t i = 0; i + 1 < curve.size(); ++i) {
    const CurvePoint& a = curve[i];
    const CurvePoint& b = curve[i + 1];
    if (trig_pct >= a.trigger && trig_pct <= b.trigger) {
      if (b.trigger == a.trigger) {
        return b.value;
      }
      const double r = (trig_pct - a.trigger) / (b.trigger - a.trigger);
      return a.value + static_cast<float>(r) * (b.value - a.value);
    }
  }
  return curve.back().value;
}

bool SG100GestureLibrary::load(const std::string& yaml_path) {
  gestures_.clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception&) {
    return false;
  }
  if (!root.IsSequence()) {
    return false;
  }

  static const char* kFields[6] = {
      "position", "torque_ff", "kp", "kd", "output_limit", "mode"};

  for (const YAML::Node& item : root) {
    if (!item.IsMap()) {
      continue;
    }

    CurveGesture g;
    if (const YAML::Node& n = item["name"]) {
      g.name = n.as<std::string>("");
    }

    bool missing = false;
    for (const char* f : kFields) {
      if (!item[f]) {
        missing = true;
        break;
      }
    }
    if (missing) {
      continue;
    }

    // 6 字段都是「11 关节的折线列表」,关节数取 position 字段的 size
    int joint_count = -1;
    std::vector<std::vector<CurvePoint>> curves[6];
    bool malformed = false;
    for (int fi = 0; fi < 6; ++fi) {
      const YAML::Node& field = item[kFields[fi]];
      if (!field.IsSequence()) {
        malformed = true;
        break;
      }
      if (joint_count < 0) {
        joint_count = static_cast<int>(field.size());
      } else if (static_cast<int>(field.size()) > joint_count) {
        // 字段元素多于关节数 → 非法
        malformed = true;
        break;
      }
      for (const YAML::Node& curveNode : field) {
        std::vector<CurvePoint> curve;
        if (!parseCurve(curveNode, curve)) {
          malformed = true;
          break;
        }
        curves[fi].push_back(std::move(curve));
      }
      if (malformed) {
        break;
      }
      // 缺的关节补空折线(默认值)
      while (static_cast<int>(curves[fi].size()) < joint_count) {
        curves[fi].push_back(std::vector<CurvePoint>());
      }
    }
    if (malformed || joint_count <= 0) {
      continue;
    }

    g.joints.resize(joint_count);
    for (int i = 0; i < joint_count; ++i) {
      g.joints[i].position = curves[0][i];
      g.joints[i].torque_ff = curves[1][i];
      g.joints[i].kp = curves[2][i];
      g.joints[i].kd = curves[3][i];
      g.joints[i].output_limit = curves[4][i];
      g.joints[i].mode = curves[5][i];
    }

    bool valid = true;
    for (int i = 0; i < joint_count && valid; ++i) {
      if (!validateCurve(g.joints[i].position)) {
        valid = false;
        break;
      }
      const std::vector<CurvePoint>* optional[5] = {
          &g.joints[i].torque_ff, &g.joints[i].kp, &g.joints[i].kd,
          &g.joints[i].output_limit, &g.joints[i].mode};
      for (const auto* c : optional) {
        if (!c->empty() && !validateCurve(*c)) {
          valid = false;
          break;
        }
      }
    }
    if (!valid) {
      continue;
    }

    gestures_.push_back(std::move(g));
  }

  loaded_path_ = yaml_path;
  return !gestures_.empty();
}

std::size_t SG100GestureLibrary::modeCount() const { return gestures_.size(); }

bool SG100GestureLibrary::buildTarget(int key_mode, float trigger,
                                      SG100HandPose& out) const {
  if (gestures_.empty()) {
    return false;
  }
  int mode = key_mode;
  if (mode < 0 || mode >= static_cast<int>(gestures_.size())) {
    mode = 0;
  }
  return buildCurveTarget(gestures_[static_cast<std::size_t>(mode)], trigger,
                          out);
}

bool SG100GestureLibrary::buildCurveTarget(const CurveGesture& gesture,
                                           float trigger,
                                           SG100HandPose& out) const {
  const float trig_pct = clampToRange(trigger * 100.0, 0.0, 100.0);
  const int dof = static_cast<int>(gesture.joints.size());

  bool has_torque = false;
  bool has_kp = false;
  bool has_kd = false;
  bool has_ol = false;
  for (const JointCurves& jc : gesture.joints) {
    if (!jc.torque_ff.empty()) has_torque = true;
    if (!jc.kp.empty()) has_kp = true;
    if (!jc.kd.empty()) has_kd = true;
    if (!jc.output_limit.empty()) has_ol = true;
  }

  out = SG100HandPose{};
  out.positions.resize(dof);
  out.modes.resize(dof);
  out.torque_ff.resize(dof, 0.0f);
  if (has_kp) out.kp.resize(dof);
  if (has_kd) out.kd.resize(dof);
  if (has_ol) out.output_limit.resize(dof);

  bool any_force = false;
  for (int i = 0; i < dof; ++i) {
    const JointCurves& jc = gesture.joints[i];
    out.positions[i] = sampleCurve(jc.position, trig_pct);

    int8_t m = kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;
    if (!jc.mode.empty()) {
      m = static_cast<int8_t>(sampleCurve(jc.mode, trig_pct));
      if (m != kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION &&
          m != kuavo_msgs::SG100HandCommand::MODE_JOINT_IMPEDANCE) {
        ROS_WARN_ONCE("SG100 joint %d: mode 非法值 %d, 兜底为 MODE_JOINT_POSITION",
                      i, m);
        m = kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;
      }
    }
    out.modes[i] = m;
    if (m == kuavo_msgs::SG100HandCommand::MODE_JOINT_IMPEDANCE) {
      any_force = true;
    }

    out.torque_ff[i] = has_torque ? sampleCurve(jc.torque_ff, trig_pct) : 0.0f;
    if (has_kp) out.kp[i] = sampleCurve(jc.kp, trig_pct);
    if (has_kd) out.kd[i] = sampleCurve(jc.kd, trig_pct);
    if (has_ol) out.output_limit[i] = sampleCurve(jc.output_limit, trig_pct);
  }

  out.has_force_control = any_force;
  return true;
}

bool SG100GestureLibrary::addGesture(
    const std::string& name, const std::vector<JointCurves>& joints) {
  if (name.empty() || joints.empty()) {
    return false;
  }
  for (const CurveGesture& g : gestures_) {
    if (g.name == name) {
      return false;
    }
  }
  for (const JointCurves& jc : joints) {
    if (!validateCurve(jc.position)) {
      return false;
    }
  }
  CurveGesture g;
  g.name = name;
  g.joints = joints;
  gestures_.push_back(std::move(g));
  return true;
}

bool SG100GestureLibrary::deleteGesture(int id) {
  if (id < 0 || id >= static_cast<int>(gestures_.size())) {
    return false;
  }
  gestures_.erase(gestures_.begin() + id);
  return true;
}

bool SG100GestureLibrary::updateGesture(
    int id, const std::string& name, const std::vector<JointCurves>& joints) {
  if (id < 0 || id >= static_cast<int>(gestures_.size()) || joints.empty()) {
    return false;
  }
  for (const JointCurves& jc : joints) {
    if (!validateCurve(jc.position)) {
      return false;
    }
  }
  gestures_[static_cast<std::size_t>(id)].name = name;
  gestures_[static_cast<std::size_t>(id)].joints = joints;
  return true;
}

bool SG100GestureLibrary::getGesture(int id, CurveGesture& out) const {
  if (id < 0 || id >= static_cast<int>(gestures_.size())) {
    return false;
  }
  out = gestures_[static_cast<std::size_t>(id)];
  return true;
}

bool SG100GestureLibrary::save(const std::string& yaml_path) const {
  if (gestures_.empty()) {
    return false;
  }

  static const char* kFields[6] = {
      "position", "torque_ff", "kp", "kd", "output_limit", "mode"};

  YAML::Emitter out;
  out.SetFloatPrecision(6);
  out.SetDoublePrecision(6);

  out << YAML::BeginSeq;  // root
  for (const CurveGesture& g : gestures_) {
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << g.name;

    auto getCurve = [&g](int fi, std::size_t joint_idx) -> const std::vector<CurvePoint>& {
      const JointCurves& jc = g.joints[joint_idx];
      switch (fi) {
        case 0: return jc.position;
        case 1: return jc.torque_ff;
        case 2: return jc.kp;
        case 3: return jc.kd;
        case 4: return jc.output_limit;
        default: return jc.mode;
      }
    };

    for (int fi = 0; fi < 6; ++fi) {
      out << YAML::Key << kFields[fi] << YAML::Value;
      out << YAML::BeginSeq;  // 字段序列，block（每关节一行）
      for (std::size_t j = 0; j < g.joints.size(); ++j) {
        out << YAML::Flow;  // 折线 flow style
        out << YAML::BeginSeq;
        for (const CurvePoint& p : getCurve(fi, j)) {
          out << YAML::BeginSeq << p.trigger << p.value << YAML::EndSeq;
        }
        out << YAML::EndSeq;
      }
      out << YAML::EndSeq;
    }

    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  std::ofstream f(yaml_path);
  if (!f) {
    return false;
  }
  f << out.c_str();
  return true;
}

}  // namespace HighlyDynamic
