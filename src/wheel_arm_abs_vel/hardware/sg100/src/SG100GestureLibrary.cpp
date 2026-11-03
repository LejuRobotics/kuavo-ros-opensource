#include "wheel_arm_abs_vel/sg100/SG100GestureLibrary.h"

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
    if (!kp.IsSequence() || kp.size() != 2) {
      return false;
    }
    CurvePoint p;
    p.trigger = kp[0].as<float>();
    p.value = kp[1].as<float>();
    out.push_back(p);
  }
  return true;
}

// All ingress paths share the same field contract. Empty optional curves use defaults.
bool validateGesture(const CurveGesture& gesture) {
  if (gesture.name.empty() || gesture.joints.size() != SG100_HAND_CMD_DOF) return false;
  const char* fields[] = {"position", "torque_ff", "kp", "kd", "output_limit", "mode"};
  for (std::size_t j = 0; j < gesture.joints.size(); ++j) {
    const JointCurves& jc = gesture.joints[j];
    const std::vector<CurvePoint>* curves[] = {
        &jc.position, &jc.torque_ff, &jc.kp, &jc.kd, &jc.output_limit, &jc.mode};
    for (int field = 0; field < 6; ++field) {
      const auto& curve = *curves[field];
      if (field != 0 && curve.empty()) continue;
      bool valid = validateCurve(curve);
      for (const auto& point : curve) {
        const float v = point.value;
        if (field == 1 || field == 4) valid = valid && v >= 0 && v <= 1;
        if (field == 2 || field == 3) valid = valid && v >= 0 && v <= 10;
        if (field == 5) valid = valid &&
            (v == kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION ||
             v == kuavo_msgs::SG100HandCommand::MODE_JOINT_IMPEDANCE);
      }
      if (!valid) {
        ROS_ERROR("[SG100GestureLibrary] Invalid gesture '%s', joint %zu, field %s",
                  gesture.name.c_str(), j + 1, fields[field]);
        return false;
      }
    }
  }
  return true;
}

// Modes are discrete: hold the last keyframe, including the last duplicate at a step.
int8_t sampleMode(const std::vector<CurvePoint>& curve, float trigger) {
  int8_t mode = kuavo_msgs::SG100HandCommand::MODE_JOINT_POSITION;
  for (const auto& point : curve) {
    if (point.trigger > trigger) break;
    mode = static_cast<int8_t>(point.value);
  }
  return mode;
}

// Position-only beta rows: [trigger_percent, joint_1_rad, ..., joint_11_rad].
// beta interprets exactly three rows as mode/init/travel, not three positions;
// reject that ambiguous force-control layout until it has a dedicated adapter.
// Convert positions only; never infer degrees, gains or force-control modes.
bool parseActionList(const YAML::Node& node, CurveGesture& gesture) {
  if (!node.IsSequence() || node.size() < 2 || node.size() == 3 || gesture.name.empty()) {
    return false;
  }
  gesture.joints.resize(SG100_HAND_CMD_DOF);
  for (const YAML::Node& row : node) {
    if (!row.IsSequence() || row.size() != SG100_HAND_CMD_DOF + 1) {
      return false;
    }
    const float trigger = row[0].as<float>();
    if (!std::isfinite(trigger) || trigger < 0.0f || trigger > 100.0f) {
      return false;
    }
    for (int joint = 0; joint < SG100_HAND_CMD_DOF; ++joint) {
      const float position = row[joint + 1].as<float>();
      if (!std::isfinite(position)) {
        return false;
      }
      gesture.joints[joint].position.push_back({trigger, position});
    }
  }
  for (const JointCurves& joint : gesture.joints) {
    if (!validateCurve(joint.position)) {
      return false;
    }
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
  for (const auto& point : curve) {
    if (!std::isfinite(point.trigger) || !std::isfinite(point.value) ||
        point.trigger < 0 || point.trigger > 100) return false;
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
      return static_cast<float>((1.0 - r) * a.value + r * b.value);
    }
  }
  return curve.back().value;
}

bool SG100GestureLibrary::load(const std::string& yaml_path) {
  gestures_.clear();
  loaded_path_.clear();

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

  const auto reject = [&]() {
    ROS_ERROR("[SG100GestureLibrary] Invalid library %s; no gestures loaded", yaml_path.c_str());
    gestures_.clear();
    return false;
  };
  try {
    for (const YAML::Node& item : root) {
      if (!item.IsMap()) return reject();
      CurveGesture g;
      g.name = item["name"].as<std::string>("");
      if (const YAML::Node& actions = item["action_list"]) {
        for (const char* field : kFields) if (item[field]) return reject();
        if (!parseActionList(actions, g)) return reject();
      } else {
        g.joints.resize(SG100_HAND_CMD_DOF);
        for (int fi = 0; fi < 6; ++fi) {
          const YAML::Node& field = item[kFields[fi]];
          if (!field || !field.IsSequence() || field.size() > SG100_HAND_CMD_DOF ||
              (fi == 0 && field.size() != SG100_HAND_CMD_DOF)) return reject();
          for (std::size_t j = 0; j < field.size(); ++j) {
            JointCurves& jc = g.joints[j];
            std::vector<CurvePoint>* curves[] = {
                &jc.position, &jc.torque_ff, &jc.kp, &jc.kd, &jc.output_limit, &jc.mode};
            if (!parseCurve(field[j], *curves[fi])) return reject();
          }
        }
      }
      if (!validateGesture(g)) return reject();
      gestures_.push_back(std::move(g));
    }
  } catch (const YAML::Exception& error) {
    ROS_ERROR("[SG100GestureLibrary] YAML conversion error: %s", error.what());
    return reject();
  }

  loaded_path_ = yaml_path;
  return !gestures_.empty();
}

std::size_t SG100GestureLibrary::modeCount() const { return gestures_.size(); }

bool SG100GestureLibrary::buildTarget(int key_mode, float trigger,
                                      SG100HandPose& out,
                                      const SG100CommandParameters* defaults) const {
  if (gestures_.empty()) {
    return false;
  }
  int mode = key_mode;
  if (mode < 0 || mode >= static_cast<int>(gestures_.size())) {
    mode = 0;
  }
  return buildCurveTarget(gestures_[static_cast<std::size_t>(mode)], trigger,
                          out, defaults);
}

bool SG100GestureLibrary::buildCurveTarget(const CurveGesture& gesture,
                                           float trigger,
                                           SG100HandPose& out,
                                      const SG100CommandParameters* defaults) const {
  if (!std::isfinite(trigger) || !validateGesture(gesture) ||
      (defaults && !defaults->valid())) return false;
  const float trig_pct = clampToRange(trigger * 100.0, 0.0, 100.0);
  const int dof = static_cast<int>(gesture.joints.size());

  bool has_torque = false;
  bool has_kp = defaults != nullptr;
  bool has_kd = defaults != nullptr;
  bool has_ol = defaults != nullptr;
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

    const int8_t m = sampleMode(jc.mode, trig_pct);
    out.modes[i] = m;
    if (m == kuavo_msgs::SG100HandCommand::MODE_JOINT_IMPEDANCE) {
      any_force = true;
    }

    out.torque_ff[i] = has_torque ? sampleCurve(jc.torque_ff, trig_pct) : 0.0f;
    if (has_kp) out.kp[i] = defaults && jc.kp.empty()
        ? defaults->kp : sampleCurve(jc.kp, trig_pct);
    if (has_kd) out.kd[i] = defaults && jc.kd.empty()
        ? defaults->kd : sampleCurve(jc.kd, trig_pct);
    if (has_ol) out.output_limit[i] = defaults && jc.output_limit.empty()
        ? defaults->output_limit : sampleCurve(jc.output_limit, trig_pct);
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
  if (!validateGesture(CurveGesture{name, joints})) return false;
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
  if (!validateGesture(CurveGesture{name, joints})) return false;
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
