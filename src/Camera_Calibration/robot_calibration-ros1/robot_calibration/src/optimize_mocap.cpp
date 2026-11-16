/*
 * Copyright (C) 2026 Leju Robotics
 *
 * Offline mocap joint-zero calibration entry point.
 *
 * Reads a capture_*.json produced by record_joint_poses.py, builds
 * robot_calibration_msgs::CalibrationData (JointState + Observation.poses),
 * and runs the Ceres optimizer with the chain3d_to_mocap error block.
 *
 * Usage (via launch, or directly):
 *   rosrun robot_calibration optimize_mocap \
 *     _capture:=/path/to/capture.json \
 *     _robot_layout:=wheel62
 *   Config (models/error_blocks/free_params) loaded from ROS params as usual.
 */

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <XmlRpcValue.h>

#include <jsoncpp/json/json.h>
#include <fstream>

#include <robot_calibration/calibration/offset_parser.h>
#include <robot_calibration/ceres/optimizer.h>
#include <robot_calibration/calibration/export.h>
#include <robot_calibration_msgs/CalibrationData.h>

#include <Eigen/Dense>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{

// Read a JSON file into a Json::Value; returns false on failure.
bool readJson(const std::string& path, Json::Value& out)
{
  std::ifstream f(path.c_str());
  if (!f.is_open())
  {
    ROS_ERROR_STREAM("Cannot open capture JSON: " << path);
    return false;
  }
  Json::CharReaderBuilder builder;
  std::string errs;
  if (!Json::parseFromStream(builder, f, &out, &errs))
  {
    ROS_ERROR_STREAM("JSON parse error: " << errs);
    return false;
  }
  return true;
}

double vec3At(const Json::Value& arr, int i)
{
  return arr[i].asDouble();
}

// Apply mocap_frame_align (axis_flip or rpy_rad) to position+orientation.
// Returns the R_align 3x3 as row-major double[9] (or identity if disabled).
bool loadAlignMatrix(const Json::Value& align, double R[9])
{
  for (int i = 0; i < 9; ++i) R[i] = (i % 4 == 0) ? 1.0 : 0.0;
  if (!align.isObject() || !align.get("enabled", Json::Value(true)).asBool())
    return true;

  if (align.isMember("axis_flip"))
  {
    const Json::Value& flip = align["axis_flip"];
    for (int i = 0; i < 3 && i < static_cast<int>(flip.size()); ++i)
    {
      R[i * 3 + i] = flip[i].asDouble();
    }
    return true;
  }
  if (align.isMember("rpy_rad"))
  {
    const Json::Value& rpy = align["rpy_rad"];
    double r = rpy[0].asDouble(), p = rpy[1].asDouble(), y = rpy[2].asDouble();
    double cr = cos(r), sr = sin(r);
    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);
    // R = Rz * Ry * Rx
    double Rx[9] = {1,0,0, 0,cr,-sr, 0,sr,cr};
    double Ry[9] = {cp,0,sp, 0,1,0, -sp,0,cp};
    double Rz[9] = {cy,-sy,0, sy,cy,0, 0,0,1};
    // Rz * Ry
    double Rzy[9];
    for (int r_ = 0; r_ < 3; ++r_)
      for (int c = 0; c < 3; ++c)
        Rzy[r_*3+c] = Rz[r_*3+0]*Ry[0*3+c] + Rz[r_*3+1]*Ry[1*3+c] + Rz[r_*3+2]*Ry[2*3+c];
    // (Rz*Ry) * Rx
    for (int r_ = 0; r_ < 3; ++r_)
      for (int c = 0; c < 3; ++c)
        R[r_*3+c] = Rzy[r_*3+0]*Rx[0*3+c] + Rzy[r_*3+1]*Rx[1*3+c] + Rzy[r_*3+2]*Rx[2*3+c];
    return true;
  }
  ROS_WARN("mocap_frame_align has neither axis_flip nor rpy_rad; using identity");
  return true;
}

void matVec3(const double R[9], const double v[3], double out[3])
{
  out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
  out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
  out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

// Apply R_align to a quaternion (xyzw): q' = R_align * q (left-multiply rotation).
// We do: R' = R_align * R(q), then convert back to quaternion.
void alignQuat(const double R[9], const double q[4], double qout[4])
{
  // quaternion -> rotation matrix
  double x=q[0], y=q[1], z=q[2], w=q[3];
  double xx=x*x, yy=y*y, zz=z*z;
  double xy=x*y, xz=x*z, yz=y*z;
  double wx=w*x, wy=w*y, wz=w*z;
  double M[9] = {
    1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy),
    2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx),
    2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)
  };
  // R' = R_align * M
  double Mp[9];
  for (int r=0; r<3; ++r)
    for (int c=0; c<3; ++c)
      Mp[r*3+c] = R[r*3+0]*M[0*3+c] + R[r*3+1]*M[1*3+c] + R[r*3+2]*M[2*3+c];
  // rotation matrix -> quaternion (wxyz, standard)
  double tr = Mp[0] + Mp[4] + Mp[8];
  double qw, qx_, qy_, qz_;
  if (tr > 0)
  {
    double S = sqrt(tr + 1.0) * 2.0;
    qw = 0.25 * S;
    qx_ = (Mp[7] - Mp[5]) / S;
    qy_ = (Mp[2] - Mp[6]) / S;
    qz_ = (Mp[3] - Mp[1]) / S;
  }
  else if (Mp[0] > Mp[4] && Mp[0] > Mp[8])
  {
    double S = sqrt(1.0 + Mp[0] - Mp[4] - Mp[8]) * 2.0;
    qw = (Mp[7] - Mp[5]) / S;
    qx_ = 0.25 * S;
    qy_ = (Mp[1] + Mp[3]) / S;
    qz_ = (Mp[2] + Mp[6]) / S;
  }
  else if (Mp[4] > Mp[8])
  {
    double S = sqrt(1.0 + Mp[4] - Mp[0] - Mp[8]) * 2.0;
    qw = (Mp[2] - Mp[6]) / S;
    qx_ = (Mp[1] + Mp[3]) / S;
    qy_ = 0.25 * S;
    qz_ = (Mp[5] + Mp[7]) / S;
  }
  else
  {
    double S = sqrt(1.0 + Mp[8] - Mp[0] - Mp[4]) * 2.0;
    qw = (Mp[3] - Mp[1]) / S;
    qx_ = (Mp[2] + Mp[6]) / S;
    qy_ = (Mp[5] + Mp[7]) / S;
    qz_ = 0.25 * S;
  }
  double n = sqrt(qw*qw + qx_*qx_ + qy_*qy_ + qz_*qz_);
  qout[0] = qx_/n; qout[1] = qy_/n; qout[2] = qz_/n; qout[3] = qw/n;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "optimize_mocap");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_global;

  bool verbose = false;
  nh.param<bool>("verbose", verbose, false);

  std::string capture_path = "";
  nh.param<std::string>("capture", capture_path, capture_path);
  if (capture_path.empty())
  {
    ROS_FATAL("optimize_mocap: required param ~capture (path to capture_*.json) not set");
    return -1;
  }

  Json::Value root;
  if (!readJson(capture_path, root))
    return -1;

  // --- Align matrix ---
  double R_align[9];
  loadAlignMatrix(root["meta"]["mocap_frame_align"], R_align);

  // --- Build CalibrationData from JSON samples ---
  const Json::Value& samples = root["samples"];
  const Json::Value& meta = root["meta"];
  const Json::Value& bodies = root["bodies"];
  const Json::Value& ji = meta["joint_q_indices"];

  // Build joint name -> index map (waist[optional] + arms + head)
  std::vector<std::string> joint_names;
  std::vector<int> joint_idxs;
  {
    // waist (optional: s45 无腰，不存在 waist 键则跳过)
    if (ji.isMember("waist"))
    {
      joint_names.push_back("waist_yaw_joint");
      joint_idxs.push_back(ji["waist"].asInt());
    }
    // left arm l1..l7
    std::vector<std::string> left_names = {
      "zarm_l1_joint","zarm_l2_joint","zarm_l3_joint","zarm_l4_joint",
      "zarm_l5_joint","zarm_l6_joint","zarm_l7_joint"};
    int lo = ji["left_arm"][0].asInt();
    for (size_t i = 0; i < left_names.size(); ++i)
    {
      joint_names.push_back(left_names[i]);
      joint_idxs.push_back(lo + static_cast<int>(i));
    }
    // right arm r1..r7
    std::vector<std::string> right_names = {
      "zarm_r1_joint","zarm_r2_joint","zarm_r3_joint","zarm_r4_joint",
      "zarm_r5_joint","zarm_r6_joint","zarm_r7_joint"};
    int rlo = ji["right_arm"][0].asInt();
    for (size_t i = 0; i < right_names.size(); ++i)
    {
      joint_names.push_back(right_names[i]);
      joint_idxs.push_back(rlo + static_cast<int>(i));
    }
  }

  // Body tip links: body name -> tip link, and sensor_name = "<body>_to_base"
  std::vector<std::string> body_names;
  std::vector<std::string> body_tips;
  std::vector<std::string> sensor_names;
  for (const auto& b : bodies)
  {
    std::string name = b["name"].asString();
    if (name == "torso")
      continue;  // reference, not an observed tip
    if (b.get("record_only", false).asBool())
      continue;  // only recorded, not used in optimization (e.g. l_shoulder)
    body_names.push_back(name);
    body_tips.push_back(b["tip_link"].asString());
    sensor_names.push_back(name + "_to_base");
  }

  std::vector<robot_calibration_msgs::CalibrationData> data;
  for (const auto& s : samples)
  {
    const Json::Value& qfull = s["joint_q_full"];
    const Json::Value& s_bodies = s["bodies"];

    robot_calibration_msgs::CalibrationData d;
    d.joint_states.name = joint_names;
    d.joint_states.position.resize(joint_names.size());
    d.joint_states.velocity.resize(joint_names.size(), 0.0);
    d.joint_states.effort.resize(joint_names.size(), 0.0);
    for (size_t i = 0; i < joint_idxs.size(); ++i)
    {
      int idx = joint_idxs[i];
      double v = (idx >= 0) ? qfull[idx].asDouble()
                            : qfull[qfull.size() + idx].asDouble();
      d.joint_states.position[i] = v;
    }

    // Build one observation per observed body
    for (size_t k = 0; k < body_names.size(); ++k)
    {
      std::string key = body_names[k] + "_in_torso";
      if (!s_bodies.isMember(key) || s_bodies[key].isNull())
        continue;

      const Json::Value& obs = s_bodies[key];
      robot_calibration_msgs::Observation o;
      o.sensor_name = sensor_names[k];
      o.poses.resize(1);
      geometry_msgs::PoseStamped& ps = o.poses[0];

      double p_mm[3] = {obs["xyz_mm"][0].asDouble(),
                        obs["xyz_mm"][1].asDouble(),
                        obs["xyz_mm"][2].asDouble()};
      double p_m[3] = {p_mm[0]/1000.0, p_mm[1]/1000.0, p_mm[2]/1000.0};
      double p_align[3];
      matVec3(R_align, p_m, p_align);
      ps.pose.position.x = p_align[0];
      ps.pose.position.y = p_align[1];
      ps.pose.position.z = p_align[2];

      double q_in[4] = {obs["quaternion_xyzw"][0].asDouble(),
                        obs["quaternion_xyzw"][1].asDouble(),
                        obs["quaternion_xyzw"][2].asDouble(),
                        obs["quaternion_xyzw"][3].asDouble()};
      double q_align[4];
      alignQuat(R_align, q_in, q_align);
      ps.pose.orientation.x = q_align[0];
      ps.pose.orientation.y = q_align[1];
      ps.pose.orientation.z = q_align[2];
      ps.pose.orientation.w = q_align[3];

      d.observations.push_back(o);
    }

    if (!d.observations.empty())
      data.push_back(d);
  }

  if (data.empty())
  {
    ROS_FATAL("optimize_mocap: no valid samples with observations in %s", capture_path.c_str());
    return -1;
  }
  ROS_INFO_STREAM("optimize_mocap: loaded " << data.size() << " samples from " << capture_path);

  // Load URDF from parameter
  std_msgs::String description_msg;
  if (!nh_global.getParam("/robot_description", description_msg.data))
  {
    ROS_FATAL("Missing global parameter: /robot_description");
    return -1;
  }

  // 健壮性校验：robot_description 是否含 waist_yaw_joint（s62 有 / s45 无），
  // 与 capture meta.fk_root 交叉验证。URDF 与采集数据不匹配会静默解出乱 bias。
  {
    const bool urdf_has_waist = description_msg.data.find("waist_yaw_joint") != std::string::npos;
    const bool meta_is_waist = (meta.isMember("fk_root") && meta["fk_root"].asString() == "waist_yaw_link");
    if (meta_is_waist != urdf_has_waist)
    {
      ROS_FATAL_STREAM("optimize_mocap: URDF 与 capture 不匹配！"
                      << " capture fk_root 含腰=" << (meta_is_waist ? "是" : "否")
                      << "，robot_description 含 waist_yaw_joint=" << (urdf_has_waist ? "是" : "否")
                      << "。请用正确的 robot_layout 选对 URDF（s62 用 wheel62，s45 用 biped45）。");
      return -1;
    }
  }

  // Create optimizer and load config (models/error_blocks/free_params) from ROS params
  robot_calibration::Optimizer opt(description_msg.data);
  robot_calibration::OptimizationParams params;
  params.LoadFromROS(nh);

  // 以采集时的事实为准：capture meta 的 fk_root 覆盖 yaml 的 base_link。
  // 避免 --layout 选错（s62 vs s45）导致 FK 根错误而静默解出乱 bias。
  if (meta.isMember("fk_root"))
  {
    const std::string meta_fk_root = meta["fk_root"].asString();
    if (params.base_link != meta_fk_root)
    {
      ROS_WARN_STREAM("optimize_mocap: capture fk_root='" << meta_fk_root
                      << "' 与 config base_link='" << params.base_link
                      << "' 不一致，以 capture 为准覆盖。"
                      << "（若为误用 --layout，请检查机型）");
      params.base_link = meta_fk_root;
    }
  }

  // Optional: single calibration step (no cal_steps). Do not export an empty
  // or unusable solution: the shell wrapper relies on this process status.
  const int optimize_rc = opt.optimize(params, data, verbose);
  const auto summary = opt.summary();
  if (optimize_rc != 0 || !summary || opt.getNumResiduals() <= 0 || !summary->IsSolutionUsable())
  {
    ROS_FATAL_STREAM("optimize_mocap: optimization failed or solution is unusable"
                     << " (rc=" << optimize_rc
                     << ", residuals=" << opt.getNumResiduals() << ")");
    return -1;
  }

  // Log solved offsets
  auto offsets = opt.getOffsets();
  if (offsets)
  {
    ROS_INFO_STREAM("Solved free_params:");
    for (const auto& name : params.free_params)
    {
      const double v_rad = offsets->get(name);
      ROS_INFO_STREAM("  " << name << ": " << v_rad << " rad (" << (v_rad * 180.0 / M_PI) << " deg)");
    }
  }

  if (!robot_calibration::exportResults(opt, description_msg.data, data))
  {
    ROS_FATAL("optimize_mocap: failed to export calibration results");
    return -1;
  }
  ROS_INFO("Done optimizing mocap.");
  return 0;
}
