#include "dagger_candidate.h"
#include <motion_capture_ik/DaggerArmSample.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace velocity_ik {
namespace {
using Pose = DaggerCandidate::Pose;
using V = Eigen::Vector3d;
template<class P> bool readPose(const P& msg, Pose& out) {
  const auto& p = msg.position;
  const auto& r = msg.orientation;
  Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
  if (!V(p.x,p.y,p.z).allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-8) return false;
  out.linear() = q.normalized().toRotationMatrix();
  out.translation() = V(p.x,p.y,p.z);
  return true;
}
template<class Vector> std::vector<double> values(const Vector& q) {
  return {q.data(), q.data() + q.size()};
}
double yaw(const Pose& chest) {
  const V forward = chest.linear().col(2);
  return std::atan2(forward.y(), forward.x());
}
double pitch(const Pose& chest) {
  const V forward = chest.linear().col(2);
  return std::atan2(-forward.z(), std::hypot(forward.x(), forward.y()));
}
}  // namespace

void DaggerCandidate::Hand::reset() {
  if (ready || requested) ++generation;
  failed_ticks = 0;
  requested = ready = false;
}
bool DaggerCandidate::fresh(const ros::Time& now, const ros::Time& stamp, double timeout) {
  const double age = (now-stamp).toSec();
  return !stamp.isZero() && age >= -.05 && age <= timeout;
}
DaggerCandidate::DaggerCandidate(ros::NodeHandle& nh, ros::NodeHandle& priv)
    : session_(std::to_string(ros::WallTime::now().toNSec())) {
  priv.param("use_incremental_hand_orientation", incremental_orientation_, true);
  for (int i = 0; i < 3; ++i) {
    translation_scale[i] = priv.param(std::string("delta_scale_") + "xyz"[i], 1.26);
  }
  const auto tcp = ros::TransportHints().tcpNoDelay();
  for (int i = 0; i < 2; ++i) {
    grip_sub_[i] = nh.subscribe<std_msgs::Bool>(i == 0 ? "/dagger/vr/left_intervention" : "/dagger/vr/right_intervention",
        8, [this, i](const std_msgs::Bool::ConstPtr& m){ grip(i, m->data); }, ros::VoidConstPtr(), tcp);
  }
  bones_sub_ = nh.subscribe("/leju_quest_bone_poses", 1, &DaggerCandidate::bones, this, tcp);
  body_sub_ = nh.subscribe("/dagger/vr/body_pose", 1, &DaggerCandidate::body, this, tcp);
  config_sub_ = nh.subscribe("/dagger/vr/waist_config", 1, &DaggerCandidate::config, this);
  local_sample_pub_ = nh.advertise<motion_capture_ik::DaggerArmSample>("/dagger/ik/arm_sample", 1);
  publisher_ = nh.advertise<std_msgs::String>("/dagger/vr/arm_candidate", 1);
}
void DaggerCandidate::grip(int i, bool pressed) {
  auto& h = hands[i];
  const auto now = ros::Time::now();
  if (pressed != h.pressed || !fresh(now, h.heartbeat, .2)) {
    h.reset();
    h.grip_edge = now;
    if (config_.side == (i ? "right" : "left") ||
        (config_.side == "both" && !hands[1-i].requested)) resetWaist();
  }
  h.pressed = pressed;
  h.heartbeat = now;
}
void DaggerCandidate::bones(const noitom_hi5_hand_udp_python::PoseInfoList::ConstPtr& msg) {
  const auto now = ros::Time::now();
  if (msg->timestamp_ms <= 0 || msg->poses.size() < 24) return;
  if (msg->timestamp_ms == bone_sequence_) return;
  if (msg->timestamp_ms < bone_sequence_ && fresh(now, last_bones_, .2)) return;
  Pose chest;
  if (!readPose(msg->poses[23], chest)) return;
  // The Quest mapper uses Rz(pi/2) Rx(pi/2) as the neutral chest axes.
  const Eigen::Matrix3d axes = Eigen::AngleAxisd(M_PI_2,V::UnitZ()) *
      Eigen::AngleAxisd(M_PI_2,V::UnitX()) * chest.linear().transpose();
  bone_sequence_ = msg->timestamp_ms;
  last_bones_ = now;
  for (int i = 0; i < 2; ++i) {
    Pose palm, elbow;
    auto& h = hands[i];
    if (!readPose(msg->poses[4+i], palm) || !readPose(msg->poses[1+2*i], elbow)) {
      h.bone_stamp = ros::Time(); h.reset(); continue;
    }
    h.human.translation() = axes * (palm.translation()-chest.translation());
    const double bias = (i ? 1. : -1.) * M_PI/12.;
    h.human.linear() = axes * palm.linear() * Eigen::AngleAxisd(i ? 0. : -M_PI,V::UnitZ())
        * Eigen::AngleAxisd(-M_PI_2+bias,V::UnitX());
    h.elbow = axes * (elbow.translation()-chest.translation());
    h.bone_stamp = now;
  }
}
void DaggerCandidate::updateRequests(const ros::Time& now) {
  for (auto& h : hands) {
    const bool active = h.pressed && fresh(now,h.heartbeat,.2) && fresh(now,h.bone_stamp,.2) && h.bone_stamp >= h.grip_edge;
    if (!active) h.reset();
    else if (!h.requested) { ++h.generation; h.requested = true; }
  }
}
void DaggerCandidate::invalidateFeedback() {
  for (auto& h : hands) h.reset();
  resetWaist();
}
Pose DaggerCandidate::target(int i, const V& tcp_to_wrist) const {
  const auto& h = hands[i];
  Pose out = h.robot_anchor;
  if (incremental_orientation_) out.linear() = h.human.linear() * h.human_anchor.linear().transpose() * h.robot_anchor.linear();
  else out.linear() = h.human.linear();
  // A rotation about the physical wrist must carry the TCP tool offset.
  out.translation() += translation_scale.cwiseProduct(h.human.translation()-h.human_anchor.translation())
      + (h.robot_anchor.linear()-out.linear()) * tcp_to_wrist;
  return out;
}
void DaggerCandidate::body(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->header.stamp > chest_stamp_ && readPose(msg->pose, chest_)) chest_stamp_ = msg->header.stamp;
}
void DaggerCandidate::resetWaist() {
  if (waist_requested_ || waist_ready_) ++waist_generation_;
  waist_requested_ = waist_ready_ = false;
  waist_stamp_ = last_step_ = waist_velocity_until_ = ros::Time();
  waist_velocity_.setZero();
}
void DaggerCandidate::config(const std_msgs::String::ConstPtr& msg) {
  try {
    const auto j = nlohmann::json::parse(msg->data);
    WaistConfig next;
    next.session = j.at("session").get<std::string>();
    next.side = j.at("which_arm").get<std::string>();
    next.max_speed = j.at("max_speed").get<double>();
    next.action_timeout = j.at("action_timeout").get<double>();
    next.signal_timeout = j.at("signal_timeout").get<double>();
    if (j.at("schema_version") != 1 || next.session.empty() ||
        (next.side != "left" && next.side != "right" && next.side != "both") ||
        !(next.max_speed > 0 && next.action_timeout > 0 && next.signal_timeout > 0))
      throw std::invalid_argument("invalid waist configuration");
    for (const char* k : {"lower","upper","lock_mask","lock_position"})
      if (!j.at(k).is_array() || j.at(k).size() != 4) throw std::invalid_argument(k);
    for (int i = 0; i < 4; ++i) {
      next.lower[i] = j.at("lower").at(i).get<double>();
      next.upper[i] = j.at("upper").at(i).get<double>();
      const double lock = j.at("lock_position").at(i).get<double>();
      if (!(next.lower[i] < next.upper[i]) || !std::isfinite(lock)) throw std::invalid_argument("limits");
      if (j.at("lock_mask").at(i).get<bool>()) {
        if (lock < next.lower[i] || lock > next.upper[i]) throw std::invalid_argument("locked position");
        next.lower[i] = next.upper[i] = lock;
      }
    }
    if (next.session != config_.session) { resetWaist(); config_ = next; }
  } catch (const std::exception& e) { ROS_WARN_THROTTLE(1., "Invalid DAgger waist config: %s", e.what()); }
}
void DaggerCandidate::updateWaist(const ros::Time& now, const ros::Time& sensor_stamp,
    const Eigen::Vector4d& measured, const Eigen::Vector4d& low, const Eigen::Vector4d& high,
    const WaistFK& fk) {
  const auto& c = config_;
  const auto active = [&](int i){ return hands[i].pressed && fresh(now,hands[i].heartbeat,c.signal_timeout); };
  const bool request = !c.session.empty() && ((c.side != "right" && active(0)) || (c.side != "left" && active(1)));
  if (!request || !fresh(now,chest_stamp_,c.action_timeout) || !fresh(now,sensor_stamp,c.action_timeout)) {
    resetWaist(); waist_position_ = measured; return;
  }
  waist_requested_ = true;
  const Eigen::Vector4d lo = low.cwiseMax(c.lower), hi = high.cwiseMin(c.upper);
  if ((lo.array() > hi.array()).any()) { waist_ready_ = false; waist_stamp_ = ros::Time(); return; }
  if (!waist_ready_) {
    waist_position_ = measured;
    waist_velocity_.setZero();
    waist_velocity_until_ = ros::Time();
    anchor_feature_ = fk(measured,nullptr);
    chest_anchor_ = chest_;
    ++waist_generation_;
  } else if (chest_stamp_ != last_step_) {
    const double dt = std::clamp((chest_stamp_-last_step_).toSec(), 0., .05);
    const auto displacement = chest_.translation()-chest_anchor_.translation();
    const double heading = yaw(chest_anchor_);
    Eigen::Vector4d goal = anchor_feature_;
    goal[0] += .6*std::clamp(displacement.dot(V(std::cos(heading),std::sin(heading),0.)), -.15,.5);
    goal[1] += std::clamp(displacement.z(),-.2,.3);
    goal[2] += std::clamp(pitch(chest_)-pitch(chest_anchor_),-M_PI/12.,M_PI/6.);
    goal[3] = std::clamp(goal[3]+std::remainder(yaw(chest_)-heading,2*M_PI),-M_PI/4.,M_PI/4.);
    Eigen::Matrix4d J;
    const Eigen::Vector4d error = goal-fk(waist_position_,&J);
    Eigen::VectorXd delta;
    int status = 0;
    const bool ok = waist_solver_.solve(J.transpose()*J+1e-5*Eigen::Matrix4d::Identity(), -J.transpose()*error,
        lo-waist_position_, hi-waist_position_, Eigen::MatrixXd(0,4), Eigen::VectorXd(), Eigen::VectorXd(), &delta,&status);
    if (!ok || !delta.allFinite()) { waist_ready_ = false; waist_stamp_ = ros::Time(); return; }
    const Eigen::Vector4d step = delta.cwiseMax(-c.max_speed*dt).cwiseMin(c.max_speed*dt);
    waist_position_ += step;
    waist_velocity_ = step / dt;  // chest timestamps strictly increase in body().
    waist_velocity_until_ = chest_stamp_ + ros::Duration(std::min(2.5 * dt, c.action_timeout));
  }
  waist_ready_ = true;
  last_step_ = chest_stamp_;
  waist_stamp_ = std::min(sensor_stamp,chest_stamp_);
}
void DaggerCandidate::publish(const ros::Time& now, const ArmVector& q, const ArmVector& dq,
                              const ArmVector& ddq) {
  ++sequence_;
  if (local_sample_pub_.getNumSubscribers()) {
    motion_capture_ik::DaggerArmSample sample;
    sample.header.stamp = now;
    sample.session = session_;
    sample.sequence = sequence_;
    for (int side = 0; side < 2; ++side) {
      sample.generation[side] = hands[side].generation;
      sample.active[side] = hands[side].requested && hands[side].ready;
    }
    std::copy_n(q.data(), 14, sample.position.begin());
    std::copy_n(dq.data(), 14, sample.velocity.begin());
    std::copy_n(ddq.data(), 14, sample.acceleration.begin());
    sample.waist_config_session = config_.session;
    sample.waist_generation = waist_generation_;
    for (int j = 0; j < 4; ++j) {
      sample.waist_position[j] = waist_position_[j];
      if (!config_.session.empty()) {
        sample.waist_lower[j] = config_.lower[j];
        sample.waist_upper[j] = config_.upper[j];
        sample.waist_active[j] = waist_requested_ && waist_ready_ && config_.lower[j] < config_.upper[j];
        if (sample.waist_active[j] && now <= waist_velocity_until_) sample.waist_velocity[j] = waist_velocity_[j];
      }
    }
    local_sample_pub_.publish(sample);
  }
  nlohmann::json packet = {{"schema_version",1},{"session",session_},{"sequence",sequence_},
    {"stamp_ns",now.toNSec()},{"position",values(q)},{"velocity",values(dq)},
    {"waist",{{"config_session",config_.session},{"requested",waist_requested_},{"ready",waist_ready_},
              {"generation",waist_generation_},{"stamp_ns",waist_stamp_.toNSec()},{"position",values(waist_position_)}}}};
  for (int i = 0; i < 2; ++i) {
    const std::string side = i ? "right_" : "left_";
    packet[side+"requested"] = hands[i].requested;
    packet[side+"ready"] = hands[i].ready;
    packet[side+"generation"] = hands[i].generation;
  }
  std_msgs::String message;
  message.data = packet.dump();
  publisher_.publish(message);
}
}  // namespace velocity_ik
