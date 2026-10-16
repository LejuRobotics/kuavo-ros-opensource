#include "wheel_arm_abs_vel/humanoid_wheel_wbc/RneaBackend.h"
#include <pinocchio/algorithm/rnea.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>

namespace ocs2 { namespace mobile_manipulator {
namespace {
double clamp(double x, double low, double high) { return std::max(low,std::min(x,high)); }
bool positiveVector(const vector_t& x, int n) {
  return x.size() == n && x.allFinite() && (x.array() > 0.0).all();
}
}

RneaBackend::RneaBackend(const PinocchioInterface& interface,
                         const ManipulatorModelInfo& info, const RneaSettings& s)
    : interface_(interface), settings_(s) {
  const auto& model = interface_.getModel();
  const int n = info.armDim;
  if (info.manipulatorModelType != ManipulatorModelType::WheelWorldBasedMobileManipulator ||
      model.nq != model.nv || model.nv != n + 3 || info.stateDim != model.nq ||
      info.inputDim != model.nv || info.dofNames.size() != static_cast<size_t>(n))
    throw std::invalid_argument("RNEA requires the world XY/yaw + scalar-joint model");
  if (s.mode != "gravity" && s.mode != "bias" && s.mode != "full")
    throw std::invalid_argument("rnea mode must be gravity, bias or full");
  if (!positiveVector(s.torqueLimit,n) || !positiveVector(s.torqueRateLimit,n) ||
      !positiveVector(s.accelerationLimit,n) || !positiveVector(s.jerkLimit,n) ||
      !std::isfinite(s.velocityCutoffHz) || s.velocityCutoffHz <= 0 ||
      !std::isfinite(s.maxMeasurementAge) || s.maxMeasurementAge <= 0 ||
      !std::isfinite(s.maxPeriod) || s.maxPeriod <= 0 ||
      !std::isfinite(s.dynamicRampSeconds) || s.dynamicRampSeconds <= 0)
    throw std::invalid_argument("invalid RNEA bounds/filter/timing configuration");
  std::set<int> seen;
  for (int i=0;i<n;++i) {
    if (!model.existJointName(info.dofNames[i]))
      throw std::invalid_argument("missing RNEA joint: " + info.dofNames[i]);
    const auto id = model.getJointId(info.dofNames[i]);
    const auto& joint = model.joints[id];
    // The controller's final reference uses this same scalar joint ordering.
    if (joint.nq()!=1 || joint.nv()!=1 || joint.idx_q()!=i+3 || joint.idx_v()!=i+3 ||
        !seen.insert(joint.idx_v()).second)
      throw std::invalid_argument("RNEA joint ordering/type mismatch: " + info.dofNames[i]);
    velocityIndices_.push_back(joint.idx_v());
  }
  q_=v_=a_=filteredV_=vector_t::Zero(model.nv);
  filteredA_=previousTorque_=gravity_=bias_=full_=vector_t::Zero(n);
}

DynamicsResult RneaBackend::update(const DynamicsReference& r, const SystemObservation& m,
                                   double dt, double measurementAge) {
  DynamicsResult out;
  const auto& model = interface_.getModel();
  auto& data = interface_.getData();
  const int n = velocityIndices_.size();
  if (!std::isfinite(dt) || dt<=0 || dt>settings_.maxPeriod ||
      !std::isfinite(measurementAge) || measurementAge < -0.02 ||
      measurementAge > settings_.maxMeasurementAge) {
    out.reason="stale_measurement_or_invalid_period"; return out;
  }
  if (m.state.size()!=model.nq || m.input.size()!=model.nv ||
      r.q.size()!=model.nq || r.v.size()!=model.nv ||
      !m.state.allFinite() || !m.input.allFinite() || !r.q.allFinite() || !r.v.allFinite()) {
    out.reason="invalid_state_or_reference"; return out;
  }
  if (r.accelerationValid && (r.acceleration.size()!=n || !r.acceleration.allFinite())) {
    out.reason="invalid_acceleration"; return out;
  }
  q_=m.state;
  // This backend is explicitly scoped to a frozen base on level ground.
  q_.head(3).setZero();
  v_=m.input; v_.head(3).setZero();
  if (!initialized_) filteredV_=v_;
  const double beta=-std::expm1(-2.0*3.141592653589793*settings_.velocityCutoffHz*dt);
  filteredV_ += beta*(v_-filteredV_);
  elapsed_ += dt;
  const double ramp=std::min(1.0,elapsed_/settings_.dynamicRampSeconds);
  a_.setZero();
  for(int j=0;j<n;++j) {
    double desired = (settings_.mode=="full" && r.accelerationValid) ? r.acceleration[j] : 0.0;
    desired=clamp(desired,-settings_.accelerationLimit[j],settings_.accelerationLimit[j]);
    // Ramp the target BEFORE slew limiting: multiplying the already limited
    // acceleration by a changing ramp would add an unbounded extra jerk term.
    desired*=ramp;
    const double step=settings_.jerkLimit[j]*dt;
    filteredA_[j] += clamp(desired-filteredA_[j],-step,step);
    a_[velocityIndices_[j]]=filteredA_[j];
  }
  // Copy each result before the next call: Pinocchio returns references into Data.
  const vector_t g=pinocchio::computeGeneralizedGravity(model,data,q_);
  for(int j=0;j<n;++j) gravity_[j]=g[velocityIndices_[j]];
  bias_=gravity_; full_=gravity_;
  if(settings_.mode!="gravity") {
    const vector_t b=pinocchio::rnea(model,data,q_,filteredV_,vector_t::Zero(model.nv));
    for(int j=0;j<n;++j) bias_[j]=b[velocityIndices_[j]];
    full_=bias_;
    if(settings_.mode=="full") {
      const vector_t f=pinocchio::rnea(model,data,q_,filteredV_,a_);
      for(int j=0;j<n;++j) full_[j]=f[velocityIndices_[j]];
    }
  }
  // Ramp only dynamic terms; do not attenuate gravity support at startup.
  out.rawTorque=gravity_+ramp*(bias_-gravity_)+(full_-bias_);
  if(!out.rawTorque.allFinite()) {out.reason="nonfinite_rnea_output";return out;}
  out.torque=out.rawTorque;
  for(int j=0;j<n;++j) {
    const double bounded=clamp(out.rawTorque[j],-settings_.torqueLimit[j],settings_.torqueLimit[j]);
    out.torqueLimited |= bounded!=out.rawTorque[j];
    // First command seeds gravity support; subsequent commands are slew-limited.
    if(!initialized_) previousTorque_[j]=clamp(gravity_[j],-settings_.torqueLimit[j],settings_.torqueLimit[j]);
    const double step=settings_.torqueRateLimit[j]*dt;
    out.torque[j]=previousTorque_[j]+clamp(bounded-previousTorque_[j],-step,step);
    out.rateLimited |= std::abs(out.torque[j]-bounded)>1e-10;
  }
  previousTorque_=out.torque;
  initialized_=true;
  saturationSeconds_ = out.torqueLimited ? saturationSeconds_+dt : 0.0;
  out.saturationSeconds=saturationSeconds_;
  out.valid=true; out.reason="ok";
  return out;
}
}}
