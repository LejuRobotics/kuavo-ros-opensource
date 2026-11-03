#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace velocity_ik {

// A reference governor, NOT a plant stabilizer or a torque safety certificate.
// Only current measured state and last committed reference/state are used.
struct ExecutionGovernorConfig {
  double error_deadband{0.20};             // rad, no permanent speed reduction
  double error_full{0.50};                 // rad, full additional smoothing
  double prediction_time{0.04};           // s, no target delay is introduced
  double penalty_ratio{4.0};              // relative to existing QP curvature
  double smoothing_ratio{0.25};           // bounded extra delta-accel cost
  double attack_time{0.01}, release_time{0.04};
};

inline bool validExecutionGovernorConfig(const ExecutionGovernorConfig& p) {
  return std::isfinite(p.error_deadband) && p.error_deadband > 0 &&
      std::isfinite(p.error_full) && p.error_full > p.error_deadband &&
      std::isfinite(p.prediction_time) && p.prediction_time > 0 &&
      std::isfinite(p.penalty_ratio) && p.penalty_ratio > 0 &&
      std::isfinite(p.smoothing_ratio) && p.smoothing_ratio >= 0 &&
      std::isfinite(p.attack_time) && p.attack_time > 0 &&
      std::isfinite(p.release_time) && p.release_time > 0;
}

struct ExecutionGovernorRow {
  int index{-1};                         // physical QP index, never a wire index
  double error{0}, relative_velocity{0}, predicted_error{0};
  double lower{0}, upper{0};             // zero-containing SOFT interval
  double risk{0}, smoothing{0}, smooth_target{0};
  double penalty_curvature{0}, smoothing_curvature{0};
};

inline ExecutionGovernorRow executionGovernorRow(
    const ExecutionGovernorConfig& p, int index, double q, double v, double a,
    double measured_q, double measured_v, double dt, double previous_risk,
    double task_curvature) {
  if (!validExecutionGovernorConfig(p) || !std::isfinite(q) ||
      !std::isfinite(v) || !std::isfinite(a) || !std::isfinite(measured_q) ||
      !std::isfinite(measured_v) || !std::isfinite(dt) || dt <= 0 ||
      !std::isfinite(previous_risk) || !std::isfinite(task_curvature) ||
      task_curvature <= 0 || index < 0)
    throw std::invalid_argument("invalid execution governor input");
  ExecutionGovernorRow r;
  r.index = index;
  r.error = q - measured_q;
  r.relative_velocity = v - measured_v;
  r.predicted_error = r.error + p.prediction_time * r.relative_velocity;
  // Project the predictive tube to include zero. Plant overshoot must NEVER
  // force the reference to follow the measured oscillation. Recovery motion
  // toward the tube remains unpenalized; crossing the opposite face does not.
  r.lower = std::min(0., measured_v + (-p.error_deadband-r.error)/p.prediction_time);
  r.upper = std::max(0., measured_v + ( p.error_deadband-r.error)/p.prediction_time);
  const double x = std::clamp((std::max(std::abs(r.error), std::abs(r.predicted_error))-
      p.error_deadband)/(p.error_full-p.error_deadband), 0., 1.);
  const double desired = x*x*(3.-2.*x);
  const double tau = desired > previous_risk ? p.attack_time : p.release_time;
  r.risk = std::clamp(previous_risk + (-std::expm1(-dt/tau))*(desired-previous_risk),0.,1.);
  // Extra smoothing is absent in the healthy band and on recovery/braking.
  // Do not turn the release memory into a slow-motion latch.
  const double widening = std::max(0., r.error*r.relative_velocity);
  const double outward = std::max(0., r.error*v);
  const double gate = widening/(widening+0.02) * outward/(outward+0.02);
  r.smoothing = r.risk * desired * gate;
  r.smooth_target = v + dt*a;
  r.penalty_curvature = p.penalty_ratio*task_curvature;
  r.smoothing_curvature = p.smoothing_ratio*r.smoothing*task_curvature;
  return r;
}

// Exact squared-hinge penalty with two nonnegative slack variables per joint.
// Original hard bounds/rows are unchanged, so the soft governor cannot make a
// feasible physical QP infeasible. No clipping after the solve, no second IK.
inline void augmentExecutionGovernorQp(
    const std::vector<ExecutionGovernorRow>& rows, Eigen::MatrixXd* h,
    Eigen::VectorXd* g, Eigen::VectorXd* lo, Eigen::VectorXd* hi,
    Eigen::MatrixXd* c, Eigen::VectorXd* cl, Eigen::VectorXd* cu) {
  if (rows.empty()) return;
  const int n=g->size(), m=c->rows(), extra=2*rows.size();
  h->conservativeResize(n+extra,n+extra);
  h->rightCols(extra).setZero(); h->bottomRows(extra).setZero();
  g->conservativeResize(n+extra); g->tail(extra).setZero();
  lo->conservativeResize(n+extra); lo->tail(extra).setZero();
  hi->conservativeResize(n+extra); hi->tail(extra).setConstant(1e20);
  c->conservativeResize(m+extra,n+extra);
  c->rightCols(extra).setZero(); c->bottomRows(extra).setZero();
  cl->conservativeResize(m+extra); cl->tail(extra).setConstant(-1e20);
  cu->conservativeResize(m+extra);
  for (int j=0;j<static_cast<int>(rows.size());++j) {
    const auto& r=rows[j]; const int s=n+2*j, k=m+2*j, i=r.index;
    if (i<0 || i>=n) throw std::invalid_argument("governor QP index out of range");
    (*h)(s,s)=(*h)(s+1,s+1)=r.penalty_curvature;
    (*h)(i,i)+=r.smoothing_curvature;
    (*g)(i)-=r.smoothing_curvature*r.smooth_target;
    (*c)(k,i)=1.; (*c)(k,s)=-1.; (*cu)(k)=r.upper;
    (*c)(k+1,i)=-1.; (*c)(k+1,s+1)=-1.; (*cu)(k+1)=-r.lower;
  }
}
}  // namespace velocity_ik
