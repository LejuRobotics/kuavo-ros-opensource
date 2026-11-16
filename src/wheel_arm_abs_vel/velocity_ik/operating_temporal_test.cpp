#include "operating_envelope.h"
#include "temporal_cost.h"
#include "weighted_qp.h"
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_motion_policy.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <random>

using namespace velocity_ik;
int main() {
  std::mt19937 random(20260921);
  std::uniform_real_distribution<double> unit(-1., 1.);
  // Arbitrary hard and soft boxes, not only boxes straddling zero.
  for (int k = 0; k < 100000; ++k) {
    double l = 8 * unit(random), u = l + 4 * std::abs(unit(random));
    double sl = 8 * unit(random), su = sl + 4 * std::abs(unit(random));
    if (k % 10 == 0) u = l;
    const auto r = feasibleOperatingInterval(l, u, sl, su);
    assert(r.valid && r.lower >= l && r.upper <= u);
    assert(r.lower <= r.baseline && r.baseline <= r.upper);
    assert(std::clamp(0., r.lower, r.upper) == std::clamp(0., l, u));
    // Multiple preferred torso rows must admit this very same baseline.
    for (double c : {-3., -.1, .1, 2.}) {
      double cl = unit(random), cu = cl + std::abs(unit(random));
      cl = std::min(cl, c * r.baseline); cu = std::max(cu, c * r.baseline);
      assert(c * r.baseline >= cl && c * r.baseline <= cu);
    }
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(!feasibleOperatingInterval(1., -1., 0., 0.).valid);
  assert(!feasibleOperatingInterval(nan, 1., 0., 0.).valid);
  assert(!feasibleOperatingInterval(0., std::numeric_limits<double>::infinity(), 0., 0.).valid);
  OperatingJoint p;
  p.lower = -1.2; p.upper = .7; p.q = .55; p.v = .3;
  p.vmax = 4; p.acceleration = 50; p.braking = 25; p.outward = 50;
  p.delay = .02; p.lower_guard = p.upper_guard = .02;
  p.extra_position = .03; p.extra_time = .08;
  for (bool separate : {false, true}) {
    p.separate_stop = separate;
    for (int k = 0; k < 1000; ++k) {
      double speed = 4. * k / 999.;
      assert(operatingStopReserve(p, speed, true, true) >=
             operatingStopReserve(p, speed, true, false));
    }
    const double comfort = predictiveOutwardCap(p, true);
    auto hard = p; hard.extra_position = hard.extra_time = 0.;
    assert(comfort < predictiveOutwardCap(hard, true));
    const auto r = operatingInterval(p, .2, .4);
    assert(r.valid && r.lower <= .2 && r.upper >= .2);
    p.upper_scale = 0.;
    const auto escaping = operatingInterval(p, -.4, .4);
    assert(escaping.lower < 0 && escaping.upper == 0);
    p.upper_scale = 1.;
  }
  // Approved lower ready endpoint: soft preference never excludes inward
  // escape or the exact stationary endpoint. The opposite side still acts.
  const auto ready = motion_capture_ik::whole_body_ik::readyJointEnvelope(0., 1., 0., .01, .02);
  p.lower = ready.lower; p.upper = ready.upper; p.q = p.v = 0.;
  p.lower_guard = ready.lower_stopping_guard; p.lower_comfort = false;
  auto r = operatingInterval(p, 0., .1);
  assert(r.valid && r.lower == 0 && r.upper > 0);
  p.extra_position = -1.; assert(!operatingInterval(p, 0., .1).valid);
  p.extra_position = nan; assert(!operatingInterval(p, 0., .1).valid);
  {
    TemporalCost invalid{-1., 0., 1., 1.};
    assert(!validTemporalCost(invalid));
    invalid.acceleration_weight = std::numeric_limits<double>::infinity();
    assert(!validTemporalCost(invalid));
    invalid.acceleration_weight = 0.; invalid.delta_acceleration_scale = 0.;
    assert(!validTemporalCost(invalid));
  }

  for (int k = 0; k < 10000; ++k) {
    TemporalCost t{.03, .01, 50., 20.};
    const double dt = .002, v = unit(random), a = 30 * unit(random), u = unit(random);
    double h = 0, g = 0;
    assert(addTemporalCost(t, dt, v, a, &h, &g) && h >= 0.);
    const auto cost = [&](double x) {
      double acc = (x - v) / dt;
      return temporalAccelerationCost(t, acc) + temporalDeltaAccelerationCost(t, acc - a);
    };
    assert(std::abs((cost(u) - cost(0)) - (.5 * h * u * u + g * u)) < 1.e-8);
    double oldh = h, oldg = g;
    t.acceleration_weight = t.delta_acceleration_weight = 0.;
    assert(addTemporalCost(t, dt, v, a, &h, &g));
    assert(std::memcmp(&h, &oldh, sizeof(h)) == 0 && std::memcmp(&g, &oldg, sizeof(g)) == 0);
    assert(!addTemporalCost(t, 0., v, a, &h, &g));
    t.acceleration_scale = nan; assert(!validTemporalCost(t));
  }
  // Actual 18x4 solver family, simultaneous box/torso activity and both costs.
  WeightedQp qp; qp.prepareWorkspaces({{18, 4}});
  for (int mode = 0; mode < 4; ++mode) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(18, 18), C = Eigen::MatrixXd::Zero(4,18);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(18, -2.), lo = Eigen::VectorXd::Constant(18, -.1), hi = -lo;
    for (int i = 0; i < 4; ++i) { C(i,i) = 1; C(i, i+1) = -.5; }
    auto cl = Eigen::VectorXd::Constant(4, -.05).eval(), cu = (-cl).eval();
    for (int i = 0; i < 18; ++i) {
      if (mode & 1) { auto b = feasibleOperatingInterval(lo(i), hi(i), -.03, .04); lo(i)=b.lower; hi(i)=b.upper; }
      if (mode & 2) assert(addTemporalCost({.003,.001,50.,50.}, .002, 0., 0., &H(i,i), &g(i)));
    }
    Eigen::VectorXd x; int status; WeightedQp::Stats stats;
    assert(qp.solve(H, g, lo, hi, C, cl, cu, &x, &status, &stats));
    assert(stats.accepted && !stats.workspace_created);
    assert((x.array() >= lo.array()-2.e-8).all() && (x.array() <= hi.array()+2.e-8).all());
  }
  std::cout << "operating/temporal: 100000 feasibility properties, 10000 cost checks, B0-B3 QPs PASS\n";
}
