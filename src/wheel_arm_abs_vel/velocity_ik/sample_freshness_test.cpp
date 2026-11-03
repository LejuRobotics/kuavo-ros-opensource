#include "sample_freshness.h"
#include <cassert>
#include <iostream>
int main() {
  using velocity_ik::transportAge;
  using velocity_ik::MeasurementFreshness;
  assert(transportAge(1000000000, 1500000000) == .5);
  assert(!std::isfinite(transportAge(0, 1)));
  assert(!std::isfinite(transportAge(200, 100)));
  MeasurementFreshness f;
  assert(f.observe(1000000000, 1010000000, 2000000000, .1));
  assert(!f.observe(1000000000, 1020000000, 2010000000, .1)); // duplicate stamp
  assert(!f.observe(999000000, 1020000000, 2010000000, .1)); // out of order
  assert(f.age(2100000000) > .1); // duplicates cannot renew the lease
  assert(!f.observe(1100000000, 1500000000, 2200000000, .1)); // queued old sample
  assert(!f.observe(1600000000, 1500000000, 2200000000, .1)); // future
  assert(f.observe(1500000000, 1500000000, 2200000000, .1));
  assert(!std::isfinite(f.age(2199999999))); // steady clock regression
  assert(f.observe(1510000000, 1510000000, 2210000000, .1)); // stationary pose allowed
  std::cout << "PASS transport delay, duplicate/out-of-order measurements, expiry and clock guards\n";
}
