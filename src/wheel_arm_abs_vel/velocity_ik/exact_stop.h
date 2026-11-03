#pragma once
#include <Eigen/Core>
#include <cmath>
namespace velocity_ik {
// Bisection may approach an accepted zero endpoint without representing it.
// Seed it exactly only when it belongs to the original bracket and the
// ORIGINAL stopping predicate explicitly certifies it. No tolerance widening.
inline double certifiedUpperZeroSeed(double lo,double hi,bool zero_valid) {
 return lo<=0. && hi>=0. && zero_valid ? 0. : lo;
}
inline double certifiedLowerZeroSeed(double lo,double hi,bool zero_valid) {
 return lo<=0. && hi>=0. && zero_valid ? 0. : hi;
}
}
