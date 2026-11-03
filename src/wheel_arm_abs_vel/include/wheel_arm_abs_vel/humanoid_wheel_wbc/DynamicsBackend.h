#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <cstdint>

namespace ocs2 { namespace mobile_manipulator {

// One cycle's FINAL reference. Acceleration is admitted only after q/dq limiters.
// The caller applies the configured acceleration gain, never a total-torque gain.
struct DynamicsReference {
  vector_t q, v, acceleration;
  bool accelerationValid = false;
  bool wholeBodyAcceleration = false;
  uint64_t session = 0, generation = 0, sequence = 0, epoch = 0;
  bool atomicReference = false;
  double accelerationGain = 0.0;
};

struct DynamicsResult {
  vector_t torque, rawTorque, solvedAcceleration;
  bool valid = false;
  bool torqueLimited = false, rateLimited = false;
  double saturationSeconds = 0.0;
  const char* reason = "uninitialized";
};

class DynamicsBackend {
 public:
  virtual ~DynamicsBackend() = default;
  virtual const char* name() const = 0;
  virtual DynamicsResult update(const DynamicsReference& reference,
                                const SystemObservation& measured,
                                double dt, double measurementAge) = 0;
};

}}  // namespace ocs2::mobile_manipulator
