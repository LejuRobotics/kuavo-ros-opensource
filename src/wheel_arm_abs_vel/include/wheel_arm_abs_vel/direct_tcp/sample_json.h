#pragma once

#include "wheel_arm_abs_vel/direct_tcp/mapper.h"

namespace wheel_arm_abs_vel::direct_tcp {
// Decode [position xyz, quaternion xyzw, linear xyz, angular xyz]. Extract
// scalars before constructing Eigen vectors: JSON's implicit conversions can
// otherwise select Eigen 3.4's nested initializer_list constructor.
inline Sample sampleFromJson(const Json& values) {
  const std::array<double, 7> pose{{
      values.at(0).get<double>(), values.at(1).get<double>(),
      values.at(2).get<double>(), values.at(3).get<double>(),
      values.at(4).get<double>(), values.at(5).get<double>(),
      values.at(6).get<double>()}};
  const Vec linear(values.at(7).get<double>(), values.at(8).get<double>(),
                   values.at(9).get<double>());
  const Vec angular(values.at(10).get<double>(), values.at(11).get<double>(),
                    values.at(12).get<double>());
  return sample(pose, linear, angular);
}
}  // namespace wheel_arm_abs_vel::direct_tcp
