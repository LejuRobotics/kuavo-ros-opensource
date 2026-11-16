#include "wheel_arm_abs_vel/direct_tcp/sample_json.h"
#include <gtest/gtest.h>

using namespace wheel_arm_abs_vel::direct_tcp;

TEST(Inputs,JsonSamplePreservesPoseAndTwist) {
  const Json values = {0.1, -2, 3, 0, 0, 0, 1, 0.25, -0.5, 2, 4, -5, 6.5};
  const auto decoded = sampleFromJson(values);
  EXPECT_TRUE(decoded.pose.translation().isApprox(Vec(0.1, -2, 3)));
  EXPECT_TRUE(decoded.pose.linear().isApprox(Rot::Identity()));
  EXPECT_TRUE(decoded.linear.isApprox(Vec(0.25, -0.5, 2)));
  EXPECT_TRUE(decoded.angular.isApprox(Vec(4, -5, 6.5)));
}
TEST(Inputs,JsonSampleRejectsInvalidFields) {
  Json values = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  values[8] = "invalid";
  EXPECT_THROW(sampleFromJson(values), Json::type_error);
  values[8] = 0;
  values.erase(values.end() - 1);
  EXPECT_THROW(sampleFromJson(values), Json::out_of_range);
}
TEST(Inputs,JsonSampleRetainsQuaternionValidation) {
  const Json values = {0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0};
  EXPECT_THROW(sampleFromJson(values), std::invalid_argument);
}
