#include <gtest/gtest.h>
#include <limits>
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_motion_policy.h"

using namespace motion_capture_ik::whole_body_ik;

TEST(MotionPolicy, PeakIsRadiansAndNeverDefinesOperatingSpeed) {
  EXPECT_DOUBLE_EQ(effectiveDriveVelocity(2.0, 2.0, 22.5), 2.0);
  EXPECT_DOUBLE_EQ(effectiveDriveVelocity(2.0, 0.38, 22.5), 0.38);
  EXPECT_DOUBLE_EQ(effectiveDriveVelocity(100.0, 100.0, 22.5), 22.5);
  EXPECT_THROW(effectiveDriveVelocity(2.0, 2.0, -22.5), std::invalid_argument);
  EXPECT_THROW(effectiveDriveVelocity(0.0, 2.0, 22.5), std::invalid_argument);
}

TEST(MotionPolicy, CostDoesNotDependOnOperatingVelocity) {
  const double cost = driveRegularization(6.0, 1.0);
  for (double cap : {0.14, 0.30, 2.0}) {
    EXPECT_GT(effectiveDriveVelocity(cap, cap, 22.5), 0.0);
    EXPECT_DOUBLE_EQ(driveRegularization(6.0, 1.0), cost);
  }
  EXPECT_DOUBLE_EQ(cost / driveRegularization(1.0, 1.0), 6.0);
  EXPECT_THROW(driveRegularization(1.0, 0.0), std::invalid_argument);
}

TEST(MotionPolicy, KneeAndLegLowEndpointsAreReachable) {
  const auto knee = readyJointEnvelope(0.0, 1.448623, 0.0, 0.01, 0.02);
  EXPECT_DOUBLE_EQ(knee.lower, 0.0);
  EXPECT_DOUBLE_EQ(knee.lower_stopping_guard, 0.0);
  EXPECT_DOUBLE_EQ(knee.upper_stopping_guard, 0.02);
  const auto leg = readyJointEnvelope(-2.809980, 0.0, 0.0, 0.01, 0.02);
  EXPECT_DOUBLE_EQ(leg.upper, 0.0);
  EXPECT_DOUBLE_EQ(leg.upper_stopping_guard, 0.0);
  EXPECT_DOUBLE_EQ(leg.lower_stopping_guard, 0.02);
}

TEST(MotionPolicy, InteriorReadyDoesNotRemoveOrdinaryGuards) {
  const auto e = readyJointEnvelope(-1.0, 1.0, 0.0, 0.01, 0.02);
  EXPECT_DOUBLE_EQ(e.lower, -0.99);
  EXPECT_DOUBLE_EQ(e.upper, 0.99);
  EXPECT_DOUBLE_EQ(e.lower_stopping_guard, 0.02);
  EXPECT_DOUBLE_EQ(e.upper_stopping_guard, 0.02);
}

TEST(MotionPolicy, InvalidReadyMustNotBeSilentlyRaisedOrClamped) {
  EXPECT_THROW(readyJointEnvelope(0.0, 1.0, -0.1, 0.01, 0.02), std::invalid_argument);
  EXPECT_THROW(readyJointEnvelope(-1.0, 0.0, 0.1, 0.01, 0.02), std::invalid_argument);
  EXPECT_THROW(readyJointEnvelope(0.0, 1.0,
      std::numeric_limits<double>::quiet_NaN(), 0.01, 0.02), std::invalid_argument);
}
