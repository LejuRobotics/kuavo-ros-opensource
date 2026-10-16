#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_recruitment.h"

#include <array>
#include <limits>

#include <gtest/gtest.h>

namespace wbik = motion_capture_ik::whole_body_ik;

TEST(WholeBodyRecruitment, SmoothStepHasExactContinuousEndpoints) {
  EXPECT_DOUBLE_EQ(wbik::smoothStep01(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(wbik::smoothStep01(0.0), 0.0);
  EXPECT_DOUBLE_EQ(wbik::smoothStep01(0.5), 0.5);
  EXPECT_DOUBLE_EQ(wbik::smoothStep01(1.0), 1.0);
  EXPECT_DOUBLE_EQ(wbik::smoothStep01(2.0), 1.0);

  const double epsilon = 1.0e-6;
  EXPECT_NEAR(wbik::smoothStep01(epsilon) / epsilon, 0.0, 1.0e-5);
  EXPECT_NEAR((1.0 - wbik::smoothStep01(1.0 - epsilon)) / epsilon,
              0.0, 1.0e-5);
}

TEST(WholeBodyRecruitment, ArmPressureUsesWorstControlledArmCondition) {
  const wbik::ArmPressure comfortable = wbik::calculateArmPressure(
      0.50, 0.58, 0.68, 0.15, 0.10, 0.025, 0.50, 0.25, 0.05);
  EXPECT_DOUBLE_EQ(comfortable.total, 0.0);

  const wbik::ArmPressure stressed = wbik::calculateArmPressure(
      0.63, 0.58, 0.68, 0.08, 0.10, 0.025, 0.10, 0.25, 0.05);
  EXPECT_GT(stressed.total, 0.0);
  EXPECT_LT(stressed.total, 1.0);
  EXPECT_DOUBLE_EQ(
      stressed.total,
      std::max({stressed.reach, stressed.singularity, stressed.joint}));

  const wbik::ArmPressure invalid = wbik::calculateArmPressure(
      std::numeric_limits<double>::quiet_NaN(), 0.58, 0.68,
      0.15, 0.10, 0.025, 0.50, 0.25, 0.05);
  EXPECT_DOUBLE_EQ(invalid.reach, 1.0);
  EXPECT_DOUBLE_EQ(invalid.total, 1.0);
}

TEST(WholeBodyRecruitment, LowerBodyPrecedesBaseContinuously) {
  std::array<wbik::ArmPressure, 2> pressure{};
  const std::array<bool, 2> controlled{true, true};

  pressure.at(0).total = 0.10;
  auto recruitment = wbik::calculateRecruitment(
      pressure, controlled, 0.20, 0.55, 0.60, 0.90, true);
  EXPECT_DOUBLE_EQ(recruitment.lower_body, 0.0);
  EXPECT_DOUBLE_EQ(recruitment.base, 0.0);

  pressure.at(1).total = 0.40;
  recruitment = wbik::calculateRecruitment(
      pressure, controlled, 0.20, 0.55, 0.60, 0.90, true);
  EXPECT_GT(recruitment.lower_body, 0.0);
  EXPECT_LT(recruitment.lower_body, 1.0);
  EXPECT_DOUBLE_EQ(recruitment.base, 0.0);

  pressure.at(1).total = 0.75;
  recruitment = wbik::calculateRecruitment(
      pressure, controlled, 0.20, 0.55, 0.60, 0.90, true);
  EXPECT_DOUBLE_EQ(recruitment.lower_body, 1.0);
  EXPECT_GT(recruitment.base, 0.0);
  EXPECT_LT(recruitment.base, 1.0);

  pressure.at(1).total = 1.0;
  recruitment = wbik::calculateRecruitment(
      pressure, controlled, 0.20, 0.55, 0.60, 0.90, true);
  EXPECT_DOUBLE_EQ(recruitment.lower_body, 1.0);
  EXPECT_DOUBLE_EQ(recruitment.base, 1.0);
}

TEST(WholeBodyRecruitment, DisabledAndUncontrolledArmsCannotRecruit) {
  std::array<wbik::ArmPressure, 2> pressure{};
  pressure.at(0).total = 1.0;
  pressure.at(1).total = 1.0;

  auto recruitment = wbik::calculateRecruitment(
      pressure, std::array<bool, 2>{false, false},
      0.20, 0.55, 0.60, 0.90, true);
  EXPECT_DOUBLE_EQ(recruitment.pressure, 0.0);
  EXPECT_DOUBLE_EQ(recruitment.lower_body, 0.0);
  EXPECT_DOUBLE_EQ(recruitment.base, 0.0);

  recruitment = wbik::calculateRecruitment(
      pressure, std::array<bool, 2>{true, true},
      0.20, 0.55, 0.60, 0.90, false);
  EXPECT_DOUBLE_EQ(recruitment.pressure, 0.0);
  EXPECT_DOUBLE_EQ(recruitment.lower_body, 0.0);
  EXPECT_DOUBLE_EQ(recruitment.base, 0.0);
}

TEST(WholeBodyRecruitment, SlewRateBoundsBothDirections) {
  EXPECT_NEAR(wbik::slewRecruitment(0.0, 1.0, 2.0, 1.0, 0.01),
              0.02, 1.0e-12);
  EXPECT_NEAR(wbik::slewRecruitment(0.8, 0.0, 2.0, 1.0, 0.01),
              0.79, 1.0e-12);
  EXPECT_DOUBLE_EQ(
      wbik::slewRecruitment(0.4, 0.5, 2.0, 1.0, 0.0), 0.4);
}
