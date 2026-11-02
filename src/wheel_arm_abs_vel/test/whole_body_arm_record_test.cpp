#include <gtest/gtest.h>
#include <limits>
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_arm_record.h"
#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_dof_layout.h"

namespace {
wheel_arm_abs_vel::WholeBodyIkReference sample() {
  wheel_arm_abs_vel::WholeBodyIkReference msg;
  msg.valid = true;
  msg.takeover_requested = true;
  msg.header.seq = 42;
  msg.header.stamp = ros::Time(123, 456);
  msg.dof_names = motion_capture_ik::whole_body_ik::DofLayout::WholeBodyStage().decisionNames();
  msg.q.assign(21, 0.0);
  msg.dq.assign(21, 0.0);
  for (int i = 0; i < 14; ++i) {
    msg.q[i + 7] = (i - 7) * 3.14159265358979323846 / 180.0;
    msg.dq[i + 7] = (14 - i) * 3.14159265358979323846 / 180.0;
  }
  return msg;
}

}  // namespace

TEST(WholeBodyArmRecord, LegacyUnitsOrderAndTimestamp) {
  const auto msg = sample();
  sensor_msgs::JointState out;
  ASSERT_TRUE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  ASSERT_EQ(out.name.size(), 14u);
  ASSERT_EQ(out.position.size(), 14u);
  ASSERT_EQ(out.velocity.size(), 14u);
  ASSERT_EQ(out.effort.size(), 14u);
  EXPECT_EQ(out.header.stamp, msg.header.stamp);
  EXPECT_EQ(out.header.seq, 42u);
  EXPECT_EQ(out.header.frame_id, wheel_arm_abs_vel::arm_record::kFrameId);
  for (int i = 0; i < 14; ++i) {
    EXPECT_EQ(out.name[i], "arm_joint_" + std::to_string(i + 1));
    EXPECT_NEAR(out.position[i], i - 7, 1e-12);
    EXPECT_NEAR(out.velocity[i], 14 - i, 1e-12);
    EXPECT_EQ(out.effort[i], 0.0);
  }
}

TEST(WholeBodyArmRecord, ResolvesNamesInsteadOfAssumingOffsets) {
  auto msg = sample();
  std::reverse(msg.dof_names.begin(), msg.dof_names.end());
  std::reverse(msg.q.begin(), msg.q.end());
  std::reverse(msg.dq.begin(), msg.dq.end());
  sensor_msgs::JointState out;
  ASSERT_TRUE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  EXPECT_NEAR(out.position.front(), -7.0, 1e-12);
  EXPECT_NEAR(out.position.back(), 6.0, 1e-12);
}

TEST(WholeBodyArmRecord, LowerBodyLegacyOrderAndUnits) {
  auto msg = sample();
  for (int i = 0; i < 4; ++i) {
    msg.q[3 + i] = (i + 1) * 3.14159265358979323846 / 180.0;
    msg.dq[3 + i] = -(i + 1) * 3.14159265358979323846 / 180.0;
  }
  sensor_msgs::JointState out;
  ASSERT_TRUE(wheel_arm_abs_vel::arm_record::convertLowerBody(msg, out));
  ASSERT_EQ(out.name.size(), 4u);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(out.name[i], "joint" + std::to_string(i + 1));
    EXPECT_NEAR(out.position[i], i + 1, 1e-12);
    EXPECT_NEAR(out.velocity[i], -(i + 1), 1e-12);
    EXPECT_EQ(out.effort[i], 0.0);
  }
  EXPECT_EQ(out.header.stamp, msg.header.stamp);
  EXPECT_EQ(out.header.frame_id, wheel_arm_abs_vel::arm_record::kFrameId);
}

TEST(WholeBodyArmRecord, RejectsInvalidAndMalformedReferences) {
  sensor_msgs::JointState out;
  auto msg = sample();
  msg.valid = false;
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.takeover_requested = false;
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.dq.pop_back();
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.dof_names[7] = "missing_left_j1";
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.dof_names[0] = msg.dof_names[7];
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.q[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
  msg = sample(); msg.dq[20] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(wheel_arm_abs_vel::arm_record::convert(msg, out));
}
