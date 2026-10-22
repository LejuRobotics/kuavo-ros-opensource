#include "wheel_arm_abs_vel/motion_capture_ik/whole_body_dof_layout.h"

#include <gtest/gtest.h>

using motion_capture_ik::whole_body_ik::DofGroup;
using motion_capture_ik::whole_body_ik::DofLayout;
using motion_capture_ik::whole_body_ik::kBaseDof;
using motion_capture_ik::whole_body_ik::kDualArmDof;
using motion_capture_ik::whole_body_ik::kLowerBodyDof;
using motion_capture_ik::whole_body_ik::kWholeBodyDof;

TEST(WholeBodyDofLayout, ArmOnlyStageKeepsFullTwentyOneDofContract) {
  const DofLayout layout = DofLayout::ArmOnlyStage();

  EXPECT_EQ(layout.fullDofCount(), kWholeBodyDof);
  EXPECT_EQ(layout.decisionDofCount(), kDualArmDof);
  EXPECT_EQ(layout.fullSpec(0).name, "base_x");
  EXPECT_EQ(layout.fullSpec(3).name, "knee_joint");
  EXPECT_EQ(layout.fullSpec(7).name, "zarm_l1_joint");
  EXPECT_EQ(layout.fullSpec(14).name, "zarm_r1_joint");
  EXPECT_FALSE(layout.fullSpec(0).decision_enabled);
  EXPECT_FALSE(layout.fullSpec(6).decision_enabled);
  EXPECT_TRUE(layout.fullSpec(7).decision_enabled);
  EXPECT_TRUE(layout.fullSpec(20).decision_enabled);
}

TEST(WholeBodyDofLayout, LowerBodyStageUsesEighteenDofSubsetOfFullLayout) {
  const DofLayout layout = DofLayout::LowerBodyStage();

  ASSERT_EQ(layout.decisionDofCount(), kLowerBodyDof + kDualArmDof);
  EXPECT_EQ(layout.decisionIndexByName("knee_joint"), 0);
  EXPECT_EQ(layout.decisionIndexByName("waist_yaw_joint"), 3);
  EXPECT_EQ(layout.armDecisionIndex(0, 0), 4);
  EXPECT_EQ(layout.armDecisionIndex(0, 6), 10);
  EXPECT_EQ(layout.armDecisionIndex(1, 0), 11);
  EXPECT_EQ(layout.armDecisionIndex(1, 6), 17);
  EXPECT_EQ(layout.decisionToFullIndex(0), 3);
  EXPECT_EQ(layout.decisionToFullIndex(17), 20);
  EXPECT_EQ(layout.decisionIndices(DofGroup::kLowerBody).size(),
            static_cast<std::size_t>(kLowerBodyDof));
  EXPECT_TRUE(layout.decisionIndices(DofGroup::kBase).empty());
}

TEST(WholeBodyDofLayout, WholeBodyStageUsesAllTwentyOneDofs) {
  const DofLayout layout = DofLayout::WholeBodyStage();

  ASSERT_EQ(layout.decisionDofCount(), kWholeBodyDof);
  EXPECT_EQ(layout.decisionIndexByName("base_x"), 0);
  EXPECT_EQ(layout.decisionIndexByName("base_y"), 1);
  EXPECT_EQ(layout.decisionIndexByName("base_yaw"), 2);
  EXPECT_EQ(layout.decisionIndexByName("knee_joint"), 3);
  EXPECT_EQ(layout.armDecisionIndex(0, 0), 7);
  EXPECT_EQ(layout.armDecisionIndex(1, 6), 20);
  EXPECT_EQ(layout.decisionIndices(DofGroup::kBase).size(),
            static_cast<std::size_t>(kBaseDof));
}

TEST(WholeBodyDofLayout, ArmDecisionOrderingMatchesLegacyOutput) {
  const DofLayout layout = DofLayout::ArmOnlyStage();
  const auto names = layout.decisionNames();
  const auto groups = layout.decisionGroups();

  ASSERT_EQ(names.size(), static_cast<std::size_t>(kDualArmDof));
  EXPECT_EQ(names.front(), "zarm_l1_joint");
  EXPECT_EQ(names.at(6), "zarm_l7_joint");
  EXPECT_EQ(names.at(7), "zarm_r1_joint");
  EXPECT_EQ(names.back(), "zarm_r7_joint");
  EXPECT_EQ(groups.front(), static_cast<std::uint8_t>(DofGroup::kLeftArm));
  EXPECT_EQ(groups.back(), static_cast<std::uint8_t>(DofGroup::kRightArm));
  for (int joint = 0; joint < 7; ++joint) {
    EXPECT_EQ(layout.armDecisionIndex(0, joint), joint);
    EXPECT_EQ(layout.armDecisionIndex(1, joint), 7 + joint);
  }
}
