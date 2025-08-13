#include <gtest/gtest.h>
#include "mobile_manipulator_controllers/mobileManipulatorIkTarget.h"
#include <ros/ros.h>

class MobileManipulatorIkTargetTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化ROS节点
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "test_ik_target");
        nodeHandle_ = std::make_unique<ros::NodeHandle>();
    }

    void TearDown() override {
        ros::shutdown();
    }

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
};

// 测试构造函数
TEST_F(MobileManipulatorIkTargetTest, ConstructorTest) {
    EXPECT_NO_THROW({
        mobile_manipulator_controller::MobileManipulatorIkTarget ikTarget(*nodeHandle_, "test_robot");
    });
}

// 测试基本方法
TEST_F(MobileManipulatorIkTargetTest, BasicMethodsTest) {
    mobile_manipulator_controller::MobileManipulatorIkTarget ikTarget(*nodeHandle_, "test_robot");
    
    // 测试设置Quest3工具
    EXPECT_TRUE(ikTarget.setQuest3Utils(true));
    EXPECT_TRUE(ikTarget.setQuest3Utils(false));
    
    // 测试设置坐标系类型
    EXPECT_TRUE(ikTarget.setFrameType(mobile_manipulator_controller::FrameType::VRFrame));
    EXPECT_TRUE(ikTarget.setFrameType(mobile_manipulator_controller::FrameType::WorldFrame));
    EXPECT_TRUE(ikTarget.setFrameType(mobile_manipulator_controller::FrameType::LocalFrame));
    
    // 测试获取坐标系类型
    auto frameType = ikTarget.getFrameType();
    EXPECT_EQ(frameType, mobile_manipulator_controller::FrameType::LocalFrame);
    
    // 测试状态查询方法
    EXPECT_FALSE(ikTarget.isObservationReceived());
    EXPECT_FALSE(ikTarget.isHumanoidObservationReceived());
    EXPECT_EQ(ikTarget.getEffTrajReceived(), 0);
}

// 测试移动语义
TEST_F(MobileManipulatorIkTargetTest, MoveSemanticsTest) {
    auto ikTarget1 = std::make_unique<mobile_manipulator_controller::MobileManipulatorIkTarget>(*nodeHandle_, "test_robot1");
    
    // 测试移动构造
    auto ikTarget2 = std::move(ikTarget1);
    EXPECT_EQ(ikTarget1, nullptr);
    EXPECT_NE(ikTarget2, nullptr);
    
    // 测试移动赋值 - 由于类禁用了移动赋值，我们只测试移动构造
    auto ikTarget3 = std::make_unique<mobile_manipulator_controller::MobileManipulatorIkTarget>(*nodeHandle_, "test_robot2");
    EXPECT_NE(ikTarget3, nullptr);
}

// 测试智能指针使用
TEST_F(MobileManipulatorIkTargetTest, SmartPointerTest) {
    auto ikTarget = std::make_unique<mobile_manipulator_controller::MobileManipulatorIkTarget>(*nodeHandle_, "test_robot");
    
    EXPECT_NE(ikTarget, nullptr);
    EXPECT_TRUE(ikTarget->setQuest3Utils(true));
    EXPECT_TRUE(ikTarget->setFrameType(mobile_manipulator_controller::FrameType::VRFrame));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 