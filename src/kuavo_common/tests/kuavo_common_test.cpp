#include <gtest/gtest.h>
#include "kuavo_common/kuavo_common.h"
#include "kuavo_common/common/kuavo_settings.h"
#include "kuavo_common/common/json_config_reader.hpp"
namespace HighlyDynamic {

/**
 * @brief Test suite for verifying KuavoCommon functionality
 * Tests the singleton pattern implementation and configuration loading
 */

/**
 * @test Verifies the getInstance pointer functionality
 * Ensures that getInstance() and getInstancePtr() return the same instance
 */

TEST(KuavoCommonTest, InstancePointerTest) {
    RobotVersion version(4, 2);
    std::string assets_path = "/root/kuavo_ws/src/kuavo_assets";

    KuavoCommon* ptr_instance = KuavoCommon::getInstancePtr(version, assets_path);
    KuavoCommon* ptr_instance2 = KuavoCommon::getInstancePtr(version, assets_path);

    // Verify that both pointers point to the same instance
    EXPECT_EQ(ptr_instance, ptr_instance2);

    // Verify that the pointer is not null
    EXPECT_NE(ptr_instance, nullptr);
}
/**
 * @test Verifies the singleton pattern of KuavoCommon
 * Ensures that multiple calls to getInstance() return the same instance
 */
TEST(KuavoCommonTest, SingletonInstance) {
    RobotVersion version(4, 2);
    std::string assets_path = "/root/kuavo_ws/src/kuavo_assets";

    KuavoCommon& instance1 = KuavoCommon::getInstance(version, assets_path);
    KuavoCommon& instance2 = KuavoCommon::getInstance(version, assets_path);

    // Verify that both references point to the same instance
    EXPECT_EQ(&instance1, &instance2);
}

/**
 * @test Validates the configuration loading functionality
 * Tests various data type retrievals from the robot configuration
 */
TEST(KuavoCommonTest, KuavoSettingsLoad) {
    RobotVersion version(4, 2);
    std::string assets_path = "/root/kuavo_ws/src/kuavo_assets";

    KuavoCommon& kuavo = KuavoCommon::getInstance(version, assets_path);
    auto *robot_config = kuavo.getRobotConfig();

    // Verify boolean configuration parameter
    EXPECT_TRUE(robot_config->getValue<bool>("swing_arm"));

    // Verify string configuration parameter
    EXPECT_EQ(robot_config->getValue<std::string>("model_path"), 
             "biped_s42/urdf/drake/biped_v3.urdf");

    // Verify integer configuration parameter
    EXPECT_EQ(robot_config->getValue<int>("NUM_JOINT"), 28);
}

/**
 * @test Verifies RobotVersion class functionality
 * Tests version string formatting and integer representation
 */
TEST(KuavoCommonTest, RobotVersion) {
    RobotVersion version(4, 2);
    // Verify string representation of version
    EXPECT_EQ(version.versionName(), "4.2");
    // Verify integer representation of version
    EXPECT_EQ(version.versionInt(), 42);
}



}  // namespace HighlyDynamic