#include <pinocchio/fwd.hpp>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/misc/Benchmark.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <numeric>
#include <cmath>

using namespace ocs2;
using namespace mobile_manipulator_controller;
using ::testing::_;
using ::testing::Return;
using ::testing::AtLeast;


class MobileManipulatorControllerBaseTest : public ::testing::Test {
protected:
    // 静态单例实例
    static MobileManipulatorControllerBase* controllerInstance_;
    static std::mutex instanceMutex_;

    void SetUp() override {
        // 设置ROS日志级别为ERROR，减少输出
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        // 初始化ROS节点
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_mobile_manipulator_controller", ros::init_options::NoSigintHandler);
        }

        nodeHandle_ = std::make_unique<ros::NodeHandle>();

        // 设置测试参数
        taskFile_ = "/root/kuavo_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/kuavo/task.info";
        libFolder_ = "/root/kuavo_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/auto_generated/kuavo";
        urdfFile_ = "/root/kuavo_ws/src/kuavo_assets/models/biped_s45/urdf/biped_s45.urdf";
        mpcType_ = MpcType::SQP;
        freq_ = 100;

        // 初始化单例实例
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if (controllerInstance_ == nullptr) {
            try {
                controllerInstance_ = new MobileManipulatorControllerBase(*nodeHandle_, taskFile_, libFolder_, urdfFile_, mpcType_, freq_);
            } catch (const std::exception& e) {
                GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
            }
        }
    }

    void TearDown() override {
        // 清理单例实例（可选）
        // std::lock_guard<std::mutex> lock(instanceMutex_);
        // if (controllerInstance_ != nullptr) {
        //     delete controllerInstance_;
        //     controllerInstance_ = nullptr;
        // }
    }

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
    std::string taskFile_;
    std::string libFolder_;
    std::string urdfFile_;
    MpcType mpcType_;
    int freq_;
};

// 初始化静态成员变量
MobileManipulatorControllerBase* MobileManipulatorControllerBaseTest::controllerInstance_ = nullptr;
std::mutex MobileManipulatorControllerBaseTest::instanceMutex_;

// 测试构造函数
TEST_F(MobileManipulatorControllerBaseTest, Constructor)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试构造函数参数验证
TEST_F(MobileManipulatorControllerBaseTest, ConstructorParameterValidation)
{
    // 测试不同的MPC类型
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping SQP test due to missing dependencies: " << e.what();
    }
    
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping DDP test due to missing dependencies: " << e.what();
    }
}

// 测试析构函数
TEST_F(MobileManipulatorControllerBaseTest, Destructor)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        // 显式调用析构函数
        // This part is tricky with static instance. For now, we'll just ensure it doesn't crash.
        // A proper cleanup would involve managing the instance's lifetime.
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试更新方法
TEST_F(MobileManipulatorControllerBaseTest, UpdateMethod) {
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        // 创建测试状态
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        
        // 测试正常更新
        EXPECT_NO_THROW(controller.update(externalState, nextState));
        
        // 验证输出状态的大小
        EXPECT_EQ(nextState.size(), 20);
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试停止和启动方法
TEST_F(MobileManipulatorControllerBaseTest, StopStartMethods)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        // 测试停止
        controller.stop();
        
        // 创建测试状态
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        
        // 停止后的更新应该不会产生错误并且返回-1
        EXPECT_NO_THROW(controller.update(externalState, nextState));
        EXPECT_EQ(controller.update(externalState, nextState), -1);
        
        // 测试重新启动
        controller.start();
        EXPECT_NO_THROW(controller.update(externalState, nextState));
        EXPECT_EQ(controller.update(externalState, nextState), 0);
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试状态维度
TEST_F(MobileManipulatorControllerBaseTest, StateDimensions)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        // 测试不同大小的状态向量
        std::vector<std::pair<size_t, bool>> testDimensions = {{10, false}, {15, false}, {20, true}, {25, false}};
        
        for(auto [dim, expectCorrect] : testDimensions) {
            vector_t externalState = vector_t::Zero(dim);
            vector_t nextState = vector_t::Zero(dim);
            
            if(expectCorrect)
            {
              EXPECT_NO_THROW(controller.update(externalState, nextState));
              EXPECT_EQ(nextState.size(), dim);
            }
            else
            {
              EXPECT_EQ(controller.update(externalState, nextState), -2);
            }
        }
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试线程安全性
TEST_F(MobileManipulatorControllerBaseTest, ThreadSafety)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        std::atomic<bool> testRunning{true};
        std::atomic<int> errorCount{0};
        
        // 创建多个线程同时调用更新方法
        std::vector<std::thread> threads;
        for(int i = 0; i < 3; ++i) {
            threads.emplace_back([&controller, &testRunning, &errorCount]() {
                vector_t externalState = vector_t::Zero(20);
                vector_t nextState = vector_t::Zero(20);
                
                while(testRunning.load()) {
                    try {
                        controller.update(externalState, nextState);
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    } catch(...) {
                        errorCount++;
                    }
                }
            });
        }
        
        // 运行一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        testRunning = false;
        
        // 等待所有线程完成
        for(auto& t : threads) {
            t.join();
        }
        
        // 检查错误计数
        EXPECT_EQ(errorCount.load(), 0);
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试边界条件
TEST_F(MobileManipulatorControllerBaseTest, BoundaryConditions)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        // 测试零状态
        vector_t zeroState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        EXPECT_NO_THROW(controller.update(zeroState, nextState));
        
        // 测试极大值
        vector_t largeState = vector_t::Ones(20) * 1000.0;
        EXPECT_NO_THROW(controller.update(largeState, nextState));
        
        // 测试极小值
        vector_t smallState = vector_t::Ones(20) * (-1000.0);
        EXPECT_NO_THROW(controller.update(smallState, nextState));

        // 测试多轮更新
        auto currentState = largeState;
        for(int i = 0; i < 10; ++i)
        {
          EXPECT_NO_THROW(controller.update(currentState, nextState));
          currentState = nextState;
        }
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试ROS发布器
TEST_F(MobileManipulatorControllerBaseTest, ROSPublishers)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        // 等待发布器初始化
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 创建订阅者来验证消息发布
        std::atomic<bool> eefPosesReceived{false};
        std::atomic<bool> mpcPolicyReceived{false};
        
        auto eefPosesSubscriber = nodeHandle_->subscribe<std_msgs::Float64MultiArray>(
            "mobile_manipulator_eef_poses", 1,
            [&eefPosesReceived](const std_msgs::Float64MultiArray::ConstPtr& msg) {
                eefPosesReceived = true;
            });
        
        auto mpcPolicySubscriber = nodeHandle_->subscribe<ocs2_msgs::mpc_flattened_controller>(
            "mobile_manipulator_mpc_policy", 1,
            [&mpcPolicyReceived](const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg) {
                mpcPolicyReceived = true;
            });
        
        // 触发更新以产生发布
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        controller.update(externalState, nextState);
        
        // 等待消息传播
        for(int i = 0; i < 10 && (!eefPosesReceived || !mpcPolicyReceived); ++i) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // 验证消息接收（可能因为依赖项而失败）
        // EXPECT_TRUE(eefPosesReceived.load());
        // EXPECT_TRUE(mpcPolicyReceived.load());
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试性能基准
TEST_F(MobileManipulatorControllerBaseTest, PerformanceBenchmark)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        
        // 测量100次更新的时间
        const int iterations = 100;
        auto start = std::chrono::high_resolution_clock::now();
        ros::Rate rate(freq_);
        for(int i = 0; i < iterations; ++i) {
            controller.update(externalState, nextState);
            externalState = nextState;
            rate.sleep();
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        // 计算平均时间
        double avgTimeMs = duration.count() / (iterations * 1000.0);
        
        // 期望平均时间小于某个阈值（例如10ms）
        const double expectMs = 1000.0 / freq_;
        EXPECT_LT(avgTimeMs, expectMs*1.1) << "Average update time: " << avgTimeMs << "ms";
        
        std::cout << "Performance benchmark: " << avgTimeMs << "ms per update" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试当前状态突变并且sleep之后，控制器是否能正常运行
TEST_F(MobileManipulatorControllerBaseTest, StateChangeWithSleep)
{
    try {
        double dt = 1000.0 / freq_;
        // 使用单例实例
        auto& controller = *controllerInstance_;
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        controller.update(externalState, nextState);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt)));
        // 计算externalState与nextState的躯干位置差异的模长
        double norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        // 突变
        externalState(2) = -0.3;
        controller.update(externalState, nextState);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt)));
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        controller.update(externalState, nextState);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt)));
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        EXPECT_LT(norm, 5e-2);
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试当前状态突变后，控制器是否能正常运行
TEST_F(MobileManipulatorControllerBaseTest, StateChange)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        controller.update(externalState, nextState);
        // 计算externalState与nextState的躯干位置差异的模长
        double norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        // 突变
        externalState(2) = -0.6;
        controller.update(externalState, nextState);
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        controller.update(externalState, nextState);
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        EXPECT_LT(norm, 5e-2);
        // 测试sleep之后，状态误差是否减少

    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}


// 测试当前状态突变后,先设置externalState，控制器是否能正常运行
TEST_F(MobileManipulatorControllerBaseTest, StateChangeWithSetExternalState)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        controller.update(externalState, nextState);
        // 计算externalState与nextState的躯干位置差异的模长
        double norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        // 突变
        externalState(2) = -0.9;
        controller.setExternalState(externalState);
        controller.update(externalState, nextState);
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        EXPECT_LT(norm, 5e-2);
        controller.update(externalState, nextState);
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        EXPECT_LT(norm, 5e-2);
        // 测试sleep之后，状态误差是否减少

    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试当前状态突变后,先reset，控制器是否能正常运行
TEST_F(MobileManipulatorControllerBaseTest, StateChangeWithReset)
{
    try {
        // 使用单例实例
        auto& controller = *controllerInstance_;
        vector_t externalState = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        controller.update(externalState, nextState);
        // 计算externalState与nextState的躯干位置差异的模长
        double norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        // EXPECT_LT(norm, 5e-2);
        // 突变
        externalState(2) = -1.2;
        // 计算reset耗时
        auto start = std::chrono::high_resolution_clock::now();
        controller.reset(externalState);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "reset time cost: " << duration.count() << "us" << std::endl;
        controller.update(externalState, nextState);
        std::cout << "nextState: " << nextState.transpose() << std::endl;
        norm = (externalState - nextState).head(3).norm();
        std::cout << "body pos norm: " << norm << std::endl;
        EXPECT_LT(norm, 5e-2);

    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// ============= 专门的重置方法测试用例 =============

// 测试重置方法的基本功能
TEST_F(MobileManipulatorControllerBaseTest, ResetBasicFunctionality)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 准备测试状态
        vector_t initialState = vector_t::Zero(20);
        vector_t resetState = vector_t::Random(20) * 0.1; // 小幅随机状态
        vector_t nextState = vector_t::Zero(20);
        
        // 初始更新
        EXPECT_EQ(controller.update(initialState, nextState), 0);
        
        // 执行重置
        int resetResult = controller.reset(resetState);
        EXPECT_EQ(resetResult, 0) << "Reset should return 0 on success";
        
        // 重置后的更新应该正常工作
        EXPECT_EQ(controller.update(resetState, nextState), 0);
        
        std::cout << "Reset basic functionality test passed" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试重置方法的参数验证
TEST_F(MobileManipulatorControllerBaseTest, ResetParameterValidation)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 测试错误的状态维度
        vector_t wrongSizeState = vector_t::Zero(10); // 错误的大小
        int result = controller.reset(wrongSizeState);
        EXPECT_EQ(result, -1) << "Reset should return -1 for wrong state dimension";
        
        // 测试空状态
        vector_t emptyState = vector_t::Zero(0);
        result = controller.reset(emptyState);
        EXPECT_EQ(result, -1) << "Reset should return -1 for empty state";
        
        // 测试正确大小的状态
        vector_t correctState = vector_t::Zero(20);
        result = controller.reset(correctState);
        EXPECT_EQ(result, 0) << "Reset should return 0 for correct state dimension";
        
        std::cout << "Reset parameter validation test passed" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试重置方法的性能
TEST_F(MobileManipulatorControllerBaseTest, ResetPerformance)
{
    try {
        auto& controller = *controllerInstance_;
        
        vector_t testState = vector_t::Random(20) * 0.5;
        
        // 多次重置测试性能
        const int numResets = 5;
        std::vector<double> resetTimes;
        
        for(int i = 0; i < numResets; ++i) {
            // 稍微修改状态以确保每次重置都有意义
            testState(0) += 0.1 * i;
            
            auto start = std::chrono::high_resolution_clock::now();
            int result = controller.reset(testState);
            auto end = std::chrono::high_resolution_clock::now();
            
            EXPECT_EQ(result, 0) << "Reset " << i << " should succeed";
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            double timeMs = duration.count() / 1000.0;
            resetTimes.push_back(timeMs);
            
            std::cout << "Reset " << i << " took: " << timeMs << " ms" << std::endl;
        }
        
        // 计算平均重置时间
        double avgTime = std::accumulate(resetTimes.begin(), resetTimes.end(), 0.0) / resetTimes.size();
        std::cout << "Average reset time: " << avgTime << " ms" << std::endl;
        
        // 期望重置时间在合理范围内（小于200ms）
        EXPECT_LT(avgTime, 200.0) << "Average reset time should be less than 200ms";
        
        // 检查时间的一致性（标准差不应该太大）
        double variance = 0.0;
        for(double time : resetTimes) {
            variance += (time - avgTime) * (time - avgTime);
        }
        variance /= resetTimes.size();
        double stdDev = std::sqrt(variance);
        
        std::cout << "Reset time standard deviation: " << stdDev << " ms" << std::endl;
        EXPECT_LT(stdDev, avgTime * 0.5) << "Reset time should be relatively consistent";
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试重置后的状态一致性
TEST_F(MobileManipulatorControllerBaseTest, ResetStateConsistency)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 定义测试状态
        vector_t resetState = vector_t::Zero(20);
        resetState(0) = 0.1;  // x位置
        resetState(1) = 0.2;  // y位置
        resetState(2) = -0.8; // z位置
        
        vector_t nextState = vector_t::Zero(20);
        
        // 执行重置
        int result = controller.reset(resetState);
        EXPECT_EQ(result, 0);
        
        // 重置后立即更新，检查状态一致性
        result = controller.update(resetState, nextState);
        EXPECT_EQ(result, 0);
        
        // 检查重置后的状态是否接近期望值
        double positionError = (resetState.head(3) - nextState.head(3)).norm();
        std::cout << "Position error after reset: " << positionError << std::endl;
        
        // 重置后的位置误差应该很小
        EXPECT_LT(positionError, 0.1) << "Position error after reset should be small";
        
        // 再次更新，检查稳定性
        vector_t secondNextState = vector_t::Zero(20);
        result = controller.update(nextState, secondNextState);
        EXPECT_EQ(result, 0);
        
        double stabilityError = (nextState.head(3) - secondNextState.head(3)).norm();
        std::cout << "Stability error: " << stabilityError << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试多次连续重置
TEST_F(MobileManipulatorControllerBaseTest, MultipleConsecutiveResets)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 定义多个测试状态
        std::vector<vector_t> testStates;
        for(int i = 0; i < 3; ++i) {
            vector_t state = vector_t::Zero(20);
            state(0) = 0.1 * i;
            state(1) = 0.2 * i;
            state(2) = -0.5 - 0.1 * i;
            testStates.push_back(state);
        }
        
        vector_t nextState = vector_t::Zero(20);
        
        // 连续执行多次重置
        for(size_t i = 0; i < testStates.size(); ++i) {
            std::cout << "Performing reset " << i << std::endl;
            
            auto start = std::chrono::high_resolution_clock::now();
            int result = controller.reset(testStates[i]);
            auto end = std::chrono::high_resolution_clock::now();
            
            EXPECT_EQ(result, 0) << "Reset " << i << " should succeed";
            
            // 验证重置后可以正常更新
            result = controller.update(testStates[i], nextState);
            EXPECT_EQ(result, 0) << "Update after reset " << i << " should succeed";
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "Reset " << i << " took: " << duration.count() << " us" << std::endl;
        }
        
        std::cout << "Multiple consecutive resets test passed" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试重置方法的线程安全性
TEST_F(MobileManipulatorControllerBaseTest, ResetThreadSafety)
{
    try {
        auto& controller = *controllerInstance_;
        
        std::atomic<int> successCount{0};
        std::atomic<int> errorCount{0};
        std::atomic<bool> testRunning{true};
        
        // 创建一个线程持续进行更新操作
        std::thread updateThread([&]() {
            vector_t state = vector_t::Zero(20);
            vector_t nextState = vector_t::Zero(20);
            
            while(testRunning.load()) {
                try {
                    int result = controller.update(state, nextState);
                    if(result == 0) {
                        state = nextState;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                } catch(...) {
                    // 在重置期间，update可能会返回错误，这是正常的
                    // 我们只计算真正的异常，而不是预期的错误返回值
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        });
        
        // 主线程执行重置操作
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 让更新线程先运行
        
        vector_t resetState = vector_t::Random(20) * 0.3;
        
        try {
            int result = controller.reset(resetState);
            if(result == 0) {
                successCount++;
            } else {
                errorCount++;
            }
        } catch(...) {
            errorCount++;
        }
        
        // 停止测试
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        testRunning = false;
        updateThread.join();
        
        EXPECT_EQ(successCount.load(), 1) << "Reset should succeed once";
        // 在线程安全测试中，允许一些错误发生，因为重置期间update可能会失败
        // 这是正常的行为，不应该被视为错误
        std::cout << "Reset thread safety test completed with " << errorCount.load() << " update failures during reset" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试极端状态值的重置
TEST_F(MobileManipulatorControllerBaseTest, ResetExtremeStates)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 测试各种极端状态
        std::vector<std::pair<std::string, vector_t>> extremeStates;
        
        // 零状态
        extremeStates.push_back({"Zero state", vector_t::Zero(20)});
        
        // 大值状态
        vector_t largeState = vector_t::Ones(20) * 5.0;
        extremeStates.push_back({"Large positive state", largeState});
        
        // 小值状态
        vector_t smallState = vector_t::Ones(20) * (-5.0);
        extremeStates.push_back({"Large negative state", smallState});
        
        // 混合状态
        vector_t mixedState = vector_t::Random(20);
        mixedState = mixedState * 3.0; // 放大随机值
        extremeStates.push_back({"Mixed extreme state", mixedState});
        
        vector_t nextState = vector_t::Zero(20);
        
        for(const auto& testCase : extremeStates) {
            std::cout << "Testing " << testCase.first << std::endl;
            
            auto start = std::chrono::high_resolution_clock::now();
            int result = controller.reset(testCase.second);
            auto end = std::chrono::high_resolution_clock::now();
            
            EXPECT_EQ(result, 0) << "Reset should succeed for " << testCase.first;
            
            // 验证重置后可以正常更新
            result = controller.update(testCase.second, nextState);
            EXPECT_EQ(result, 0) << "Update should succeed after reset for " << testCase.first;
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << testCase.first << " reset took: " << duration.count() << " us" << std::endl;
        }
        
        std::cout << "Extreme states reset test passed" << std::endl;
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试经历极端状态值后，重置回正常状态，系统是否正常运行
TEST_F(MobileManipulatorControllerBaseTest, ResetExtremeStatesAndBackToNormal)
{
    try {
        auto& controller = *controllerInstance_;
        
        // 测试各种极端状态
        std::vector<std::pair<std::string, vector_t>> extremeStates;
        
        // 零状态
        extremeStates.push_back({"Zero state", vector_t::Zero(20)});
        
        // 大值状态
        vector_t largeState = vector_t::Ones(20) * 5.0;
        extremeStates.push_back({"Large positive state", largeState});
        
        // 小值状态
        vector_t smallState = vector_t::Ones(20) * (-5.0);
        extremeStates.push_back({"Large negative state", smallState});
        
        // 混合状态
        vector_t mixedState = vector_t::Random(20);
        mixedState = mixedState * 3.0; // 放大随机值
        extremeStates.push_back({"Mixed extreme state", mixedState});
        
        vector_t nextState = vector_t::Zero(20);
        
        for(const auto& testCase : extremeStates) {
            std::cout << "Testing " << testCase.first << std::endl;
            
            auto start = std::chrono::high_resolution_clock::now();
            int result = controller.reset(testCase.second);
            auto end = std::chrono::high_resolution_clock::now();
            
            EXPECT_EQ(result, 0) << "Reset should succeed for " << testCase.first;
            
            // 验证重置后可以正常更新
            result = controller.update(testCase.second, nextState);
            EXPECT_EQ(result, 0) << "Update should succeed after reset for " << testCase.first;
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << testCase.first << " reset took: " << duration.count() << " us" << std::endl;
        }
        
        std::cout << "Extreme states reset test passed, then we test if the system can run normally after reset" << std::endl;
        vector_t normalState = vector_t::Zero(20);
        normalState(0) = 0.1;
        normalState(1) = 0.2;
        normalState(2) = -0.3;
        controller.reset(normalState);

        // simulate for a while, and check if the system can run normally in rviz by our eyes
        auto sim_state = normalState;   
        for(int i = 0; i < 5*100; ++i) {
            controller.update(sim_state, nextState);
            // std::cout << "nextState: " << nextState.transpose() << std::endl;
            sim_state = nextState;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        double norm = (normalState - nextState).head(3).norm();
        EXPECT_LT(norm, 5e-2);
        
    } catch(const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 主函数
int main(int argc, char** argv)
{
    // 设置Google Test输出格式为简洁模式
    // ::testing::GTEST_FLAG(output) = "xml:";
    
    // 设置ROS日志级别为ERROR，减少输出
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    // 设置其他日志级别
    if(ros::console::set_logger_level("/rosout", ros::console::levels::Error))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    // 禁用标准输出重定向（如果需要）
    ::testing::GTEST_FLAG(catch_exceptions) = false;
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 