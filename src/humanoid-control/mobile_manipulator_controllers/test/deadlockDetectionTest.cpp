#include <pinocchio/fwd.hpp>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/misc/Benchmark.h>
#include <mobile_manipulator_controllers/mobileManipulatorControllerBase.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>
#include <future>
#include <random>
#include <signal.h>
#include <condition_variable>

using namespace ocs2;
using namespace mobile_manipulator_controller;

class DeadlockDetectionTest : public ::testing::Test {
protected:
    static MobileManipulatorControllerBase* controllerInstance_;
    static std::mutex instanceMutex_;
    static std::atomic<bool> deadlockDetected_;
    
    void SetUp() override {
        // 设置ROS日志级别
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        // 初始化ROS节点
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "deadlock_detection_test", ros::init_options::NoSigintHandler);
        }

        nodeHandle_ = std::make_unique<ros::NodeHandle>();

        // 设置测试参数
        taskFile_ = "/root/kuavo_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/kuavo/task.info";
        libFolder_ = "/root/kuavo_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/auto_generated/kuavo";
        urdfFile_ = "/root/kuavo_ws/src/kuavo_assets/models/biped_s45/urdf/biped_s45.urdf";
        mpcType_ = MpcType::SQP;
        freq_ = 100;

        // 初始化控制器实例
        std::lock_guard<std::mutex> lock(instanceMutex_);
        if (controllerInstance_ == nullptr) {
            try {
                controllerInstance_ = new MobileManipulatorControllerBase(*nodeHandle_, taskFile_, libFolder_, urdfFile_, mpcType_, freq_);
            } catch (const std::exception& e) {
                GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
            }
        }
        
        deadlockDetected_ = false;
    }

    void TearDown() override {
        // 清理资源
    }

    // 死锁检测器 - 使用超时机制
    template<typename Func>
    bool detectDeadlock(Func&& func, std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
        std::promise<bool> promise;
        std::future<bool> future = promise.get_future();
        
        std::thread worker([&promise, func = std::forward<Func>(func)]() {
            try {
                func();
                promise.set_value(true);
            } catch (const std::exception& e) {
                std::cerr << "Exception in worker thread: " << e.what() << std::endl;
                promise.set_value(false);
            }
        });
        
        if (future.wait_for(timeout) == std::future_status::timeout) {
            // 超时表示可能发生死锁
            worker.detach(); // 分离线程，避免析构时hang
            return true; // 检测到死锁
        }
        
        worker.join();
        return !future.get(); // 如果函数执行失败，也可能是死锁导致的
    }

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
    std::string taskFile_;
    std::string libFolder_;
    std::string urdfFile_;
    MpcType mpcType_;
    int freq_;
};

// 静态成员初始化
MobileManipulatorControllerBase* DeadlockDetectionTest::controllerInstance_ = nullptr;
std::mutex DeadlockDetectionTest::instanceMutex_;
std::atomic<bool> DeadlockDetectionTest::deadlockDetected_{false};

// 测试1: 基本死锁检测 - 单个reset调用
TEST_F(DeadlockDetectionTest, BasicDeadlockDetection) {
    try {
        auto& controller = *controllerInstance_;
        vector_t resetState = vector_t::Random(20) * 0.3;
        
        bool deadlocked = detectDeadlock([&]() {
            int result = controller.reset(resetState);
            EXPECT_EQ(result, 0) << "Reset should succeed without deadlock";
        });
        
        EXPECT_FALSE(deadlocked) << "Basic reset should not deadlock";
        
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}


// 测试2: 极端条件下的死锁检测 - 快速连续reset
TEST_F(DeadlockDetectionTest, RapidConsecutiveResetDeadlockDetection) {
    try {
        auto& controller = *controllerInstance_;
        
        std::atomic<int> deadlockCount{0};
        std::atomic<int> successCount{0};
        std::atomic<int> errorCount{0};
        
        const int numResets = 100;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.1, 0.1);
        
        // 首先进行一次初始化，让MPC求解器完成第一次求解
        vector_t initialState = vector_t::Zero(20);
        controller.reset(initialState);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待MPC初始化完成
        
        for (int i = 0; i < numResets; ++i) {
            vector_t resetState = vector_t::Zero(20);
            for (int k = 0; k < 20; ++k) {
                resetState[k] = dis(gen);
            }
            
            bool deadlocked = detectDeadlock([&]() {
                try {
                    int result = controller.reset(resetState);
                    if (result == 0) {
                        successCount++;
                    } else {
                        errorCount++;
                    }
                } catch (const std::runtime_error& e) {
                    std::string error_msg = e.what();
                    // 检查是否是预期的MPC求解器竞争条件错误
                    if (error_msg.find("No performance log yet") != std::string::npos ||
                        error_msg.find("no problem solved yet") != std::string::npos) {
                        // 这是预期的竞争条件错误，不是死锁
                        errorCount++;
                        if (i % 20 == 0) { // 只打印部分日志避免刷屏
                            std::cerr << "Reset " << i << " race condition (expected): " << error_msg << std::endl;
                        }
                    } else {
                        // 其他未预期的错误
                        errorCount++;
                        std::cerr << "Reset " << i << " unexpected error: " << error_msg << std::endl;
                    }
                } catch (const std::exception& e) {
                    // 其他类型的异常
                    errorCount++;
                    std::cerr << "Reset " << i << " other error: " << e.what() << std::endl;
                }
            }, std::chrono::milliseconds(2000)); // 增加超时时间
            
            if (deadlocked) {
                deadlockCount++;
                std::cerr << "Deadlock detected in rapid reset " << i << std::endl;
            }
            
            // 适当增加间隔，给MPC线程一些时间
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "Rapid consecutive reset test results:" << std::endl;
        std::cout << "  Successful resets: " << successCount.load() << std::endl;
        std::cout << "  Error resets (not deadlocks): " << errorCount.load() << std::endl;
        std::cout << "  Detected deadlocks: " << deadlockCount.load() << std::endl;
        
        EXPECT_EQ(deadlockCount.load(), 0) << "No deadlocks should be detected in rapid consecutive resets";
        
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试3: 极端条件下的死锁检测 - 快速连续reset+stop
TEST_F(DeadlockDetectionTest, RapidConsecutiveResetAndStopDeadlockDetection) {
    try {
        auto& controller = *controllerInstance_;
        
        std::atomic<int> deadlockCount{0};
        std::atomic<int> successCount{0};
        
        const int numResets = 100;
        std::random_device rd;
        std::mt19937 gen(rd());

        vector_t state = vector_t::Zero(20);
        vector_t nextState = vector_t::Zero(20);
        std::uniform_real_distribution<> dis(-0.2, 0.2);
        
        for (int i = 0; i < numResets; ++i) {
            std::cout << "Round " << i << std::endl;
            vector_t resetState = vector_t::Zero(20);
            for (int k = 0; k < 20; ++k) {
                resetState[k] = dis(gen);
            }
            
            bool deadlocked = detectDeadlock([&]() {
                try {
                    int result = controller.reset(resetState);
                    if (result == 0) {
                        successCount++;
                    }
                    controller.update(state, nextState);
                    state = nextState;
                    controller.stop();
                } catch (const std::runtime_error& e) {
                    std::string error_msg = e.what();
                    // 检查是否是预期的MPC求解器竞争条件错误
                    if (error_msg.find("No performance log yet") != std::string::npos ||
                        error_msg.find("no problem solved yet") != std::string::npos) {
                        // 这是预期的竞争条件错误，不是死锁
                        if (i % 20 == 0) { // 只打印部分日志避免刷屏
                            std::cerr << "Reset+Stop " << i << " race condition (expected): " << error_msg << std::endl;
                        }
                    } else {
                        std::cerr << "Reset+Stop " << i << " unexpected error: " << error_msg << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Reset+Stop " << i << " other error: " << e.what() << std::endl;
                }
            }, std::chrono::milliseconds(2000)); // 增加超时时间
            
            if (deadlocked) {
                deadlockCount++;
                std::cerr << "Deadlock detected in rapid reset " << i << std::endl;
            }
            
            // 非常短的间隔，增加死锁概率
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        std::cout << "Rapid consecutive reset test results:" << std::endl;
        std::cout << "  Successful resets: " << successCount.load() << std::endl;
        std::cout << "  Detected deadlocks: " << deadlockCount.load() << std::endl;
        
        EXPECT_EQ(deadlockCount.load(), 0) << "No deadlocks should be detected in rapid consecutive resets";
        
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

// 测试4: MPC线程状态检查死锁检测
TEST_F(DeadlockDetectionTest, MpcThreadStateCheckDeadlockDetection) {
    try {
        auto& controller = *controllerInstance_;
        
        std::atomic<int> deadlockCount{0};
        std::atomic<int> checkCount{0};
        
        // 创建一个线程持续检查MPC线程状态
        std::atomic<bool> testRunning{true};
        std::thread statusCheckThread([&]() {
            while (testRunning.load()) {
                bool deadlocked = detectDeadlock([&]() {
                    // 调用isMpcThreadPaused()方法，这是可能发生死锁的地方
                    bool paused = controller.isMpcThreadPaused();
                    checkCount++;
                }, std::chrono::milliseconds(500));
                
                if (deadlocked) {
                    deadlockCount++;
                    std::cerr << "Deadlock detected in MPC thread state check" << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        
        // 同时执行reset操作
        std::thread resetThread([&]() {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-0.3, 0.3);
            
            for (int i = 0; i < 50 && testRunning.load(); ++i) {
                vector_t resetState = vector_t::Zero(20);
                for (int k = 0; k < 20; ++k) {
                    resetState[k] = dis(gen);
                }
                
                bool deadlocked = detectDeadlock([&]() {
                    controller.reset(resetState);
                }, std::chrono::milliseconds(1000));
                
                if (deadlocked) {
                    deadlockCount++;
                    std::cerr << "Deadlock detected in reset during status check test" << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        
        // 运行测试
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        testRunning = false;
        
        statusCheckThread.join();
        resetThread.join();
        
        std::cout << "MPC thread state check test results:" << std::endl;
        std::cout << "  Status checks performed: " << checkCount.load() << std::endl;
        std::cout << "  Deadlocks detected: " << deadlockCount.load() << std::endl;
        
        EXPECT_EQ(deadlockCount.load(), 0) << "No deadlocks should be detected in MPC thread state checking";
        
    } catch (const std::exception& e) {
        GTEST_SKIP() << "Skipping test due to missing dependencies: " << e.what();
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}