#include "wheel_arm_abs_vel/model/FullModelContract.h"
#include <ros/package.h>
#include <atomic>
#include "wheel_arm_abs_vel/motion_capture_ik/runtime/E2eDeadline.h"

#include "wheel_arm_abs_vel/humanoid_controllers/absolute/humanoidController_wheel_absolute.h"
#include <clocale>
#include <thread>
#include <chrono>
#include <time.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <errno.h>
#include <cstring>
#include "ocs2_core/thread_support/SetThreadPriority.h"
#include <std_msgs/Bool.h>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;

namespace {
constexpr char kHardwareStateParam[] = "/nodelet_manager/hardware_state";
constexpr int kHardwareDeinitialized = 2;
}  // namespace

class WheelAbsoluteControllerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        std::setlocale(LC_ALL, "en_US.UTF-8");  // 修复 log4cxx ROS_INFO 中文乱码
        NODELET_INFO("Initializing WheelAbsoluteControllerNodelet nodelet...");
        nh = getNodeHandle();
        std::string controller_type;
        nh.param<std::string>("/controller_type", controller_type, "legacy");
        if (controller_type != "wheel_arm_abs_vel")
            throw std::runtime_error("Explicit controller_type=wheel_arm_abs_vel required");
        bool absolute_mode=false, external_mpc=false;
        int robot_version=62;
        int robot_type=0, wheel_type=0, estimator=1;
        nh.param("/vr_ik/use_velocity_ik_reference", absolute_mode, false);
        nh.param("/use_external_mpc", external_mpc, false);
        nh.param("/robot_version", robot_version, 62);
        nh.param("/nodelet_robot_type", robot_type, 0);
        nh.param("/wheel_control_type", wheel_type, 0);
        nh.param("/estimator_type", estimator, 1);
        if (!absolute_mode || robot_type != 1 || wheel_type == 0 || estimator != 1 || external_mpc || (robot_version != 62 && robot_version != 63 && robot_version != 400062))
        {
            NODELET_ERROR("WheelAbsoluteControllerNodelet requires explicit wheel absolute mode");
            throw std::runtime_error("WheelAbsoluteControllerNodelet requires explicit wheel absolute mode");
        }

        {
            std::string urdf;
            nh.getParam("/urdfFile", urdf);
            wheel_arm_abs_vel::requireFullModel(
                robot_version, ros::package::getPath("kuavo_assets"), urdf);
        }

        lifecycle_active_ = true;
        stop_pub_ = nh.advertise<std_msgs::Bool>("/stop_robot", 1, true);
        ros::param::set("/nodelet_manager/controller_state", 0);
        pause_sub = nh.subscribe<std_msgs::Bool>("pauseFlag", 1, &WheelAbsoluteControllerNodelet::pauseCallback, this);
        stop_sub_ = nh.subscribe<std_msgs::Bool>("/stop_robot", 1, &WheelAbsoluteControllerNodelet::stopCallback, this);
        control_thread = std::thread(&WheelAbsoluteControllerNodelet::controlLoop, this);

        // 设置控制循环线程的实时调度策略和CPU亲和性，减少系统调度延迟
        setControlThreadScheduling();

        NODELET_INFO("WheelAbsoluteControllerNodelet nodelet initialized.");
    }
    void controllerExit()
    {
        if (exit_started_.exchange(true)) return;
        //waao：退出控制器
        std::cerr << "[controllerNodelet] controllerExit called, ros::ok=" << ros::ok() << ", is_running=" << is_running << std::endl;
        ros::param::set("/nodelet_manager/controller_state", 1);
        is_running = false;// 先停止控制器
        // Use the original HAL stop subscriber for init failures as well as
        // explicit stops. The guard above prevents recursive stop callbacks.
        std_msgs::Bool stop;
        stop.data = true;
        stop_pub_.publish(stop);


        // 检查nodelet状态
        const double max_wait_seconds = 20.0; // 最大等待5秒
        const int check_interval_ms = 100;   // 每100ms检查一次
        const int max_checks = static_cast<int>(max_wait_seconds * 1000 / check_interval_ms);
        int check_count = 0;
        int hardware_state = 0;
        while (check_count < max_checks && ros::ok())
        {
            if (ros::param::get(kHardwareStateParam, hardware_state))
            {
                if (hardware_state == kHardwareDeinitialized)
                {
                    ROS_INFO("[WheelAbsoluteControllerNodelet] Hardware deinitialization completed");
                    break;
                }
            }

            usleep(check_interval_ms * 1000); // 转换为微秒
            check_count++;

            // 输出剩余等待时间
            double remaining_time = max_wait_seconds - (check_count * check_interval_ms / 1000.0);
            if (check_count % 10 == 0) // 每秒输出一次
            {
                ROS_INFO_STREAM("[WheelAbsoluteControllerNodelet] Waiting for nodelets to be ready... " << std::fixed << std::setprecision(1)
                              << remaining_time << "s remaining, current state: "
                              << hardware_state);
            }
        }

        if (check_count >= max_checks)
        {
            ROS_WARN("[WheelAbsoluteControllerNodelet] Timeout after %.1f seconds waiting for nodelets", max_wait_seconds);
        }

        //waao
        std::cerr << "[controllerNodelet] controllerExit requesting ROS-thread shutdown" << std::endl;
        ros::requestShutdown();

    }
    ~WheelAbsoluteControllerNodelet()
    {
       is_running.store(false, std::memory_order_release);
       // Disable callbacks before publishing our final stop, so this nodelet's
       // own stop subscriber cannot enter controllerExit during destruction.
       stop_sub_.shutdown();
       pause_sub.shutdown();
       if (lifecycle_active_ && !exit_started_.load() && ros::ok()) {
           std_msgs::Bool stop;
           stop.data = true;
           stop_pub_.publish(stop);
           ros::param::set("/nodelet_manager/controller_state", 1);
       }
       if (control_thread.joinable()) control_thread.join();
       // Release all ROS endpoints before the base Nodelet destroys its queues.
       delete controller_wheel_real_ptr_; controller_wheel_real_ptr_ = nullptr;
       std::cerr << "[WheelAbsoluteControllerNodelet] control thread joined before destruction" << std::endl;
    }

private:
    bool lifecycle_active_{false};
    ros::Publisher stop_pub_;
    std::atomic<bool> pause_flag{false};
    ros::NodeHandle nh;
    std::atomic<bool> is_running{true};
    std::atomic<bool> exit_started_{false};
    ros::Subscriber pause_sub;

    wheel_absolute_control::humanoidControllerWheelWbc *controller_wheel_real_ptr_{nullptr};
    ros::Subscriber stop_sub_;
    std::chrono::high_resolution_clock::time_point lastTime;
    std::thread control_thread;
    void pauseCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        pause_flag = msg->data;
        std::cerr << "pause_flag: " << pause_flag << std::endl;
    }
    void stopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        //waao
        std::cerr << "[controllerNodelet] stopCallback received /stop_robot=" << msg->data
                  << ", ros::ok=" << ros::ok() << ", is_running(before)=" << is_running << std::endl;

        if (msg->data)
        {
            is_running = false;
            std::cerr << "[controllerNodelet] stopCallback setting is_running=false and entering controllerExit()" << std::endl;
            controllerExit();
        }
    }

    void setControlThreadScheduling()
    {
        // 等待线程启动
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        pthread_t thread_handle = control_thread.native_handle();

        // 1. 设置实时调度策略 (SCHED_FIFO) 和优先级
        struct sched_param param;
        param.sched_priority = 90;  // 高优先级 (1-99, 99最高)

        int ret = pthread_setschedparam(thread_handle, SCHED_FIFO, &param);
        if (ret != 0) {
            ROS_WARN_STREAM("[controllerNodelet] Failed to set SCHED_FIFO for control thread: "
                          << strerror(ret) << ". You may need to run with sudo or set capabilities.");
        } else {
            ROS_INFO_STREAM("[controllerNodelet] Control thread set to SCHED_FIFO with priority " << param.sched_priority);
        }

        // // 2. 设置CPU亲和性到隔离核心（如果可用）
        // auto isolate_core = ocs2::getIsolatedCpus();
        // if (isolate_core.size() >= 2) {
        //     cpu_set_t cpuset;
        //     CPU_ZERO(&cpuset);
        //     // 使用隔离核心的前两个核心
        //     CPU_SET(isolate_core[0], &cpuset);
        //     CPU_SET(isolate_core[1], &cpuset);

        //     ret = pthread_setaffinity_np(thread_handle, sizeof(cpu_set_t), &cpuset);
        //     if (ret != 0) {
        //         ROS_WARN_STREAM("[controllerNodelet] Failed to set CPU affinity for control thread: " << strerror(ret));
        //     } else {
        //         ROS_INFO_STREAM("[controllerNodelet] Control thread bound to CPU cores: "
        //                       << isolate_core[0] << ", " << isolate_core[1]);
        //     }
        // } else {
        //     ROS_WARN_STREAM("[controllerNodelet] No isolated CPUs found. Control thread may experience scheduling delays.");
        // }

        // 3. 设置内存锁定（可选，防止内存被交换到磁盘）
        // 注意：这需要CAP_IPC_LOCK权限或root权限
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            ROS_WARN_STREAM("[controllerNodelet] Failed to lock memory: " << strerror(errno)
                          << ". This is optional but recommended for real-time performance.");
        } else {
            ROS_INFO("[controllerNodelet] Memory locked successfully");
        }
    }

    void controlLoop()
    {
        try {
            runControlLoop();
        } catch (const std::exception& error) {
            ROS_ERROR("Absolute controller failed: %s", error.what());
            controllerExit();
        } catch (...) {
            ROS_ERROR("Absolute controller failed with an unknown exception");
            controllerExit();
        }
    }

    void runControlLoop()
    {
        // This plugin has one implementation. Mode validation happens before
        // starting the worker; never consume/delete shared launch selectors.
        controller_wheel_real_ptr_ = new wheel_absolute_control::humanoidControllerWheelWbc();
        if (!controller_wheel_real_ptr_->init(nh, true))
        {
            ROS_ERROR("Failed to initialize the wheel absolute controller!");
            controllerExit();
            return;
        }

        if (!is_running.load(std::memory_order_acquire)) return;

        // Record the ROS start time used to initialize the control state
        // Calls controller's starting() method to initialize the control state
        // Sets up control frequency (default 500Hz, configurable via parameter)
        auto startTimeROS = ros::Time::now();

        // 调用 starting() 并检查是否成功
        const bool starting_success = controller_wheel_real_ptr_->starting(startTimeROS, &is_running);

        // 检查starting是否成功
        if (!starting_success) {
            if (!is_running.load(std::memory_order_acquire)) return;
            ROS_ERROR("Controller starting failed! Data not ready within timeout. Exiting...");
            controllerExit();
            return;
        }

        lastTime = std::chrono::high_resolution_clock::now();
        double controlFrequency = 500.0; // 500Hz
        nh.getParam("/wbc_frequency", controlFrequency);
        ROS_INFO_STREAM("Wbc control frequency: " << controlFrequency);
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        ros::Rate rate(controlFrequency);
        kuavo_e2e::Deadline e2eDeadline(1000000);
        // uint64_t cycle_count = 0;

        while (is_running && ros::ok() && !controller_wheel_real_ptr_->preUpdateComplete())
        {
            if (!controller_wheel_real_ptr_->preUpdate(ros::Time::now())) {
                ROS_INFO_THROTTLE(1.0, "Waiting for measured startup state...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        // main control loop
        // Can be paused with pause_flag
        /* For each cycle:
           1. Calculates elapsed time since last cycle
           2. Calls controller's update() method with current time and elapsed time
           3. Uses precise sleep with clock_nanosleep for accurate timing
           4. Monitors cycle time to detect performance issues
        */
        while (is_running && ros::ok())
        {
            // std::cout << "\n\nControlLoop: "<<cycle_count++ << std::endl;
            if (!pause_flag)
            {
                // std::cout << "controlLoop: Running control loop" << std::endl;
                const auto currentTime = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> time_span = currentTime - lastTime;
                ros::Duration elapsedTime(time_span.count());
                lastTime = currentTime;

                // 轮臂定频：绝对时间轴 sleep。
                // 相对 sleep(period-work) 会把每次醒晚误差累加 → 频率稳定偏低于 500Hz。
                // 绝对网格：醒晚后下一拍少睡，平均仍贴近 500Hz；仅严重落后才丢拍重同步。
                const double periodSec = 1.0 / std::max(controlFrequency, 1.0);
                auto advanceNextTime = [&](struct timespec& t) {
                    t.tv_nsec += static_cast<long>(periodSec * 1e9);
                    while (t.tv_nsec >= 1000000000L) {
                        t.tv_sec += 1;
                        t.tv_nsec -= 1000000000L;
                    }
                };
                auto timespecDiffSec = [](const struct timespec& a, const struct timespec& b) {
                    return static_cast<double>(a.tv_sec - b.tv_sec) +
                           static_cast<double>(a.tv_nsec - b.tv_nsec) * 1e-9;
                };

                controller_wheel_real_ptr_->update(ros::Time::now(), elapsedTime);

                if (e2eDeadline.enabled()) { e2eDeadline.wait(); } else {
                advanceNextTime(next_time);
                struct timespec now {};
                clock_gettime(CLOCK_MONOTONIC, &now);
                const double lagSec = timespecDiffSec(now, next_time);  // >0 表示已经超过目标时刻

                if (lagSec <= 0.0) {
                    // 早到：睡到绝对目标；醒晚由下一拍少睡补偿，不累积
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
                } else if (lagSec < 2.0 * periodSec) {
                    // 轻微超时：不睡、不改时间轴（避免相对 sleep 那样永久掉频）
                } else {
                    // 严重落后：丢拍对齐，避免长时间追赶连打
                    int skipped = 0;
                    while (timespecDiffSec(now, next_time) > 0.0) {
                        advanceNextTime(next_time);
                        ++skipped;
                        if (skipped > 1000) {  // 防护
                            next_time = now;
                            advanceNextTime(next_time);
                            break;
                        }
                    }
                }
                }

                // Add print statement if the cycle time exceeds 1 second
                const auto cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count();
                if (cycleTime > 1000)
                {
                    ROS_ERROR_STREAM("WBC Cycle time exceeded 1 second: " << cycleTime << "ms");
                }
            }
            else
            {
                // 暂停状态：休眠避免 CPU 空转
                usleep(10000);  // 10ms
            }
        }
        std::cout << "controlLoop: Exiting control loop" << std::endl;
    }
};

PLUGINLIB_EXPORT_CLASS(WheelAbsoluteControllerNodelet, nodelet::Nodelet)
