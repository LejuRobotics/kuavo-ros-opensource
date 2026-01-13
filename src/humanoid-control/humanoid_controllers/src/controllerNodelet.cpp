
#include "humanoid_controllers/humanoidController.h"
#include "humanoid_controllers/humanoidWheelController.h"
#include "humanoid_controllers/humanoidController_wheel_wbc.h"
#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

using Duration = std::chrono::duration<double>;
using Clock = std::chrono::high_resolution_clock;
static ros::Publisher stop_pub_; 

class HumanoidControllerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        NODELET_INFO("Initializing HumanoidControllerNodelet nodelet...");
        nh = getNodeHandle();
        ros::param::set("/nodelet_manager/controller_state", 0);
        robot_hw = new humanoid_controller::HybridJointInterface(); // TODO:useless
        pause_sub = nh.subscribe<std_msgs::Bool>("pauseFlag", 1, &HumanoidControllerNodelet::pauseCallback, this);
        stop_pub_ = nh.advertise<std_msgs::Bool>("/stop_robot", 10);
        stop_sub_ = nh.subscribe<std_msgs::Bool>("/stop_robot", 1, &HumanoidControllerNodelet::stopCallback, this);
        control_thread = std::thread(&HumanoidControllerNodelet::controlLoop, this);
        // signal(SIGINT, signalHandler);
        // signal(SIGTERM, signalHandler);

        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = signalHandler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        // sigaction(SIGINT, &sa, NULL);
        sigaction(SIGTERM, &sa, NULL); 

        NODELET_INFO("HumanoidControllerNodelet nodelet initialized.");
    }
    void controllerExit()
    {
        std::cerr << "[controllerNodelet] controllerExit called" << std::endl;
        ros::param::set("/nodelet_manager/controller_state", 1);
        is_running = false;// 先停止控制器


        // 检查nodelet状态
        const double max_wait_seconds = 20.0; // 最大等待5秒
        const int check_interval_ms = 100;   // 每100ms检查一次
        const int max_checks = static_cast<int>(max_wait_seconds * 1000 / check_interval_ms);
        int check_count = 0;
        while (check_count < max_checks && ros::ok())
        {
            int logger_state, controller_state, hardware_state;
            //  ros::param::has("/nodelet_manager/controller_state") && ros::param::get("/nodelet_manager/controller_state", controller_state);
            // ros::param::has("/nodelet_manager/hardware_state") && ros::param::get("/nodelet_manager/hardware_state", hardware_state);
            if (ros::param::has("/nodelet_manager/logger_state") && ros::param::get("/nodelet_manager/logger_state", logger_state))
            {
                if (logger_state == 2)
                {
                    ROS_INFO("[HumanoidControllerNodelet] Nodelets ready to exit, state: 1");
                    break;
                }
            }
            
            usleep(check_interval_ms * 1000); // 转换为微秒
            check_count++;
            
            // 输出剩余等待时间
            double remaining_time = max_wait_seconds - (check_count * check_interval_ms / 1000.0);
            if (check_count % 10 == 0) // 每秒输出一次
            {
                ROS_INFO_STREAM("[HumanoidControllerNodelet] Waiting for nodelets to be ready... " << std::fixed << std::setprecision(1) 
                              << remaining_time << "s remaining, current state: " 
                              << logger_state);
            }
        }
        
        if (check_count >= max_checks)
        {
            ROS_WARN("[HumanoidControllerNodelet] Timeout after %.1f seconds waiting for nodelets", max_wait_seconds);
        }
        
        ros::shutdown();

    }
    static void signalHandler(int sig)
    {
        std::cerr << "[HumanoidControllerNodelet] signal handler called with SIGINT"<< std::endl;
        // 发布停止信号
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        
        // 发布几次确保消息被接收
        for(int i = 0; i < 3; i++) {
            stop_pub_.publish(stop_msg);
            usleep(10000); // 等待10ms
        }
        
    }
    ~HumanoidControllerNodelet()
    {
       std::cerr << "[HumanoidControllerNodelet] destructor called" << std::endl;
    }

private:
    bool pause_flag{false};
    ros::NodeHandle nh;
    bool is_running{true};
    humanoid_controller::HybridJointInterface *robot_hw;
    ros::Subscriber pause_sub;

    humanoid_controller::humanoidController *controller_ptr_;
    humanoid_wheel_controller::humanoidWheelController *controller_wheel_ptr_;
    humanoidController_wheel_wbc::humanoidControllerWheelWbc *controller_wheel_real_ptr_;
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
        if (msg->data)
        {
            is_running = false;
            std::cerr << "[controllerNodelet] stopCallback: " << is_running << std::endl;
            controllerExit();
        }
    }

    void controlLoop()
    {
        // Choose the controller type with estimator based on the parameter
        int estimator_type = 1;
        // 先设置默认值
        int nodelet_robot_type = 2;     // 默认值：2表示双足人形机器人
        // 从参数服务器读取参数（如果存在则覆盖默认值）
        if (nh.hasParam("/nodelet_robot_type"))
        {
            nh.getParam("/nodelet_robot_type", nodelet_robot_type);
            std::cout << "[controllerNodelet] Found /nodelet_robot_type parameter: " << nodelet_robot_type << std::endl;
            // 读取后删除旧参数，确保下次启动时使用launch文件设置的新值
            nh.deleteParam("/nodelet_robot_type");
        }
        else
        {
            std::cout << "[controllerNodelet] /nodelet_robot_type not found, using default: " << nodelet_robot_type << std::endl;
        }
        
        int wheel_control_type = 0;
        if (nh.hasParam("/wheel_control_type"))
        {
            nh.getParam("/wheel_control_type", wheel_control_type);
        }
        else
        {
            wheel_control_type = 0;     // 没设置 wheel_control_type 一定是测试的轮臂控制器
        }

        bool with_estimation = false;
        if (nh.hasParam("/estimator_type"))
        {
            nh.getParam("/estimator_type", estimator_type);
        }
        else
        {
            ROS_INFO("estimator_type not found in parameter server");
        }
        if (estimator_type == 1)
        {
            std::cout << "Using nomal estimator" << std::endl;
            controller_ptr_ = new humanoid_controller::humanoidController();
            controller_wheel_ptr_ = new humanoid_wheel_controller::humanoidWheelController();
            controller_wheel_real_ptr_ = new humanoidController_wheel_wbc::humanoidControllerWheelWbc();
        }
        else if (estimator_type == 2)
        {
#ifdef KUAVO_CONTROL_LIB_FOUND

            std::cout << "Using inEKF estimator" << std::endl;
            controller_ptr_ = new humanoid_controller::humanoidKuavoController();
#else
            ROS_ERROR("Kuavo control library not found. Please make sure to clone submodule kuavo-ros-control-lejulib into src/ and rebuild the workspace.");
            exit(1);
#endif
        }
        else
        {
            std::cout << "Using cheater estimator" << std::endl;
            controller_ptr_ = new humanoid_controller::humanoidCheaterController();
        }
        // Initialize the controller
        if(nodelet_robot_type == 1)
        {
            if(wheel_control_type == 0)
            {
                if (!controller_wheel_ptr_->init(nh, true))
                {
                    ROS_ERROR("Failed to initialize the wheel humanoid controller!");
                    return;
                }
            }
            else
            {
                if (!controller_wheel_real_ptr_->init(nh, true))
                {
                    ROS_ERROR("Failed to initialize the wheel humanoid controller!");
                    return;
                }
            }
            
        } 
        else
        {
            if (!controller_ptr_->init(robot_hw, nh, true))
            {
                ROS_ERROR("Failed to initialize the humanoid controller!");
                return;
            }
        }
        
        // Time setup record start time in both system and ros time
        // Calls controller's starting() method to initialize the control state
        // Sets up control frequency (default 500Hz, configurable via parameter)
        auto startTime = std::chrono::high_resolution_clock::now();
        auto startTimeROS = ros::Time::now();
        
        // 调用 starting() 并检查是否成功
        bool starting_success = true;
        if(nodelet_robot_type == 1)
        {
            if(wheel_control_type == 0)
            {
                // humanoidWheelController::starting() 返回 void
                controller_wheel_ptr_->starting(startTimeROS);
                starting_success = true;
            }
            else
            {
                // humanoidControllerWheelWbc::starting() 返回 bool
                starting_success = controller_wheel_real_ptr_->starting(startTimeROS);
            }
        }
        else
        {
            // humanoidController::starting() 返回 void
            controller_ptr_->starting(startTimeROS);
            starting_success = true;
        }
        
        // 检查starting是否成功
        if (!starting_success) {
            ROS_ERROR("Controller starting failed! Data not ready within timeout. Exiting...");
            is_running = false;
            ros::shutdown();
            return;
        }
        
        lastTime = startTime;
        double controlFrequency = 500.0; // 500Hz
        nh.getParam("/wbc_frequency", controlFrequency);
        ROS_INFO_STREAM("Wbc control frequency: " << controlFrequency);
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        ros::Rate rate(controlFrequency);
        // uint64_t cycle_count = 0;

        // Pre-Update Initialization Phase
        // Runs until preUpdateComplete() returns true
        // This phase likely waits for sensors to provide initial data
        if(nodelet_robot_type == 1)
        {
            if(wheel_control_type == 0)
            {
                while (is_running && ros::ok() && controller_wheel_ptr_->preUpdateComplete() != true)
                {
                    bool preUpdateExeFlag = controller_wheel_ptr_->preUpdate(ros::Time::now());
                    if(!preUpdateExeFlag)
                    {
                        ROS_INFO_STREAM("re-squat state, waitting is_real signal...");
                        controller_wheel_ptr_->starting(startTimeROS);  // 返回 void
                    }
                }
            }
            else
            {
                while (is_running && ros::ok() && controller_wheel_real_ptr_->preUpdateComplete() != true)
                {
                    bool preUpdateExeFlag = controller_wheel_real_ptr_->preUpdate(ros::Time::now());
                    if(!preUpdateExeFlag)
                    {
                        ROS_INFO_THROTTLE(1.0, "re-squat state, waitting is_real signal...");
                        // if (!controller_wheel_real_ptr_->starting(startTimeROS)) {  // 返回 bool，检查失败
                        //     ROS_ERROR("Re-starting failed! Exiting...");
                        //     is_running = false;
                        //     ros::shutdown();
                        //     return;
                        // }
                    }
                }
            }
        }
        else
        {
            while (is_running && ros::ok() && controller_ptr_->preUpdateComplete() != true)
            {
                bool preUpdateExeFlag = controller_ptr_->preUpdate(ros::Time::now());
                if(!preUpdateExeFlag)
                {
                    ROS_INFO_STREAM("re-squat state, waitting is_real signal...");
                    controller_ptr_->starting(startTimeROS);
                }
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

                // Control
                if(nodelet_robot_type == 1)
                {
                    if(wheel_control_type == 0)
                    {
                        controller_wheel_ptr_->update(ros::Time::now(), elapsedTime);
                    }
                    else
                    {
                        controller_wheel_real_ptr_->update(ros::Time::now(), elapsedTime);
                    }
                    // 轮式机器人使用固定频率
                    next_time.tv_sec += (next_time.tv_nsec + 1 / controlFrequency * 1e9) / 1e9;
                    next_time.tv_nsec = (int)(next_time.tv_nsec + 1 / controlFrequency * 1e9) % (int)1e9;
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
                }
                else
                {
                    controller_ptr_->update(ros::Time::now(), elapsedTime);
                    // 双足机器人使用控制器自身的频率控制（支持不同控制器不同频率）
                    controller_ptr_->waitForNextCycle();
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

PLUGINLIB_EXPORT_CLASS(HumanoidControllerNodelet, nodelet::Nodelet)
