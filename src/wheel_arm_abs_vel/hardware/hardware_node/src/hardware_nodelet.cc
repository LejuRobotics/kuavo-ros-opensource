#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <signal.h>
#include <exception>
#include <iostream>
#include "hardware_node.h"

static volatile sig_atomic_t shutdown_signal_received_ = 0;

class AbsoluteHardwareNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        NODELET_INFO("Initializing hardware nodelet...");
        try
        {
            nh = getNodeHandle();
            double controlFrequency = 1000;
            if (nh.hasParam("/sensor_frequency"))
            {
                nh.getParam("/sensor_frequency", controlFrequency);
            }
            NODELET_INFO("Sensor data frequency: %f", controlFrequency);
            NODELET_INFO("[AbsoluteHardwareNodelet] creating HardwareNode...");
            hardware_node_ptr = std::make_unique<HighlyDynamic::HardwareNode>(nh, 1 / controlFrequency);
            NODELET_INFO("[AbsoluteHardwareNodelet] HardwareNode created, calling init()...");
            hardware_node_ptr->init();
            NODELET_INFO("[AbsoluteHardwareNodelet] HardwareNode init() finished, starting wait thread...");
            control_thread = std::thread(&AbsoluteHardwareNodelet::controlLoop, this);

            stop_pub_ = nh.advertise<std_msgs::Bool>("/stop_robot", 10, true);
            shutdown_timer_ = nh.createWallTimer(ros::WallDuration(0.01),
                                                 &AbsoluteHardwareNodelet::shutdownTimerCallback,
                                                 this);

            // Register signal handlers
            signal(SIGINT, AbsoluteHardwareNodelet::signalHandler);
            signal(SIGTERM, AbsoluteHardwareNodelet::signalHandler);
            signal(SIGHUP, AbsoluteHardwareNodelet::signalHandler);

            NODELET_INFO("Hardware nodelet initialized.");
        }
        catch (const std::exception& e)
        {
            NODELET_FATAL_STREAM("[AbsoluteHardwareNodelet] onInit exception: " << e.what());
            std::cerr << "[AbsoluteHardwareNodelet] onInit exception: " << e.what() << std::endl;
            throw;
        }
        catch (...)
        {
            NODELET_FATAL("[AbsoluteHardwareNodelet] onInit unknown exception");
            std::cerr << "[AbsoluteHardwareNodelet] onInit unknown exception" << std::endl;
            throw;
        }
    }
    ~AbsoluteHardwareNodelet() override {
        NODELET_INFO("AbsoluteHardwareNodelet is shutting down... joinable=%d", control_thread.joinable());
        shutdown_timer_.stop();
        if (hardware_node_ptr) hardware_node_ptr->requestShutdown();
        if (control_thread.joinable()) control_thread.join();
        hardware_node_ptr.reset();
    }


private:
    std::thread control_thread;
    ros::Publisher stop_pub_;
    ros::WallTimer shutdown_timer_;

    static void signalHandler(int signum)
    {
        (void)signum;
        if (shutdown_signal_received_ == 0) {
            shutdown_signal_received_ = 1;
        }
    }

    void shutdownTimerCallback(const ros::WallTimerEvent &)
    {
        if (shutdown_signal_received_ != 1) {
            return;
        }
        shutdown_signal_received_ = 2;

        NODELET_INFO("Shutdown signal received, publishing latched /stop_robot");
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        stop_pub_.publish(stop_msg);
    }

    void controlLoop()
    {
        try {
            hardware_node_ptr->real_init_wait();
        } catch (const std::exception& error) {
            NODELET_FATAL("Hardware startup failed: %s", error.what());
            hardware_node_ptr->requestShutdown();
            ros::shutdown();
        }
    }
    ros::NodeHandle nh;
    std::unique_ptr<HighlyDynamic::HardwareNode> hardware_node_ptr;
};

PLUGINLIB_EXPORT_CLASS(AbsoluteHardwareNodelet, nodelet::Nodelet)
