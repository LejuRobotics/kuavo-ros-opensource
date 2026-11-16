#include <ros/ros.h>
#include <csignal>
#include "hardware_node/sg100_hand_driver.h"

std::shared_ptr<SG100HandROSNode> g_node;
volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int) {
    // Never join threads or acquire the driver mutex from a signal handler.
    g_stop_requested = 1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sg100_hand_test", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("[sg100_test] Starting SG100 hand driver...");
    ROS_INFO("  Using hardware_node embedded driver");

    // ---- 配置断言(不依赖硬件即可执行) ----
    {
        std::vector<std::string> joint_names;
        std::vector<float> left_rest, right_rest;
        if (!loadSg100HardcodedDefaults(joint_names, left_rest, right_rest) ||
            joint_names.size() != 11 || left_rest.size() != 11 || right_rest.size() != 11) {
            ROS_FATAL("[sg100_test] config assertion failed: expected 11 joints, got "
                      "%zu/%zu/%zu",
                      joint_names.size(), left_rest.size(), right_rest.size());
            return 1;
        }
        for (const std::string& name : joint_names) {
            if (name.empty()) {
                ROS_FATAL("[sg100_test] config assertion failed: empty joint name");
                return 1;
            }
        }
        ROS_INFO("[sg100_test] config assertions passed (11 joint names + rest poses)");
    }

    g_node = std::make_shared<SG100HandROSNode>(nh, pnh);
    if (!g_node->initialize()) {
        ROS_FATAL("[sg100_test] Init failed");
        return 1;
    }

    ROS_INFO("[sg100_test] Initialized. Spinning...");
    ros::WallRate rate(100);
    while (ros::ok() && !g_stop_requested) {
        ros::spinOnce();
        rate.sleep();
    }
    g_node->shutdown();
    ros::shutdown();
    return 0;
}
