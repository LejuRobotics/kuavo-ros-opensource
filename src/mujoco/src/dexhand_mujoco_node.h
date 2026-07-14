#ifndef TOUCH_DEXHAND_ROS1_H
#define TOUCH_DEXHAND_ROS1_H
#include <ros/ros.h>
#include <thread>
#include <string>
#include "joint_address.hpp"
#include "dexhand/dexhand_def.h"
#include "dexhand/dexhand_controller.h"
#include "dexhand/mujoco_hand_base.hpp"
#include "sensor_msgs/JointState.h"

#include "kuavo_msgs/dexhandCommand.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/gestureExecute.h"
#include "kuavo_msgs/gestureList.h"
#include "kuavo_msgs/gestureExecuteState.h"
#include "std_msgs/Bool.h"
#include <atomic>

namespace mujoco_node {
using namespace eef_controller;

enum class HandType {
    QIANGNAO,  // 老的强脑手，范围0-100
    LINKER_L6, // 新的LinkerL6灵巧手，范围0-255
    LINKER_O6  // 新的LinkerO6灵巧手，范围0-255
};

class DexHandMujocoRosNode {
public:
    DexHandMujocoRosNode() = default;
    ~DexHandMujocoRosNode();

    /**
     * @brief Initialize the TouchDexHand node
     * @return true if initialization is successful, false otherwise
     */
    bool init(ros::NodeHandle& nh,
        const mjModel* model,
        const JointGroupAddress &r_hand_address,
        const JointGroupAddress &l_hand_address,
        HandType hand_type = HandType::QIANGNAO,
        double frequency = 500.0);

    /**
     * @brief Stop the TouchDexHand node and cleanup resources
     */
    void stop();

    /**
     * @brief Get the hand joints num
     * @return the hand joints num
     */
    int get_hand_joints_num();
    
    void readCallback(const mjData *d);
    void writeCallback(mjData *d);

private:
    void dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    void controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    // 兼容原来的 control_robot_hand_position 接口（强脑手使用，范围0-100）
    void controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg);

    // LinkerO6手使用的control_robot_hand_position接口（范围0-100）
    void linkerO6ControlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg);

    // Linker系列灵巧手的控制指令回调
    void linkerLeftHandCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void linkerRightHandCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // enable control callback
    void enableControlCallback(const std_msgs::Bool::ConstPtr& msg);

    /* gesture execute service. */
    bool gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                          kuavo_msgs::gestureExecuteResponse &res);
    /* gesture list service. */
    bool gestureListCallback(kuavo_msgs::gestureListRequest &req,
                          kuavo_msgs::gestureListResponse &res);
    /* gesture execute state service. */
    bool gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
                          kuavo_msgs::gestureExecuteStateResponse &res);
    void publish_loop();
    
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Subscriber r_hand_command_sub_;
    ros::Subscriber l_hand_command_sub_;
    ros::Publisher status_pub_;

    // Linker系列灵巧手的话题订阅者
    ros::Subscriber linker_l_hand_command_sub_;
    ros::Subscriber linker_r_hand_command_sub_;
    // Linker系列灵巧手的状态发布者
    ros::Publisher l_hand_state_pub_;
    ros::Publisher r_hand_state_pub_;

    // enable control
    ros::Subscriber enable_control_state_sub_;
    std::atomic<bool> enable_control_{true};
    
    // 兼容原来的 control_robot_hand_position 接口
    ros::Subscriber hand_sub_;  
    
    // 兼容原来的手势接口
    ros::ServiceServer gesture_execute_srv_;
    ros::ServiceServer gesture_list_srv_;
    ros::ServiceServer gesture_exec_state_srv_;
    
    DexhandControllerPtr controller_ = nullptr;
    std::thread publish_thread_;
    bool running_{false};

    int finger_count_;
    int hand_count_;
    double frequency_;
    HandType hand_type_;

    MujocoHandBasePtr r_dexhand_ = nullptr;
    MujocoHandBasePtr l_dexhand_ = nullptr;
};

} // namespace eef_controller
#endif