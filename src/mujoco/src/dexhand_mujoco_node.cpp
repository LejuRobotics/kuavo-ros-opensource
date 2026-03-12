#include "dexhand_mujoco_node.h"
#include "sensor_msgs/JointState.h"
#include <tuple>
#include <string>
#include <array>
#include <iostream>
#include "kuavo_assets/include/package_path.h"


namespace mujoco_node {
using namespace eef_controller;



DexHandMujocoRosNode::~DexHandMujocoRosNode() {
    stop();
}

bool DexHandMujocoRosNode::init(ros::NodeHandle& nh, 
    const mjModel* model, 
    const JointGroupAddress &r_hand_address,
    const JointGroupAddress &l_hand_address,
    double frequency) {

    nh_ = nh;
    frequency_ = frequency;

    finger_count_ = std::tuple_size<UnsignedFingerArray>::value;
    hand_count_ = std::tuple_size<UnsignedDualHandsArray>::value;

    /* ROS */
    auto right_hand_callback = [this](const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
        this->controlSingleHand(HandSide::RIGHT, msg);
    };

    auto left_hand_callback = [this](const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
        this->controlSingleHand(HandSide::LEFT, msg);
    };

    command_sub_ = nh_.subscribe("dexhand/command", 10, &DexHandMujocoRosNode::dualHandCommandCallback, this);
    r_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/right/command", 10, right_hand_callback);
    l_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/left/command", 10, left_hand_callback);
    status_pub_ = nh_.advertise<sensor_msgs::JointState>("dexhand/state", 10);
    
    // 兼容原来的接口
    hand_sub_ = nh_.subscribe("control_robot_hand_position", 10, &DexHandMujocoRosNode::controlHandCallback, this);
    gesture_execute_srv_ = nh_.advertiseService("gesture/execute", &DexHandMujocoRosNode::gestureExecuteCallback, this);
    gesture_list_srv_ = nh_.advertiseService("gesture/list", &DexHandMujocoRosNode::gestureListCallback, this);
    gesture_exec_state_srv_ = nh_.advertiseService("gesture/execute_state", &DexHandMujocoRosNode::gestureExecuteStateCallback, this);
    
    /* Initialize controller */
    l_dexhand_ = std::make_shared<MujocoDexHand>(model, l_hand_address);
    r_dexhand_ = std::make_shared<MujocoDexHand>(model, r_hand_address);

    auto kuavo_assets_path = ocs2::kuavo_assets::getPath();
    std::string gesture_filepath = kuavo_assets_path + "/config/gesture/preset_gestures.json";
    controller_ = DexhandController::Create(gesture_filepath);
    if(!controller_->init(l_dexhand_, r_dexhand_)) {
        std::cerr << "[DexHandMujocoRosNode] Failed to initialize." << std::endl;
        return false;
    }
    
    /* create publish thread */
    running_ = true;
    publish_thread_ = std::thread(&DexHandMujocoRosNode::publish_loop, this);
    
    return true;
}

void DexHandMujocoRosNode::stop() {
    
    running_ = false;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
}

void DexHandMujocoRosNode::readCallback(const mjData *d)
{
    if(l_dexhand_) l_dexhand_->readCallback(d);
    if(r_dexhand_) r_dexhand_->readCallback(d);
}

void DexHandMujocoRosNode::writeCallback(mjData *d)
{
    if(l_dexhand_) l_dexhand_->writeCallback(d);
    if(r_dexhand_) r_dexhand_->writeCallback(d);
}

void DexHandMujocoRosNode::publish_loop()
{
    while (running_)  {
        auto finger_status = controller_->get_finger_status();

        // Publish finger status
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"l_thumb", "l_thumb_aux", "l_index", "l_middle", "l_ring", "l_pinky",
                           "r_thumb", "r_thumb_aux", "r_index", "r_middle", "r_ring", "r_pinky"};

        // Add joint names and positions for both hands
        for (int hand = 0; hand < 2; hand++) {
            for (int finger = 0; finger < 6; finger++) {
                joint_state.position.push_back(finger_status[hand]->positions[finger]);
                joint_state.velocity.push_back(finger_status[hand]->speeds[finger]); 
                joint_state.effort.push_back(finger_status[hand]->currents[finger]);
            }
        }

        status_pub_.publish(joint_state);

        // Sleep to maintain publish frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0/frequency_)));
    }
}

int DexHandMujocoRosNode::get_hand_joints_num() {
    return finger_count_ * hand_count_;
}

void DexHandMujocoRosNode::dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    /* check control mode*/
    if(msg->control_mode != kuavo_msgs::dexhandCommand::POSITION_CONTROL 
        && msg->control_mode != kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
            std::cerr << "[DexHandMujocoRosNode] Unsupported control mode:" << static_cast<int>(msg->control_mode) << std::endl;
            return ;
    }

    constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value;
    constexpr int hand_count = std::tuple_size<UnsignedDualHandsArray>::value;

    if (msg->data.size() != finger_count*hand_count) {
        std::cerr << "[DexHandMujocoRosNode] Invalid data size:" << msg->data.size() << "!= " << finger_count*hand_count << std::endl;
    }

    if(msg->control_mode == kuavo_msgs::dexhandCommand::POSITION_CONTROL) {
        UnsignedDualHandsArray positions;
        // Assign values to left and right hands
        for(int i = 0; i < finger_count; i++) {
            positions[0][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i]));
            positions[1][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i + finger_count]));
        }
        controller_->send_position(positions);
    } else if(msg->control_mode == kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
        DualHandsArray vels;
        // Assign values to left and right hands
        for(int i = 0; i < finger_count; i++) {
            vels[0][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i]));
            vels[1][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i + finger_count]));
        }
        controller_->send_speed(vels);
    }
}

void DexHandMujocoRosNode::controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    /* check control mode*/
    if(msg->control_mode != kuavo_msgs::dexhandCommand::POSITION_CONTROL 
        && msg->control_mode != kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
            std::cerr << "[DexHandMujocoRosNode] Unsupported control mode:" << static_cast<int>(msg->control_mode) << std::endl;
            return ;
    }

    constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value; // 获取大小
    if (msg->data.size() != finger_count) {
        std::cerr << "[DexHandMujocoRosNode] Invalid data size:" << msg->data.size() << "!= " << finger_count << std::endl;
    }

    if(msg->control_mode == kuavo_msgs::dexhandCommand::POSITION_CONTROL) {
        UnsignedFingerArray positions;
        for(int i = 0; i < finger_count; i++) {
            positions[i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i]));
        }
        
       if (side == HandSide::LEFT) {
            controller_->send_left_position(positions);
        }
        else if (side == HandSide::RIGHT) {
            controller_->send_right_position(positions);
        }
    } else if(msg->control_mode == kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
        FingerArray vels;
        for(int i = 0; i < finger_count; i++) {
            vels[i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i]));
        }
        if (side == HandSide::LEFT) {
            controller_->send_left_speed(vels);
        }
        else if (side == HandSide::RIGHT) {
            controller_->send_right_speed(vels);
        }
    }
}


void DexHandMujocoRosNode::controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    constexpr int finger_count = std::tuple_size<UnsignedFingerArray>::value;
    constexpr int hand_count = std::tuple_size<UnsignedDualHandsArray>::value;

    if (msg->left_hand_position.size() != finger_count || msg->right_hand_position.size() != finger_count) 
    {
        ROS_WARN("Received desired positions vector of incorrect size");
        return;
    }

    UnsignedDualHandsArray positions;
    // Assign values to left and right hands
    for(int i = 0; i < finger_count; i++) {
        positions[0][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->left_hand_position[i]));
        positions[1][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->right_hand_position[i]));
    }
    
    controller_->send_position(positions);
}

/* gesture execute service. */
bool DexHandMujocoRosNode::gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                        kuavo_msgs::gestureExecuteResponse &res)
{
    if (!controller_) {
        res.success = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    std::string err_msg;
    GestureExecuteInfoVec gesture_tasks;
    for(const auto & gs : req.gestures) {
        gesture_tasks.push_back(GestureExecuteInfo{static_cast<HandSide>(gs.hand_side), gs.gesture_name});
    }
    
    if(gesture_tasks.size() == 0) {
        res.success = false;
        res.message = "No gesture to execute.";
        return false;
    }

    /* execute gestures */
    bool exec_ret = controller_->execute_gestures(gesture_tasks, err_msg);
    if(exec_ret) {
        res.message = "success.";
        res.success = true;
    }    
    else {
        res.message = err_msg;
        res.success = false;
    }
    return true;
}

/* gesture list service. */
bool DexHandMujocoRosNode::gestureListCallback(kuavo_msgs::gestureListRequest &req,
                        kuavo_msgs::gestureListResponse &res)
{
    if (!controller_) {
        res.gesture_infos.clear();
        res.gesture_count  = 0;
        res.success = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    /* convert gesture info to kuavo_msgs::gestureInfo */
    for(const auto &gesture_info : controller_->list_gestures()) {
        kuavo_msgs::gestureInfo msg;
        msg.gesture_name = gesture_info->gesture_name;
        msg.alias = gesture_info->alias;
        msg.description = gesture_info->description;
        res.gesture_infos.push_back(msg);
    }

    res.success = true;
    res.message = "success";
    res.gesture_count = res.gesture_infos.size();
    return true;
}

/* gesture execute state service. */
bool DexHandMujocoRosNode::gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
                        kuavo_msgs::gestureExecuteStateResponse &res)
{
    if (!controller_) {
        res.is_executing = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    // res.is_executing = controller_->is_gesture_executing();        
    res.message = "success";
    
    return true;
}

} // namespace 
