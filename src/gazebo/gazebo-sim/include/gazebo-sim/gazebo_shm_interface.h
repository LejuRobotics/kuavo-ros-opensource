#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "humanoid_controllers/shm_manager.h"
#include <ros/ros.h>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

namespace gazebo
{

class GazeboShmInterface : public ModelPlugin
{
public:
    GazeboShmInterface();
    ~GazeboShmInterface();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    
    // 退出相关函数
    void cleanupAndExit();

private:
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    bool simStartCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void OnUpdate(const common::UpdateInfo& _info);
    bool ParseImu(const sdf::ElementPtr& _sdf);
    bool ParseJoints(const sdf::ElementPtr& _sdf);
    bool ParseContacts(const sdf::ElementPtr& _sdf);
    
    // 初始化相关函数
    void waitForParams();
    void setInitialState();
    void setModelConfiguration(const std::vector<double>& positions);
    void setModelState(const std::vector<double>& pose);
    
    physics::ModelPtr model_;
    event::ConnectionPtr updateConnection_;
    
    // IMU相关
    physics::LinkPtr imu_link_;
    std::string imu_frame_id_;
    
    // 关节相关
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    
    // 接触传感器相关
    std::vector<physics::LinkPtr> contact_links_;
    std::vector<std::string> contact_link_names_;
    physics::ContactManager* contact_manager_;
    
    // 共享内存
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    gazebo_shm::SensorsData sensors_data_;
    
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Subscriber stop_sub_;
    ros::ServiceServer sim_start_srv_;
    
    // 参数和状态
    std::vector<double> robot_init_state_param_;
    bool params_loaded_ = false;
    std::atomic<bool> sim_start_{false};
};

} 
