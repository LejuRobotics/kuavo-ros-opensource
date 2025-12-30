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
    
    // 新增退出相关函数 - 移到公有部分以便静态函数访问
    void cleanupAndExit();

private:
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    bool simStartCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void OnUpdate(const common::UpdateInfo& _info);
    bool ParseImu(const sdf::ElementPtr& _sdf);
    bool ParseJoints(const sdf::ElementPtr& _sdf);
    bool ParseContacts(const sdf::ElementPtr& _sdf);
    
    // 新增初始化相关函数
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
    
    // 共享内存管理
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    
    // 命令读取线程
    std::unique_ptr<std::thread> cmd_thread_;
    std::atomic<bool> running_;
    
    // 数据缓存
    gazebo_shm::SensorsData sensors_data_;
    gazebo_shm::JointCommand joint_cmd_;
    std::mutex cmd_mutex_;
    
    // 新增初始化相关变量
    std::vector<double> robot_init_state_param_;
    bool params_loaded_ = false;
    bool sim_start_ = false;
    
    // ROS相关
    ros::NodeHandle* nh_;
    ros::Subscriber stop_sub_;
    ros::ServiceServer sim_start_srv_;
};

} 
