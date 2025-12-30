#include "gazebo-sim/gazebo_shm_interface.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <signal.h>
#include <thread>

namespace gazebo
{

GazeboShmInterface::GazeboShmInterface()
    
{
    shm_manager_ = std::make_unique<gazebo_shm::ShmManager>();
}

GazeboShmInterface::~GazeboShmInterface()
{
    if (nh_) {
        delete nh_;
        nh_ = nullptr;
    }
}

void GazeboShmInterface::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        std::cout << "Received stop command, shutting down simulation..." << std::endl;
        // 3. 快速清理共享内存
        if (shm_manager_) {
            shm_manager_.reset();
            std::cout << "✓ Shared memory cleaned up" << std::endl;
        }
        // 启动一个线程来执行退出，避免阻塞
        std::thread exit_thread([this]() {
            cleanupAndExit();
        });
        exit_thread.detach();  // 分离线程，让它独立运行
        
        // 同时立即发送信号
        std::cout << "Immediately sending shutdown signals..." << std::endl;
        system("pkill -9 gazebo &");
        system("pkill -9 gzserver &");
        system("pkill -9 gzclient &");
        
        // 发送SIGTERM
        std::raise(SIGTERM);
    }
}

bool GazeboShmInterface::simStartCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    if (req.data) {
        std::cout << "Starting simulation..." << std::endl;
        if (model_ && model_->GetWorld()) {
            sim_start_ = true;
            model_->GetWorld()->SetPaused(false);
            res.success = true;
            res.message = "Simulation started successfully";
        } else {
            res.success = false;
            res.message = "Failed to start simulation: world or model not available";
        }
    } else {
        std::cout << "Pausing simulation..." << std::endl;
        if (model_ && model_->GetWorld()) {
            sim_start_ = false;
            model_->GetWorld()->SetPaused(true);
            res.success = true;
            res.message = "Simulation paused successfully";
        } else {
            res.success = false;
            res.message = "Failed to pause simulation: world or model not available";
        }
    }
    return true;
}

void GazeboShmInterface::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    model_ = _parent;

    // 打印模型总质量
    double total_mass = 0.0;
    auto links = model_->GetLinks();
    for (const auto& link : links) {
        total_mass += link->GetInertial()->Mass();
    }
    std::cout << "GazeboShmInterface: Model total mass: " << total_mass << " kg" << std::endl;

    // 初始化ROS节点
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_shm_interface",
                 ros::init_options::NoSigintHandler);
    }
    nh_ = new ros::NodeHandle();
    
    stop_sub_ = nh_->subscribe("/stop_robot", 1, &GazeboShmInterface::stopCallback, this);
    sim_start_srv_ = nh_->advertiseService("sim_start", &GazeboShmInterface::simStartCallback, this);

    // 注册信号处理器
    signal(SIGTERM, [](int sig) {
        std::cout << "Received SIGTERM, forcing Gazebo shutdown..." << std::endl;
        // 立即发送pkill命令
        system("pkill -9 gazebo &");
        system("pkill -9 gzserver &");
        system("pkill -9 gzclient &");
        // 触发Gazebo关闭
        event::Events::sigInt();
    });
    
    signal(SIGINT, [](int sig) {
        std::cout << "Received SIGINT, forcing Gazebo shutdown..." << std::endl;
        // 立即发送pkill命令
        system("pkill -9 gazebo &");
        system("pkill -9 gzserver &");
        system("pkill -9 gzclient &");
        // 触发Gazebo关闭
        event::Events::sigInt();
    });
    
    signal(SIGKILL, [](int sig) {
        std::cout << "Received SIGKILL, immediately killing Gazebo..." << std::endl;
        // 强制杀死所有Gazebo相关进程
        system("pkill -9 -f gazebo");
        system("pkill -9 -f gzserver");
        system("pkill -9 -f gzclient");
        exit(1);
    });

    // 从ROS参数服务器读取dt，如果没有设置则使用默认值0.002
    double dt = 0.002;  // 默认值
    if (nh_->hasParam("/sensor_frequency")) {
        double freq;
        nh_->getParam("/sensor_frequency", freq);
        if (freq > 0) {
            dt = 1.0 / freq;
        }
    }
    
    // 设置Gazebo的更新频率
    auto physics = model_->GetWorld()->Physics();
    physics->SetMaxStepSize(dt);
    physics->SetRealTimeUpdateRate(1.0/dt);
    
    std::cout << "Setting simulation dt to: " << dt << " seconds (frequency: " << 1.0/dt << " Hz)" << std::endl;

    // 初始化共享内存
    std::cout << "Initializing shared memory..." << std::endl;
    if (!shm_manager_->initializeSensorsShm() || !shm_manager_->initializeCommandShm()) {
        gzerr << "Failed to initialize shared memory" << std::endl;
        return;
    }
    std::cout << "Shared memory initialized successfully" << std::endl;

    // 等待并加载参数
    waitForParams();
    
    // 设置初始状态
    setInitialState();

    // 解析配置
    if (!ParseImu(_sdf)) {
        gzerr << "Failed to parse IMU configuration" << std::endl;
        return;
    }

    std::cout << "Parsing joints configuration..." << std::endl;
    if (!ParseJoints(_sdf)) {
        gzerr << "Failed to parse joints configuration" << std::endl;
        return;
    }
    std::cout << "Joints configuration parsed successfully" << std::endl;

    if (!ParseContacts(_sdf)) {
        gzerr << "Failed to parse contacts configuration" << std::endl;
        return;
    }

    // 获取接触管理器
    contact_manager_ = model_->GetWorld()->Physics()->GetContactManager();
    contact_manager_->SetNeverDropContacts(true);

    // 直接注册更新回调
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboShmInterface::OnUpdate, this, std::placeholders::_1));

    gzlog << "GazeboShmInterface plugin loaded successfully" << std::endl;
}

void GazeboShmInterface::waitForParams()
{
    std::cout << "GazeboShmInterface waitForParams" << std::endl;
    int retry_count = 0;
    const int sleep_ms = 100;
    
    while (!params_loaded_  && ros::ok()) {
        // 首先检查参数是否存在
        if (!nh_->hasParam("robot_init_state_param")) {
            if (retry_count%20 == 0) {
                std::cout << "[GazeboShmInterface] Waiting for /robot_init_state_param ( "
                        << retry_count/10 << " s)" << std::endl;
            }
            retry_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            continue;
        }

        // 尝试获取参数
        XmlRpc::XmlRpcValue param;
        try {
            if (nh_->getParam("robot_init_state_param", param)) {
                if (param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                    std::cerr << "错误：'robot_init_state_param' 不是数组类型" << std::endl;
                    retry_count++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                    continue;
                }
                std::cout << "[GazeboShmInterface] get /robot_init_state_param param success" << std::endl;
                robot_init_state_param_.clear();
                bool param_valid = true;
                
                // 验证并转换参数
                for (int i = 0; i < param.size(); ++i) {
                    try {
                        double value = static_cast<double>(param[i]);
                        robot_init_state_param_.push_back(value);
                    } catch (const std::exception& e) {
                        std::cerr << "Error: parameter conversion failed, index " << i << ": " << e.what() << std::endl;
                        param_valid = false;
                        break;
                    }
                }
                
                if (param_valid && !robot_init_state_param_.empty()) {

                    // 打印参数值用于调试
                    std::cout << "robot_init_state_param: ";
                    for (size_t i = 0; i < robot_init_state_param_.size(); ++i) {
                        std::cout << robot_init_state_param_[i] << " ";
                    }
                    std::cout << std::endl;
                    
                    params_loaded_ = true;
                    break;
                }
            }
        } catch (const ros::Exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
        
        retry_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    
    if (!params_loaded_) {
        std::cerr << "Cannot load initial state: parameters not loaded" << std::endl;
        exit(1);
    }
}

void GazeboShmInterface::setInitialState()
{
    if (!params_loaded_) {
        std::cerr << "Cannot set initial state: parameters not loaded" << std::endl;
        return;
    }
    std::cout << "[GazeboShmInterface] setInitialState: " << robot_init_state_param_.size() << std::endl;

    // 设置base位姿
    if (robot_init_state_param_.size() >= 7) {
        std::vector<double> base_pose(robot_init_state_param_.begin(), robot_init_state_param_.begin() + 7);
        ignition::math::Pose3d new_pose;
        new_pose.Pos().Set(base_pose[0], base_pose[1], base_pose[2]);
        new_pose.Rot().Set(base_pose[3], base_pose[4], base_pose[5], base_pose[6]);  // w,x,y,z
        model_->SetWorldPose(new_pose);
        
        std::cout << "[GazeboShmInterface] Setting model pose to: "
                  << "pos[" << base_pose[0] << "," << base_pose[1] << "," << base_pose[2] << "] "
                  << "rot[" << base_pose[3] << "," << base_pose[4] << "," << base_pose[5] << "," << base_pose[6] << "]" << std::endl;
    }

    // 设置关节位置
    if (robot_init_state_param_.size() > 7) {
        std::vector<double> joint_positions(robot_init_state_param_.begin() + 7, robot_init_state_param_.end());
        
        // 使用map存储关节名称和位置
        std::map<std::string, double> joint_pos_map;
        
        for (size_t i = 0; i < joints_.size() && i < joint_positions.size(); ++i) {
            std::string joint_name = joints_[i]->GetName();
            joint_pos_map[joint_name] = joint_positions[i];
            std::cout << "[GazeboShmInterface] Setting joint " << joint_name << " to position: " << joint_positions[i] << std::endl;
        }
        
        // 使用Gazebo的接口设置关节位置
        model_->SetJointPositions(joint_pos_map);
    }
}

void GazeboShmInterface::setModelConfiguration(const std::vector<double>& positions)
{
    if (positions.empty()) return;
    
    // 准备关节名称和位置的map
    std::map<std::string, double> joint_positions;
    
    for (size_t i = 0; i < joints_.size() && i < positions.size(); ++i) {
        std::string joint_name = joints_[i]->GetName();
        joint_positions[joint_name] = positions[i];
        
        // 直接设置关节状态
        joints_[i]->SetPosition(0, positions[i], true);  // true表示强制更新
        joints_[i]->SetVelocity(0, 0.0);
        joints_[i]->SetForce(0, 0.0);
        
        // 重置关节的物理状态
        joints_[i]->Reset();
        
        std::cout << "Setting joint " << joint_name 
                  << " to position: " << positions[i] 
                  << ", velocity: 0.0, effort: 0.0" << std::endl;
    }
    
    // 使用Model的接口设置关节配置
    model_->SetJointPositions(joint_positions);
    
    // 重置模型的动力学状态
    model_->Reset();
    
    // 强制更新物理引擎
    if (model_->GetWorld()) {
        model_->GetWorld()->Physics()->InitForThread();
        model_->GetWorld()->Physics()->UpdatePhysics();
    }
}

void GazeboShmInterface::setModelState(const std::vector<double>& pose)
{
    if (pose.size() < 7) return;  // 需要x,y,z和四元数

    ignition::math::Pose3d new_pose;
    new_pose.Pos().Set(pose[0], pose[1], pose[2]);
    new_pose.Rot().Set(pose[3], pose[4], pose[5], pose[6]);  // w,x,y,z
    
    // 强制设置世界坐标系下的位姿
    model_->SetWorldPose(new_pose);
    
    // 确保速度为0
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
    
    std::cout << "Setting model pose to: "
              << "pos[" << pose[0] << "," << pose[1] << "," << pose[2] << "] "
              << "rot[" << pose[3] << "," << pose[4] << "," << pose[5] << "," << pose[6] << "]" 
              << " with zero velocity" << std::endl;
}

void GazeboShmInterface::OnUpdate(const common::UpdateInfo& _info)
{
    static bool first_update = true;
    if (first_update) {
        // 在第一次更新时设置初始状态
        setInitialState();
    }

    // 更新IMU数据
    if (imu_link_) {
        auto pose = imu_link_->WorldPose();
        auto rot = pose.Rot();
        
        sensors_data_.imu_data.orientation[0] = rot.X();
        sensors_data_.imu_data.orientation[1] = rot.Y();
        sensors_data_.imu_data.orientation[2] = rot.Z();
        sensors_data_.imu_data.orientation[3] = rot.W();

        auto ang_vel = imu_link_->RelativeAngularVel();
        sensors_data_.imu_data.angular_velocity[0] = ang_vel.X();
        sensors_data_.imu_data.angular_velocity[1] = ang_vel.Y();
        sensors_data_.imu_data.angular_velocity[2] = ang_vel.Z();

        ignition::math::Vector3d gravity = { 0., 0., -9.81 };
        ignition::math::Vector3d accel = imu_link_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
        sensors_data_.imu_data.linear_acceleration[0] = accel.X();
        sensors_data_.imu_data.linear_acceleration[1] = accel.Y();
        sensors_data_.imu_data.linear_acceleration[2] = accel.Z();
    }

    // 更新关节数据
    sensors_data_.num_joints = joints_.size();
    for (size_t i = 0; i < joints_.size(); ++i) {
        sensors_data_.joint_data[i].position = joints_[i]->Position(0);
        sensors_data_.joint_data[i].velocity = joints_[i]->GetVelocity(0);
        sensors_data_.joint_data[i].effort = joints_[i]->GetForce(0);
    }

    // 接触状态始终为false
    sensors_data_.end_effector_data.contact = false;

    // 更新时间戳
    sensors_data_.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 更新仿真器时间
    sensors_data_.sensor_time = _info.simTime.Double();
    
    // 写入共享内存
    shm_manager_->writeSensorsData(sensors_data_);

    // 应用关节命令
    if (first_update)
    {
        std::cout << "[GazeboShmInterface] first update" << std::endl;   
        first_update = false;
        for (size_t i = 0; i < joints_.size(); ++i) {
            joints_[i]->SetForce(0, 0);
        }
        return; 
    }

    // 直接使用同步读取接口读取命令
    gazebo_shm::JointCommand cmd;
    if (shm_manager_->readJointCommandSync(cmd, 100.0)) {  // 100ms超时，快速检查是否有新命令
        // 应用新的关节命令
        for (size_t i = 0; i < joints_.size() && i < cmd.num_joints; ++i) {
            double effort = cmd.tau[i];
     
            joints_[i]->SetForce(0, effort);
        }
    }
    while (!sim_start_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "[GazeboShmInterface] Waiting for simulation to start..." << std::endl;
    }
}

bool GazeboShmInterface::ParseImu(const sdf::ElementPtr& _sdf)
{
    if (!_sdf->HasElement("imu")) {
        gzerr << "No IMU configuration found in SDF" << std::endl;
        return false;
    }

    auto imu_elem = _sdf->GetElement("imu");
    if (!imu_elem->HasElement("frame_id")) {
        gzerr << "No frame_id element in IMU configuration" << std::endl;
        return false;
    }
    
    imu_frame_id_ = imu_elem->Get<std::string>("frame_id");
    std::cout << "IMU frame_id: " << imu_frame_id_ << std::endl;
    
    auto links = model_->GetLinks();
    for (const auto& link : links) {
        std::cout << "Link name: " << link->GetName() << std::endl;
    }
    
    // 尝试最多5次获取IMU link
    int max_retries = 50;
    for (int i = 0; i < max_retries; ++i) {
        imu_link_ = model_->GetLink(imu_frame_id_);
        if (imu_link_) {
            std::cout << "Found IMU link on attempt " << (i + 1) << std::endl;
            return true;
        }
        std::cout << "Attempt " << (i + 1) << " failed to get IMU link, retrying..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    gzerr << "IMU link '" << imu_frame_id_ << "' not found in model after " << max_retries << " attempts" << std::endl;
    return false;
}

bool GazeboShmInterface::ParseJoints(const sdf::ElementPtr& _sdf)
{
    if (!_sdf->HasElement("joints")) {
        gzerr << "No joints configuration found" << std::endl;
        return false;
    }

    auto joints_elem = _sdf->GetElement("joints");
    for (auto joint_elem = joints_elem->GetElement("joint"); joint_elem;
         joint_elem = joint_elem->GetNextElement("joint")) {
        std::string joint_name = joint_elem->Get<std::string>("name");
        auto joint = model_->GetJoint(joint_name);
        if (!joint) {
            gzerr << "Joint '" << joint_name << "' not found" << std::endl;
            return false;
        }
        joints_.push_back(joint);
        joint_names_.push_back(joint_name);
        // std::cout << "joint_name: " << joint_name << std::endl;
    }

    return true;
}

bool GazeboShmInterface::ParseContacts(const sdf::ElementPtr& _sdf)
{
    // 不解析接触传感器，直接返回成功
    std::cout << "Skipping contacts configuration..." << std::endl;
    return true;
}

void GazeboShmInterface::cleanupAndExit()
{
    std::cout << "=== Starting fast cleanup and exit process ===" << std::endl;
    
    // 1. 立即暂停仿真
    if (model_ && model_->GetWorld()) {
        model_->GetWorld()->SetPaused(true);
        std::cout << "✓ Simulation paused" << std::endl;
    }
    
    // 2. 立即断开事件连接
    if (updateConnection_) {
        updateConnection_.reset();
        std::cout << "✓ Event connections disconnected" << std::endl;
    }
    
    
    
    // 4. 快速关闭ROS节点
    if (nh_) {
        stop_sub_.shutdown();
        sim_start_srv_.shutdown();
        std::cout << "✓ ROS subscribers and services shutdown" << std::endl;
    }
    
    // 5. 使用多种方法快速关闭Gazebo
    std::cout << "Forcing Gazebo shutdown..." << std::endl;
    
    // 方法1: 直接发送SIGKILL给Gazebo进程
    std::cout << "Sending SIGKILL to Gazebo..." << std::endl;
    system("pkill -9 gazebo");
    system("pkill -9 gzserver");
    system("pkill -9 gzclient");
    
    // 方法2: 使用Gazebo的事件系统
    try {
        event::Events::sigInt();
        std::cout << "✓ Gazebo shutdown event triggered" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Warning: Error triggering Gazebo shutdown event: " << e.what() << std::endl;
    }
    
    // 方法3: 发送SIGTERM信号
    std::cout << "Sending SIGTERM..." << std::endl;
    std::raise(SIGTERM);
    
    // 方法4: 等待很短时间后强制退出
    std::cout << "Waiting for quick shutdown..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 方法5: 强制退出当前进程
    std::cout << "Force exiting current process..." << std::endl;
    exit(0);
}

} 

// 在命名空间外添加插件注册
GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboShmInterface) 
