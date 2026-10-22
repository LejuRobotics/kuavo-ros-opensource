#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>  // for getenv
#include <unistd.h> // for read, STDIN_FILENO
#include <cmath>    // for sin, cos, M_PI
#include <fstream>  // for file operations
#include <mutex>    // for mutex
#include "kuavo_common/common/json_config_reader.hpp" // for JSON parsing

// ROS相关头文件
#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <sensor_msgs/JointState.h>
#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointData.h"

#include "hardware_plant.h"

#define LB_LEG_JOINT_NUM 4
#define ARM_JOINT_NUM 14
#define HEAD_JOINT_NUM 2
#define TOTAL_JOINT_NUM (LB_LEG_JOINT_NUM + ARM_JOINT_NUM + HEAD_JOINT_NUM)
#define PI 3.14159265359

using namespace HighlyDynamic;

class MotorTest
{
public:
    MotorTest() {
        // 初始化ROS节点
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "motor_test", ros::init_options::AnonymousName);
        }
        
        // 创建ROS节点句柄
        nh_ = std::make_unique<ros::NodeHandle>();
        
        // 创建发布器
        joint_cmd_pub_ = nh_->advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
        joint_state_pub_ = nh_->advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 10);
        
        // 设置默认配置文件路径
        config_file_path_ = "motor_test_config.json";
        
        std::cout << "电机测试ROS节点初始化完成" << std::endl;
        ROS_INFO("电机测试节点已启动，节点名称: %s", ros::this_node::getName().c_str());

        start_time = ros::Time::now();
    }
    
    ~MotorTest() {
        // 停止关节数据发布线程
        stopJointDataThread();
        
        if (hardware_plant_) {
            hardware_plant_.reset();
        }
    }

    // 新增：对外公开的配置文件路径设置接口
    void configureFromFilePath(const std::string& file_path) {
        setConfigFilePath(file_path);
    }

    void init(const std::string &kuavo_assets_path="") {
        const char* robot_version_env = std::getenv("ROBOT_VERSION");
        if (robot_version_env == nullptr) {
            std::cerr << "错误：未设置环境变量 ROBOT_VERSION" << std::endl;
            std::cerr << "使用默认版本 42" << std::endl;
        }
        else{
            this->robot_version_int = 60; // std::atoi(robot_version_env);
            std::cout << "使用环境变量 ROBOT_VERSION: " << this->robot_version_int << std::endl;
        }
        
        hardware_param = HardwareParam();
        try
        {
            hardware_param.robot_version = RobotVersion::create(this->robot_version_int);
        }
        catch (const std::exception &e)
        {
            std::cerr << "无效的机器人版本号: " << this->robot_version_int << "，错误信息: " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (kuavo_assets_path == ""){
            hardware_param.kuavo_assets_path = KUAVO_ASSETS_PATH; // 使用编译时定义的 KUAVO_ASSETS_PATH
        }
        else{
            hardware_param.kuavo_assets_path = kuavo_assets_path;
        }
        
        std::cout << "kuavo_assets_path: " << hardware_param.kuavo_assets_path << std::endl;
        std::cout << "准备初始化轮臂硬件..." << std::endl;
        
        hardware_plant_ = std::make_unique<HardwarePlant>(dt_, hardware_param, std::string(PROJECT_SOURCE_DIR));
        hardware_plant_->HWPlantInit();
        
        if (hardware_plant_ == nullptr) {
            std::cout << "轮臂硬件初始化失败" << std::endl;
            exit(1);
        }
        else{
            std::cout << "轮臂硬件初始化成功" << std::endl;
        }

        // 构建关节ID列表
        for (int i = 1; i <= TOTAL_JOINT_NUM; ++i) {
            joint_ids.push_back(i);
        }
        joint_data.resize(joint_ids.size());
        
        // 加载测试参数配置文件
        loadTestConfig();
        
        // 加载PD参数配置文件
        loadPDConfig();
        
        // 启动关节数据发布线程
        startJointDataThread();
    }

    // 正弦曲线位置跟踪测试
    void testPositionTracking(int joint_id, double frequency_hz) {
        std::cout << "开始位置跟踪测试 - 关节 " << joint_id << " 频率 " << frequency_hz << "Hz" << std::endl;
        
        std::cout << "测试持续时间: " << test_config_.test_duration_seconds << "秒" << std::endl;
        std::cout << "采样频率: " << test_config_.sampling_rate_hz << "Hz" << std::endl;
        
        // 准备关节命令数组
        std::vector<double> joint_positions(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_velocities(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_torques(TOTAL_JOINT_NUM, 0.0);
        
        // 初始化所有关节为当前位置
        {
            std::lock_guard<std::mutex> lock(joint_data_mutex_);
            for(int i = 0; i < TOTAL_JOINT_NUM; i++){
                joint_positions[i] = joint_data[i].position;
                joint_velocities[i] = joint_data[i].velocity;
                joint_torques[i] = joint_data[i].torque;
            }
        }

        int total_samples = static_cast<int>(test_config_.test_duration_seconds * test_config_.sampling_rate_hz);
        
        // 使用ros::Rate控制循环频率
        ros::Rate rate(test_config_.sampling_rate_hz);
        
        for (int i = 0; i < total_samples; ++i) {
            double time = static_cast<double>(i) / test_config_.sampling_rate_hz;
            double commanded_pos = test_config_.position_amplitude_deg * (PI/180.0) * sin(2.0 * PI * frequency_hz * time);
            
            // 更新目标关节的位置命令
            joint_positions[joint_id - 1] = commanded_pos;
            
            // 通过HardwarePlant直接控制电机
            if (hardware_plant_) {
                // 构建命令向量 (q, v, tau, tau_max, tau_ratio)
                Eigen::VectorXd cmd(TOTAL_JOINT_NUM * 5);
                Eigen::VectorXd cmd_out(TOTAL_JOINT_NUM * 5);
                std::vector<int> control_modes(TOTAL_JOINT_NUM, 2); // 位置控制模式
                Eigen::VectorXd joint_kp(TOTAL_JOINT_NUM);
                Eigen::VectorXd joint_kd(TOTAL_JOINT_NUM);
                
                // 填充命令数据
                for (int j = 0; j < TOTAL_JOINT_NUM; ++j) {
                    cmd[TOTAL_JOINT_NUM * 0 + j] = joint_positions[j];      // 位置
                    cmd[TOTAL_JOINT_NUM * 1 + j] = joint_velocities[j];     // 速度
                    cmd[TOTAL_JOINT_NUM * 2 + j] = joint_torques[j];        // 扭矩
                    cmd[TOTAL_JOINT_NUM * 3 + j] = joint_data[j].maxTorque; // 最大扭矩
                    cmd[TOTAL_JOINT_NUM * 4 + j] = 1.0;                     // 扭矩比例
                    
                    // 获取PD参数
                    auto [kp, kd] = getJointPDParams(j + 1);
                    joint_kp[j] = kp;
                    joint_kd[j] = kd;
                }
                
                // 转换命令格式
                hardware_plant_->cmds2Cmdr(cmd, TOTAL_JOINT_NUM, cmd_out, TOTAL_JOINT_NUM);
                
                // 写入硬件命令
                hardware_plant_->writeCommand(cmd_out, TOTAL_JOINT_NUM, control_modes, joint_kp, joint_kd);
            }
            
            // 同时发布到ROS话题供测试人员监控
            publishJointCommand(joint_positions, joint_velocities, joint_torques);
            
            // 使用ros::Rate控制频率
            rate.sleep();
        }
        
        std::cout << "位置跟踪测试完成" << std::endl;
    }

    // 正弦曲线速度跟踪测试
    void testVelocityTracking(int joint_id, double frequency_hz) {
        std::cout << "开始速度跟踪测试 - 关节 " << joint_id << " 频率 " << frequency_hz << "Hz" << std::endl;
        
        std::cout << "测试持续时间: " << test_config_.test_duration_seconds << "秒" << std::endl;
        std::cout << "采样频率: " << test_config_.sampling_rate_hz << "Hz" << std::endl;
        
        // 准备关节命令数组
        std::vector<double> joint_positions(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_velocities(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_torques(TOTAL_JOINT_NUM, 0.0);
        
        // 初始化所有关节为当前位置
        {
            std::lock_guard<std::mutex> lock(joint_data_mutex_);
            for(int i = 0; i < TOTAL_JOINT_NUM; i++){
                joint_positions[i] = joint_data[i].position;
                joint_velocities[i] = joint_data[i].velocity;
                joint_torques[i] = joint_data[i].torque;
            }
        }

        int total_samples = static_cast<int>(test_config_.test_duration_seconds * test_config_.sampling_rate_hz);
        
        // 使用ros::Rate控制循环频率
        ros::Rate rate(test_config_.sampling_rate_hz);
        
        for (int i = 0; i < total_samples; ++i) {
            double time = static_cast<double>(i) / test_config_.sampling_rate_hz;
            double commanded_vel = test_config_.velocity_amplitude_rad_per_s * sin(2.0 * PI * frequency_hz * time);
            
            // 更新目标关节的速度命令
            joint_velocities[joint_id - 1] = commanded_vel;
            
            // 通过HardwarePlant直接控制电机
            if (hardware_plant_) {
                // 构建命令向量 (q, v, tau, tau_max, tau_ratio)
                Eigen::VectorXd cmd(TOTAL_JOINT_NUM * 5);
                Eigen::VectorXd cmd_out(TOTAL_JOINT_NUM * 5);
                std::vector<int> control_modes(TOTAL_JOINT_NUM, 1); // 速度控制模式
                Eigen::VectorXd joint_kp(TOTAL_JOINT_NUM);
                Eigen::VectorXd joint_kd(TOTAL_JOINT_NUM);
                
                // 填充命令数据
                for (int j = 0; j < TOTAL_JOINT_NUM; ++j) {
                    cmd[TOTAL_JOINT_NUM * 0 + j] = joint_positions[j];      // 位置
                    cmd[TOTAL_JOINT_NUM * 1 + j] = joint_velocities[j];     // 速度
                    cmd[TOTAL_JOINT_NUM * 2 + j] = joint_torques[j];        // 扭矩
                    cmd[TOTAL_JOINT_NUM * 3 + j] = joint_data[j].maxTorque; // 最大扭矩
                    cmd[TOTAL_JOINT_NUM * 4 + j] = 1.0;                     // 扭矩比例
                    
                    // 获取PD参数
                    auto [kp, kd] = getJointPDParams(j + 1);
                    joint_kp[j] = kp;
                    joint_kd[j] = kd;
                }
                
                // 转换命令格式
                hardware_plant_->cmds2Cmdr(cmd, TOTAL_JOINT_NUM, cmd_out, TOTAL_JOINT_NUM);
                
                // 写入硬件命令
                hardware_plant_->writeCommand(cmd_out, TOTAL_JOINT_NUM, control_modes, joint_kp, joint_kd);
            }
            
            // 同时发布到ROS话题供测试人员监控
            publishJointCommand(joint_positions, joint_velocities, joint_torques);
            
            // 使用ros::Rate控制频率
            rate.sleep();
        }
        
        std::cout << "速度跟踪测试完成" << std::endl;
    }

    // 正弦曲线扭矩跟踪测试
    void testTorqueTracking(int joint_id, double frequency_hz) {
        std::cout << "开始扭矩跟踪测试 - 关节 " << joint_id << " 频率 " << frequency_hz << "Hz" << std::endl;
        
        // 获取关节最大扭矩（使用线程中已更新的数据）
        double max_torque;
        {
            std::lock_guard<std::mutex> lock(joint_data_mutex_);
            max_torque = joint_data[joint_id - 1].maxTorque;
        }
        double amplitude_torque = max_torque * test_config_.torque_amplitude_ratio;
        
        std::cout << "测试持续时间: " << test_config_.test_duration_seconds << "秒" << std::endl;
        std::cout << "采样频率: " << test_config_.sampling_rate_hz << "Hz" << std::endl;
        std::cout << "最大扭矩: " << max_torque << " Nm" << std::endl;
        std::cout << "测试幅值: " << amplitude_torque << " Nm" << std::endl;
        
        // 准备关节命令数组
        std::vector<double> joint_positions(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_velocities(TOTAL_JOINT_NUM, 0.0);
        std::vector<double> joint_torques(TOTAL_JOINT_NUM, 0.0);
        
        // 初始化所有关节为当前位置
        {
            std::lock_guard<std::mutex> lock(joint_data_mutex_);
            for(int i = 0; i < TOTAL_JOINT_NUM; i++){
                joint_positions[i] = joint_data[i].position;
                joint_velocities[i] = joint_data[i].velocity;
                joint_torques[i] = joint_data[i].torque;
            }
        }

        int total_samples = static_cast<int>(test_config_.test_duration_seconds * test_config_.sampling_rate_hz);
        
        // 使用ros::Rate控制循环频率
        ros::Rate rate(test_config_.sampling_rate_hz);
        
        for (int i = 0; i < total_samples; ++i) {
            double time = static_cast<double>(i) / test_config_.sampling_rate_hz;
            double commanded_torque = amplitude_torque * sin(2.0 * PI * frequency_hz * time);
            
            // 更新目标关节的扭矩命令
            joint_torques[joint_id - 1] = commanded_torque;
            
            // 通过HardwarePlant直接控制电机
            if (hardware_plant_) {
                // 构建命令向量 (q, v, tau, tau_max, tau_ratio)
                Eigen::VectorXd cmd(TOTAL_JOINT_NUM * 5);
                Eigen::VectorXd cmd_out(TOTAL_JOINT_NUM * 5);
                std::vector<int> control_modes(TOTAL_JOINT_NUM, 3); // 扭矩控制模式
                Eigen::VectorXd joint_kp(TOTAL_JOINT_NUM);
                Eigen::VectorXd joint_kd(TOTAL_JOINT_NUM);
                
                // 填充命令数据
                for (int j = 0; j < TOTAL_JOINT_NUM; ++j) {
                    cmd[TOTAL_JOINT_NUM * 0 + j] = joint_positions[j];      // 位置
                    cmd[TOTAL_JOINT_NUM * 1 + j] = joint_velocities[j];     // 速度
                    cmd[TOTAL_JOINT_NUM * 2 + j] = joint_torques[j];        // 扭矩
                    cmd[TOTAL_JOINT_NUM * 3 + j] = joint_data[j].maxTorque; // 最大扭矩
                    cmd[TOTAL_JOINT_NUM * 4 + j] = 1.0;                     // 扭矩比例
                    
                    // 获取PD参数
                    auto [kp, kd] = getJointPDParams(j + 1);
                    joint_kp[j] = kp;
                    joint_kd[j] = kd;
                }
                
                // 转换命令格式
                hardware_plant_->cmds2Cmdr(cmd, TOTAL_JOINT_NUM, cmd_out, TOTAL_JOINT_NUM);
                
                // 写入硬件命令
                hardware_plant_->writeCommand(cmd_out, TOTAL_JOINT_NUM, control_modes, joint_kp, joint_kd);
            }
            
            // 同时发布到ROS话题供测试人员监控
            publishJointCommand(joint_positions, joint_velocities, joint_torques);
            
            // 使用ros::Rate控制频率
            rate.sleep();
        }
        
        std::cout << "扭矩跟踪测试完成" << std::endl;
    }

    // 打印当前关节角度
    void printCurrentJointAngles() {
        std::cout << "\n=== 当前关节角度 ===" << std::endl;
        
        // 打印轮臂腿部关节角度
        std::cout << "轮臂腿部关节:" << std::endl;
        std::vector<std::string> lb_joint_names = {"joint1", "joint2", "joint3", "joint4"};
        for (int i = 0; i < LB_LEG_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i].position;
            std::cout << "  " << lb_joint_names[i] << " (ID:" << (i+1) << "): " 
                      << std::fixed << std::setprecision(2) << angle_deg << "° (" 
                      << std::setprecision(4) << joint_data[i].position  / (180.0 / PI) << " rad)" << std::endl;
        }
        
        // 打印手臂关节角度
        std::cout << "\n手臂关节:" << std::endl;
        std::vector<std::string> arm_joint_names = {
            "左臂Pitch", "左臂Roll", "左臂Yaw", "左前臂Pitch", "左手Yaw", "左手Pitch", "左手Roll",
            "右臂Pitch", "右臂Roll", "右臂Yaw", "右前臂Pitch", "右手Yaw", "右手Pitch", "右手Roll"
        };
        for (int i = 0; i < ARM_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i + LB_LEG_JOINT_NUM].position;
            std::cout << "  " << arm_joint_names[i] << " (ID:" << (i + LB_LEG_JOINT_NUM + 1) << "): " 
                      << std::fixed << std::setprecision(5) << angle_deg << "° (" 
                      << std::setprecision(5) << joint_data[i + LB_LEG_JOINT_NUM].position / (180.0 / PI) << " rad)" << std::endl;
        }
        
        // 打印头部关节角度
        std::cout << "\n头部关节:" << std::endl;
        std::vector<std::string> head_joint_names = {"头部偏航", "头部俯仰"};
        for (int i = 0; i < HEAD_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM].position;
            std::cout << "  " << head_joint_names[i] << " (ID:" << (i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM + 1) << "): " 
                      << std::fixed << std::setprecision(5) << angle_deg << "° (" 
                      << std::setprecision(5) << joint_data[i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM].position / (180.0 / PI)<< " rad)" << std::endl;
        }
        std::cout << std::endl;
    }

private:
    // ROS相关成员变量
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher joint_cmd_pub_;
    ros::Publisher joint_state_pub_;

    ros::Time start_time;
    // 硬件传感器数据
    SensorData_t sensor_data_;
    SensorData_t sensor_data_joint;

    kuavo_msgs::sensorsData msg;

    // 线程相关
    std::thread joint_data_thread_;
    bool joint_data_thread_running_ = false;
    std::mutex joint_data_mutex_;
    
    // 硬件相关
    std::vector<uint8_t> joint_ids;
    std::vector<JointParam_t> joint_data;
    double dt_ = 0.001;
    int robot_version_int = 42;
    HardwareParam hardware_param;
    std::unique_ptr<HardwarePlant> hardware_plant_;
    
    // JSON配置相关
    std::unique_ptr<HighlyDynamic::JSONConfigReader> pd_config_;
    std::string config_file_path_;
    bool config_loaded_ = false;
    
    // 测试配置结构体
    struct TestConfig {
        double position_amplitude_deg = 20.0;        // 位置测试幅值 ±20度
        double velocity_amplitude_rad_per_s = 0.8;   // 速度测试幅值 0.8 rad/s
        double torque_amplitude_ratio = 0.5;         // 扭矩测试幅值比例 50%
        double test_duration_seconds = 5.0;          // 测试持续时间 5秒
        int sampling_rate_hz = 1000;                 // 采样频率 1000Hz
    };
    
    TestConfig test_config_;  // 测试配置实例
    
    // 加载测试参数配置文件
    void loadTestConfig() {
        try {
            // 尝试加载测试配置文件
            std::ifstream config_file(config_file_path_);
            if (config_file.is_open()) {
                // 如果配置文件存在，读取测试参数
                std::string config_content((std::istreambuf_iterator<char>(config_file)),
                                         std::istreambuf_iterator<char>());
                config_file.close();
                
                // 使用JSONConfigReader解析配置
                auto temp_config = std::make_unique<HighlyDynamic::JSONConfigReader>(config_content);
                
                // 读取测试参数
                try {
                    test_config_.position_amplitude_deg = temp_config->getValue<double>("test_config.position_amplitude_deg");
                    test_config_.velocity_amplitude_rad_per_s = temp_config->getValue<double>("test_config.velocity_amplitude_rad_per_s");
                    test_config_.torque_amplitude_ratio = temp_config->getValue<double>("test_config.torque_amplitude_ratio");
                    test_config_.test_duration_seconds = temp_config->getValue<double>("test_config.test_duration_seconds");
                    test_config_.sampling_rate_hz = temp_config->getValue<int>("test_config.sampling_rate_hz");
                    
                    std::cout << "成功加载测试参数配置文件: " << config_file_path_ << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "部分测试参数读取失败，使用默认值: " << e.what() << std::endl;
                }
            } else {
                // 如果配置文件不存在，创建默认配置
                createDefaultTestConfig();
            }
        } catch (const std::exception& e) {
            std::cerr << "错误: 加载测试配置文件失败: " << e.what() << std::endl;
            createDefaultTestConfig();
        }
    }
    
    // 创建默认测试配置文件
    void createDefaultTestConfig() {
        std::ofstream default_config_file(config_file_path_);
        if (default_config_file.is_open()) {
            default_config_file << "{\n";
            default_config_file << "  \"test_config\": {\n";
            default_config_file << "    \"position_amplitude_deg\": 20.0,\n";
            default_config_file << "    \"velocity_amplitude_rad_per_s\": 0.8,\n";
            default_config_file << "    \"torque_amplitude_ratio\": 0.5,\n";
            default_config_file << "    \"test_duration_seconds\": 5.0,\n";
            default_config_file << "    \"sampling_rate_hz\": 1000\n";
            default_config_file << "  },\n";
            default_config_file << "  \"pd_config\": {\n";
            default_config_file << "    \"default\": {\n";
            default_config_file << "      \"kp\": 100.0,\n";
            default_config_file << "      \"kd\": 10.0\n";
            default_config_file << "    },\n";
            
            // 为每个关节设置默认PD值
            for (int i = 1; i <= TOTAL_JOINT_NUM; ++i) {
                default_config_file << "    \"joint_" << i << "\": {\n";
                default_config_file << "      \"kp\": 100.0,\n";
                default_config_file << "      \"kd\": 10.0\n";
                default_config_file << "    }";
                if (i < TOTAL_JOINT_NUM) {
                    default_config_file << ",";
                }
                default_config_file << "\n";
            }
            
            default_config_file << "  }\n";
            default_config_file << "}\n";
            default_config_file.close();
            
            std::cout << "已创建默认测试配置文件: " << config_file_path_ << std::endl;
        } else {
            std::cerr << "无法创建默认配置文件" << std::endl;
        }
    }

    // 加载PD参数配置文件
    void loadPDConfig() {
        try {
            pd_config_ = std::make_unique<HighlyDynamic::JSONConfigReader>(config_file_path_);
            config_loaded_ = true;
            
            std::cout << "成功加载PD参数配置文件: " << config_file_path_ << std::endl;
            ROS_INFO("成功加载PD参数配置文件: %s", config_file_path_.c_str());
            
            // 验证配置
            validatePDConfig();
            
        } catch (const std::exception& e) {
            std::cerr << "错误: 解析配置文件失败: " << e.what() << std::endl;
            ROS_ERROR("解析配置文件失败: %s", e.what());
            createDefaultTestConfig();
        }
    }
    
    // 验证PD配置
    void validatePDConfig() {
        if (!pd_config_) {
            std::cout << "警告: 配置对象为空，无法验证" << std::endl;
            return;
        }
        
        // 检查默认值是否存在
        try {
            double default_kp = pd_config_->getValue<double>("pd_config.default.kp");
            double default_kd = pd_config_->getValue<double>("pd_config.default.kd");
            std::cout << "默认PD参数 - Kp: " << default_kp << ", Kd: " << default_kd << std::endl;
        } catch (...) {
            std::cout << "警告: 配置文件中缺少默认值" << std::endl;
        }
        
        // 检查每个关节的配置
        for (int i = 1; i <= TOTAL_JOINT_NUM; ++i) {
            std::string joint_name = "pd_config.joint_" + std::to_string(i);
            try {
                double kp = pd_config_->getValue<double>(joint_name + ".kp");
                double kd = pd_config_->getValue<double>(joint_name + ".kd");
                std::cout << "关节 " << joint_name << " - Kp: " << kp << ", Kd: " << kd << std::endl;
            } catch (...) {
                std::cout << "警告: 关节 " << joint_name << " 缺少配置，将使用默认值" << std::endl;
            }
        }
    }
    
    // 获取关节的PD参数
    std::pair<double, double> getJointPDParams(int joint_id) {
        if (!config_loaded_ || !pd_config_) {
            return {100.0, 10.0}; // 默认值
        }
        
        std::string joint_name = "pd_config.joint_" + std::to_string(joint_id);
        
        try {
            // 尝试获取特定关节的配置
            double kp = pd_config_->getValue<double>(joint_name + ".kp");
            double kd = pd_config_->getValue<double>(joint_name + ".kd");
            return {kp, kd};
        } catch (...) {
            // 如果没有特定关节的配置，尝试使用默认值
            try {
                double kp = pd_config_->getValue<double>("pd_config.default.kp");
                double kd = pd_config_->getValue<double>("pd_config.default.kd");
                return {kp, kd};
            } catch (...) {
                // 最后的默认值
                return {100.0, 10.0};
            }
        }
    }
    
    // 设置PD参数配置文件路径（保持为私有，仅通过公开包装函数调用）
    void setConfigFilePath(const std::string& file_path) {
        config_file_path_ = file_path;
        if (config_loaded_) {
            // 重新加载配置
            loadPDConfig();
        }
    }
    
    // 发布关节命令
    void publishJointCommand(const std::vector<double>& joint_positions,
                           const std::vector<double>& joint_velocities,
                           const std::vector<double>& joint_torques) {
        kuavo_msgs::jointCmd cmd_msg;
        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.header.frame_id = "base_link";
        
        cmd_msg.joint_q = joint_positions;
        cmd_msg.joint_v = joint_velocities;
        cmd_msg.tau = joint_torques;
        
        // 从JSON配置文件读取PD参数
        cmd_msg.joint_kp.resize(joint_positions.size());
        cmd_msg.joint_kd.resize(joint_positions.size());
        cmd_msg.control_modes.resize(joint_positions.size(), 1); // 位置控制模式
        
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            auto [kp, kd] = getJointPDParams(i + 1);
            cmd_msg.joint_kp[i] = kp;
            cmd_msg.joint_kd[i] = kd;
        }
        
        joint_cmd_pub_.publish(cmd_msg);
    }

    // 启动关节数据发布线程
    void startJointDataThread() {
        if (joint_data_thread_running_) {
            ROS_WARN("关节数据发布线程已在运行");
            return;
        }
        
        joint_data_thread_running_ = true;
        joint_data_thread_ = std::thread(&MotorTest::jointDataThreadFunction, this);
        
        ROS_INFO("关节数据发布线程已启动");
    }
    
    // 停止关节数据发布线程
    void stopJointDataThread() {
        if (!joint_data_thread_running_) {
            return;
        }
        
        joint_data_thread_running_ = false;
        
        if (joint_data_thread_.joinable()) {
            joint_data_thread_.join();
        }
        
        ROS_INFO("关节数据发布线程已停止");
    }
    
    // 关节数据发布线程函数
    void jointDataThreadFunction() {
        const int publish_rate_hz = 100; // 100Hz发布频率
        ros::Rate rate(publish_rate_hz);
        
        ROS_INFO("关节数据发布线程开始运行，发布频率: %d Hz", publish_rate_hz);
        sensor_data_joint.resizeJoint(TOTAL_JOINT_NUM);

        while (joint_data_thread_running_ && ros::ok()) {
            // 使用readSensor获取传感器数据并更新joint_data
            if (hardware_plant_) {
                hardware_plant_->readSensor(sensor_data_);
            }

            hardware_plant_->motor2joint(sensor_data_, sensor_data_joint);
            hardware_plant_->setState(sensor_data_, sensor_data_joint);

            msg.header.stamp = ros::Time::now();
            msg.sensor_time = start_time;
            msg.joint_data.joint_q = eigenToStdVector(sensor_data_joint.joint_q);
            msg.joint_data.joint_v = eigenToStdVector(sensor_data_joint.joint_v);
            msg.joint_data.joint_vd = eigenToStdVector(sensor_data_joint.joint_vd);
            msg.joint_data.joint_torque = eigenToStdVector(sensor_data_joint.joint_current);
            msg.imu_data.acc.x = sensor_data_joint.acc.x();
            msg.imu_data.acc.y = sensor_data_joint.acc.y();
            msg.imu_data.acc.z = sensor_data_joint.acc.z();
            msg.imu_data.gyro.x = sensor_data_joint.gyro.x();
            msg.imu_data.gyro.y = sensor_data_joint.gyro.y();
            msg.imu_data.gyro.z = sensor_data_joint.gyro.z();
            msg.imu_data.free_acc.x = sensor_data_joint.free_acc.x();
            msg.imu_data.free_acc.y = sensor_data_joint.free_acc.y();
            msg.imu_data.free_acc.z = sensor_data_joint.free_acc.z();
            msg.imu_data.quat.x = sensor_data_joint.quat[1];
            msg.imu_data.quat.y = sensor_data_joint.quat[2];
            msg.imu_data.quat.z = sensor_data_joint.quat[3];
            msg.imu_data.quat.w = sensor_data_joint.quat[0];
            
            // 发布关节状态
            joint_state_pub_.publish(msg);
            
            // 使用ros::Rate控制发布频率
            rate.sleep();
        }
        
        ROS_INFO("关节数据发布线程结束");
    }
    
    // 发布关节状态
    void publishJointStates() {
        if (!hardware_plant_) return;
        
        std::lock_guard<std::mutex> lock(joint_data_mutex_);
        
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.header.frame_id = "base_link";
        
        // 设置关节名称
        for (int i = 1; i <= TOTAL_JOINT_NUM; ++i) {
            joint_state_msg.name.push_back("joint_" + std::to_string(i));
        }
        
        // 设置关节位置、速度和扭矩
        for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
            joint_state_msg.position.push_back(joint_data[i].position);
            joint_state_msg.velocity.push_back(joint_data[i].velocity);
            joint_state_msg.effort.push_back(joint_data[i].torque);
        }
        
        joint_state_pub_.publish(joint_state_msg);
    }
};

int main(int argc, char const *argv[])
{
    std::string kuavo_assets_path = "";
    std::string config_file_path = "";
    
    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        if (argv[i] != nullptr) {
            if (i == 1) {
                kuavo_assets_path = std::string(argv[i]);
            } else if (i == 2) {
                config_file_path = std::string(argv[i]);
            }
        }
    }
    
    std::cout << "电机测试程序" << std::endl;
    using namespace HighlyDynamic;
    
    std::cout << "初始化电机测试" << std::endl;
    auto motor_test = std::make_shared<MotorTest>();
    
    // 设置配置文件路径（如果提供）
    if (!config_file_path.empty()) {
        motor_test->configureFromFilePath(config_file_path);
    }
    
    motor_test->init(kuavo_assets_path);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto output_test_menu = [](){
        std::cout << "\n[MotorTest] 测试菜单:" << std::endl;
        std::cout << "按下 '1' 位置跟踪测试" << std::endl;
        std::cout << "按下 '2' 速度跟踪测试" << std::endl;
        std::cout << "按下 '3' 扭矩跟踪测试" << std::endl;
        std::cout << "按下 'p' 打印当前关节角度" << std::endl;
        std::cout << "按下 'q' 退出" << std::endl;
    };

    output_test_menu();
    bool running = true;
    
    while (running)
    {
        char input;
        if (read(STDIN_FILENO, &input, 1) > 0) {
            switch (input) {
                case '1':
                    {
                        std::cout << "请输入关节ID (1-" << TOTAL_JOINT_NUM << "): ";
                        int joint_id;
                        std::cin >> joint_id;
                        if (joint_id >= 1 && joint_id <= TOTAL_JOINT_NUM) {
                            motor_test->testPositionTracking(joint_id, 2.0);
                        }
                    }
                    output_test_menu();
                    break;
                case '2':
                    {
                        std::cout << "请输入关节ID (1-" << TOTAL_JOINT_NUM << "): ";
                        int joint_id;
                        std::cin >> joint_id;
                        if (joint_id >= 1 && joint_id <= TOTAL_JOINT_NUM) {
                            motor_test->testVelocityTracking(joint_id, 2.0);
                        }
                    }
                    output_test_menu();
                    break;
                case '3':
                    {
                        std::cout << "请输入关节ID (1-" << TOTAL_JOINT_NUM << "): ";
                        int joint_id;
                        std::cin >> joint_id;
                        if (joint_id >= 1 && joint_id <= TOTAL_JOINT_NUM) {
                            motor_test->testTorqueTracking(joint_id, 10.0);
                        }
                    }
                    output_test_menu();
                    break;
                case 'p':
                    motor_test->printCurrentJointAngles();
                    output_test_menu();
                    break;
                case 'q':
                    std::cout << "[MotorTest] 退出" << std::endl;
                    running = false;
                    break;
            }
        }
        
        // 处理ROS消息
        ros::spinOnce();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
} 