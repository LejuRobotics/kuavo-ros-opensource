#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <cmath>
#include <numeric>
#include <unistd.h>
#include <signal.h>
#include <atomic>
#include <mutex>
#include <fstream>
#include <cstdio>
#include <sys/wait.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>

#include "hipnuc_imu_receiver.h"
#include "imu_receiver.h"


// IMU data storage structure (public for external access)
struct IMUTestData {
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Quaterniond quat;
    bool valid = false;
    std::chrono::steady_clock::time_point timestamp;
};

// Statistics
struct IMUStats {
    int sample_count = 0;
    double acc_magnitude_sum = 0.0;
    double gyro_magnitude_sum = 0.0;
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    
    void reset() {
        sample_count = 0;
        acc_magnitude_sum = 0.0;
        gyro_magnitude_sum = 0.0;
        acc_sum = Eigen::Vector3d::Zero();
        gyro_sum = Eigen::Vector3d::Zero();
    }
    
    void addSample(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        sample_count++;
        acc_magnitude_sum += acc.norm();
        gyro_magnitude_sum += gyro.norm();
        acc_sum += acc;
        gyro_sum += gyro;
    }
};

class IMUTestScript
{

public:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    
    std::atomic<bool> running_;
    std::atomic<bool> publishing_enabled_;
    
    // IMU类型控制
    std::string imu_type_ = "xsens";  // "xsens" or "hipnuc"
    
    IMUTestData xsens_data_;
    IMUTestData hipnuc_data_;
    std::mutex data_mutex_;
    
    std::thread publishing_thread_;

    // Constructor and public methods
    IMUTestScript() : nh_("~"), running_(false), publishing_enabled_(false) {
        // Initialize ROS publishers
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
        
        std::cout << "IMU测试脚本初始化完成" << std::endl;
    }
    
    void setIMUType(const std::string& imu_type) {
        if (imu_type == "xsens" || imu_type == "hipnuc") {
            imu_type_ = imu_type;
            std::cout << "设置IMU类型为: " << imu_type_ << std::endl;
        } else {
            std::cout << "错误: 不支持的IMU类型 '" << imu_type << "'，支持的类型: xsens, hipnuc" << std::endl;
            std::cout << "使用默认类型: " << imu_type_ << std::endl;
        }
    }
    
    ~IMUTestScript() {
        // 停止发布线程
        std::cout << "正在停止发布线程..." << std::endl;
        running_ = false;
        publishing_enabled_ = false;
        if (publishing_thread_.joinable()) {
            publishing_thread_.join();
        }
        
        // 停止IMU
        std::cout << "正在停止IMU..." << std::endl;
        xsens_IMU::imu_stop();
        HIPNUC_IMU::imu_stop();
        std::cout << "IMU已停止" << std::endl;
    }
    
    bool init() {
        
        std::cout << "准备初始化IMU类型: " << imu_type_ << std::endl;
        
        int init_result = -1;
        
        if (imu_type_ == "xsens") {
            // 初始化Xsens IMU
            std::cout << "初始化Xsens IMU..." << std::endl;
            init_result = xsens_IMU::imu_init();
            if (init_result == 0) {
                std::cout << "✓ Xsens IMU初始化成功" << std::endl;
            } else {
                std::cerr << "✗ Xsens IMU初始化失败，错误代码: " << init_result << std::endl;
            }
        } else if (imu_type_ == "hipnuc") {
            // 初始化HIPNUC IMU
            std::cout << "初始化HIPNUC IMU..." << std::endl;
            HIPNUC_IMU::imu_stop();
            init_result = HIPNUC_IMU::imu_init();
            if (HIPNUC_IMU::get_imu_running_flag()) {
                std::cout << "✓ HIPNUC IMU初始化成功" << std::endl;
            } else {
                init_result = -1;
                std::cerr << "✗ HIPNUC IMU初始化失败，请检查IMU是否连接正常" << std::endl;
            }
        }
        
        // 检查初始化结果
        if (init_result == 0) {
            std::cout << "IMU初始化成功，测试可以继续" << std::endl;
            
            // 自动启动后台发布
            std::cout << "自动启动后台ROS数据发布..." << std::endl;
            startPublishing();
            
            return true;
        } else {
            std::cerr << "IMU初始化失败，无法继续测试" << std::endl;
            return false;
        }
    }
    
    bool testSingleIMU(const std::string& imu_type, IMUTestData& data) {
        bool success = false;
        
        if (imu_type == "xsens") {
            success = xsens_IMU::getImuDataFrame(data.acc, data.gyro, data.quat);
            if (success) {
                data.valid = true;
                data.timestamp = std::chrono::steady_clock::now();
            }
        } else if (imu_type == "hipnuc") {
            success = HIPNUC_IMU::getImuDataFrame(data.acc, data.gyro, data.quat);
            if (success) {
                data.valid = true;
                data.timestamp = std::chrono::steady_clock::now();
            }
        }
        
        if (!success) {
            std::cerr << "读取" << imu_type << " IMU数据失败" << std::endl;
        }
        
        return success;
    }
    
    void printIMUData(const std::string& name, const IMUTestData& data) {
        if (!data.valid) {
            std::cout << name << " IMU: 数据无效" << std::endl;
            return;
        }
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << name << " IMU:" << std::endl;
        std::cout << "  加速度: [" << data.acc.x() << ", " << data.acc.y() << ", " << data.acc.z() << "] m/s²" << std::endl;
        std::cout << "  角速度: [" << data.gyro.x() << ", " << data.gyro.y() << ", " << data.gyro.z() << "] rad/s" << std::endl;
        std::cout << "  四元数: [" << data.quat.w() << ", " << data.quat.x() << ", " << data.quat.y() << ", " << data.quat.z() << "]" << std::endl;
        
        // 转换为欧拉角显示
        Eigen::Vector3d euler = data.quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
        std::cout << "  姿态角: Roll=" << euler[2] * 180.0 / M_PI << "°, Pitch=" << euler[1] * 180.0 / M_PI << "°, Yaw=" << euler[0] * 180.0 / M_PI << "°" << std::endl;
    }
    
    // Yaw角偏移测试
    bool testYawOffset() {

        std::cout << "测试 IMU 单元" << std::endl;
        const int SAMPLE_COUNT = 100;         // 检测样本数（可根据需求调整）
        const double TIME_INTERVAL = 0.001;     // 采样间隔（秒，可根据IMU频率调整）
        const double MAX_ALLOWED_DRIFT = 0.1; // 允许的最大漂移（弧度，约5.7度）

        std::vector<double> yawAngles; // 存储所有采样的yaw角

        std::cout << "正在检测Yaw角稳定性（保持静止）..." << std::endl;
        Eigen::Vector3d acc, gyro;
        Eigen::Quaterniond quat;
        
        for (int i = 0; i < SAMPLE_COUNT; ++i)
        {
            // 读取最新IMU数据
            bool readflag;
            if (imu_type_ == "xsens")
                readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat);
            else
                readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
            if (!readflag)
            {
                std::cerr << "检测过程中IMU数据读取失败!" << std::endl;
            }

            // 将四元数转换为Yaw角（ZYX欧拉角顺序，Yaw为绕Z轴的旋转角）
            Eigen::Vector3d eulerAngles = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
            double yaw = eulerAngles[0];                                                // Yaw角（单位：弧度）

            yawAngles.push_back(yaw);
            std::cout << "采样 " << i + 1 << "/" << SAMPLE_COUNT << "，Yaw: " << yaw << " rad" << std::endl;

            // 等待采样间隔
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIME_INTERVAL * 1000)));
        }

        // 计算Yaw角的波动范围（最大值-最小值）
        double maxYaw = *std::max_element(yawAngles.begin(), yawAngles.end());
        double minYaw = *std::min_element(yawAngles.begin(), yawAngles.end());
        double yawDrift = maxYaw - minYaw;

        // 判断是否超过漂移阈值
        if (yawDrift > MAX_ALLOWED_DRIFT)
        {
            std::cerr << "警告：Yaw角漂移检测失败！漂移量：" << yawDrift << " rad（允许最大值：" << MAX_ALLOWED_DRIFT << " rad）" << std::endl;
            std::cerr << "可能原因：IMU未校准、陀螺仪零偏过大或设备晃动" << std::endl;
            return false;
        }

        std::cout << "Yaw角稳定性检测通过！漂移量：" << yawDrift << " rad" << std::endl;
        return true;
    }
    
    // 新增：频率测试
    bool testFrequency() {
        std::cout << "\n======= IMU频率测试 =======" << std::endl;
        std::cout << "测试说明：使用rostopic hz检测IMU数据发布频率" << std::endl;
        std::cout << "预期频率：500Hz (根据发布线程设置)" << std::endl;
        
        if (!publishing_enabled_) {
            std::cout << "启动数据发布..." << std::endl;
            startPublishing();
            std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待发布稳定
        }
        
        std::cout << "开始频率测试..." << std::endl;
        std::cout << "将运行rostopic hz命令检测以下话题的频率：" << std::endl;
        std::cout << "  - /imu/data" << std::endl;
        
        bool all_tests_passed = true;
        bool frequency_test_passed = false;
        
        // 测试/imu/data话题频率
        std::cout << "\n--- 测试 /imu/data 话题频率 ---" << std::endl;
        
        // 使用临时文件来捕获rostopic hz的输出
        std::string temp_file = "/tmp/rostopic_hz_output.txt";
        std::string cmd1 = "timeout 10s rostopic hz /imu/data > " + temp_file + " 2>&1";
        std::cout << "执行命令: " << cmd1 << std::endl;
        int status = system(cmd1.c_str());
        int exit_code = -1;
        
        if (status == -1) {
            std::cout << "✗ 无法执行 rostopic hz 命令" << std::endl;
            all_tests_passed = false;
        } else if (WIFEXITED(status)) {
            exit_code = WEXITSTATUS(status); // 例如 timeout 的退出码是 124
        } else if (WIFSIGNALED(status)) {
            std::cout << "✗ rostopic hz 被信号终止, signal=" << WTERMSIG(status) << std::endl;
            all_tests_passed = false;
        }
        
        if (exit_code != 0 && exit_code != 124) {
            std::cout << "警告: /imu/data 话题频率测试未成功执行, exit_code=" << exit_code << std::endl;
            all_tests_passed = false;
        } else {
            // 读取输出文件并解析频率
            std::ifstream file(temp_file);
            if (file.is_open()) {
                std::string line;
                std::vector<double> frequencies;
                
                while (std::getline(file, line)) {
                    // 查找包含"average rate:"的行
                    if (line.find("average rate:") != std::string::npos) {
                        // 提取频率数值
                        size_t pos = line.find("average rate:");
                        if (pos != std::string::npos) {
                            std::string freq_str = line.substr(pos + 13); // "average rate:" 长度为13
                            // 移除前导空格
                            freq_str.erase(0, freq_str.find_first_not_of(" \t"));
                            // 提取数字部分
                            size_t end_pos = freq_str.find_first_not_of("0123456789.");
                            if (end_pos != std::string::npos) {
                                freq_str = freq_str.substr(0, end_pos);
                            }
                            
                            try {
                                double freq = std::stod(freq_str);
                                frequencies.push_back(freq);
                                std::cout << "检测到频率: " << freq << " Hz" << std::endl;
                            } catch (const std::exception& e) {
                                std::cout << "无法解析频率值: " << freq_str << std::endl;
                            }
                        }
                    }
                }
                file.close();
                
                // 计算平均频率并判断是否通过
                if (!frequencies.empty()) {
                    const double detected_frequency = std::accumulate(frequencies.begin(), frequencies.end(), 0.0) / frequencies.size();
                    
                    std::cout << "平均检测频率: " << detected_frequency << " Hz" << std::endl;
                    
                    // 判断频率是否达到要求 (允许±10%的误差)
                    const double target_freq = 500.0;
                    const double tolerance = 0.1; // 10% 容差
                    const double min_freq = target_freq * (1.0 - tolerance);
                    const double max_freq = target_freq * (1.0 + tolerance);
                    
                    if (detected_frequency >= min_freq && detected_frequency <= max_freq) {
                        std::cout << "✓ 频率测试通过！检测频率 " << detected_frequency << " Hz 在目标范围 [" 
                                  << min_freq << ", " << max_freq << "] Hz 内" << std::endl;
                        frequency_test_passed = true;
                    } else {
                        std::cout << "✗ 频率测试失败！检测频率 " << detected_frequency << " Hz 超出目标范围 [" 
                                  << min_freq << ", " << max_freq << "] Hz" << std::endl;
                        std::cout << "  目标频率: " << target_freq << " Hz" << std::endl;
                        frequency_test_passed = false;
                        all_tests_passed = false;
                    }
                } else {
                    std::cout << "✗ 未能检测到有效的频率数据" << std::endl;
                    frequency_test_passed = false;
                    all_tests_passed = false;
                }
            } else {
                std::cout << "✗ 无法读取频率测试输出文件" << std::endl;
                frequency_test_passed = false;
                all_tests_passed = false;
            }
            
            // 清理临时文件
            std::remove(temp_file.c_str());
        }
        
        std::cout << "\n--- 频率测试完成 ---" << std::endl;
        if (frequency_test_passed) {
            std::cout << "✓ 频率测试通过！IMU数据发布频率正常" << std::endl;
        } else {
            std::cout << "✗ 频率测试失败，请检查系统负载和硬件连接" << std::endl;
        }
        
        return all_tests_passed;
    }
    
    // 新增：综合测试函数
    void runComprehensiveTest() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "         IMU 综合测试开始" << std::endl;
        std::cout << "========================================" << std::endl;
        
        bool all_passed = true;
        
        // 1. 基本数据读取测试
        std::cout << "\n1. 基本数据读取测试:" << std::endl;
        IMUTestData data;
        if (testSingleIMU(imu_type_, data)) {
            std::cout << "✓ 基本数据读取测试通过" << std::endl;
            printIMUData(imu_type_, data);
        } else {
            std::cout << "✗ 基本数据读取测试失败" << std::endl;
            all_passed = false;
        }
        
        // 2. 频率测试 (先进行)
        std::cout << "\n2. 频率测试:" << std::endl;
        if (!testFrequency()) {
            all_passed = false;
        }
        
        // 等待一段时间让用户观察频率测试结果
        std::cout << "\n等待15秒以便观察频率测试结果..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(15));
        
        // 3. Yaw角偏移测试 (然后进行)
        std::cout << "\n3. Yaw角偏移测试:" << std::endl;
        if (!testYawOffset()) {
            all_passed = false;
        }
        
        // 测试总结
        std::cout << "\n========================================" << std::endl;
        std::cout << "         IMU 综合测试结果" << std::endl;
        std::cout << "========================================" << std::endl;
        if (all_passed) {
            std::cout << "✓ 所有测试通过！IMU工作正常" << std::endl;
        } else {
            std::cout << "✗ 部分测试失败，请检查IMU状态和配置" << std::endl;
        }
        
        std::cout << "\n注意事项:" << std::endl;
        std::cout << "- Yaw角偏移超出±5°范围时，建议检查IMU校准" << std::endl;
        std::cout << "- 对于HIPNUC IMU，可通过 ~/.config/lejuconfig/hipimuEulerOffset.csv 调整偏移" << std::endl;
        std::cout << "- 频率应接近500Hz，如偏差过大请检查系统负载和硬件连接" << std::endl;
    }
    
    void publishROSData(const std::string& imu_type, const IMUTestData& data) {
        if (!data.valid) return;
        
        // 发布标准sensor_msgs::Imu消息
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = imu_type + "_imu_link";
        
        imu_msg.linear_acceleration.x = data.acc.x();
        imu_msg.linear_acceleration.y = data.acc.y();
        imu_msg.linear_acceleration.z = data.acc.z();
        
        imu_msg.angular_velocity.x = data.gyro.x();
        imu_msg.angular_velocity.y = data.gyro.y();
        imu_msg.angular_velocity.z = data.gyro.z();
        
        imu_msg.orientation.w = data.quat.w();
        imu_msg.orientation.x = data.quat.x();
        imu_msg.orientation.y = data.quat.y();
        imu_msg.orientation.z = data.quat.z();
        
        // 设置协方差矩阵（简单示例）
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }
        // 对角线元素设置为小值表示高置信度
        imu_msg.orientation_covariance[0] = 0.001;
        imu_msg.orientation_covariance[4] = 0.001;
        imu_msg.orientation_covariance[8] = 0.001;
        
        imu_pub_.publish(imu_msg);
    }
    
    void startPublishing() {
        if (!publishing_enabled_) {
            publishing_enabled_ = true;
            running_ = true;
            publishing_thread_ = std::thread(&IMUTestScript::publishingThreadFunction, this);
            std::cout << "✓ 开始后台发布IMU数据" << std::endl;
        } else {
            std::cout << "发布线程已在运行" << std::endl;
        }
    }
    
    void stopPublishing() {
        if (publishing_enabled_) {
            publishing_enabled_ = false;
            std::cout << "✓ 停止后台发布IMU数据" << std::endl;
        }
    }
    
    void publishingThreadFunction() {
        ros::Rate rate(500); // 500Hz
        
        std::cout << "发布线程启动，频率: 500Hz" << std::endl;
        std::cout << "发布话题:" << std::endl;
        std::cout << "  - /imu/data (sensor_msgs/Imu)" << std::endl;
        
        while (running_ && ros::ok()) {
            if (publishing_enabled_) {
                // 根据指定的IMU类型读取并发布数据
                IMUTestData imu_temp;
                if (testSingleIMU(imu_type_, imu_temp)) {
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        if (imu_type_ == "xsens") {
                            xsens_data_ = imu_temp;
                        } else if (imu_type_ == "hipnuc") {
                            hipnuc_data_ = imu_temp;
                        }
                    }
                    publishROSData(imu_type_, imu_temp);
                }
            }
            
            rate.sleep();
        }
        
        std::cout << "发布线程已退出" << std::endl;
    }
    
    void runContinuousTest() {
        std::cout << "\n======= 连续IMU数据发布模式 =======" << std::endl;
        std::cout << "正在连续发布IMU数据到ROS话题..." << std::endl;
        std::cout << "话题:" << std::endl;
        std::cout << "  /imu/data (sensor_msgs/Imu)" << std::endl;
        std::cout << "按 Ctrl+C 停止..." << std::endl;
        
        running_ = true;
        ros::Rate rate(500); // 500Hz
        
        while (running_ && ros::ok()) {
            // 尝试读取并发布Xsens数据
            if (testSingleIMU("xsens", xsens_data_)) {
                publishROSData("xsens", xsens_data_);
            }
            
            // 尝试读取并发布HIPNUC数据
            if (testSingleIMU("hipnuc", hipnuc_data_)) {
                publishROSData("hipnuc", hipnuc_data_);
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    void stop() {
        running_ = false;
        publishing_enabled_ = false;
    }
};

// 全局变量用于信号处理
IMUTestScript* g_test_script = nullptr;

void signalHandler(int signum) {
    std::cout << "\n接收到中断信号，正在停止测试..." << std::endl;
    if (g_test_script) {
        g_test_script->stop();
    }
    ros::shutdown();
    exit(0);
}



void printUsage(const char* program_name) {
    std::cout << "使用方法: " << program_name << " [IMU类型]" << std::endl;
    std::cout << "参数说明:" << std::endl;
    std::cout << "  IMU类型    : xsens 或 hipnuc (必需)" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "测试内容:" << std::endl;
    std::cout << "  - 基本数据读取测试" << std::endl;
    std::cout << "  - Yaw角偏移测试 (±5°范围)" << std::endl;
    std::cout << "  - 频率测试 (使用rostopic hz)" << std::endl;
    std::cout << "  - 连续数据发布" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << program_name << " xsens" << std::endl;
    std::cout << "  " << program_name << " hipnuc" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_test_script");
    
    // 检查参数数量
    if (argc < 2) {
        std::cerr << "错误: 缺少IMU类型参数" << std::endl;
        printUsage(argv[0]);
        return -1;
    }
    
    // 解析IMU类型参数
    std::string imu_type = std::string(argv[1]);
    if (imu_type != "xsens" && imu_type != "hipnuc") {
        std::cerr << "错误: 不支持的IMU类型 '" << imu_type << "'" << std::endl;
        printUsage(argv[0]);
        return -1;
    }
    
    IMUTestScript test_script;
    g_test_script = &test_script;
    
    // 设置指定的IMU类型
    test_script.setIMUType(imu_type);
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    if (!test_script.init()) {
        std::cerr << "初始化失败，退出" << std::endl;
        return -1;
    }
    
    std::cout << "初始化成功，开始IMU综合测试" << std::endl;
    
    // 运行综合测试
    test_script.runComprehensiveTest();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "         开始连续发布ROS数据" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "程序将持续运行，发布IMU数据到ROS话题:" << std::endl;
    std::cout << "  - /imu/data (sensor_msgs/Imu)" << std::endl;
    std::cout << "按 Ctrl+C 退出程序" << std::endl;
    
    // 保持程序运行，让后台线程持续发布数据
    ros::Rate rate(10); // 10Hz 主循环
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
} 