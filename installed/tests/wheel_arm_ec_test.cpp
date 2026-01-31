#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <unordered_map>
#include <cstdlib>  // for getenv
#include <unistd.h> // for read, STDIN_FILENO
#include <sys/stat.h>  // for stat
#include <sys/types.h> // for stat

#include "hardware_plant.h"
#include "joint_test_poses.h"

#define LB_LEG_JOINT_NUM 4
#define ARM_JOINT_NUM 14
#define HEAD_JOINT_NUM 2
#define TOTAL_JOINT_NUM (LB_LEG_JOINT_NUM + ARM_JOINT_NUM + HEAD_JOINT_NUM)
#define PI 3.14159265359

std::vector<double> init_joints_q(TOTAL_JOINT_NUM, 0);

std::vector<double> test_lb_leg_joints_q;
std::vector<double> test_arm_joints_q;
std::vector<double> test_head_joints_q;

using namespace HighlyDynamic;

// 检查路径是否存在且包含config目录
bool checkAssetsPath(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        return false;  // 路径不存在
    }
    if (!(info.st_mode & S_IFDIR)) {
        return false;  // 不是目录
    }
    
    // 检查是否存在config子目录
    std::string config_path = path + "/config";
    if (stat(config_path.c_str(), &info) != 0) {
        return false;  // config目录不存在
    }
    if (!(info.st_mode & S_IFDIR)) {
        return false;  // config不是目录
    }
    
    return true;
}

// 查找可能的kuavo_assets路径
std::string findKuavoAssetsPath() {
    // 可能的路径列表（按优先级排序）
    std::vector<std::string> possible_paths = {
        "/home/lab/kuavo-ros-control/src/kuavo_assets",
        "/home/lab/kuavo-ros-opensource/src/kuavo_assets",
        // 可以添加更多可能的路径
    };
    
    std::cout << "正在查找 kuavo_assets 路径..." << std::endl;
    
    // 遍历可能的路径
    for (const auto& path : possible_paths) {
        std::cout << "  检查路径: " << path << std::endl;
        if (checkAssetsPath(path)) {
            std::cout << "  ✓ 找到有效路径: " << path << std::endl;
            return path;
        } else {
            std::cout << "  ✗ 路径不存在或无效" << std::endl;
        }
    }
    
    // 如果都找不到，尝试使用编译时的路径
    std::string compile_time_path = KUAVO_ASSETS_PATH;
    std::cout << "  检查编译时路径: " << compile_time_path << std::endl;
    if (checkAssetsPath(compile_time_path)) {
        std::cout << "  ✓ 使用编译时路径: " << compile_time_path << std::endl;
        return compile_time_path;
    } else {
        std::cout << "  ✗ 编译时路径也无效" << std::endl;
    }
    
    // 如果都找不到，返回编译时的路径作为后备（即使可能无效）
    std::cerr << "警告: 未找到有效的 kuavo_assets 路径，使用编译时路径: " << compile_time_path << std::endl;
    return compile_time_path;
}

void initializeTestPoses() {
    auto test_poses = joint_test_poses::test_pos_list();
    
    // 轮臂腿部测试位置（4个关节）
    test_lb_leg_joints_q = {0.25, -0.4, 0.03, 0.0}; // [knee, leg, waist_pitch, waist_yaw]
    
    // 手臂测试位置（14个关节）
    if (test_poses.size() > 1) {
        test_arm_joints_q = test_poses[1];
    } else {
        // 默认手臂位置
        test_arm_joints_q = std::vector<double>(ARM_JOINT_NUM, 0.0);
        test_arm_joints_q[0] = 0.35;  // 左肩roll
        test_arm_joints_q[6] = 0.35;  // 右肩roll
        test_arm_joints_q[3] = -0.52; // 左肘
        test_arm_joints_q[9] = -0.52; // 右肘
    }
    
    // 头部测试位置（2个关节）
    if (test_poses.size() > 2) {
        test_head_joints_q = test_poses[2];
    } else {
        test_head_joints_q = {0.0, 0.0}; // [yaw, pitch]
    }
}

class WheelArmECTest
{
public:
    WheelArmECTest() {
        // 关节运动速度参数（度/秒）
        // 建议值：1.0-10.0，越小越慢越平滑
        // 1.0 = 很慢，适合精细调试
        // 3.0 = 较慢，适合安全测试
        // 5.0 = 中等速度（默认）
        // 10.0 = 较快
        joint_move_speed_ = 6.0;  // 默认速度：3度/秒（可调整，越小越慢）
        
        // 插值时间步长（秒）
        // 建议值：0.01-0.05，越小插值越平滑但计算频率越高
        // 0.01 = 10ms，非常平滑但计算量大
        // 0.02 = 20ms，平衡（默认）
        // 0.05 = 50ms，较快但可能不够平滑
        joint_move_dt_ = 0.001;
    }
    std::vector<uint8_t> joint_ids;
    std::vector<JointParam_t> joint_data;
    
    ~WheelArmECTest()
    {
        if (hardware_plant_) {
            hardware_plant_.reset();
        }
    }

    void init(const std::string &kuavo_assets_path="") {
        const char* robot_version_env = std::getenv("ROBOT_VERSION");
        if (robot_version_env == nullptr) {
            std::cerr << "错误：未设置环境变量 ROBOT_VERSION" << std::endl;
            std::cerr << "使用默认版本 42" << std::endl;
        }
        else{
            std::cout << "检测到环境变量 ROBOT_VERSION: \"" << robot_version_env << "\"" << std::endl;
            this->robot_version_int = std::atoi(robot_version_env);
            std::cout << "解析后的机器人版本: " << this->robot_version_int << std::endl;
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
            // 自动查找可能的路径
            hardware_param.kuavo_assets_path = findKuavoAssetsPath();
        }
        else{
            // 使用用户指定的路径
            std::cout << "使用用户指定的路径: " << kuavo_assets_path << std::endl;
            hardware_param.kuavo_assets_path = kuavo_assets_path;
        }
        
        std::cout << "最终使用的 kuavo_assets_path: " << hardware_param.kuavo_assets_path << std::endl;
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
    }

    void printCurrentJointAngles() {
        std::cout << "\n=== 当前关节角度 ===" << std::endl;
        
        // 获取当前关节数据
        hardware_plant_->GetMotorData(joint_ids, joint_data);
        
        // 打印轮臂腿部关节角度
        std::cout << "轮臂腿部关节:" << std::endl;
        std::vector<std::string> lb_joint_names = {"膝关节", "腿部关节", "腰部俯仰", "腰部偏航"};
        for (int i = 0; i < LB_LEG_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i].position;
            std::cout << "  " << lb_joint_names[i] << " (ID:" << (i+1) << "): " 
                      << std::fixed << std::setprecision(2) << angle_deg << "° (" 
                      << std::setprecision(4) << joint_data[i].position  / (180.0 / PI) << " rad)" << std::endl;
        }
        
        // 打印手臂关节角度
        std::cout << "\n手臂关节:" << std::endl;
        std::vector<std::string> arm_joint_names = {
            "左肩Roll", "左肩Pitch", "左肩Yaw", "左肘", "左前臂", "左腕Roll", "左腕Pitch",
            "右肩Roll", "右肩Pitch", "右肩Yaw", "右肘", "右前臂", "右腕Roll", "右腕Pitch"
        };
        for (int i = 0; i < ARM_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i + LB_LEG_JOINT_NUM].position;
            std::cout << "  " << arm_joint_names[i] << " (ID:" << (i + LB_LEG_JOINT_NUM + 1) << "): " 
                      << std::fixed << std::setprecision(2) << angle_deg << "° (" 
                      << std::setprecision(4) << joint_data[i + LB_LEG_JOINT_NUM].position / (180.0 / PI) << " rad)" << std::endl;
        }
        
        // 打印头部关节角度
        std::cout << "\n头部关节:" << std::endl;
        std::vector<std::string> head_joint_names = {"头部偏航", "头部俯仰"};
        for (int i = 0; i < HEAD_JOINT_NUM; ++i) {
            double angle_deg = joint_data[i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM].position;
            std::cout << "  " << head_joint_names[i] << " (ID:" << (i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM + 1) << "): " 
                      << std::fixed << std::setprecision(2) << angle_deg << "° (" 
                      << std::setprecision(4) << joint_data[i + LB_LEG_JOINT_NUM + ARM_JOINT_NUM].position / (180.0 / PI)<< " rad)" << std::endl;
        }
        std::cout << std::endl;
    }

    void sendJointMoveToRequest(const std::vector<double>& joint_values, const std::string& joint_type) {
        std::cout << "发送 " << joint_type << " 关节运动请求" << std::endl;
        
        for(int i = 0; i < joint_values.size(); ++i){

            std::cout << joint_values[i] << " ";
        }
        std::cout << std::endl;
        // 发送关节运动命令（使用配置的速度和插值步长）
        std::cout << "运动速度: " << joint_move_speed_ << " 度/秒, 插值步长: " << joint_move_dt_ << " 秒" << std::endl;
        hardware_plant_->jointMoveTo(joint_values, joint_move_speed_, joint_move_dt_);
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 打印运动后的关节角度
        printCurrentJointAngles();
    }
    
    void testIndividualJoint() {
        std::cout << "测试单个关节控制" << std::endl;
        std::cout << "请输入关节ID (1-" << TOTAL_JOINT_NUM << "): ";
        
        int joint_id;
        std::cin >> joint_id;
        
        if (joint_id < 1 || joint_id > TOTAL_JOINT_NUM) {
            std::cout << "无效的关节ID" << std::endl;
            return;
        }
        
        // 检查关节状态
        auto allJointsStatus = hardware_plant_->getAllJointsStatus();
        auto it = allJointsStatus.find(joint_id);
        if (it != allJointsStatus.end()) {
            std::string status_str;
            switch (it->second) {
                case MotorStatus::ENABLE: status_str = "已启用"; break;
                case MotorStatus::DISABLED: status_str = "已禁用"; break;
                case MotorStatus::ERROR: status_str = "错误"; break;
                default: status_str = "未知"; break;
            }
            std::cout << "关节 " << joint_id << " 当前状态: " << status_str << std::endl;
            
            if (it->second == MotorStatus::DISABLED || it->second == MotorStatus::ERROR) {
                std::cerr << "警告: 关节 " << joint_id << " 处于 " << status_str << " 状态，可能无法控制！" << std::endl;
                std::cerr << "建议: 请先检查硬件连接和EC状态，或使用 'c' 命令查看详细状态" << std::endl;
                return;
            }
        } else {
            std::cout << "警告: 未找到关节 " << joint_id << " 的状态信息" << std::endl;
        }
        
        std::cout << "请输入目标角度 (度): ";
        double target_angle_deg;
        std::cin >> target_angle_deg;
        
        hardware_plant_->GetMotorData(joint_ids, joint_data);
        
        std::vector<double> joint_command;
        joint_command.resize(joint_data.size());
        for(int i = 0; i < joint_data.size(); ++i){

            joint_command[i] = joint_data[i].position;
        }
        joint_command[joint_id - 1] = target_angle_deg;
        
        std::cout << "移动关节 " << joint_id << " 到 " << target_angle_deg << "°" << std::endl;
        sendJointMoveToRequest(joint_command, "单个关节");
    }

    void testECStatus() {
        std::cout << "检查EC状态" << std::endl;
        
        // 获取所有关节状态
        auto allJointsStatus = hardware_plant_->getAllJointsStatus();
        
        std::cout << "关节状态:" << std::endl;
        for (const auto& jointStatus : allJointsStatus) {
            int joint_id = jointStatus.first;
            auto status = jointStatus.second;
            
            std::string status_str;
            switch (status) {
                case MotorStatus::ENABLE: status_str = "已启用"; break;
                case MotorStatus::DISABLED: status_str = "已禁用"; break;
                case MotorStatus::ERROR: status_str = "错误"; break;
                default: status_str = "未知"; break;
            }
            
            std::cout << "  关节 " << joint_id << ": " << status_str << std::endl;
        }
        
        // 打印当前关节角度
        printCurrentJointAngles();
    }

private:
    double dt_ = 0.001;
    int robot_version_int = 42;
    HardwareParam hardware_param;
    std::unique_ptr<HardwarePlant> hardware_plant_;
    
    // 关节运动参数
    double joint_move_speed_;  // 关节运动速度（度/秒），越小越慢
    double joint_move_dt_;      // 插值时间步长（秒），影响插值平滑度
};

int main(int argc, char const *argv[])
{
    std::string kuavo_assets_path = "";
    if (argc > 1 && argv[1] != nullptr) {
        kuavo_assets_path = std::string(argv[1]);
    }
    
    std::cout << "轮臂EC测试程序" << std::endl;
    using namespace HighlyDynamic;
    
    initializeTestPoses();

    std::cout << "初始化轮臂EC测试" << std::endl;
    auto wheel_arm_test = std::make_shared<WheelArmECTest>();
    wheel_arm_test->init(kuavo_assets_path);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 打印当前关节角度
    wheel_arm_test->printCurrentJointAngles();

    auto output_test_menu = [](){
        std::cout << "\n[WheelArmECTest] 测试菜单:" << std::endl;
        std::cout << "按下 'i' 测试单个关节控制" << std::endl;
        std::cout << "按下 'c' 检查EC状态和当前关节角度" << std::endl;
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
                case 'i':
                    wheel_arm_test->testIndividualJoint();
                    output_test_menu();
                    break;
                case 'c':
                    wheel_arm_test->testECStatus();
                    output_test_menu();
                    break;
                case 'p':
                    wheel_arm_test->printCurrentJointAngles();
                    output_test_menu();
                    break;
                case 'q':
                    std::cout << "[WheelArmECTest] 退出" << std::endl;
                    // wheel_arm_test->sendJointMoveToRequest(init_joints_q, "初始化");
                    // std::this_thread::sleep_for(std::chrono::seconds(3));
                    running = false;
                    break;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
} 