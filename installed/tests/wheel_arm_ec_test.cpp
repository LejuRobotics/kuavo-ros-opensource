#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <unordered_map>
#include <cstdlib>  // for getenv
#include <unistd.h> // for read, STDIN_FILENO

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
        // 发送关节运动命令
        hardware_plant_->jointMoveTo(joint_values, 30.0, 0.02);
        
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