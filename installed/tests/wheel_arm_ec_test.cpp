#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <unordered_map>
#include <cstdlib>  // for getenv
#include <unistd.h> // for read, STDIN_FILENO, dup, dup2
#include <sys/stat.h>  // for stat
#include <sys/types.h> // for stat
#include <iomanip>  // for std::setprecision
#include <cstdio>   // for freopen, fflush
#include <fcntl.h>  // for open, O_WRONLY

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

// ============================================
// 磨线程序配置参数（参考 arm_breakin_kuavo_v53_dual_config.yaml）
// ============================================
// 动作配置：
//   - 每帧动作的时长（秒）
//   - 腿部动作关键帧数组（4个关节，单位：度）
//     电机ID：1, 2, 3, 4（下肢3个和腰部）
//   - 左臂动作关键帧数组（7个关节，单位：度）
//     电机ID：5, 6, 7, 8, 9, 10, 11
//     注意：ID 8 必须为负值（小于0）
//   - 右臂电机（ID 12-18）通过左臂动作映射：
//     * ID 12 = ID 5（相同方向）
//     * ID 13 = -ID 6（相反方向）
//     * ID 14 = -ID 7（相反方向）
//     * ID 15 = ID 8（相同方向，必须是负值）
//     * ID 16 = -ID 9（相反方向）
//     * ID 17 = -ID 10（相反方向）
//     * ID 18 = ID 11（相同方向）
//   - 头部动作关键帧数组（2个关节，单位：度）
//     电机ID：19, 20
//     注意：ID 19 可以正负值，ID 20 只能是正值（大于等于0）
// ============================================

// 每帧动作的时长（秒）
const double BREAKIN_FRAME_DURATION_SEC = 3.0;

// 腿部动作关键帧数组（4个关节，单位：度）
// 电机ID：1, 2, 3, 4（下肢3个和腰部）
// 共10组动作，最前和最后都是0
const std::vector<std::vector<double>> BREAKIN_LEG_ACTIONS = {
    {0.0, 0.0, 0.0, 0.0},      // 帧0：第0秒（起始位置）
    {20.0, -30.0, 0.0, 0.0},    // 帧1：第2秒
    {30.0, -80.0, 40.0, 0.0},    // 帧2：第4秒
    {30.0, -80.0, 40.0, 165.0}, // 帧3：第6秒
    {30.0, -80.0, 40.0, 0.0}, // 帧4：第8秒
    {30.0, -80.0, 40.0, -165.0}, // 帧5：第10秒
    {20.0, -80.0, 160.0, 0.0}, // 帧6：第12秒
    {20.0, -80.0, 55.0, 0.0}, // 帧7：第14秒
    {20.0, -40.0, 20.0, 0.0},    // 帧8：第16秒   # shoubu
    {0.0, 0.0, 0.0, 0.0}       // 帧9：第18秒（结束位置）
};

// const std::vector<std::vector<double>> BREAKIN_LEG_ACTIONS = {
//     {0.0, 0.0, 0.0, 0.0},      // 帧0：第0秒（起始位置）
//     {0.0, 0.0, 0.0, 0.0},    // 帧1：第2秒
//     {0.0, 0.0, 0.0, 0.0},    // 帧2：第4秒
//     {0.0, 0.0, 0.0, 0.0}, // 帧3：第6秒
//     {0.0, 0.0, 0.0, 0.0}, // 帧4：第8秒
//     {0.0, 0.0, 0.0, 0.0}, // 帧5：第10秒
//     {0.0, 0.0, 0.0, 0.0}, // 帧6：第12秒
//     {0.0, 0.0, 0.0, 0.0}, // 帧7：第14秒
//     {0.0, 0.0, 0.0, 0.0},    // 帧8：第16秒   # shoubu
//     {0.0, 0.0, 0.0, 0.0}       // 帧9：第18秒（结束位置）
// };

// 左臂动作关键帧数组（7个关节，单位：度）
// 电机ID：5, 6, 7, 8, 9, 10, 11
// 注意：ID 8 肘部（索引3）必须为负值（小于0）
// 共10组动作，最前和最后都是0
const std::vector<std::vector<double>> BREAKIN_LEFT_ARM_ACTIONS = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},              // 帧0：第0秒（起始位置）
    {10.0, 15.0, 15.0, -60.0, 6.0, 6.0, 6.0},            // 帧1：第2秒
    {45.0, 45.0, 30.0, -100.0, 55.0, 15.0, 15.0},     // 帧2：第4秒
    {85.0, 60.0, 45.0, -110.0, 80.0, 25.0, 25.0},     // 帧3：第6秒
    {20.0, 120.0, 70.0, -100.0, 0.0, 25.0, 25.0},     // 帧4：第8秒
    {0.0, 60.0, 0.0, -90.0, -50.0, 25.0, 25.0},     // 帧5：第10秒
    {-50.0, 25.0, -35.0, -90.0, -80.0, 25.0, 25.0},     // 帧6：第12秒   ///保持
    {-145.0, 15.0, -15.0, -50.0, -40.0, 15.0, 15.0},     // 帧7：第14秒
    {-50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},             // 帧8：第16秒
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}               // 帧9：第18秒（结束位置）
};

// 头部动作关键帧数组（2个关节，单位：度）
// 电机ID：19, 20
// 注意：ID 19 可以正负值，ID 20 只能是正值（大于等于0）
// 共10组动作，最前和最后都是0
const std::vector<std::vector<double>> BREAKIN_HEAD_ACTIONS = {
    {0.0, 0.0},      // 帧0：第0秒（起始位置）
    {6.0, 6.0},      // 帧1：第2秒
    {-8.0, 15.0},    // 帧2：第4秒
    {12.0, 22.0},    // 帧3：第6秒
    {15.0, 25.0},    // 帧4：第8秒
    {15.0, 25.0},    // 帧5：第10秒
    {12.0, 22.0},    // 帧6：第12秒
    {-8.0, 15.0},    // 帧7：第14秒
    {6.0, 6.0},      // 帧8：第16秒
    {0.0, 0.0}       // 帧9：第18秒（结束位置）
};

// 动作组结构：定义在特定时间点的目标位置
struct BreakinActionFrame {
    double time_sec;  // 时间点（秒）
    std::vector<double> leg_positions;    // 4个腿部电机的目标位置（度）：ID 1-4
    std::vector<double> left_arm_positions;  // 7个左臂电机的目标位置（度）：ID 5,6,7,8,9,10,11
    std::vector<double> head_positions;     // 2个头部电机的目标位置（度）：ID 19,20
    // 右臂电机（ID 12,13,14,15,16,17,18）通过左臂映射：
    //   ID 12 = ID 5（相同方向）
    //   ID 13 = -ID 6（相反方向）
    //   ID 14 = -ID 7（相反方向）
    //   ID 15 = ID 8（相同方向，必须是负值）
    //   ID 16 = -ID 9（相反方向）
    //   ID 17 = -ID 10（相反方向）
    //   ID 18 = ID 11（相同方向）
    // 头部电机：
    //   ID 19：可以正负值
    //   ID 20：只能是正值（大于等于0）
};

// 磨线动作组定义（腿部+手部+头部）
std::vector<BreakinActionFrame> createBreakinActionSequence() {
    std::vector<BreakinActionFrame> sequence;
    
    // 从配置参数创建动作序列
    size_t max_frames = std::max({BREAKIN_LEG_ACTIONS.size(), 
                                   BREAKIN_LEFT_ARM_ACTIONS.size(), 
                                   BREAKIN_HEAD_ACTIONS.size()});
    
    for (size_t i = 0; i < max_frames; ++i) {
        double time_sec = i * BREAKIN_FRAME_DURATION_SEC;
        
        // 获取各部分的动作帧（如果存在）
        std::vector<double> leg_pos = (i < BREAKIN_LEG_ACTIONS.size()) ? 
                                      BREAKIN_LEG_ACTIONS[i] : std::vector<double>(4, 0.0);
        std::vector<double> arm_pos = (i < BREAKIN_LEFT_ARM_ACTIONS.size()) ? 
                                      BREAKIN_LEFT_ARM_ACTIONS[i] : std::vector<double>(7, 0.0);
        std::vector<double> head_pos = (i < BREAKIN_HEAD_ACTIONS.size()) ? 
                                       BREAKIN_HEAD_ACTIONS[i] : std::vector<double>(2, 0.0);
        
        sequence.push_back({
            time_sec,
            leg_pos,      // 腿部：ID 1-4
            arm_pos,      // 左臂：ID 5,6,7,8,9,10,11
            head_pos      // 头部：ID 19,20
        });
    }
    
    return sequence;
}

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

// 查找可能的kuavo_assets路径（使用rospack）
std::string findKuavoAssetsPath() {
    std::cout << "正在使用 rospack 查找 kuavo_assets 路径..." << std::endl;

    // 使用 rospack find 命令查找 kuavo_assets 包路径
    FILE* pipe = popen("rospack find kuavo_assets 2>/dev/null", "r");
    if (pipe == nullptr) {
        std::cerr << "  ✗ 无法执行 rospack 命令" << std::endl;
    } else {
        char buffer[512];
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            // 移除末尾的换行符
            std::string path(buffer);
            if (!path.empty() && path.back() == '\n') {
                path.pop_back();
            }

            int status = pclose(pipe);
            if (status == 0) {
                std::cout << "  rospack 返回路径: " << path << std::endl;
                if (checkAssetsPath(path)) {
                    std::cout << "  ✓ 找到有效路径: " << path << std::endl;
                    return path;
                } else {
                    std::cout << "  ✗ rospack 返回的路径不存在或无效" << std::endl;
                }
            } else {
                std::cout << "  ✗ rospack find kuavo_assets 失败" << std::endl;
            }
        } else {
            pclose(pipe);
            std::cout << "  ✗ rospack 未返回结果" << std::endl;
        }
    }

    // 如果 rospack 找不到，尝试使用编译时的路径
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
        // 磨线功能速度：20.0（较快，适合批量磨线）
        breakin_speed_ = 35.0;
        // 单关节控制速度：10.0（较慢，适合精细调试）
        individual_joint_speed_ = 10.0;
        
        // 插值时间步长（秒）
        // 建议值：0.01-0.05，越小插值越平滑但计算频率越高
        // 0.01 = 10ms，非常平滑但计算量大
        // 0.02 = 20ms，平衡（默认）
        // 0.05 = 50ms，较快但可能不够平滑
        joint_move_dt_ = 0.005;
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

    void sendJointMoveToRequest(const std::vector<double>& joint_values, const std::string& joint_type, double speed) {
        std::cout << "发送 " << joint_type << " 关节运动请求" << std::endl;
        
        for(int i = 0; i < joint_values.size(); ++i){

            std::cout << joint_values[i] << " ";
        }
        std::cout << std::endl;
        // 发送关节运动命令（使用指定的速度和插值步长）
        std::cout << "运动速度: " << speed << " 度/秒, 插值步长: " << joint_move_dt_ << " 秒" << std::endl;
        hardware_plant_->jointMoveTo(joint_values, speed, joint_move_dt_);
        
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
        sendJointMoveToRequest(joint_command, "单个关节", individual_joint_speed_);
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

    // 静默调用jointMoveTo（抑制底层库的输出）
    void jointMoveToSilent(const std::vector<double>& target_joints, double speed, double dt) {
        #ifndef _WIN32
        // Linux/Unix系统：使用dup2保存和恢复文件描述符
        fflush(stdout);
        fflush(stderr);
        
        // 保存原始的stdout和stderr文件描述符
        int saved_stdout = dup(STDOUT_FILENO);
        int saved_stderr = dup(STDERR_FILENO);
        
        // 打开null设备并重定向stdout和stderr
        int null_fd = open("/dev/null", O_WRONLY);
        if (null_fd >= 0) {
            dup2(null_fd, STDOUT_FILENO);
            dup2(null_fd, STDERR_FILENO);
            close(null_fd);
        }
        
        // 调用jointMoveTo（此时输出被抑制）
        hardware_plant_->jointMoveTo(target_joints, speed, dt);
        
        // 恢复原始的stdout和stderr
        fflush(stdout);
        fflush(stderr);
        if (saved_stdout >= 0) {
            dup2(saved_stdout, STDOUT_FILENO);
            close(saved_stdout);
        }
        if (saved_stderr >= 0) {
            dup2(saved_stderr, STDERR_FILENO);
            close(saved_stderr);
        }
        #else
        // Windows系统：直接调用（Windows下的重定向较复杂，暂时保留输出）
        hardware_plant_->jointMoveTo(target_joints, speed, dt);
        #endif
    }

    // 根据动作帧构建完整的关节命令
    std::vector<double> buildJointCommandFromFrame(const BreakinActionFrame& frame, 
                                                    const std::vector<double>& current_all_joints) {
        std::vector<double> target_all_joints = current_all_joints;
        
        // 设置腿部电机（ID 1-4，索引0-3）
        for (int i = 0; i < LB_LEG_JOINT_NUM && i < frame.leg_positions.size(); ++i) {
            target_all_joints[i] = frame.leg_positions[i];
        }
        
        // 设置左臂电机（ID 5,6,7,8,9,10,11，索引4-10）
        // 注意：ID 8（索引7）必须确保是负值
        if (frame.left_arm_positions.size() >= 7) {
            target_all_joints[4] = frame.left_arm_positions[0];  // ID 5
            target_all_joints[5] = frame.left_arm_positions[1];  // ID 6
            target_all_joints[6] = frame.left_arm_positions[2];  // ID 7
            // ID 8：确保是负值（小于0）
            double motor8_value = frame.left_arm_positions[3];
            target_all_joints[7] = (motor8_value > 0) ? -motor8_value : motor8_value;  // ID 8（强制为负值）
            target_all_joints[8] = frame.left_arm_positions[4];  // ID 9
            target_all_joints[9] = frame.left_arm_positions[5];  // ID 10
            target_all_joints[10] = frame.left_arm_positions[6]; // ID 11
        }
        
        // 设置右臂电机（ID 12,13,14,15,16,17,18，索引11-17）
        // 根据对称关系映射：
        //   ID 12 = ID 5（相同方向）
        //   ID 13 = -ID 6（相反方向）
        //   ID 14 = -ID 7（相反方向）
        //   ID 15 = ID 8（相同方向，但必须是负值）
        //   ID 16 = -ID 9（相反方向）
        //   ID 17 = -ID 10（相反方向）
        //   ID 18 = ID 11（相同方向）
        if (frame.left_arm_positions.size() >= 7 && TOTAL_JOINT_NUM > 17) {
            target_all_joints[11] = frame.left_arm_positions[0];  // ID 12 = ID 5（相同）
            target_all_joints[12] = -frame.left_arm_positions[1]; // ID 13 = -ID 6（相反）
            target_all_joints[13] = -frame.left_arm_positions[2]; // ID 14 = -ID 7（相反）
            // ID 15：与ID 8相同方向，但必须确保是负值
            double motor15_value = target_all_joints[7];  // 使用ID 8的值（已经是负值）
            target_all_joints[14] = motor15_value;  // ID 15 = ID 8（相同，确保为负值）
            target_all_joints[15] = -frame.left_arm_positions[4]; // ID 16 = -ID 9（相反）
            target_all_joints[16] = -frame.left_arm_positions[5]; // ID 17 = -ID 10（相反）
            target_all_joints[17] = frame.left_arm_positions[6];  // ID 18 = ID 11（相同）
        }
        
        // 设置头部电机（ID 19,20，索引18-19）
        // 注意：ID 19 可以正负值，ID 20 只能是正值（大于等于0）
        if (frame.head_positions.size() >= 2 && TOTAL_JOINT_NUM > 19) {
            // ID 19：可以正负值，直接使用配置值
            target_all_joints[18] = frame.head_positions[0];  // ID 19
            
            // ID 20：只能是正值（大于等于0），如果配置值为负则强制为0
            double motor20_value = frame.head_positions[1];
            target_all_joints[19] = (motor20_value < 0) ? 0.0 : motor20_value;  // ID 20（强制为非负值）
        }
        
        return target_all_joints;
    }

    // 执行一轮动作序列（所有动作帧，按关键帧顺序逐帧执行，不做额外插值）
    void executeOneRound(const std::vector<BreakinActionFrame>& action_sequence) {
        if (action_sequence.empty()) {
            return;
        }
        
        // 获取当前所有关节位置
        hardware_plant_->GetMotorData(joint_ids, joint_data);
        std::vector<double> current_all_joints(TOTAL_JOINT_NUM);
        for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
            current_all_joints[i] = joint_data[i].position;
        }
        // 逐帧执行：每一帧调用一次 jointMoveTo（内部包含平滑插值），并等待腿部4个关节到位
        const double LEG_REACH_TOL_DEG = 5;        // 腿部关节到位容差（度）
        const int    LEG_REACH_MAX_WAIT_MS = 5000;   // 最长等待时间（毫秒）

        for (size_t frame_index = 0; frame_index < action_sequence.size(); ++frame_index) {
            const auto& frame = action_sequence[frame_index];

            // 基于当前姿态构建这一帧的完整目标关节命令
            std::vector<double> target_all_joints = buildJointCommandFromFrame(frame, current_all_joints);

            std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
            std::cout << "  [关键帧] 帧 " << (frame_index + 1) << " / " << action_sequence.size()
                      << ", 计划时间戳: " << std::fixed << std::setprecision(1)
                      << frame.time_sec << " 秒" << std::endl;
            std::cout << "  目标腿部关节 (ID 1-4): ";
            for (int i = 0; i < LB_LEG_JOINT_NUM; ++i) {
                std::cout << std::fixed << std::setprecision(2) << target_all_joints[i] << " ";
            }
            std::cout << " (单位: 度)" << std::endl;

            // 调用底层 jointMoveTo（内部会做平滑插值并阻塞，直到动作完成）
            jointMoveToSilent(target_all_joints, breakin_speed_, joint_move_dt_);

            // 额外等待：确认腿部4个关节已经到位（在一定容差内）
            bool leg_reached = false;
            auto wait_start = std::chrono::steady_clock::now();
            while (true) {
                hardware_plant_->GetMotorData(joint_ids, joint_data);

                double max_leg_error = 0.0;
                for (int i = 0; i < LB_LEG_JOINT_NUM; ++i) {
                    double err = std::fabs(joint_data[i].position - target_all_joints[i]);
                    if (err > max_leg_error) {
                        max_leg_error = err;
                    }
                }

                if (max_leg_error <= LEG_REACH_TOL_DEG) {
                    leg_reached = true;
                    std::cout << "  腿部关节已到位，最大误差: "
                              << std::fixed << std::setprecision(3) << max_leg_error << " 度" << std::endl;
                    break;
                }

                auto now = std::chrono::steady_clock::now();
                double wait_ms = std::chrono::duration<double, std::milli>(now - wait_start).count();
                if (wait_ms >= LEG_REACH_MAX_WAIT_MS) {
                    std::cout << "  警告: 腿部关节未在期望时间内完全到位，最大误差: "
                              << std::fixed << std::setprecision(3) << max_leg_error << " 度" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (!leg_reached) {
                std::cout << "  提示: 即使腿部误差稍大，也将继续执行后续关键帧。" << std::endl;
            }

            // 更新当前关节位置为实际读取到的位置
            for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
                current_all_joints[i] = joint_data[i].position;
            }
        }

        // 最后一帧完成后，稍作停顿
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void runBreakinSequence() {
        // ANSI颜色代码
        const char* RED = "\033[0;31m";
        const char* RESET = "\033[0m";
        
        std::cout << "\n=== 开始磨线功能（腿部+手部+头部） ===" << std::endl;
        std::cout << RED << "⚠ 请确认轮臂机器人标定过零点，且目前位于零点位置" << RESET << std::endl;
        std::cout << std::endl;
        
        // 检查相关关节的状态
        auto allJointsStatus = hardware_plant_->getAllJointsStatus();
        bool all_enabled = true;
        
        // 检查腿部电机（ID 1-4）
        std::vector<int> leg_motor_ids = {1, 2, 3, 4};
        // 检查手部电机（左臂ID 5-11，右臂ID 12-18）
        std::vector<int> arm_motor_ids = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
        // 检查头部电机（ID 19-20）
        std::vector<int> head_motor_ids = {19, 20};
        
        std::vector<int> all_check_ids = leg_motor_ids;
        all_check_ids.insert(all_check_ids.end(), arm_motor_ids.begin(), arm_motor_ids.end());
        all_check_ids.insert(all_check_ids.end(), head_motor_ids.begin(), head_motor_ids.end());
        
        for (int motor_id : all_check_ids) {
            auto it = allJointsStatus.find(motor_id);
            if (it != allJointsStatus.end()) {
                if (it->second != MotorStatus::ENABLE) {
                    std::cerr << "警告: 关节 " << motor_id << " 未启用，状态: ";
                    switch (it->second) {
                        case MotorStatus::DISABLED: std::cerr << "已禁用"; break;
                        case MotorStatus::ERROR: std::cerr << "错误"; break;
                        default: std::cerr << "未知"; break;
                    }
                    std::cerr << std::endl;
                    all_enabled = false;
                }
            } else {
                std::cerr << "警告: 未找到关节 " << motor_id << " 的状态信息" << std::endl;
                all_enabled = false;
            }
        }
        
        if (!all_enabled) {
            std::cerr << "错误: 部分关节未启用，无法执行磨线功能" << std::endl;
            return;
        }
        
        // 创建动作序列
        auto action_sequence = createBreakinActionSequence();
        
        // 计算每轮大约时长（最后一帧的时间 + 缓冲时间）
        double round_duration_sec = 0.0;
        if (!action_sequence.empty()) {
            round_duration_sec = action_sequence.back().time_sec + 1.0;  // 最后一帧时间 + 1秒缓冲
        }
        
        // 提示用户输入轮数
        std::cout << "\n每轮动作大约需要: " << std::fixed << std::setprecision(1) << round_duration_sec << " 秒" << std::endl;
        std::cout << "请输入要磨线的轮数: ";
        
        int target_rounds = 0;
        std::cin >> target_rounds;
        
        if (target_rounds <= 0) {
            std::cout << "无效的轮数，取消磨线操作" << std::endl;
            return;
        }
        
        std::cout << "\n将执行 " << target_rounds << " 轮磨线动作" << std::endl;
        std::cout << "预计总时长: " << std::fixed << std::setprecision(1) 
                  << (target_rounds * round_duration_sec) << " 秒" << std::endl;
        std::cout << "\n开始执行磨线动作序列..." << std::endl;
        std::cout << std::endl;
        
        // 循环执行指定轮数
        for (int round = 1; round <= target_rounds; ++round) {
            std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
            std::cout << "开始第 " << round << " / " << target_rounds << " 轮" << std::endl;
            std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
            
            // 执行一轮动作序列
            executeOneRound(action_sequence);
            
            // 显示完成进度
            std::cout << "\n✓ 第 " << round << " / " << target_rounds << " 轮完成" << std::endl;
            
            // 如果不是最后一轮，稍作停顿
            if (round < target_rounds) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        // 打印最终关节角度
        std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "磨线动作序列执行完成，共完成 " << target_rounds << " 轮" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        printCurrentJointAngles();
    }

private:
    double dt_ = 0.001;
    int robot_version_int = 42;
    HardwareParam hardware_param;
    std::unique_ptr<HardwarePlant> hardware_plant_;
    
    // 关节运动参数
    double breakin_speed_;           // 磨线功能速度（度/秒），默认20.0
    double individual_joint_speed_;  // 单关节控制速度（度/秒），默认10.0
    double joint_move_dt_;           // 插值时间步长（秒），影响插值平滑度
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
        std::cout << "按下 'm' 执行磨线功能" << std::endl;
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
                case 'm':
                    wheel_arm_test->runBreakinSequence();
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