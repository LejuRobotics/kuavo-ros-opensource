/*
 * ============================================================================
 * 机型配置说明：kuavo_v52_dual
 * ============================================================================
 * 当前代码适配的机型：kuavo_v52_dual（双CAN版本，机器人版本52）
 * 
 * 电机配置：
 *   - 总电机数量：14个（ID 1-14）
 *   - 手臂电机：12个（ID 1-12，左臂1-6，右臂7-12）
 *   - 头部电机：2个（ID 13-14）
 * 
 * 动作序列配置：
 *   - 左臂动作：6关节，7帧动作序列（小动作±0.2弧度）
 *   - 右臂映射：通过左臂动作序列映射
 *     * ID 7 = -ID 1（反方向）
 *     * ID 8 = -ID 2（反方向）
 *     * ID 9 = ID 3（同方向）
 *     * ID 10 = -ID 4（反方向）
 *     * ID 11 = -ID 5（反方向）
 *     * ID 12 = ID 6（同方向）
 *   - 头部动作：2关节，7帧动作序列
 * 
 * 注意：此配置仅适用于 kuavo_v52_dual 型机器人
 * ============================================================================
 */

#include "motorevo/motorevo_actuator.h"
#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <limits>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <atomic>
#include <csignal>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sstream>
#include <libgen.h>
#include <limits.h>
#include <cstring>

// 全局时间戳工具函数（[HH:MM:SS.mmm]）
static std::string get_timestamp() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm_local{};
#if defined(_WIN32)
    localtime_s(&tm_local, &t);
#else
    localtime_r(&t, &tm_local);
#endif
    std::ostringstream oss;
    oss << '[' << std::put_time(&tm_local, "%H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms.count() << ']';
    return oss.str();
}

class ArmBreakinNode {
private:
    ros::NodeHandle* nh_;
    ros::Publisher pub_arm_ready_;
    ros::Publisher pub_arm_started_;  // 发布手臂已开始运行
    ros::Publisher pub_arm_running_;  // 发布手臂正在运行状态（1Hz）：正在执行动作帧时为True，等待下一轮时为False
    ros::Subscriber sub_start_together_;
    ros::Subscriber sub_allow_run_;
    ros::Subscriber sub_start_new_round_arm_;  // 订阅主程序发布的"开始新一轮"信号
    ros::Subscriber sub_standalone_mode_;  // 订阅单独运行模式
    
    std::atomic<bool> arm_ready_;
    std::atomic<bool> arm_running_;  // 手臂是否正在执行动作帧（执行时为True，等待下一轮时为False）
    std::atomic<bool> start_received_;  // 是否收到开始第一轮的信号（start_together或standalone模式）
    std::atomic<bool> start_together_received_;  // 用于记录是否收到start_together信号（即使allow_run为false也记录）
    std::atomic<bool> start_new_round_arm_;  // 是否收到开始新一轮的信号
    std::atomic<bool> allow_run_;
    std::atomic<bool> standalone_mode_;  // 单独运行模式标志
    std::atomic<bool> should_exit_;
    std::atomic<bool> arm_started_;  // 手臂是否已开始运行
    
    motorevo::MotorevoActuator* actuator_;
    std::string config_file_path_;
    std::string action_config_file_path_;  // 动作配置文件路径
    
    std::thread publish_thread_;
    std::thread publish_running_thread_;  // 发布arm_running的线程
    std::atomic<bool> stop_publish_thread_;
    std::atomic<bool> stop_publish_running_thread_;
    
public:
    ArmBreakinNode(int argc, char** argv) : nh_(nullptr), arm_ready_(false), arm_running_(false), start_received_(false), 
                                             start_together_received_(false), start_new_round_arm_(false), allow_run_(true), 
                                             standalone_mode_(false), should_exit_(false), arm_started_(false), 
                                             actuator_(nullptr), stop_publish_thread_(false), stop_publish_running_thread_(false) {
        // 初始化ROS节点（必须在创建NodeHandle之前）
        ros::init(argc, argv, "arm_breakin_kuavo_v52_dual_node");
        
        // 创建NodeHandle（在ros::init()之后）
        nh_ = new ros::NodeHandle();
        
        // 创建发布者（使用latch确保退出时的False消息能被接收）
        pub_arm_ready_ = nh_->advertise<std_msgs::Bool>("/breakin/arm_ready", 10, true);
        pub_arm_started_ = nh_->advertise<std_msgs::Bool>("/breakin/arm_started", 10, true);
        pub_arm_running_ = nh_->advertise<std_msgs::Bool>("/breakin/arm_running", 10);
        
        // 创建订阅者
        sub_start_together_ = nh_->subscribe("/breakin/start_together", 10, &ArmBreakinNode::startTogetherCallback, this);
        sub_allow_run_ = nh_->subscribe("/breakin/allow_run", 10, &ArmBreakinNode::allowRunCallback, this);
        sub_start_new_round_arm_ = nh_->subscribe("/breakin/start_new_round_arm", 10, &ArmBreakinNode::startNewRoundArmCallback, this);
        sub_standalone_mode_ = nh_->subscribe("/breakin/standalone_mode", 10, &ArmBreakinNode::standaloneModeCallback, this);
        
        // 等待发布者注册
        ros::Duration(0.5).sleep();
        
        // 从ROS参数服务器获取配置文件路径
        nh_->param<std::string>("config_file", config_file_path_, "");
        if (config_file_path_.empty()) {
            config_file_path_ = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
        }
        
        // 从ROS参数服务器获取动作配置文件路径
        nh_->param<std::string>("action_config_file", action_config_file_path_, "");
        if (action_config_file_path_.empty()) {
            // 辅助函数：检查文件是否存在
            auto file_exists = [](const std::string& path) -> bool {
                struct stat buffer;
                return (stat(path.c_str(), &buffer) == 0);
            };
            
            // 辅助函数：拼接路径
            auto join_path = [](const std::string& base, const std::vector<std::string>& parts) -> std::string {
                std::ostringstream oss;
                oss << base;
                for (const auto& part : parts) {
                    if (!oss.str().empty() && oss.str().back() != '/') {
                        oss << "/";
                    }
                    oss << part;
                }
                return oss.str();
            };
            
            // 尝试多个可能的路径：优先使用可执行文件路径，其次使用当前工作目录
            std::vector<std::string> search_paths;
            
            // 1. 尝试使用可执行文件路径（最可靠的方法）
            char exe_path[PATH_MAX];
            ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
            if (len != -1) {
                exe_path[len] = '\0';
                // dirname可能会修改字符串，所以需要先复制
                char exe_path_copy[PATH_MAX];
                strncpy(exe_path_copy, exe_path, sizeof(exe_path_copy) - 1);
                exe_path_copy[sizeof(exe_path_copy) - 1] = '\0';
                char* exe_dir = dirname(exe_path_copy);
                std::string workspace_root = exe_dir;
                // 可执行文件可能在以下位置：
                // - .../joint_breakin_ros/devel/lib/arm_breakin/
                // - .../joint_breakin_ros/build/lib/arm_breakin/
                // - .../joint_breakin_ros/build_lib/lib/arm_breakin/  (新增)
                // 需要向上找到工作空间根目录
                size_t pos1 = workspace_root.find("/devel/lib/arm_breakin");
                size_t pos2 = workspace_root.find("/build/lib/arm_breakin");
                size_t pos3 = workspace_root.find("/build_lib/lib/arm_breakin");
                if (pos1 != std::string::npos) {
                    workspace_root = workspace_root.substr(0, pos1);
                    search_paths.push_back(join_path(workspace_root, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                } else if (pos2 != std::string::npos) {
                    workspace_root = workspace_root.substr(0, pos2);
                    search_paths.push_back(join_path(workspace_root, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                } else if (pos3 != std::string::npos) {
                    workspace_root = workspace_root.substr(0, pos3);
                    search_paths.push_back(join_path(workspace_root, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                } else {
                    // 如果无法从路径模式匹配，尝试向上遍历目录树查找包含 config/arm_breakin 的目录
                    std::string current_path = exe_dir;
                    for (int i = 0; i < 10; ++i) {  // 最多向上10级
                        std::string test_path = join_path(current_path, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"});
                        if (file_exists(test_path)) {
                            search_paths.push_back(test_path);
                            break;
                        }
                        // 向上移动一级
                        size_t last_slash = current_path.find_last_of('/');
                        if (last_slash == std::string::npos || last_slash == 0) {
                            break;
                        }
                        current_path = current_path.substr(0, last_slash);
                    }
                }
            }
            
            // 2. 尝试使用环境变量（如果设置了ROS_WORKSPACE环境变量）
            const char* ros_workspace = getenv("ROS_WORKSPACE");
            if (ros_workspace != nullptr) {
                search_paths.push_back(join_path(ros_workspace, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
            }
            
            // 3. 尝试使用当前工作目录（作为备选）
            char cwd[PATH_MAX];
            if (getcwd(cwd, sizeof(cwd)) != nullptr) {
                std::string workspace_path = cwd;
                search_paths.push_back(join_path(workspace_path, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                search_paths.push_back(join_path(workspace_path, {"..", "config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                // 如果在src目录下
                size_t src_pos = workspace_path.find("/src/");
                if (src_pos != std::string::npos) {
                    std::string workspace_root = workspace_path.substr(0, src_pos);
                    search_paths.push_back(join_path(workspace_root, {"config", "arm_breakin", "arm_breakin_kuavo_v52_dual_config.yaml"}));
                }
            }
            
            // 遍历所有可能的路径
            std::cout << get_timestamp() << " [信息] 正在查找动作配置文件，搜索路径：" << std::endl;
            for (const auto& path : search_paths) {
                std::cout << get_timestamp() << "  - " << path;
                if (file_exists(path)) {
                    action_config_file_path_ = path;
                    std::cout << " [找到]" << std::endl;
                    break;
                } else {
                    std::cout << " [不存在]" << std::endl;
                }
            }
            if (action_config_file_path_.empty()) {
                std::cout << get_timestamp() << " [警告] 未找到动作配置文件，将使用默认硬编码配置" << std::endl;
            }
        }
        
        std::cout << get_timestamp() << " 手臂磨线ROS节点已启动 [机型: kuavo_v52_dual]" << std::endl;
        std::cout << get_timestamp() << " 使用配置文件: " << config_file_path_ << std::endl;
        if (!action_config_file_path_.empty()) {
            std::cout << get_timestamp() << " 使用动作配置文件: " << action_config_file_path_ << std::endl;
        }
        
        // 启动1Hz发布线程
        stop_publish_thread_ = false;
        publish_thread_ = std::thread(&ArmBreakinNode::publishArmReadyLoop, this);
        
        // 启动1Hz发布arm_running线程
        stop_publish_running_thread_ = false;
        publish_running_thread_ = std::thread(&ArmBreakinNode::publishArmRunningLoop, this);
        
        // 等待线程启动并发布一次arm_ready = False（表示节点已启动但还未准备好）
        ros::Duration(0.1).sleep();  // 给线程一点时间启动
        publishArmReady(false);
        publishArmRunning(false);  // 初始状态为False
        std::cout << get_timestamp() << " 手臂磨线节点已启动，arm_ready = False（等待初始化）" << std::endl;
    }
    
    ArmBreakinNode(const ArmBreakinNode&) = delete;
    ArmBreakinNode& operator=(const ArmBreakinNode&) = delete;
    ArmBreakinNode(ArmBreakinNode&&) = delete;
    ArmBreakinNode& operator=(ArmBreakinNode&&) = delete;
    
    ~ArmBreakinNode() {
        // 停止发布线程
        stop_publish_thread_ = true;
        stop_publish_running_thread_ = true;
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
        if (publish_running_thread_.joinable()) {
            publish_running_thread_.join();
        }
        
        // 发布False并清理
        publishArmReady(false);
        publishArmStarted(false);
        publishArmRunning(false);
        if (actuator_) {
            actuator_->disable();
            actuator_->reset();
            delete actuator_;
        }
        if (nh_) {
            delete nh_;
            nh_ = nullptr;
        }
    }
    
    void publishArmReadyLoop() {
        // 1Hz发布arm_ready状态
        ros::Rate rate(1);
        while (!stop_publish_thread_) {
            if (!ros::ok()) {
                break;
            }
            // 发布当前arm_ready状态
            std_msgs::Bool msg;
            msg.data = arm_ready_;
            pub_arm_ready_.publish(msg);
            rate.sleep();
        }
        // 退出时发布False
        std_msgs::Bool msg;
        msg.data = false;
        pub_arm_ready_.publish(msg);
    }
    
    void publishArmRunningLoop() {
        // 1Hz发布arm_running状态
        ros::Rate rate(1);
        while (!stop_publish_running_thread_) {
            if (!ros::ok()) {
                break;
            }
            // 发布当前arm_running状态
            std_msgs::Bool msg;
            msg.data = arm_running_;
            pub_arm_running_.publish(msg);
            rate.sleep();
        }
        // 退出时发布False
        std_msgs::Bool msg;
        msg.data = false;
        pub_arm_running_.publish(msg);
    }
    
    void publishArmRunning(bool running) {
        std_msgs::Bool msg;
        msg.data = running;
        pub_arm_running_.publish(msg);
        arm_running_ = running;
    }
    
    void startNewRoundArmCallback(const std_msgs::Bool::ConstPtr& msg) {
        // 收到开始新一轮的信号
        if (msg->data) {
            start_new_round_arm_.store(true);
            std::cout << get_timestamp() << " 收到 start_new_round_arm = True，准备开始新一轮动作" << std::endl;
        } else {
            start_new_round_arm_.store(false);
        }
    }
    
    void standaloneModeCallback(const std_msgs::Bool::ConstPtr& msg) {
        standalone_mode_ = msg->data;
        if (standalone_mode_) {
            std::cout << get_timestamp() << " 检测到单独运行模式，无需等待腿部信号" << std::endl;
            // 单独运行模式下，自动设置start_received_为true
            start_received_ = true;
        }
    }
    
    void publishArmStarted(bool started) {
        std_msgs::Bool msg;
        msg.data = started;
        pub_arm_started_.publish(msg);
        arm_started_ = started;
    }
    
    void startTogetherCallback(const std_msgs::Bool::ConstPtr& msg) {
        static bool last_start_together_state = false;
        bool current_state = msg->data;
        
        if (current_state) {
            start_together_received_.store(true);
            // 只在状态变化时打印（从False变为True时）
            if (!last_start_together_state) {
                std::cout << get_timestamp() << " 收到 start_together = True" << std::endl;
            }
            // 如果allow_run也为true，立即设置start_received_
            if (allow_run_ && !start_received_) {
                std::cout << get_timestamp() << " allow_run = True，开始执行动作" << std::endl;
                start_received_ = true;
            } else if (!allow_run_ && !last_start_together_state) {
                // 只在第一次收到start_together时打印等待信息
                std::cout << get_timestamp() << " allow_run = False，等待allow_run变为True..." << std::endl;
            }
        } else {
            start_together_received_.store(false);
            // 只在状态变化时打印（从True变为False时）
            if (last_start_together_state && start_received_) {
                std::cout << get_timestamp() << " 收到 start_together = False，重置标志" << std::endl;
                start_received_ = false;
            }
        }
        last_start_together_state = current_state;
    }
    
    void allowRunCallback(const std_msgs::Bool::ConstPtr& msg) {
        bool old_allow_run = allow_run_;
        allow_run_ = msg->data;
        if (!allow_run_ && old_allow_run) {
            std::cout << get_timestamp() << " [警告] 收到 allow_run = False，立即停止运动" << std::endl;
            should_exit_ = true;
            publishArmRunning(false);  // 停止时发布False
            // 立即失能电机
            if (actuator_) {
                std::cout << get_timestamp() << " 正在失能所有电机..." << std::endl;
                actuator_->disable();
            }
        } else if (allow_run_ && !old_allow_run && !standalone_mode_) {
            // 在同时运行模式下，如果allow_run从false变为true，且已经收到start_together信号，则开始执行
            std::cout << get_timestamp() << " allow_run 从 False 变为 True" << std::endl;
            if (start_together_received_.load() && !start_received_) {
                std::cout << get_timestamp() << " ✓ 同时满足 start_together=True 和 allow_run=True，开始执行动作" << std::endl;
                start_received_ = true;
            }
        }
    }
    
    void publishArmReady(bool ready) {
        std_msgs::Bool msg;
        msg.data = ready;
        pub_arm_ready_.publish(msg);
        arm_ready_ = ready;
    }
    
    // 检查目标电机是否失能，如果失能则返回true并打印错误信息
    bool checkMotorDisabled(const std::vector<uint8_t>& target_motor_ids) {
        // 停止流程（收到 allow_run = false / 退出信号）时不再检测，避免主动失能后重复报警
        if (!actuator_ || should_exit_ || !allow_run_) {
            return false;
        }
        
        auto motor_states = actuator_->get_motor_state();
        std::vector<uint8_t> disabled_motors;
        
        for (uint8_t motor_id : target_motor_ids) {
            // 在motor_states中查找对应的电机
            bool found = false;
            for (size_t idx = 0; idx < motor_states.size(); ++idx) {
                auto [id, state] = motor_states[idx];
                
                if (id == motor_id) {
                    found = true;
                    // 检查状态是否为Disabled（失能）
                    if (state == RuiwoActuatorBase::State::Disabled) {
                        disabled_motors.push_back(motor_id);
                    }
                    break;
                }
            }
            if (!found) {
                // 如果找不到电机，也认为是异常
                std::cout << get_timestamp() << " [警告] 未找到电机 ID " << static_cast<int>(motor_id) << " 的状态" << std::endl;
            }
        }
        
        if (!disabled_motors.empty()) {
            std::cout << get_timestamp() << " [错误] 检测到以下电机失能: ";
            for (size_t i = 0; i < disabled_motors.size(); ++i) {
                std::cout << static_cast<int>(disabled_motors[i]);
                if (i < disabled_motors.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "，立即停止运动！" << std::endl;
            return true;
        }
        
        return false;
    }
    
    // 从YAML文件读取动作配置
    bool loadActionConfig(std::vector<std::array<double,6>>& left_arm_actions,
                         std::vector<std::array<double,2>>& head_actions,
                         int& frame_duration_ms) {
        if (action_config_file_path_.empty()) {
            std::cout << get_timestamp() << " [信息] 未指定动作配置文件，使用默认配置" << std::endl;
            return false;
        }
        
        try {
            YAML::Node config = YAML::LoadFile(action_config_file_path_);
            
            if (!config["action"]) {
                std::cout << get_timestamp() << " [警告] YAML文件中未找到'action'节点" << std::endl;
                return false;
            }
            
            auto action_node = config["action"];
            
            // 读取frame_duration_ms
            if (action_node["frame_duration_ms"]) {
                frame_duration_ms = action_node["frame_duration_ms"].as<int>();
            }
            
            // 读取左臂动作序列
            if (action_node["left_arm_actions"]) {
                left_arm_actions.clear();
                for (const auto& frame : action_node["left_arm_actions"]) {
                    std::array<double, 6> frame_data;
                    for (size_t i = 0; i < frame.size() && i < 6; ++i) {
                        frame_data[i] = frame[i].as<double>();
                    }
                    left_arm_actions.push_back(frame_data);
                }
            }
            
            // 读取头部动作序列
            if (action_node["head_actions"]) {
                head_actions.clear();
                for (const auto& frame : action_node["head_actions"]) {
                    std::array<double, 2> frame_data;
                    for (size_t i = 0; i < frame.size() && i < 2; ++i) {
                        frame_data[i] = frame[i].as<double>();
                    }
                    head_actions.push_back(frame_data);
                }
            }
            
            std::cout << get_timestamp() << " [信息] 成功从YAML文件加载动作配置" << std::endl;
            std::cout << get_timestamp() << " [信息] 左臂动作帧数: " << left_arm_actions.size() << std::endl;
            std::cout << get_timestamp() << " [信息] 头部动作帧数: " << head_actions.size() << std::endl;
            std::cout << get_timestamp() << " [信息] 每帧时长: " << frame_duration_ms << " 毫秒" << std::endl;
            
            return true;
        } catch (const YAML::Exception& e) {
            std::cout << get_timestamp() << " [错误] 读取YAML配置文件失败: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cout << get_timestamp() << " [错误] 读取配置文件异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool initActuator() {
        try {
            std::cout << get_timestamp() << " 正在创建MotorevoActuator实例..." << std::endl;
            actuator_ = new motorevo::MotorevoActuator(config_file_path_, false, 250);
            
            std::cout << get_timestamp() << " 正在初始化执行器..." << std::endl;
            if (actuator_->init() != 0) {
                std::cout << get_timestamp() << " [错误] 执行器初始化失败" << std::endl;
                return false;
            }
            
            std::cout << get_timestamp() << " 执行器初始化成功" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cout << get_timestamp() << " [错误] 初始化异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    void run() {
        // 初始化执行器
        if (!initActuator()) {
            publishArmReady(false);
            publishArmRunning(false);  // 初始化失败时也发布False
            return;
        }
        
        // 等待actuator初始化完成
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // ====================================================================
        // kuavo_v52_dual 机型配置：电机数量与ID范围
        // ====================================================================
        // 电机配置：14个电机（ID 1-14）
        //   - 手臂：12个电机（ID 1-12）
        //     * 左臂：ID 1-6（6个关节）
        //     * 右臂：ID 7-12（6个关节，通过左臂动作映射）
        //   - 头部：2个电机（ID 13-14）
        // ====================================================================
        auto motor_states = actuator_->get_motor_state();
        std::vector<uint8_t> target_indices;
        std::vector<uint8_t> target_motor_ids;
        
        for (size_t idx = 0; idx < motor_states.size(); ++idx) {
            auto [motor_id, motor_state] = motor_states[idx];
            if (motor_id >= 0x1 && motor_id <= 0xE) {  // ID 1-14 (0xE = 14) - kuavo_v52_dual机型
                target_indices.push_back(static_cast<uint8_t>(idx));
                target_motor_ids.push_back(motor_id);
            }
        }
        
        if (target_indices.empty()) {
            std::cout << get_timestamp() << " [错误] 未找到目标电机（ID 1-14，kuavo_v52_dual机型）" << std::endl;
            publishArmReady(false);
            publishArmRunning(false);  // 错误时也发布False
            return;
        }
        
        std::cout << get_timestamp() << " [kuavo_v52_dual] 找到 " << target_indices.size() 
                  << " 个目标电机（手臂1-12 + 头部13-14）" << std::endl;
        
        // 记录零点位置
        auto current_positions = actuator_->get_positions();
        std::vector<double> zero_positions;
        zero_positions.reserve(target_indices.size());
        for (uint8_t idx : target_indices) {
            if (idx < current_positions.size()) {
                zero_positions.push_back(current_positions[idx]);
            } else {
                zero_positions.push_back(0.0);
            }
        }
        
        // ====================================================================
        // kuavo_v52_dual 机型配置：动作序列定义（从YAML文件读取）
        // ====================================================================
        std::vector<std::array<double,6>> left_arm_actions;
        std::vector<std::array<double,2>> head_actions;
        int frame_duration_ms = 2000;  // 默认值
        
        // 尝试从YAML文件读取配置
        if (!loadActionConfig(left_arm_actions, head_actions, frame_duration_ms)) {
            // 如果读取失败，使用默认硬编码配置
            std::cout << get_timestamp() << " [信息] 使用默认硬编码动作配置" << std::endl;
            left_arm_actions = {
                {  0.00,  0.00,  0.00,  0.00,  0.00,  0.00 },
                {  0.40,  0.35, -0.50,  0.15, -0.20,  0.15 },
                {  0.40,  0.20,  0.00, -0.20,  0.20, -0.20 },
                {  0.35, -0.35,  0.00,  0.20, -0.15,  0.20 },
                {  0.35,  0.20, -0.50,  0.15,  0.20, -0.15 },
                {  0.40, -0.30,  0.00, -0.15,  0.15,  0.20 },
                {  0.00,  0.00,  0.00,  0.00,  0.00,  0.00 }
            };
            head_actions = {
                {  0.00,  0.00 },
                {  0.10,  0.10 },
                { -0.20,  0.20 },
                {  0.15,  0.15 },
                { -0.15,  0.20 },
                {  0.20,  0.20 },
                {  0.00,  0.00 }
            };
        }
        
        // 发布arm_ready = True
        publishArmReady(true);
        std::cout << get_timestamp() << " 手臂电机初始化完成，arm_ready = True" << std::endl;
        
        // 等待start_together信号或standalone_mode信号
        if (!standalone_mode_) {
            std::cout << get_timestamp() << " 等待 start_together 信号..." << std::endl;
            std::cout << get_timestamp() << " 同时运行模式：需要同时满足 start_together=True 和 allow_run=True" << std::endl;
            ros::Rate wait_rate(50);
            while (ros::ok() && !start_received_ && !should_exit_ && !standalone_mode_) {
                ros::spinOnce();
                // 检查是否同时满足start_together和allow_run
                if (start_together_received_.load() && allow_run_ && !start_received_) {
                    std::cout << get_timestamp() << " ✓ 同时满足 start_together=True 和 allow_run=True，开始执行动作" << std::endl;
                    start_received_ = true;
                    break;
                }
                wait_rate.sleep();
            }
            if (start_received_) {
                std::cout << get_timestamp() << " ✓ 已收到 start_together 信号，准备开始执行动作" << std::endl;
            }
        } else {
            std::cout << get_timestamp() << " 单独运行模式，无需等待 start_together 信号" << std::endl;
        }
        
        if (should_exit_ || !ros::ok()) {
            std::cout << get_timestamp() << " 收到退出信号，停止执行" << std::endl;
            publishArmRunning(false);  // 退出时发布False
            return;
        }
        
        // 执行动作序列
        const int frame_ms = frame_duration_ms;  // 从配置文件读取的每帧时长
        
        // ====================================================================
        // kuavo_v52_dual 机型配置：电机ID到动作序列的映射关系
        // ====================================================================
        // 映射规则（仅适用于 kuavo_v52_dual）：
        //   左臂（ID 1-6）：直接使用左臂动作序列的对应关节
        //     - ID 1 -> left_offsets[0] (左臂关节1)
        //     - ID 2 -> left_offsets[1] (左臂关节2)
        //     - ID 3 -> left_offsets[2] (左臂关节3)
        //     - ID 4 -> left_offsets[3] (左臂关节4)
        //     - ID 5 -> left_offsets[4] (左臂关节5)
        //     - ID 6 -> left_offsets[5] (左臂关节6)
        //   
        //   右臂（ID 7-12）：通过左臂动作序列映射
        //     - ID 7  -> -left_offsets[0] (右臂关节1 = -左臂关节1，反方向)
        //     - ID 8  -> -left_offsets[1] (右臂关节2 = -左臂关节2，反方向)
        //     - ID 9  -> left_offsets[2]  (右臂关节3 = 左臂关节3，同方向)
        //     - ID 10 -> -left_offsets[3] (右臂关节4 = -左臂关节4，反方向)
        //     - ID 11 -> -left_offsets[4] (右臂关节5 = -左臂关节5，反方向)
        //     - ID 12 -> left_offsets[5]  (右臂关节6 = 左臂关节6，同方向)
        //   
        //   头部（ID 13-14）：直接使用头部动作序列
        //     - ID 13 -> head_offsets[0] (头部关节1)
        //     - ID 14 -> head_offsets[1] (头部关节2)
        // ====================================================================
        auto send_offsets = [&](const std::array<double,6>& left_offsets, const std::array<double,2>& head_offsets = {0.0, 0.0}) {
            std::vector<uint8_t> indices;
            std::vector<double> positions_deg;
            std::vector<double> torques;
            std::vector<double> velocities;
            
            for (size_t i = 0; i < target_indices.size(); ++i) {
                uint8_t motor_id = target_motor_ids[i];
                double off = 0.0;
                
                // kuavo_v52_dual 机型映射逻辑
                if (motor_id >= 1 && motor_id <= 6) {
                    // 左臂：直接映射
                    off = left_offsets[motor_id - 1];
                } else if (motor_id == 7) {
                    // 右臂关节1 = -左臂关节1（反方向）
                    off = -left_offsets[0];
                } else if (motor_id == 8) {
                    // 右臂关节2 = -左臂关节2（反方向）
                    off = -left_offsets[1];
                } else if (motor_id == 9) {
                    // 右臂关节3 = 左臂关节3（同方向）
                    off = left_offsets[2];
                } else if (motor_id == 10) {
                    // 右臂关节4 = -左臂关节4（反方向）
                    off = -left_offsets[3];
                } else if (motor_id == 11) {
                    // 右臂关节5 = -左臂关节5（反方向）
                    off = -left_offsets[4];
                } else if (motor_id == 12) {
                    // 右臂关节6 = 左臂关节6（同方向）
                    off = left_offsets[5];
                } else if (motor_id == 13) {
                    // 头部关节1
                    off = head_offsets[0];
                } else if (motor_id == 14) {
                    // 头部关节2
                    off = head_offsets[1];
                } else {
                    continue;
                }
                
                double target_pos_rad = zero_positions[i] + off;
                double target_pos_deg = target_pos_rad * 180.0 / M_PI;
                
                indices.push_back(target_indices[i]);
                positions_deg.push_back(target_pos_deg);
                torques.push_back(0.0);
                velocities.push_back(0.0);
            }
            
            if (!indices.empty()) {
                actuator_->set_positions(indices, positions_deg, torques, velocities);
            }
        };
        
        auto do_frame_pair = [&](const std::array<double,6>& start_left,
                                  const std::array<double,6>& end_left,
                                  const std::array<double,2>& start_head,
                                  const std::array<double,2>& end_head,
                                  int duration_ms) -> bool {
            auto t0 = std::chrono::steady_clock::now();
            ros::Rate frame_rate(50);  // 50Hz运动控制频率
            int check_counter = 0;  // 用于降低电机状态检查频率（每50次检查一次，即1Hz）
            while (true) {
                // 持续检查allow_run状态（50Hz）
                ros::spinOnce();
                if (should_exit_ || !allow_run_ || !ros::ok()) {
                    return false;
                }
                
                // 每50次循环检查一次电机状态（1Hz检查频率）
                check_counter++;
                if (check_counter >= 50) {
                    check_counter = 0;
                    if (checkMotorDisabled(target_motor_ids)) {
                        std::cout << get_timestamp() << " [错误] 检测到电机失能，立即停止运动" << std::endl;
                        should_exit_ = true;
                        publishArmRunning(false);
                        return false;
                    }
                }
                
                auto now = std::chrono::steady_clock::now();
                int elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count());
                if (elapsed >= duration_ms) break;
                
                double alpha = static_cast<double>(elapsed) / duration_ms;
                std::array<double,6> cur_arm{};
                for (int i = 0; i < 6; ++i) {
                    cur_arm[i] = start_left[i] + (end_left[i] - start_left[i]) * alpha;
                }
                
                std::array<double,2> cur_head{};
                cur_head[0] = start_head[0] + (end_head[0] - start_head[0]) * alpha;
                cur_head[1] = start_head[1] + (end_head[1] - start_head[1]) * alpha;
                
                send_offsets(cur_arm, cur_head);
                
                frame_rate.sleep();
            }
            
            if (!should_exit_ && allow_run_ && ros::ok()) {
                send_offsets(end_left, end_head);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            return true;
        };
        
        // 发布arm_started，通知主程序节点已准备好
        publishArmStarted(true);
        std::cout << get_timestamp() << " 节点已准备好，arm_started = True" << std::endl;
        
        // 初始状态：发布第一帧位置，arm_running = False（等待开始第一轮）
        send_offsets(left_arm_actions[0], head_actions[0]);
        publishArmRunning(false);
        std::cout << get_timestamp() << " 已发布第一帧位置，arm_running = False（等待开始第一轮）" << std::endl;
        
        // 等待开始第一轮的信号（同时运行模式等待start_together，单独运行模式直接开始）
        if (!standalone_mode_) {
            std::cout << get_timestamp() << " 同时运行模式：等待 start_together 信号..." << std::endl;
            ros::Rate wait_rate(50);  // 50Hz检查
            int check_counter = 0;  // 用于降低电机状态检查频率（每50次检查一次，即1Hz）
                while (ros::ok() && !start_received_ && !should_exit_ && allow_run_) {
                // 持续发布第一帧位置
                send_offsets(left_arm_actions[0], head_actions[0]);
                ros::spinOnce();
                
                // 每50次循环检查一次电机状态（1Hz检查频率）
                check_counter++;
                if (check_counter >= 50) {
                    check_counter = 0;
                    if (checkMotorDisabled(target_motor_ids)) {
                        std::cout << get_timestamp() << " [错误] 检测到电机失能，立即停止运动" << std::endl;
                        should_exit_ = true;
                        publishArmRunning(false);
                        break;
                    }
                }
                
                if (start_together_received_.load() && allow_run_ && !start_received_) {
                    std::cout << get_timestamp() << " ✓ 收到 start_together 信号，准备开始第一轮动作" << std::endl;
                    start_received_ = true;
                    break;
                }
                wait_rate.sleep();
            }
        } else {
            std::cout << get_timestamp() << " 单独运行模式，准备开始第一轮动作" << std::endl;
            start_received_ = true;
        }
        
        if (should_exit_ || !ros::ok() || !allow_run_) {
            std::cout << get_timestamp() << " 收到退出信号，停止执行" << std::endl;
            publishArmRunning(false);
            return;
        }
        
        // 执行动作循环
        int completed_rounds = 0;
        bool is_first_round = true;
        ros::Rate loop_rate(50);  // 50Hz主循环
        
        while (ros::ok() && allow_run_ && !should_exit_) {
            // 等待开始新一轮的信号
            // 第一轮已经收到（start_together或standalone模式直接开始）
            // 从第二轮开始：
            //   - 无论单独运行模式还是同时运行模式，都等待start_new_round_arm信号
            if (!is_first_round) {
                // 持续发布第一帧位置，等待start_new_round_arm信号
                start_new_round_arm_.store(false);
                std::cout << get_timestamp() << " 第 " << completed_rounds
                          << " 轮完成，arm_running = False，等待 start_new_round_arm 信号..."
                          << (standalone_mode_ ? "（单独运行模式）" : "（同时运行模式）") << std::endl;
                
                int check_counter = 0;  // 用于降低电机状态检查频率（每50次检查一次，即1Hz）
                while (ros::ok() && allow_run_ && !should_exit_ && !start_new_round_arm_.load()) {
                    // 持续发布第一帧位置
                    send_offsets(left_arm_actions[0], head_actions[0]);
                    ros::spinOnce();
                    
                    // 每50次循环检查一次电机状态（1Hz检查频率）
                    check_counter++;
                    if (check_counter >= 50) {
                        check_counter = 0;
                        if (checkMotorDisabled(target_motor_ids)) {
                            std::cout << get_timestamp() << " [错误] 检测到电机失能，立即停止运动" << std::endl;
                            should_exit_ = true;
                            publishArmRunning(false);
                            break;
                        }
                    }
                    
                    loop_rate.sleep();
                }
                
                if (should_exit_ || !ros::ok() || !allow_run_) {
                    std::cout << get_timestamp() << " 收到退出信号，停止执行" << std::endl;
                    publishArmRunning(false);
                    break;
                }
                
                std::cout << get_timestamp() << " ✓ 收到 start_new_round_arm 信号，开始第 "
                          << (completed_rounds + 1) << " 轮动作" << std::endl;
                start_new_round_arm_.store(false);  // 重置信号
            }
            
            // 开始执行一轮动作
            publishArmRunning(true);
            std::cout << get_timestamp() << " 开始执行第 " << (completed_rounds + 1) << " 轮动作，arm_running = True" << std::endl;
            
            // 执行一轮动作（一旦开始，必须完成整轮）
            bool round_completed = true;
            size_t head_action_index = 0;
            for (size_t i = 0; i + 1 < left_arm_actions.size(); ++i) {
                ros::spinOnce();
                if (should_exit_ || !allow_run_ || !ros::ok()) {
                    std::cout << get_timestamp() << " 检测到停止信号，立即停止" << std::endl;
                    publishArmRunning(false);
                    round_completed = false;
                    break;
                }
                
                // 在执行动作帧前检查电机状态
                if (checkMotorDisabled(target_motor_ids)) {
                    std::cout << get_timestamp() << " [错误] 检测到电机失能，立即停止运动" << std::endl;
                    should_exit_ = true;
                    publishArmRunning(false);
                    round_completed = false;
                    break;
                }
                
                // 计算头部动作索引
                size_t head_start_idx = head_action_index % head_actions.size();
                size_t head_end_idx = (head_action_index + 1) % head_actions.size();
                
                if (!do_frame_pair(left_arm_actions[i], left_arm_actions[i + 1], 
                                   head_actions[head_start_idx], head_actions[head_end_idx], frame_ms)) {
                    std::cout << get_timestamp() << " 执行动作帧失败" << std::endl;
                    publishArmRunning(false);
                    round_completed = false;
                    break;
                }
                
                head_action_index++;
                std::cout << get_timestamp() << " 动作帧 " << (i + 1) << " 执行完成" << std::endl;
            }
            
            if (!round_completed) {
                break;
            }
            
            // 一轮动作完成
            completed_rounds++;
            publishArmRunning(false);  // 本轮完成，设置为False
            std::cout << get_timestamp() << " 第 " << completed_rounds << " 轮动作完成，arm_running = False" << std::endl;
            is_first_round = false;
        }
        
        // 检查是否因为电机失能而退出
        if (should_exit_ && actuator_) {
            // 再次检查电机状态，确认是否有失能
            if (checkMotorDisabled(target_motor_ids)) {
                std::cout << get_timestamp() << " [错误] 确认电机失能，正在失能所有电机..." << std::endl;
            } else {
                std::cout << get_timestamp() << " 动作执行完成，共完成 " << completed_rounds << " 轮" << std::endl;
            }
        } else {
            std::cout << get_timestamp() << " 动作执行完成，共完成 " << completed_rounds << " 轮" << std::endl;
        }
        
        // 失能电机
        if (actuator_) {
            std::cout << get_timestamp() << " 正在失能所有电机..." << std::endl;
            actuator_->disable();
        }
        
        // 确保发布False
        arm_ready_ = false;
        arm_running_ = false;
        publishArmReady(false);
        publishArmRunning(false);
    }
};

// 全局节点指针，用于信号处理
ArmBreakinNode* g_node_ptr = nullptr;

void signalHandler(int sig) {
    if (g_node_ptr) {
        std::cout << get_timestamp() << " [警告] 收到信号 " << sig << "，正在退出..." << std::endl;
        g_node_ptr->publishArmReady(false);
    }
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    try {
        // 创建节点（在构造函数中初始化ROS）
        ArmBreakinNode node(argc, argv);
        g_node_ptr = &node;
        
        // 设置信号处理
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        // 注册ROS shutdown回调
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();
        
        // 运行节点
        node.run();
        
        // 确保发布False
        node.publishArmReady(false);
        
        spinner.stop();
        std::cout << get_timestamp() << " 手臂磨线节点退出" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cout << get_timestamp() << " [错误] 程序异常: " << e.what() << std::endl;
        if (g_node_ptr) {
            g_node_ptr->publishArmReady(false);
        }
        return -1;
    }
}

