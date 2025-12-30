#ifndef LEJUCLAW_CPP_IMPL_H
#define LEJUCLAW_CPP_IMPL_H
#define POSITION_TOLERANCE 1
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <iostream>
#include <iterator>
#include <time.h>
#include <chrono>
#include "ruiwoSDK.h"

class LeJuClaw
{
//#define ENABLE_JOINT_DEBUG  // 取消注释此行以启用关节调试打印
 public:
    
     enum class PawMoveState  : int8_t {
        ERROR = -1,                       // 出现错误
        LEFT_REACHED_RIGHT_REACHED = 0,   // 所有夹爪到位
        LEFT_REACHED_RIGHT_GRABBED = 1,   // 左夹爪到位，右夹爪抓取到物品
        LEFT_GRABBED_RIGHT_REACHED = 2,   // 右夹爪到位，左夹爪抓取到物品
        LEFT_GRABBED_RIGHT_GRABBED = 3,   // 所有夹爪均夹取
        
    };

    enum class State {
        None,
        Enabled,
        Disabled
    };

    struct MotorStateData {
        uint8_t id;
        State   state;
        MotorStateData():id(0x0), state(State::None) {}
        MotorStateData(uint8_t id_, State state_):id(id_), state(state_) {}
    };    
    using MotorStateDataVec = std::vector<MotorStateData>;
public:
    LeJuClaw(std::string unused = "");
    ~LeJuClaw();
    int initialize(bool init_bmlib);
    void enable();
    void disable();
    void go_to_zero();
    void set_zero();
    void set_positions(const std::vector<uint8_t> &index, const std::vector<double> &positions, const std::vector<double> &torque, const std::vector<double> &velocity);
    void set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque);
    void set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity);
    void set_joint_state(int index, const std::vector<float> &state);
    PawMoveState move_paw(const std::vector<double> &positions, const std::vector<double> &velocity,const std::vector<double> &torque);
    std::vector<std::vector<float>> get_joint_state();
    std::vector<double> get_positions();
    std::vector<double> get_torque();
    std::vector<double> get_velocity();
    void close();
    MotorStateDataVec get_motor_state();
    std::vector<int> disable_joint_ids;
    
private:
    void control_thread();
    // void get_parameter();
    void get_config(const std::string &config_file);
    std::string get_home_path();
    void interpolate_move(const std::vector<float> &start_positions, const std::vector<float> &target_positions, const std::vector<float> &speeds, float dt);
    std::vector<std::vector<float>> interpolate_positions_with_speed(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &speeds, float dt);
    void send_positions(const std::vector<int> &index, const std::vector<float> &pos, const std::vector<float> &torque, const std::vector<float> &velocity);
    std::vector<int> get_joint_addresses(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<bool> get_joint_online_status(const YAML::Node &config, const std::string &joint_type, int count);
    std::vector<std::vector<int>> get_joint_parameters(const YAML::Node &config, const std::string &joint_type, int count);
    // 关节ID
    std::vector<int> Claw_joint_address = {0x0F, 0x10};
    
    static inline const std::vector<std::vector<int>> Claw_parameter = {{0, 10, 2, 0, 0, 0, 0},
                                                                        {0, 10, 2, 0, 0, 0, 0}};

                                                                           
    // 反转电机ID 可能会用上?
    static inline  std::vector<int> Negtive_joint_address_list = {};
    
    // // ptm:力控模式 servo:伺服模式
    static inline  std::string Control_mode = "ptm";

    
    std::vector<bool> Claw_joint_online;

    std::vector<std::vector<int>> Joint_parameter_list;
    std::vector<int> Joint_address_list;
    std::vector<bool> Joint_online_list;

    // RUIWOTools ruiwo;
    bool thread_running;
    bool thread_end;
    std::thread control_thread_;
    std::mutex sendpos_lock;
    std::mutex recvpos_lock;
    std::mutex sendvel_lock;
    std::mutex recvvel_lock;
    std::mutex sendtor_lock;
    std::mutex recvtor_lock;
    std::mutex state_lock;
    std::mutex update_lock;
    std::mutex can_lock;

    bool target_update;
    std::vector<float> target_positions;
    std::vector<float> target_velocity;
    std::vector<float> target_torque;
    std::vector<int> target_pos_kp;
    std::vector<int> target_pos_kd;
    std::vector<int> target_vel_kp;
    std::vector<int> target_vel_kd;
    std::vector<int> target_vel_ki;

    std::vector<float> old_target_positions;
    std::vector<float> current_positions;
    std::vector<float> current_torque;
    std::vector<float> current_velocity;
    std::vector<std::vector<float>> joint_status;
    std::vector<float> joint_start_positions;  // 行程起点位置
    std::vector<float> joint_end_positions;    // 行程终点位置

};

#endif // LEJUCLAW_CPP_H