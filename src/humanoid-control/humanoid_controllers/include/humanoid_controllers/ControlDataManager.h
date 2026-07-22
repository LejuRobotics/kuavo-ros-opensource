#ifndef HUMANOID_CONTROLLERS_CONTROL_DATA_MANAGER_H
#define HUMANOID_CONTROLLERS_CONTROL_DATA_MANAGER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mutex>
#include <memory>
#include <functional>
#include <atomic>
#include <Eigen/Dense>

#include <kuavo_msgs/sensorsData.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include <kuavo_msgs/SetIncrementalArmTrajLink.h>
#include <kuavo_msgs/lbBaseLinkPoseCmdSrv.h>
#include <kuavo_msgs/robotHeadMotionData.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <chrono>

#include "humanoid_interface/common/Types.h"
#include "kuavo_common/common/sensor_data.h"
#include "humanoid_interface_drake/kuavo_data_buffer.h"
#include "humanoid_controllers/shm_manager.h"
#include "humanoid_controllers/shm_data_structure.h"
#include "kuavo_common/common/arm_traj_shm.h"
#include <thread>

namespace humanoidController_wheel_wbc {
using namespace ocs2;
using namespace humanoid;

// 传感器数据结构（在这里完整定义）
struct SensorData
{
    ros::Time timeStamp_;
    vector_t jointPos_;
    vector_t jointVel_;
    vector_t jointAcc_;
    vector_t jointTorque_;
    vector3_t angularVel_;
    vector3_t linearAccel_;
    Eigen::Quaternion<scalar_t> quat_;
    matrix3_t orientationCovariance_;
    matrix3_t angularVelCovariance_;
    matrix3_t linearAccelCovariance_;
    
    void resize_joint(size_t num) {
        jointPos_.resize(num);
        jointVel_.resize(num);
        jointAcc_.resize(num);
        jointTorque_.resize(num);
    }
};

// 手臂关节轨迹数据结构
struct ArmJointTrajectory {
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd tau;
    
    void init(int arm_num) {
        pos = Eigen::VectorXd::Zero(arm_num);
        vel = Eigen::VectorXd::Zero(arm_num);
        tau = Eigen::VectorXd::Zero(arm_num);
    }
};

// 带时间戳的数据容器
template<typename T>
struct TimestampedData {
    T data;
    ros::Time timestamp;
    bool valid = false;
    
    void update(const T& new_data) {
        data = new_data;
        timestamp = ros::Time::now();
        valid = true;
    }
    
    bool isValid(double timeout_sec = 1.0) const {
        if (!valid) return false;
        return (ros::Time::now() - timestamp).toSec() < timeout_sec;
    }
};

class ControlDataManager {
public:
    explicit ControlDataManager(ros::NodeHandle& nh, bool is_real, int arm_num, int low_joint_num, int head_num, 
                                const vector_t& leg_initial_state, const vector_t& arm_initial_state);
    ~ControlDataManager();

    // 初始化所有订阅者
    void initializeSubscribers();
    
    // ========== 数据获取接口（线程安全，使用引用输出优化性能） ==========
    
    // ========== 共享内存相关 ==========
    bool updateSensorDataFromShm(SensorData& out) const;  // 从共享内存更新传感器数据
    void publishJointCmdToShm(const kuavo_msgs::jointCmd& jointCmdMsg) const;  // 发布关节命令到共享内存

    // ========== 需要时间戳校验的接口 ==========
    // 传感器数据
    bool getRealtimeSensorData(SensorData& out) const;
    
    // 里程计数据 [x, y, yaw, vx, vy, vyaw]
    bool getRealtimeOdomData(vector6_t& out) const;
    void setOdomReset(void) {is_odom_reset_ = true; }  // 设置里程计重置标志位
    
    // 基座位姿 [x, y, z, qw, qx, qy, qz]
    bool getRealtimeBaseLinkPose(vector_t& out) const;
    
    // 速度命令（带超时检查）
    bool getRealtimeCmdVel(geometry_msgs::Twist& out) const;
    
    // VR相关数据（带时间戳校验）
    bool getRealtimeWaistYawLinkPose(vector_t& out) const;
    bool getRealtimeVrTorsoPose(vector_t& out) const;
    
    // ========== 直接返回最新数据的接口 ==========
    // 轮臂关节外部控制状态 [4个关节]
    vector_t getLbWaistExternalControlState() const;
    void setLbWaistExternalControlState(const vector_t& joint_state);  // 设置轮臂关节初始值
    
    // 头部外部控制状态 [yaw, pitch]（rad）
    // 非 const：读前先 applyHeadVelDeltaCommands 做开环积分
    vector_t getHeadExternalControlState();

    // 带 disable 保护的关节轨迹读取 helper
    ArmJointTrajectory getJointTrajectoryWithDisableGuard(const TimestampedData<ArmJointTrajectory>& storage) const;

    // 计算头部控制扭矩（内部获取传感器数据）
    vector_t computeHeadControl(const vector_t& target_pos) const;

    // VR控制模式
    bool getWholeTorsoCtrl() const;
    void resetVrTorsoPose();  // 重置VR躯干姿态为单位姿态
    
    // 手臂轨迹
    ArmJointTrajectory getArmExternalControlState() const;

    // 下肢轨迹
    ArmJointTrajectory getLegExternalControlState() const;

    // 轮臂MPC控制模式
    int8_t getLbMpcControlMode() const;
    
    // 从当前状态更新手臂外部控制状态
    void updateArmExternalControlState(const Eigen::VectorXd& current_pos,
                                     const Eigen::VectorXd& current_vel,
                                     const Eigen::VectorXd& current_tau);
    
    // 从当前状态更新下肢外部控制状态
    void updateLegExternalControlState(const Eigen::VectorXd& current_pos,
                                     const Eigen::VectorXd& current_vel,
                                     const Eigen::VectorXd& current_tau);
    
    // ========== 批量获取接口（一次加锁，性能最优） ==========
    struct ControlData {
        SensorData sensors;
        vector6_t odom;
        vector_t base_link_pose;
        geometry_msgs::Twist cmd_vel;
        bool valid;
    };
    
    ControlData getAllControlData() const;
    
    // ========== 状态查询接口 ==========
    /**
     * @brief 检查数据是否就绪
     * @param check_motion_data 是否检查运动相关数据（odom和base_link_pose），默认为true
     * @return 如果所有需要的数据都就绪，返回true
     */
    bool isDataReady(bool check_motion_data = true) const;

    bool setIncrementalArmTrajLink(int8_t transport, std::string* message = nullptr);
    bool handleSetIncrementalArmTrajLink(typename kuavo_msgs::SetIncrementalArmTrajLink::Request& req,
                                         typename kuavo_msgs::SetIncrementalArmTrajLink::Response& res);
    
    // ========== 服务注册接口 ==========
    
    /**
     * @brief 注册单个ROS服务（模板方法，支持任意服务类型）
     * @tparam ServiceT ROS服务类型（如 kuavo_msgs::changeArmCtrlMode）
     * @param service_name 服务名称
     * @param callback 服务回调函数
     * @example 
     *   control_data_manager_->registerService<kuavo_msgs::changeArmCtrlMode>(
     *       "/enable_wbc_arm_trajectory_control",
     *       [this](auto& req, auto& res) { return handleArmTraj(req, res); }
     *   );
     */
    template<typename ServiceT>
    void registerService(
        const std::string& service_name,
        std::function<bool(typename ServiceT::Request&, typename ServiceT::Response&)> callback
    ) {
        auto server = nh_.advertiseService<typename ServiceT::Request, typename ServiceT::Response>(
            service_name,
            [callback](typename ServiceT::Request& req, typename ServiceT::Response& res) -> bool {
                return callback(req, res);
            }
        );
        registered_services_.push_back(server);
        ROS_INFO("[ControlDataManager] Registered service: %s", service_name.c_str());
    }

private:
    ros::NodeHandle& nh_;
    bool is_real_;
    bool whole_torso_ctrl_{false};  // VR全身控制模式默认关闭
    int8_t lb_mpc_control_mode_{2}; // 轮臂MPC控制模式，默认2（baseonly模式）
    bool use_shm_communication_{false};  // 是否使用共享内存通信
    bool incremental_arm_traj_link_capable_{true};
    std::atomic<int8_t> incremental_arm_traj_transport_{
        kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_NONE};
    int arm_num_{-1};
    int low_joint_num_{-1};
    int head_num_{-1};

    // enable 控制（订阅 /enable_control_state，同进程同步触发 callback）
    std::atomic<bool> prev_enable_control_{true};
    ros::Subscriber enable_control_state_sub_;
    void enableControlStateCallback(const std_msgs::Bool::ConstPtr& msg);

    // 头部控制参数
    vector_t head_kp_;  // 头部 PD 控制 Kp 增益
    vector_t head_kd_;  // 头部 PD 控制 Kd 增益

    bool is_odom_reset_{false};  // 里程计重置标志位
    Eigen::Vector3d resetOrigin_{0.0, 0.0, 0.0};
    Eigen::Matrix2d R_resetOrigin_;  // 重置机器人世界系方向的旋转矩阵（2D）

    // 共享内存管理器
    std::unique_ptr<gazebo_shm::ShmManager> shm_manager_;
    std::unique_ptr<kuavo_common::ArmTrajShmManager> arm_traj_shm_;
    std::thread arm_traj_shm_reader_thread_;
    std::atomic<bool> arm_traj_shm_reader_running_{false};
    std::mutex arm_traj_sub_mutex_;
    std::chrono::steady_clock::time_point last_arm_traj_shm_receive_wall_{};
    bool has_last_arm_traj_shm_receive_wall_{false};
    double arm_traj_shm_stale_timeout_sec_{0.3};
    void armTrajShmReaderThreadFunction();
    void ensureArmTrajRosSubscription(bool enable);
    void deactivateIncrementalArmTrajLink(const char* reason);

    enum class ArmTrajSource { kRosTopic, kIncrementalShm };
    bool shouldAcceptArmTrajFrom(ArmTrajSource source) const;
    void ingestArmJointTrajectory(ArmTrajSource source,
                                  const double* pos_rad,
                                  const double* vel_rad,
                                  const double* tau,
                                  int count,
                                  uint64_t stamp_nsec = 0);
    void recordArmTrajReceiveTiming(uint64_t stamp_nsec);
    void publishArmTrajReceiveStatus();

     // 头部关节限位 [yaw, pitch]（角度）
     static constexpr std::array<std::pair<double, double>, 2> HEAD_JOINT_LIMITS = {
         std::pair<double, double>{-88.0, 88.0},  // yaw: ±80度
         std::pair<double, double>{-25.0, 25.0}   // pitch: ±25度
     };

    // 头部速度 / 相对位移接口（与躯干 /cmd_torso_vel、/cmd_torso_delta 对齐）
    // 开环绝对位姿只在 head_external_control_state_（rad），本类为唯一积分终点。
    // vel: joint_data=[yaw_vel, pitch_vel] deg/s, sticky + 0.3s timeout zero;
    // delta: joint_data=[d_yaw, d_pitch] deg，oneshot 用后即清。
    static constexpr double kHeadVelTimeout_{0.3};  // vel 超时清零秒数
    vector_t cmd_head_vel_;          // 2D: [yaw, pitch] rad/s
    vector_t cmd_head_delta_;        // 2D oneshot: [d_yaw, d_pitch] rad
    mutable std::mutex cmd_head_vel_mtx_;
    mutable std::mutex cmd_head_delta_mtx_;
    bool is_cmd_head_vel_updated_{false};      // sticky，对齐 isCmdTorsoVelUpdated_
    bool is_cmd_head_vel_time_update_{false}; // 刷新 last_cmd_head_vel_time_
    bool is_cmd_head_delta_updated_{false};   // oneshot，用后即清
    double last_cmd_head_vel_time_{0.0};      // 时间轴（ros::Time::now().toSec()）
    double last_head_vel_integrate_time_{0.0};
    bool has_head_vel_integrate_time_{false};
    void applyHeadVelDeltaCommands();        // 在 update 取数前积分
    void headVelCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg);
    void headDeltaCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg);
    static vector_t clampHead2D(const vector_t& pose2d);

    // ========== ROS订阅者 ==========
    ros::Subscriber sensors_data_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber lb_waist_external_control_sub_;
    ros::Subscriber head_external_control_sub_;
    ros::Subscriber head_vel_sub_;
    ros::Subscriber head_delta_sub_;
    ros::Subscriber waist_yaw_link_pose_sub_;
    ros::Subscriber torso_pose_sub_;
    ros::Subscriber whole_torso_ctrl_sub_;
    ros::Subscriber arm_joint_traj_sub_;
    ros::Subscriber leg_joint_traj_sub_;
    ros::Subscriber lb_mpc_control_mode_sub_;

    // 手臂/下肢指令独立回调队列，避免与默认队列里高频 debug/录包回调互相饿死
    ros::CallbackQueue cmd_traj_callback_queue_;
    ros::NodeHandle cmd_traj_nh_;
    std::unique_ptr<ros::AsyncSpinner> cmd_traj_spinner_;
    
    // ========== 已注册的ROS服务列表 ==========
    std::vector<ros::ServiceServer> registered_services_;
    
    // ========== 数据缓存（线程安全） ==========
    // 1. 传感器数据（最高优先级）
    mutable std::mutex sensor_mutex_;
    TimestampedData<SensorData> sensor_data_;  // 关节位置、速度、IMU等关键数据

    // 2. 位置和速度命令（高优先级）
    mutable std::mutex motion_mutex_;
    TimestampedData<vector6_t> odom_data_;        // 里程计
    TimestampedData<vector_t> base_link_pose_;    // 基座位姿
    TimestampedData<geometry_msgs::Twist> cmd_vel_;  // 速度命令

    // 3. 外部控制状态（低优先级）
    mutable std::mutex external_state_mutex_;
    TimestampedData<vector_t> lb_waist_external_control_state_;
    TimestampedData<vector_t> head_external_control_state_;
    TimestampedData<vector_t> waist_yaw_link_pose_;
    TimestampedData<vector_t> vr_torso_pose_;
    TimestampedData<ArmJointTrajectory> arm_external_control_state_;
    TimestampedData<ArmJointTrajectory> leg_external_control_state_;
    
    // ========== 回调函数（私有，仅用于数据接收） ==========
    void sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void lbWaistExternalControlCallback(const kuavo_msgs::jointCmd::ConstPtr& msg);
    void headExternalControlCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg);
    void waistYawLinkPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void vrTorsoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wholeTorsoCtrlCallback(const std_msgs::Bool::ConstPtr& msg);
    void armJointTrajCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void legJointTrajCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void lbMpcControlModeCallback(const std_msgs::Int8::ConstPtr& msg);

    // /kuavo_arm_traj 订阅端到达诊断（PlotJuggler: /ik_debug/arm_traj_receive/*）
    bool enable_arm_traj_receive_timing_log_{false};
    bool arm_traj_receive_timing_initialized_{false};
    ros::Publisher arm_traj_receive_latency_ms_pub_;
    ros::Publisher arm_traj_receive_period_ms_pub_;
    ros::Publisher arm_traj_receive_stamp_period_ms_pub_;
    ros::Publisher arm_traj_using_shm_pub_;
    ros::Publisher arm_traj_transport_pub_;
    bool arm_traj_receive_status_initialized_{false};
    std::chrono::steady_clock::time_point last_arm_traj_receive_wall_{};
    ros::Time last_arm_traj_header_stamp_;
    bool has_last_arm_traj_receive_wall_{false};
    bool has_last_arm_traj_header_stamp_{false};
    void ensureArmTrajReceiveTimingPublishers();
    void publishArmTrajReceiveTimingMs(const ros::Publisher& publisher, double ms) const;
    
    // ========== 辅助函数 ==========
    double rosQuaternionToYaw(const geometry_msgs::Quaternion& ros_quat) const;
    static vector_t makeIdentityPose7D();  // 7维单位位姿 [0,0,0,1,0,0,0]
};

} // namespace humanoidController_wheel_wbc

#endif // HUMANOID_CONTROLLERS_CONTROL_DATA_MANAGER_H

