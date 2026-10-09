#include "humanoid_controllers/ControlDataManager.h"
#include <kuavo_msgs/SetIncrementalArmTrajLink.h>
#include <iostream>
#include <vector>
#include <ocs2_core/misc/LoadData.h>
#include <angles/angles.h>

namespace humanoidController_wheel_wbc {

using namespace ocs2;

ControlDataManager::ControlDataManager(ros::NodeHandle& nh, bool is_real, int arm_num, int low_joint_num, int head_num, 
                                       const vector_t& leg_initial_state, const vector_t& arm_initial_state)
    : nh_(nh),
      is_real_(is_real),
      arm_num_(arm_num),
      low_joint_num_(low_joint_num),
      head_num_(head_num),
      cmd_traj_nh_(nh)
{
    // 初始化轮臂关节状态（4个关节）
    vector_t initial_lb_joint = vector_t::Zero(low_joint_num);
    lb_waist_external_control_state_.data = initial_lb_joint;
    lb_waist_external_control_state_.valid = true;  // 使用零位作为默认值

    // 初始化头部状态（2个关节：yaw, pitch）
    vector_t initial_head = vector_t::Zero(head_num_);
    head_external_control_state_.data = initial_head;
    head_external_control_state_.valid = true;  // 使用零位作为默认值

    // 头部 vel/delta 缓冲（rad/s, rad）
    cmd_head_vel_ = vector_t::Zero(2);
    cmd_head_delta_ = vector_t::Zero(2);

    // 初始化头部PD控制增益（从referenceFile加载）
    head_kp_.resize(head_num_);
    head_kd_.resize(head_num_);

    if (head_num_ > 0) {
        std::string referenceFile;
        nh_.getParam("/referenceFile", referenceFile);
        loadData::loadEigenMatrix(referenceFile, "head_kp_", head_kp_);
        loadData::loadEigenMatrix(referenceFile, "head_kd_", head_kd_);
    }

    // 初始化VR躯干位姿（7维：位置xyz + 四元数wxyz）
    vr_torso_pose_.data = makeIdentityPose7D();
    vr_torso_pose_.valid = true;  // 使用零位作为默认值

    // 初始化odom位置
    odom_data_.data = vector6_t::Zero();
    std::cout << "odom init: " << odom_data_.data << std::endl;
    
    // 初始化手臂轨迹
    arm_external_control_state_.data.init(arm_num);
    ArmJointTrajectory arm_traj;
    arm_traj.init(arm_num_);
    arm_traj.pos = arm_initial_state;  
    arm_external_control_state_.update(arm_traj);

    // 初始化下肢轨迹
    leg_external_control_state_.data.init(low_joint_num);
    ArmJointTrajectory leg_traj;
    leg_traj.init(low_joint_num_);
    leg_traj.pos = leg_initial_state;  
    leg_external_control_state_.update(leg_traj);

    // 初始化重置旋转矩阵
    R_resetOrigin_ = Eigen::Matrix2d::Identity();

    // 初始化通信方式
    use_shm_communication_ = false;
    if (nh_.hasParam("/use_shm_communication"))
    {
        nh_.getParam("/use_shm_communication", use_shm_communication_);
    }

    if (use_shm_communication_) 
    {
        shm_manager_ = std::make_unique<gazebo_shm::ShmManager>();
        use_shm_communication_ = shm_manager_->initializeSensorsShm() && shm_manager_->initializeCommandShm();
    }

    ROS_INFO_STREAM("[ControlDataManager] Using " << 
        (use_shm_communication_ ? "shared memory" : "ROS topics") << " for communication");
    
    ROS_INFO("ControlDataManager initialized: is_real=%d, arm_num=%d, low_joint_num=%d",
             is_real_, arm_num_, low_joint_num_);
}

ControlDataManager::~ControlDataManager() {
    arm_traj_receiver_.reset();
    if (cmd_traj_spinner_) {
        cmd_traj_spinner_->stop();
        cmd_traj_spinner_.reset();
    }
    leg_joint_traj_sub_.shutdown();
}

void ControlDataManager::initializeSubscribers() {
    sensors_data_sub_ = nh_.subscribe<kuavo_msgs::sensorsData>(
        "/sensors_data_raw", 10, 
        &ControlDataManager::sensorsDataCallback, this);
    
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/odom", 10, 
        &ControlDataManager::odomCallback, this);
    
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, 
        &ControlDataManager::cmdVelCallback, this);
    
    lb_waist_external_control_sub_ = nh_.subscribe<kuavo_msgs::jointCmd>(
        "/lb_joint_cmd", 10, 
        &ControlDataManager::lbWaistExternalControlCallback, this);
    
    head_external_control_sub_ = nh_.subscribe(
        "/robot_head_motion_data", 10,
        &ControlDataManager::headExternalControlCallback, this);

    // 头部速度 / 相对位移（开环积分在本类；绝对话题仍保留给 SDK/VR）
    head_vel_sub_ = nh_.subscribe(
        "/cmd_head_vel", 1,
        &ControlDataManager::headVelCallback, this);
    head_delta_sub_ = nh_.subscribe(
        "/cmd_head_delta", 1,
        &ControlDataManager::headDeltaCallback, this);

    waist_yaw_link_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/waist_yaw_link_pose", 10, 
        &ControlDataManager::waistYawLinkPoseCallback, this);
    
    torso_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/cmd_torso_pose_vr", 10, 
        &ControlDataManager::vrTorsoPoseCallback, this);
    
    whole_torso_ctrl_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/vr_whole_torso_ctrl", 10, 
        &ControlDataManager::wholeTorsoCtrlCallback, this);
    
    // 下肢指令：独立 CallbackQueue；手臂 traj 由 ArmTrajReceiver 接管
    cmd_traj_nh_.setCallbackQueue(&cmd_traj_callback_queue_);
    arm_traj_receiver_ = std::make_unique<humanoid_controller::ArmTrajReceiver>();
    arm_traj_receiver_->init(
        nh_, arm_num_,
        [this](const double* pos, const double* vel, const double* tau, int n,
               uint64_t stamp_nsec) {
          ingestArmJointTrajectory(pos, vel, tau, n, stamp_nsec);
        },
        "/humanoid_wheel/set_incremental_arm_traj_link",
        kuavo_msgs::SetIncrementalArmTrajLink::Request::TRANSPORT_NONE);

    leg_joint_traj_sub_ = cmd_traj_nh_.subscribe<sensor_msgs::JointState>(
        "/lb_leg_traj", 1,
        &ControlDataManager::legJointTrajCallback, this,
        ros::TransportHints().tcpNoDelay());

    cmd_traj_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &cmd_traj_callback_queue_);
    cmd_traj_spinner_->start();
    
    lb_mpc_control_mode_sub_ = nh_.subscribe<std_msgs::Int8>(
        "/mobile_manipulator/lb_mpc_control_mode", 10,
        &ControlDataManager::lbMpcControlModeCallback, this);

    enable_control_state_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/enable_control_state", 1,
        &ControlDataManager::enableControlStateCallback, this);

    ROS_INFO("ControlDataManager: All subscribers initialized "
             "(arm traj via ArmTrajReceiver, leg traj on dedicated CallbackQueue)");
    ensureArmTrajReceiveTimingPublishers();
}

// ========== 回调函数实现 ==========

void ControlDataManager::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg) {
    SensorData sensor_data;
    auto& joint_data = msg->joint_data;
    auto& imu_data = msg->imu_data;
    int num_joint = low_joint_num_ + arm_num_ + head_num_;  // 下肢 + 手臂 + 头部
    
    sensor_data.resize_joint(num_joint);
    
    // 关节数据
    for(int i = 0; i < num_joint; i++) {
        sensor_data.jointPos_(i) = joint_data.joint_q[i];
        sensor_data.jointVel_(i) = joint_data.joint_v[i];
        sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
        sensor_data.jointTorque_(i) = joint_data.joint_torque[i];
    }
    
    // IMU数据
    sensor_data.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data.orientationCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    
    // 时间戳
    sensor_data.timeStamp_ = msg->sensor_time;
    
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_data_.update(sensor_data);
    }
}

void ControlDataManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 在锁外处理数据
    vector6_t odom;
    vector_t base_pose = vector_t::Zero(7);
    
    if(is_odom_reset_)
    {
        resetOrigin_[0] = msg->pose.pose.position.x;
        resetOrigin_[1] = msg->pose.pose.position.y;
        resetOrigin_[2] = rosQuaternionToYaw(msg->pose.pose.orientation);
        R_resetOrigin_ << std::cos(resetOrigin_[2]), -std::sin(resetOrigin_[2]),
                          std::sin(resetOrigin_[2]),  std::cos(resetOrigin_[2]);
        is_odom_reset_ = false;
    }
    
    Eigen::Vector2d pos_reset = Eigen::Vector2d(msg->pose.pose.position.x - resetOrigin_[0], 
                                                msg->pose.pose.position.y - resetOrigin_[1]).transpose() 
                                * R_resetOrigin_;

    // 处理里程计数据 [x, y, yaw, vx, vy, vyaw]
    odom[0] = pos_reset[0];
    odom[1] = pos_reset[1];
    odom[2] = angles::shortest_angular_distance(resetOrigin_[2], rosQuaternionToYaw(msg->pose.pose.orientation));
    odom[3] = msg->twist.twist.linear.x;
    odom[4] = msg->twist.twist.linear.y;
    odom[5] = msg->twist.twist.angular.z;
    
    // 处理基座位姿 [x, y, z, qw, qx, qy, qz]
    auto z_offset = is_real_ ? 0.185 : 0.0;
    base_pose[0] = msg->pose.pose.position.x;
    base_pose[1] = msg->pose.pose.position.y;
    base_pose[2] = msg->pose.pose.position.z + z_offset;
    base_pose[3] = msg->pose.pose.orientation.w;
    base_pose[4] = msg->pose.pose.orientation.x;
    base_pose[5] = msg->pose.pose.orientation.y;
    base_pose[6] = msg->pose.pose.orientation.z;
    
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        odom_data_.update(odom);
        base_link_pose_.update(base_pose);
    }
}

void ControlDataManager::enableControlStateCallback(const std_msgs::Bool::ConstPtr& msg) {
    bool requested = msg->data;
    bool was_enabled = prev_enable_control_.load(std::memory_order_acquire);

    // true→false：进入 disable。
    // Pose storage is written back from WBC planned values; ControlDataManager no longer overwrites with sensor snapshots.
    // 速度/偏移类命令置零，避免底盘/VR 躯干/头部继续运动。
    if (was_enabled && !requested) {
        {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            cmd_vel_.update(geometry_msgs::Twist());
        }
        {
            std::lock_guard<std::mutex> lock(external_state_mutex_);
            vr_torso_pose_.update(makeIdentityPose7D());
        }
        // Head vel/delta: clear sticky velocity and oneshot on freeze to avoid residual integration after unpause
        {
            std::lock_guard<std::mutex> lock(cmd_head_vel_mtx_);
            cmd_head_vel_.setZero();
            is_cmd_head_vel_updated_ = false;
            is_cmd_head_vel_time_update_ = false;
            last_cmd_head_vel_time_ = 0.0;
            has_head_vel_integrate_time_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(cmd_head_delta_mtx_);
            cmd_head_delta_.setZero();
            is_cmd_head_delta_updated_ = false;
        }
    }

    prev_enable_control_.store(requested, std::memory_order_release);
}

void ControlDataManager::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // disable 期间丢弃新的速度命令
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;

    std::lock_guard<std::mutex> lock(motion_mutex_);
    cmd_vel_.update(*msg);
}

void ControlDataManager::lbWaistExternalControlCallback(const kuavo_msgs::jointCmd::ConstPtr& msg) {
    // disable 期间丢弃新的轮臂指令
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;

    if (msg->joint_q.size() < 4 || msg->tau.size() < 4) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Insufficient lb_joint_cmd data, need at least 4 joints");
        return;
    }
    
    // 在锁外处理数据
    vector_t lb_joint = vector_t::Zero(4);
    for (size_t i = 0; i < 4; ++i) {
        lb_joint[i] = msg->joint_q[i];
    }
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        lb_waist_external_control_state_.update(lb_joint);
    }
}

void ControlDataManager::headExternalControlCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg) {
    // disable 期间丢弃新的头部运动指令
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;

    if (msg->joint_data.size() != 2) {
        ROS_WARN_THROTTLE(1.0, "Invalid head motion data size: %lu (expected 2)", msg->joint_data.size());
        return;
    }

    // 在锁外处理数据
    // 限位约束（角度单位为度）
    double yaw = std::clamp(msg->joint_data[0], HEAD_JOINT_LIMITS[0].first, HEAD_JOINT_LIMITS[0].second);
    double pitch = std::clamp(msg->joint_data[1], HEAD_JOINT_LIMITS[1].first, HEAD_JOINT_LIMITS[1].second);

    vector_t head_cmd = vector_t::Zero(2);
    head_cmd[0] = yaw * M_PI / 180.0;  // yaw, 转换为弧度
    head_cmd[1] = pitch * M_PI / 180.0;  // pitch, 转换为弧度

    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        head_external_control_state_.update(head_cmd);
    }
}

void ControlDataManager::headVelCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg) {
    // Soft-pause discards. Hold mirrors torso /cmd_torso_vel: sticky + 0.3s timeout zero.
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;
    if (msg->joint_data.size() != 2) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Invalid /cmd_head_vel size: %lu (expected 2)",
                          msg->joint_data.size());
        return;
    }

    std::lock_guard<std::mutex> lock(cmd_head_vel_mtx_);
    // joint_data = [yaw, pitch] deg/s → rad/s
    cmd_head_vel_[0] = msg->joint_data[0] * M_PI / 180.0;
    cmd_head_vel_[1] = msg->joint_data[1] * M_PI / 180.0;
    is_cmd_head_vel_updated_ = true;
    is_cmd_head_vel_time_update_ = true;
}

void ControlDataManager::headDeltaCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr& msg) {
    // Soft-pause discards. Oneshot relative displacement.
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;
    if (msg->joint_data.size() != 2) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Invalid /cmd_head_delta size: %lu (expected 2)",
                          msg->joint_data.size());
        return;
    }

    std::lock_guard<std::mutex> lock(cmd_head_delta_mtx_);
    // joint_data = [d_yaw, d_pitch] deg → rad
    cmd_head_delta_[0] = msg->joint_data[0] * M_PI / 180.0;
    cmd_head_delta_[1] = msg->joint_data[1] * M_PI / 180.0;
    is_cmd_head_delta_updated_ = true;
}

vector_t ControlDataManager::clampHead2D(const vector_t& pose2d) {
    vector_t out = pose2d;
    if (out.size() < 2) {
        out = vector_t::Zero(2);
    }
    // HEAD_JOINT_LIMITS 是度，内部状态是 rad
    const double yaw_min = HEAD_JOINT_LIMITS[0].first * M_PI / 180.0;
    const double yaw_max = HEAD_JOINT_LIMITS[0].second * M_PI / 180.0;
    const double pitch_min = HEAD_JOINT_LIMITS[1].first * M_PI / 180.0;
    const double pitch_max = HEAD_JOINT_LIMITS[1].second * M_PI / 180.0;
    out[0] = std::clamp(out[0], yaw_min, yaw_max);
    out[1] = std::clamp(out[1], pitch_min, pitch_max);
    return out;
}

void ControlDataManager::applyHeadVelDeltaCommands() {
    // Velocity hold aligned with torso applyTorsoVelDeltaCommands:
    //   - is_cmd_head_vel_updated_ sticky (not cleared while non-zero)
    //   - is_cmd_head_vel_time_update_ stamps last_cmd_head_vel_time_
    //   - >0.3s without new msg → cmd_head_vel_ = 0
    //   - explicit/timeout zero → clear sticky flag (stop integrating)
    // Oneshot delta: consume once.
    // Time axis: ros::Time::now() (ControlDataManager has no ReferenceManager loop; lazy-integrate on getHead)
    const double now = ros::Time::now().toSec();

    bool integrateVel = false;
    bool hasDelta = false;
    vector_t vel2 = vector_t::Zero(2);
    vector_t delta2 = vector_t::Zero(2);

    {
        std::lock_guard<std::mutex> lock(cmd_head_vel_mtx_);
        if (is_cmd_head_vel_time_update_) {
            last_cmd_head_vel_time_ = now;
            is_cmd_head_vel_time_update_ = false;
        }
        if ((now - last_cmd_head_vel_time_) > kHeadVelTimeout_) {
            cmd_head_vel_.setZero();
            is_cmd_head_vel_updated_ = false;
        }

        if (is_cmd_head_vel_updated_) {
            vel2 = cmd_head_vel_;
            if (vel2.squaredNorm() < 1e-18) {
                // zero (publisher zero or timeout): exit velocity hold
                is_cmd_head_vel_updated_ = false;
                cmd_head_vel_.setZero();
            } else {
                integrateVel = true;
                // sticky: keep is_cmd_head_vel_updated_
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(cmd_head_delta_mtx_);
        if (is_cmd_head_delta_updated_) {
            delta2 = cmd_head_delta_;
            is_cmd_head_delta_updated_ = false;
            hasDelta = true;
        }
    }

    if (!integrateVel && !hasDelta) {
        // Next non-zero burst starts a fresh Δt base (avoid huge first step after idle).
        {
            std::lock_guard<std::mutex> lock(cmd_head_vel_mtx_);
            has_head_vel_integrate_time_ = false;
        }
        return;
    }

    // Scheme A: integrate with real control period. Nominal ~WBC 100Hz → 0.01s; clamp outliers.
    constexpr double dt_nom = 0.01;
    double dt = dt_nom;
    {
        std::lock_guard<std::mutex> lock(cmd_head_vel_mtx_);
        if (has_head_vel_integrate_time_) {
            dt = now - last_head_vel_integrate_time_;
        }
        constexpr double dt_min = 0.5 * dt_nom;
        constexpr double dt_max = 3.0 * dt_nom;
        if (dt < dt_min) {
            dt = dt_min;
        } else if (dt > dt_max) {
            dt = dt_max;
        }
        last_head_vel_integrate_time_ = now;
        has_head_vel_integrate_time_ = true;
    }

    // Read current open-loop absolute target (same store absolute teleop writes into)
    vector_t cmd2 = vector_t::Zero(2);
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        if (head_external_control_state_.data.size() >= 2) {
            cmd2 = head_external_control_state_.data.head(2);
        }
    }

    if (integrateVel) {
        cmd2[0] += vel2[0] * dt;
        cmd2[1] += vel2[1] * dt;
    }
    if (hasDelta) {
        cmd2[0] += delta2[0];
        cmd2[1] += delta2[1];
    }

    cmd2 = clampHead2D(cmd2);

    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        head_external_control_state_.update(cmd2);
    }
}

void ControlDataManager::waistYawLinkPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!msg) return;
    
    // 在锁外处理数据
    vector_t pose = vector_t::Zero(7);
    pose[0] = msg->pose.pose.position.x;
    pose[1] = msg->pose.pose.position.y;
    pose[2] = msg->pose.pose.position.z;
    pose[3] = msg->pose.pose.orientation.w;
    pose[4] = msg->pose.pose.orientation.x;
    pose[5] = msg->pose.pose.orientation.y;
    pose[6] = msg->pose.pose.orientation.z;
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        waist_yaw_link_pose_.update(pose);
    }
}

void ControlDataManager::vrTorsoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) return;

    // disable 期间丢弃新的 VR 躯干位姿指令
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;

    // 在锁外处理数据
    vector_t pose = vector_t::Zero(7);
    pose[0] = msg->pose.position.x;
    pose[1] = msg->pose.position.y;
    pose[2] = msg->pose.position.z;
    pose[3] = msg->pose.orientation.w;
    pose[4] = msg->pose.orientation.x;
    pose[5] = msg->pose.orientation.y;
    pose[6] = msg->pose.orientation.z;
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        vr_torso_pose_.update(pose);
    }
}

void ControlDataManager::wholeTorsoCtrlCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg) return;
    
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    whole_torso_ctrl_ = msg->data;
}

void ControlDataManager::ensureArmTrajReceiveTimingPublishers() {
    if (arm_traj_receive_timing_initialized_) {
        return;
    }
    arm_traj_receive_timing_initialized_ = true;
    nh_.param("/vr_ik/enable_arm_traj_receive_timing_log", enable_arm_traj_receive_timing_log_, false);
    if (!enable_arm_traj_receive_timing_log_) {
        return;
    }
    arm_traj_receive_latency_ms_pub_ =
        nh_.advertise<std_msgs::Float64>("/ik_debug/arm_traj_receive/receive_latency_ms", 10);
    arm_traj_receive_period_ms_pub_ =
        nh_.advertise<std_msgs::Float64>("/ik_debug/arm_traj_receive/receive_period_ms", 10);
    arm_traj_receive_stamp_period_ms_pub_ =
        nh_.advertise<std_msgs::Float64>("/ik_debug/arm_traj_receive/stamp_period_ms", 10);
    ROS_INFO("[ControlDataManager] arm_traj receive timing log enabled "
             "(/ik_debug/arm_traj_receive/*)");
}

void ControlDataManager::publishArmTrajReceiveTimingMs(const ros::Publisher& publisher, double ms) const {
    if (!enable_arm_traj_receive_timing_log_ || !publisher) {
        return;
    }
    std_msgs::Float64 msg;
    msg.data = ms;
    publisher.publish(msg);
}

void ControlDataManager::recordArmTrajReceiveTiming(uint64_t stamp_nsec) {
    ensureArmTrajReceiveTimingPublishers();
    if (!enable_arm_traj_receive_timing_log_) {
        return;
    }

    const ros::Time header_stamp(static_cast<uint32_t>(stamp_nsec / 1000000000ULL),
                                 static_cast<uint32_t>(stamp_nsec % 1000000000ULL));
    const ros::Time receive_time = ros::Time::now();
    const auto receive_wall = std::chrono::steady_clock::now();
    publishArmTrajReceiveTimingMs(arm_traj_receive_latency_ms_pub_,
                                  (receive_time - header_stamp).toSec() * 1000.0);
    if (has_last_arm_traj_receive_wall_) {
        publishArmTrajReceiveTimingMs(
            arm_traj_receive_period_ms_pub_,
            std::chrono::duration<double, std::milli>(receive_wall - last_arm_traj_receive_wall_).count());
    }
    if (has_last_arm_traj_header_stamp_) {
        publishArmTrajReceiveTimingMs(
            arm_traj_receive_stamp_period_ms_pub_,
            (header_stamp - last_arm_traj_header_stamp_).toSec() * 1000.0);
    }
    last_arm_traj_receive_wall_ = receive_wall;
    has_last_arm_traj_receive_wall_ = true;
    last_arm_traj_header_stamp_ = header_stamp;
    has_last_arm_traj_header_stamp_ = true;
}

void ControlDataManager::ingestArmJointTrajectory(const double* pos_rad, const double* vel_rad,
                                                  const double* tau, int count,
                                                  uint64_t stamp_nsec) {
    if (!prev_enable_control_.load(std::memory_order_acquire)) {
        return;
    }
    if (count != arm_num_) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Arm joint count mismatch: %d vs %d", count, arm_num_);
        return;
    }

    if (stamp_nsec != 0) {
        recordArmTrajReceiveTiming(stamp_nsec);
    }

    ArmJointTrajectory traj;
    traj.init(arm_num_);
    for (int i = 0; i < arm_num_; ++i) {
        traj.pos[i] = pos_rad[i];
        traj.vel[i] = (vel_rad != nullptr) ? vel_rad[i] : 0.0;
        traj.tau[i] = (tau != nullptr) ? tau[i] : 0.0;
    }

    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        arm_external_control_state_.update(traj);
    }
}

void ControlDataManager::legJointTrajCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // disable 期间丢弃新的下肢轨迹指令
    if (!prev_enable_control_.load(std::memory_order_acquire)) return;

    if(msg->name.size() != low_joint_num_) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Leg joint count mismatch: %lu vs %d", msg->name.size(), low_joint_num_);
        return;
    }
    
    // 在锁外处理数据
    ArmJointTrajectory traj;
    traj.init(low_joint_num_);
    
    for(int i = 0; i < low_joint_num_; i++) {
        traj.pos[i] = msg->position[i] * M_PI / 180.0;  // 角度转弧度
        if(msg->velocity.size() == low_joint_num_)
            traj.vel[i] = msg->velocity[i] * M_PI / 180.0;
        if(msg->effort.size() == low_joint_num_)
            traj.tau[i] = msg->effort[i];
    }
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        leg_external_control_state_.update(traj);
    }
}

void ControlDataManager::lbMpcControlModeCallback(const std_msgs::Int8::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    lb_mpc_control_mode_ = msg->data;
}

// ========== 数据获取接口实现 ==========

// ---------- 需要时间戳校验的接口 ----------
bool ControlDataManager::updateSensorDataFromShm(SensorData& out) const 
{
    if (!use_shm_communication_ || !shm_manager_) {
        return false;
    }

    gazebo_shm::SensorsData shm_raw_data;
    if (!shm_manager_->readSensorsData(shm_raw_data)) 
    {
        ROS_WARN_THROTTLE(1.0, "[updateSensorDataFromShm] Failed to read sensors data from shared memory");
        return false;
    }
    
    // 直接修改输出参数
    auto joint_num = low_joint_num_ + arm_num_ + head_num_;
    out.resize_joint(joint_num);
    
    // 关节数据
    for (size_t i = 0; i < joint_num; ++i) {
        out.jointPos_(i) = shm_raw_data.joint_data[i].position;
        out.jointVel_(i) = shm_raw_data.joint_data[i].velocity;
        out.jointAcc_(i) = 0.0;  // 加速度在共享内存中未提供
        out.jointTorque_(i) = shm_raw_data.joint_data[i].effort;
    }
    
    // IMU数据
    out.quat_.coeffs() << shm_raw_data.imu_data.orientation[0],
                        shm_raw_data.imu_data.orientation[1],
                        shm_raw_data.imu_data.orientation[2],
                        shm_raw_data.imu_data.orientation[3];
                           
    out.angularVel_ << shm_raw_data.imu_data.angular_velocity[0],
                      shm_raw_data.imu_data.angular_velocity[1],
                      shm_raw_data.imu_data.angular_velocity[2];
                         
    out.linearAccel_ << shm_raw_data.imu_data.linear_acceleration[0],
                       shm_raw_data.imu_data.linear_acceleration[1],
                       shm_raw_data.imu_data.linear_acceleration[2];
    
    out.orientationCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    out.angularVelCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    out.linearAccelCovariance_ = Eigen::Matrix<double, 3, 3>::Zero();
    
    // 更新时间戳
    out.timeStamp_ = ros::Time(shm_raw_data.sensor_time);
    
    return true;
}

void ControlDataManager::publishJointCmdToShm(const kuavo_msgs::jointCmd& jointCmdMsg) const
{
    if (!use_shm_communication_ || !shm_manager_) 
    {
        return;
    }

    gazebo_shm::JointCommand joint_cmd;
    joint_cmd.num_joints = low_joint_num_ + arm_num_ + head_num_;

    // 检查数据大小是否匹配
    if (jointCmdMsg.joint_q.size() != joint_cmd.num_joints) 
    {
        ROS_ERROR("[ControlDataManager] Joint command size mismatch: expected %zu, got %zu", 
                  joint_cmd.num_joints, jointCmdMsg.joint_q.size());
        return;
    }

    // 从jointCmdMsg中复制数据到共享内存结构
    for (size_t i = 0; i < joint_cmd.num_joints; ++i) 
    {
        joint_cmd.joint_q[i] = jointCmdMsg.joint_q[i];
        joint_cmd.joint_v[i] = jointCmdMsg.joint_v[i];
        joint_cmd.tau[i] = jointCmdMsg.tau[i];
        joint_cmd.tau_max[i] = jointCmdMsg.tau_max[i];
        joint_cmd.joint_kp[i] = jointCmdMsg.joint_kp[i];
        joint_cmd.joint_kd[i] = jointCmdMsg.joint_kd[i];
        joint_cmd.control_modes[i] = jointCmdMsg.control_modes[i];
    }

    // 写入共享内存
    if (!shm_manager_->writeJointCommandNext(joint_cmd)) {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Failed to write joint command to shared memory");
    }
}

bool ControlDataManager::getRealtimeSensorData(SensorData& out) const {
    // 如果使用共享内存，直接从共享内存获取数据
    if (use_shm_communication_) 
    {
        if (updateSensorDataFromShm(out)) 
        {
            return true;
        }
        else
        {
            // ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Failed to update sensor data from shared memory");
            return false;
        }
    }
    
    // 如果共享内存获取失败或未启用，使用缓存数据
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (sensor_data_.isValid()) {  // 使用默认1秒超时
        out = sensor_data_.data;
        return true;
    }
    out = sensor_data_.data;
    return false;
}

bool ControlDataManager::getRealtimeOdomData(vector6_t& out) const {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (odom_data_.isValid()) {
        out = odom_data_.data;
        return true;
    }
    out = odom_data_.data;
    return false;
}

bool ControlDataManager::getRealtimeBaseLinkPose(vector_t& out) const {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (base_link_pose_.isValid()) {
        out = base_link_pose_.data;
        return true;
    }
    return false;
}

bool ControlDataManager::getRealtimeCmdVel(geometry_msgs::Twist& out) const {
    // disable 期间返回零速度（冻结底盘）
    if (!prev_enable_control_.load(std::memory_order_acquire)) {
        out = geometry_msgs::Twist();
        return true;
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (cmd_vel_.isValid(0.2)) {  // 速度命令使用500ms超时，确保安全性
        out = cmd_vel_.data;
        return true;
    }
    return false;
}

bool ControlDataManager::getRealtimeWaistYawLinkPose(vector_t& out) const {
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    if (waist_yaw_link_pose_.isValid()) {  // 使用默认1秒超时
        out = waist_yaw_link_pose_.data;
        return true;
    }
    return false;
}

bool ControlDataManager::getRealtimeVrTorsoPose(vector_t& out) const {
    // disable 期间 VR 躯干相对偏移保持为零（原地不动语义）
    if (!prev_enable_control_.load(std::memory_order_acquire)) {
        out = makeIdentityPose7D();
        return true;
    }

    std::lock_guard<std::mutex> lock(external_state_mutex_);
    if (vr_torso_pose_.isValid()) {  // 使用默认1秒超时
        out = vr_torso_pose_.data;
        return true;
    }
    return false;
}

// ---------- 直接返回最新数据的接口 ----------
vector_t ControlDataManager::getLbWaistExternalControlState() const {
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    return lb_waist_external_control_state_.data;  // 直接返回最新的轮臂关节命令
}

void ControlDataManager::setLbWaistExternalControlState(const vector_t& joint_state) {
    if (joint_state.size() != 4) {
        ROS_WARN("Invalid lb waist joint state size: %ld (expected 4)", joint_state.size());
        return;
    }
    
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    lb_waist_external_control_state_.update(joint_state);
}

vector_t ControlDataManager::getHeadExternalControlState() {
    // vel/delta first: 读前积分到 head_external_control_state_（唯一开环终点）。
    // 绝对 /robot_head_motion_data 在回调里直接写同一 store，同周期仍可覆盖。
    applyHeadVelDeltaCommands();
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    return head_external_control_state_.data;
}

vector_t ControlDataManager::computeHeadControl(const vector_t& target_pos) const {

    // 初始化反馈扭矩
    vector_t feedback_tau = vector_t::Zero(head_num_);

    if(is_real_ || head_num_ == 0)
    {
        return feedback_tau;
    }

    // 仿真时计算PD反馈扭矩
    SensorData sensor_data;
    bool sensor_data_valid = getRealtimeSensorData(sensor_data);
    if(!sensor_data_valid)
    {
        return feedback_tau;
    }

    // 提取头部关节位置和速度（假设头部在关节数组最后）
    int total_joints = sensor_data.jointPos_.size();
    if (total_joints >= head_num_)
    {
        vector_t current_pos = sensor_data.jointPos_.tail(head_num_);
        vector_t current_vel = sensor_data.jointVel_.tail(head_num_);

        // 计算PD反馈扭矩
        feedback_tau = head_kp_.cwiseProduct(target_pos - current_pos) + head_kd_.cwiseProduct(-current_vel);
    }

    return feedback_tau;
}

bool ControlDataManager::getWholeTorsoCtrl() const {
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    return whole_torso_ctrl_;  // 直接返回VR全身控制模式状态
}

ArmJointTrajectory ControlDataManager::getJointTrajectoryWithDisableGuard(const TimestampedData<ArmJointTrajectory>& storage) const
{
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    ArmJointTrajectory traj = storage.data;
    // disable 期间速度/力矩保持为零
    if (!prev_enable_control_.load(std::memory_order_acquire)) {
        traj.vel.setZero();
        traj.tau.setZero();
    }
    return traj;
}

ArmJointTrajectory ControlDataManager::getArmExternalControlState() const
{
    return getJointTrajectoryWithDisableGuard(arm_external_control_state_);
}

ArmJointTrajectory ControlDataManager::getLegExternalControlState() const
{
    return getJointTrajectoryWithDisableGuard(leg_external_control_state_);
}

int8_t ControlDataManager::getLbMpcControlMode() const 
{
    std::lock_guard<std::mutex> lock(external_state_mutex_);
    return lb_mpc_control_mode_;
}

void ControlDataManager::updateArmExternalControlState(const Eigen::VectorXd& current_pos, const Eigen::VectorXd& current_vel, const Eigen::VectorXd& current_tau) 
{
    if (current_pos.size() != arm_num_ || current_vel.size() != arm_num_ || current_tau.size() != arm_num_) 
    {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Invalid arm state dimensions");
        return;
    }
    
    // 在锁外准备数据
    ArmJointTrajectory state;
    state.init(arm_num_);
    state.pos = current_pos;
    state.vel = current_vel;
    state.tau = current_tau;
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        arm_external_control_state_.update(state);
    }
}

void ControlDataManager::updateLegExternalControlState(const Eigen::VectorXd& current_pos, const Eigen::VectorXd& current_vel, const Eigen::VectorXd& current_tau) 
{
    if (current_pos.size() != low_joint_num_ || current_vel.size() != low_joint_num_ || current_tau.size() != low_joint_num_) 
    {
        ROS_WARN_THROTTLE(1.0, "[ControlDataManager] Invalid leg state dimensions");
        return;
    }
    
    // 在锁外准备数据
    ArmJointTrajectory state;
    state.init(low_joint_num_);
    state.pos = current_pos;
    state.vel = current_vel;
    state.tau = current_tau;
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        leg_external_control_state_.update(state);
    }
}

void ControlDataManager::resetVrTorsoPose() {
    vector_t pose = vector_t::Zero(7);
    pose[3] = 1.0;  // qw = 1 (单位四元数)
    
    {
        std::lock_guard<std::mutex> lock(external_state_mutex_);
        vr_torso_pose_.update(pose);
    }
}

// 批量获取接口（性能最优）
ControlDataManager::ControlData ControlDataManager::getAllControlData() const {
    ControlData data;
    bool sensor_valid = false;
    bool motion_valid = false;
    
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        data.sensors = sensor_data_.data;
        sensor_valid = sensor_data_.isValid();
    }
    
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        data.odom = odom_data_.data;
        data.base_link_pose = base_link_pose_.data;
        data.cmd_vel = cmd_vel_.data;
        motion_valid = odom_data_.isValid() && base_link_pose_.isValid();
    }
    
    data.valid = sensor_valid && motion_valid;
    return data;
}

bool ControlDataManager::isDataReady(bool check_motion_data) const {
    // 如果使用共享内存，先尝试更新数据
    if (use_shm_communication_) 
    {
        SensorData out;
        if (updateSensorDataFromShm(out)) 
        {
            return true; 
        }
        else
        {
            return false;
        }
    }

    // 检查传感器数据
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!sensor_data_.isValid()) {
        return false;
    }
    
    if (!check_motion_data) {
        return true;
    }
    
    // 检查运动数据
    std::lock_guard<std::mutex> lock2(motion_mutex_);
    return odom_data_.isValid() && base_link_pose_.isValid();
}

// ========== 辅助函数 ==========

double ControlDataManager::rosQuaternionToYaw(const geometry_msgs::Quaternion& ros_quat) const {
    Eigen::Quaterniond eigen_quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
    Eigen::Matrix3d R = eigen_quat.toRotationMatrix();
    return std::atan2(R(1, 0), R(0, 0));
}

vector_t ControlDataManager::makeIdentityPose7D() {
    vector_t pose = vector_t::Zero(7);
    pose[3] = 1.0;  // qw = 1，单位旋转
    return pose;
}

} // namespace humanoidController_wheel_wbc

