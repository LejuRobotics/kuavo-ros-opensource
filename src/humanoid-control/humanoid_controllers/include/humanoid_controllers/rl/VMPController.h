#pragma once

// Pinocchio must be included before Boost headers
#include <pinocchio/fwd.hpp>

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/armController.h"
#include "humanoid_controllers/LowPassFilter.h"
#include "kuavo_solver/ankle/ankle_solver.h"

// VR相关头文件
#include "kuavo_msgs/picoPoseRetarget.h"
// Mocap相关头文件
#include "kuavo_msgs/MocapPoseRetarget.h"
// Xsense相关头文件
#include "kuavo_msgs/xsensePoseRetarget.h"
// PICO/Xsens 共用手柄消息
#include "kuavo_msgs/JoySticks.h"
// Ruiwo电机参数切换服务
#include "kuavo_msgs/ExecuteArmAction.h"
// ROS服务
#include <std_srvs/SetBool.h>
#include <std_msgs/Header.h>

#include <openvino/openvino.hpp>
#include <memory>
#include <map>
#include <mutex>
#include <deque>
#include <atomic>
#include <functional>
#include <thread>

namespace humanoid_controller
{
  /**
   * @brief VMP配置参数结构体
   * 用于存储VMP VAE和参考运动的配置参数
   */
  struct VMPConfig {
    int in_c = 77;              // 输入通道数（参考运动维度）
    int window_l = 30;          // 左窗口长度
    int window_r = 15;          // 右窗口长度
    int base_c = 128;           // 基础通道数
    int latent_d = 64;          // 潜在维度
    int scalors = 3;            // 缩放器数量
    bool encode_atten = true;   // 是否使用注意力编码
    double a = 0.01;            // 参数a
    double d = 2;               // 参数d
    double beta = 0.7;          // beta参数
    std::string encode_method = "TCN";  // 编码方法
    bool use_global_coef = true;  // 是否使用全局均值/标准差归一化（与Python对齐）
    
    // 参考运动数据索引配置
    int h_start_id = 0;         // 高度开始ID
    int h_end_id = 1;           // 高度结束ID
    int theta_start_id = 1;     // 方向开始ID
    int theta_end_id = 7;       // 方向结束ID
    int v_start_id = 7;         // 速度开始ID
    int v_end_id = 13;          // 速度结束ID
    int q_start_id = 13;        // 关节位置开始ID
    int q_end_id = 39;          // 关节位置结束ID
    int q_dot_start_id = 39;    // 关节速度开始ID
    int q_dot_end_id = 65;      // 关节速度结束ID
    int p_start_id = 65;        // 末端位置开始ID
    int p_end_id = 77;          // 末端位置结束ID
    int num_tracklink = 4;      // 追踪链接数量
  };

  /**
   * @brief 预加载轨迹数据结构
   */
  struct PreloadedTrajectory {
    std::vector<float> task_data;              // 原始轨迹数据
    std::vector<int> task_lengths;             // 轨迹长度数据
    std::vector<float> processed_data;         // 处理后的数据（包含静止帧）
    bool is_loaded = false;                    // 是否已加载
    bool is_processed = false;                 // 是否已处理（添加静止帧）
    std::string name;                          // 轨迹名称
    size_t original_frames = 0;                // 原始帧数
    size_t total_frames = 0;                   // 包含静止帧的总帧数
  };

  /**
   * @brief 在线参考数据环形缓冲区
   */
  struct OnlineReferenceBuffer {
    std::vector<std::vector<float>> data_buffer;  // 固定大小的环形缓冲区
    int buffer_size;                              // 缓冲区大小
    int current_write_index;                      // 当前写入位置
    int current_read_index;                       // 当前读取位置
    int total_frames_received;                    // 已接收的总帧数
    bool is_initialized;                          // 是否已初始化
    ros::Time last_update_time;                   // 上次更新时间
    
    OnlineReferenceBuffer() 
        : buffer_size(200), current_write_index(0), current_read_index(0),
          total_frames_received(0), is_initialized(false) {}
    
    void reset() {
      data_buffer.clear();
      current_write_index = 0;
      current_read_index = 0;
      total_frames_received = 0;
      is_initialized = false;
    }
    
    int getBufferFrameCount() const {
      return std::min(total_frames_received, buffer_size);
    }
    
    bool isFull() const {
      return total_frames_received >= buffer_size;
    }
  };

  /**
   * @brief 数据段结构（用于多话题数据融合）
   */
  struct DataSegment {
    std::vector<double> data;
    ros::Time timestamp;
    bool valid;
    double max_age_sec;
    
    DataSegment() : valid(false), max_age_sec(0.5) {}
    
    bool isFresh(const ros::Time& now) const {
      if (!valid) return false;
      return (now - timestamp).toSec() < max_age_sec;
    }
    
    void update(const std::vector<double>& new_data, const ros::Time& stamp) {
      data = new_data;
      timestamp = stamp;
      valid = true;
    }
    
    void reset() {
      valid = false;
      data.clear();
    }
  };

  /**
   * @brief 手臂数据结构
   */
  struct ArmData {
    DataSegment left_joint_angles;
    DataSegment right_joint_angles;
    DataSegment left_hand_pos;
    DataSegment right_hand_pos;
    DataSegment left_hand_quat;
    DataSegment right_hand_quat;
    
    ArmData() {
      left_joint_angles.data.resize(7, 0.0);
      right_joint_angles.data.resize(7, 0.0);
      left_hand_pos.data.resize(3, 0.0);
      right_hand_pos.data.resize(3, 0.0);
      left_hand_quat.data.resize(4, 0.0);
      right_hand_quat.data.resize(4, 0.0);
    }
  };

  /**
   * @brief 脚部数据结构
   */
  struct FootData {
    DataSegment left_foot_pos;
    DataSegment right_foot_pos;
    
    FootData() {
      left_foot_pos.data.resize(3, 0.0);
      right_foot_pos.data.resize(3, 0.0);
    }
  };

  /**
   * @brief Qpos平滑：pending buffer 中的单个样本
   * 用于检测重复帧并回溯插值
   */
  struct PendingQposSample {
    std::vector<float> frame;   // 完整VMP帧数据 (in_c维)
    bool anchor;                // true=真正新数据, false=重复帧(待插值)
    bool smoothed;              // 是否已被回溯插值覆盖
  };

  /**
   * @brief VMP控制器类
   * 实现基于VAE的模仿学习控制，支持离线轨迹播放和在线VR遥操作
   */
  class VMPController : public RLControllerBase
  {
  public:
    /**
     * @brief 构造函数
     * @param name 控制器名称
     * @param config_file 配置文件路径
     * @param nh ROS节点句柄
     * @param ros_logger ROS日志发布器（可选）
     */
    VMPController(const std::string& name, const std::string& config_file,
                  ros::NodeHandle& nh,
                  ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    /**
     * @brief 析构函数
     */
    ~VMPController() override;

    /**
     * @brief 初始化控制器
     * @return 是否初始化成功
     */
    bool initialize() override;

    /**
     * @brief 加载配置文件
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    bool loadConfig(const std::string& config_file) override;

    /**
     * @brief 重置控制器状态
     */
    void reset() override;

    /**
     * @brief 暂停控制器（切回正常 Ruiwo 电机参数）
     */
    void pause() override;

    /**
     * @brief 恢复控制器（切换到 VMP 专用 Ruiwo 电机参数）
     */
    void resume() override;

    /**
     * @brief 恢复使用最新的重定向数据（等效于 RT+X）
     */
    void resumeRetargetedStreaming();

    /**
     * @brief 切入 VMP 前将在线参考朝向对齐到机器人，并等待 GMR 完成 RT+X
     */
    void prepareRetargetedStreamingResume(const Eigen::Quaterniond& robot_imu_quat,
                                          const ros::Time& robot_sensor_stamp);

    /**
     * @brief MPC→VMP 插值完成后的回调，启动在线采样
     */
    void onInterpolationComplete() override;

    /**
     * @brief 是否请求退出当前 RL 模式（与 RLControllerBase 一致）
     * 当前实现调用基类默认，恒为 false，不主动请求由上层自动退出；若需运动结束自动切出，可在此扩展条件。
     * @return 恒为 false
     */
    bool requestToExit() const override;

    /**
     * @brief 是否允许从当前 VMP 控制器切出（如切回 MPC）
     * 循环/多轨迹模式下始终允许；单次离线轨迹需播放完成后才允许。
     */
    bool isAllowToExit() const override;

    int getArmControlModeOnControllerActivate() const override;

    void setExternalCommandBufferCallback(std::function<bool()> callback) override;

  protected:
    /**
     * @brief 更新控制器实现
     */
    bool updateImpl(const ros::Time& time, 
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 推理实现
     */
    bool inference(const Eigen::VectorXd& observation,
                   Eigen::VectorXd& action) override;

    /**
     * @brief 更新观测
     */
    void updateObservation(const Eigen::VectorXd& state_est,
                           const SensorData& sensor_data) override;

    /**
     * @brief 动作转关节命令
     */
    void actionToJointCmd(const Eigen::VectorXd& actuation,
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 传感器数据预处理
     */
    void preprocessSensorData(SensorData& sensor_data) override;

    /**
     * @brief 推理条件检查
     */
    bool shouldRunInference() const override;

    bool updateArmCommand(const ros::Time& time,
                          const SensorData& sensor_data,
                          kuavo_msgs::jointCmd& joint_cmd) override;

  private:
    // ========== VMP核心推理函数 ==========
    
    /**
     * @brief VMP推理入口
     */
    void vmpInference(const Eigen::VectorXd& state_est, const SensorData& sensor_data);
    
    /**
     * @brief 计算VMP动作
     */
    Eigen::VectorXd computeVMPAction(const Eigen::VectorXd& observation);
    
    /**
     * @brief 更新VMP参考运动（离线/在线模式分支处理）
     */
    void updateVMPReferenceMotion();
    
    /**
     * @brief VMP命令计算（PD控制）
     */
    Eigen::VectorXd updateVMPcmd(const Eigen::VectorXd& state);

    // ========== VMP初始化和模型管理函数 ==========
    
    /**
     * @brief 初始化VMP系统
     * @return 成功返回true，失败返回false
     */
    bool initializeVMP();
    
    /**
     * @brief 设置VMP模型
     */
    void setupVMPModels();
    
    /**
     * @brief 模型预热
     */
    void warmupVMPModels();
    
    // ========== 离线轨迹相关函数 ==========
    
    /**
     * @brief 加载VMP参考数据
     */
    void loadVMPRefData();
    
    /**
     * @brief 加载单轨迹数据
     */
    void loadSingleTrajectoryData();
    
    /**
     * @brief 加载当前轨迹数据（多轨迹模式）
     */
    void loadCurrentTrajectoryData();
    
    /**
     * @brief 切换到下一条轨迹
     */
    void switchToNextTrajectory();
    
    /**
     * @brief 添加静止帧到轨迹数据
     */
    void appendStandingFramesToTaskData();
    
    // ========== 预加载相关函数 ==========
    
    /**
     * @brief 预加载所有轨迹
     */
    bool preloadAllTrajectories();
    
    /**
     * @brief 预加载单个轨迹
     */
    bool preloadSingleTrajectory(size_t index, const std::string& dataFile);
    
    /**
     * @brief 处理预加载的轨迹
     */
    void processPreloadedTrajectory(size_t index);
    
    /**
     * @brief 切换到预加载的轨迹
     */
    bool switchToPreloadedTrajectory(size_t index);
    
    /**
     * @brief 清除预加载轨迹
     */
    void clearPreloadedTrajectories();
    
    /**
     * @brief 获取预加载内存使用量
     */
    size_t getPreloadedMemoryUsage() const;

    // ========== 在线VR遥操作相关函数 ==========
    
    /**
     * @brief 初始化在线参考缓冲区
     */
    void initializeOnlineReferenceBuffer();
    
    /**
     * @brief 更新在线参考缓冲区
     */
    void updateOnlineReferenceBuffer(const std::vector<float>& new_frame);
    
    /**
     * @brief 从缓冲区获取帧
     */
    std::vector<float> getFrameFromOnlineBuffer(int relative_index);
    
    /**
     * @brief 加载bin文件用于在线播放（测试用）
     */
    void loadBinFileForOnlinePlayback(const std::string& data_file);
    
    /**
     * @brief 从bin文件更新在线缓冲区
     */
    void updateOnlineBufferFromBinFile();
    
    /**
     * @brief PICO重定向姿态回调
     * @param msg PICO重定向姿态消息，包含基座位姿、速度、关节位置/速度、末端位置
     * 
     * 将77维VMP数据格式：
     * [0]: 机身高度 (base_link z position)
     * [1-6]: 旋转矩阵前两行（从四元数转换）
     * [7-12]: 机身速度 (6D: linear xyz + angular xyz)
     * [13-38]: 关节位置 (26D)
     * [39-64]: 关节速度 (26D)
     * [65-76]: 末端位置 (12D: left_hand, right_hand, left_foot, right_foot)
     */
    void picoRetargetedPoseCallback(const kuavo_msgs::picoPoseRetarget::ConstPtr& msg);
    
    /**
     * @brief 处理Mocap重定向姿态消息（/mocap/retargeted_pose）
     * @param msg Mocap重定向姿态消息（使用与PICO相同的消息格式）
     * 
     * 消息格式与PICO相同，使用picoPoseRetarget消息类型
     */
    void mocapRetargetedPoseCallback(const kuavo_msgs::MocapPoseRetarget::ConstPtr& msg);
    
    /**
     * @brief 处理Xsense重定向姿态消息（/xsense/retargeted_pose）
     * @param msg Xsense重定向姿态消息（字段格式与Mocap/PICO一致）
     */
    void xsenseRetargetedPoseCallback(const kuavo_msgs::xsensePoseRetarget::ConstPtr& msg);

    /**
     * @brief GMR 已发布完成偏航重定向的新帧
     */
    void retargetYawResumeReadyCallback(const std_msgs::Header::ConstPtr& msg);

    void recordRetargetedFrameStamp(const ros::Time& stamp);

    bool extractRetargetedFrameYaw(const std::vector<float>& frame, double& yaw) const;
    void applyRetargetedHeadingAlignment(std::vector<float>& frame) const;
    void applyRetargetedHeadingAlignment(float* frames, int frame_count) const;
    void prefillRetargetedReferenceBuffers(const std::vector<float>& canonical_frame);
    
    /**
     * @brief PICO推流暂停/恢复 ROS 服务回调
     * @param req SetBool请求 (true=暂停推流, false=恢复推流)
     * @param res SetBool响应
     * @return 服务是否成功执行
     * 
     * 由 pico-body-tracking-server 的 PICO 手柄按键回调触发：
     * - RT + Y：暂停推流（调用 service data=true）
     * - RT + X：恢复推流（调用 service data=false）
     * 也可通过 rosservice call 直接调用
     */
    bool picoStreamControlServiceCallback(std_srvs::SetBool::Request& req,
                                          std_srvs::SetBool::Response& res);

    /**
     * @brief Xsens 手柄推流暂停/恢复回调，同步冻结/恢复外部手臂控制
     */
    void picoJoyStreamControlCallback(const kuavo_msgs::JoySticks::ConstPtr& msg);
    
    /**
     * @brief 融合多话题数据
     */
    std::vector<float> fuseMultiTopicData();
    
    /**
     * @brief 预处理在线运动数据
     */
    std::vector<float> preprocessOnlineMotionData(const std::vector<float>& raw_data);
    
    /**
     * @brief 在线采样循环
     */
    void onlineSamplingLoop();
    
    /**
     * @brief 启动在线采样
     */
    void startOnlineSampling();
    
    /**
     * @brief 停止在线采样
     */
    void stopOnlineSampling();
    
    /**
     * @brief 判断是否为在线设备模式（PICO或Mocap）
     */
    bool isOnlineVRDeviceMode() const;

    // ========== 数学工具函数 ==========
    
    /**
     * @brief 参考运动归一化
     */
    void normalizeRefMotion(Eigen::VectorXd& ref_motion);

    /**
     * @brief 计算 motion_anchor_euler_b 观测项（参考 anchor 在 robot anchor 坐标系下的相对姿态误差）
     */
    Eigen::Vector3d computeAnchorEulerB(const SensorData& sensor_data);
    
    /**
     * @brief 矩阵转四元数
     */
    Eigen::Vector4d mat_to_quat(const Eigen::Matrix3d& matrix);
    
    /**
     * @brief 获取欧拉角
     */
    Eigen::Vector3d get_euler_xyz(const Eigen::Vector4d& quat);
    
    /**
     * @brief 欧拉角转旋转矩阵
     */
    Eigen::Matrix3d euler_to_rotation_matrix(const Eigen::Vector3d& euler);
    
    /**
     * @brief 应用时间归一化
     */
    void applyTemporalNormalization(float* input_data, size_t data_size);
    
    /**
     * @brief 应用高斯滤波
     */
    std::vector<float> applyGaussianFilter(const std::vector<float>& velocities);
    
    /**
     * @brief GMR数据平滑
     */
    std::vector<float> applyGMRSmoothing(const std::vector<float>& raw_frame);
    
    // ========== Qpos平滑（重复帧回溯插值） ==========
    /**
     * @brief 判断新帧是否真正包含新的运动数据
     * 对比关节位置q维度的最大绝对差值，超过阈值则视为新数据
     */
    bool isNewQpos(const std::vector<float>& frame);
    
    /**
     * @brief 线性插值两个VMP帧（所有维度均做线性插值）
     */
    std::vector<float> interpolateFrame(const std::vector<float>& f0,
                                         const std::vector<float>& f1,
                                         float alpha);
    
    /**
     * @brief 回溯插值pending buffer尾部
     * 当最新样本是anchor时，从最近的前一个anchor到当前anchor之间的held帧做线性插值
     */
    void retroInterpolatePendingTail();
    
    /**
     * @brief 从pending buffer弹出最旧样本（如果buffer已满）
     * @return 弹出的帧数据，如果buffer未满则返回空
     */
    std::vector<float> popOldestPendingIfReady();
    
    /**
     * @brief 四元数球面线性插值
     */
    Eigen::Quaterniond slerpQuaternion(const Eigen::Quaterniond& q1, 
                                        const Eigen::Quaterniond& q2, double t);
    
    /**
     * @brief 帧序列插值
     */
    std::vector<std::vector<float>> interpolateFrameSequence(
        const std::vector<float>& frame1,
        const std::vector<float>& frame2,
        double alpha);
    
    /**
     * @brief 时间戳驱动的插值
     */
    std::vector<float> interpolateWithTimestamp(
        const std::vector<float>& frame_prev,
        const std::vector<float>& frame_next,
        ros::Time t_prev, ros::Time t_next, ros::Time t_target);

    // ========== Estimator模块函数 ==========
    
    /**
     * @brief 更新Estimator观测
     * @param robot_observation 已构建的机器人观测向量（与Policy输入共享）
     * @param sensor_data 传感器数据（保留参数，当前未使用）
     */
    void updateEstimatorObservation(const Eigen::VectorXd& robot_observation, 
                                    const SensorData& sensor_data);
    
    /**
     * @brief 运行Estimator推理
     */
    Eigen::VectorXd runEstimatorInference();

    // ========== 辅助函数 ==========
    
    /**
     * @brief 检查文件是否存在
     */
    bool fileExists(const std::string& path);
    
    /**
     * @brief 确定VMP数据路径
     */
    std::string determineVMPDataPath(const std::string& filename);
    
    /**
     * @brief 打印多轨迹状态
     */
    void printMultiTrajectoryStatus();
    
    /**
     * @brief 打印在线缓冲区状态
     */
    void printOnlineBufferStatus();

    void initArmControl(const std::string& urdf_path);
    void activateExternalArmControlIfNeeded();

  private:
    // ========== 基本控制参数 ==========
    double arm_max_tracking_velocity_{0.5};
    double arm_tracking_error_threshold_{0.05};
    double arm_mode_interpolation_velocity_{1.0};
    double arm_mode2_cutoff_freq_{2.0}; ///< 外部手臂 mode2 输入低通截止频率 (Hz)
    std::function<bool()> external_command_buffer_callback_;

    double dt_{0.002};                    // 控制周期
    bool is_real_{false};                 // 是否真机
  bool is_roban_{false};                // 是否是roban机器人
    kuavo_solver::AnkleSolver ankleSolver_;  // 脚踝解算器
    
    // ========== VMP模式标志 ==========
    bool vmp_initialized_{false};         // VMP是否初始化
    bool vmp_enable_theta_normalization_{true};  // 是否启用theta归一化
    
    // ========== Ruiwo 电机参数切换 ==========
    bool use_vmp_ruiwo_kpkd_{false};      // 是否使用 VMP 专用 Ruiwo 增益（由 vmp_param.info 中 use_vmp_ruiwo_kpkd 配置）
    ros::ServiceClient srv_change_motor_param_;  // Ruiwo 电机参数切换服务客户端
    
    /**
     * @brief 异步切换 Ruiwo 电机参数，避免在控制循环中阻塞
     * @param param_name kuavo.json 中的参数配置名称（如 "vmp_kpkd", "normal_kpkd"）
     */
    void changeRuiwoMotorParamAsync(const std::string& param_name);
    
    // ========== VMP播放模式 ==========
    std::string vmp_playback_mode_{"offline_trajectory"};  // 播放模式
    bool enable_online_vr_mode_{false};   // 是否启用在线VR模式
    
    // ========== VMP模型相关 ==========
    std::string baseModelPath_;           // 基础模型路径
    std::string vmpModelPath_;            // VMP策略模型路径
    std::string vmpEncoderPath_;          // VAE编码器路径
    std::string vmpEstimatorPath_;        // Estimator路径（离线）
    std::string vmpOnlinePolicyPath_;     // 在线策略路径
    std::string vmpOnlineEncoderPath_;    // 在线编码器路径
    std::string vmpOnlineEstimatorPath_;  // 在线Estimator路径
    std::string vmpRefDataDir_;           // 参考数据目录
    std::string vmpTaskDataFile_;         // 任务数据文件
    
    ov::Core core_;
    ov::CompiledModel vmp_policy_model_;
    ov::CompiledModel vmp_encoder_model_;
    ov::CompiledModel vmp_estimator_model_;
    ov::InferRequest vmp_policy_request_;
    ov::InferRequest vmp_encoder_request_;
    ov::InferRequest vmp_estimator_request_;
    
    // ========== VMP参数配置 ==========
    VMPConfig vmp_config_;
    double vmp_actionScale_{0.25};
    int vmp_frameStack_{1};
    int vmp_numSingleObs_{90};       // 观测维度 (v46=87, v52/v53=90)
    double vmp_clipObservations_{18.0};
    double vmp_clipActions_{18.0};
    int numRefMotionObs_{77};        // 参考运动维度 (v46=77, v52=79)
    int numEncoderObs_{64};
    int numActions_{26};             // 关节/动作维度 (v46=26, v52=27)
    int episodeLengthS_{30};
    int decimation_{10};
    
    // ========== VMP观测缩放参数 ==========
    double vmp_obsScale_dof_pos_{1.0};
    double vmp_obsScale_dof_vel_{0.05};
    double vmp_obsScale_base_lin_vel_{2.0};
    double vmp_obsScale_base_ang_vel_{0.25};
    double vmp_obsScale_height_measurements_{5.0};
    double vmp_obsScale_quat_{1.0};

    // Entry alignment: maps the robot IMU at VMP activation onto the current
    // motion-anchor heading, removing any fixed yaw error at switch-in.
    Eigen::Quaterniond vmp_entry_imu_quat_{Eigen::Quaterniond::Identity()};
    bool vmp_entry_imu_quat_valid_{false};
    
    // ========== VMP关节控制参数 ==========
    Eigen::VectorXd vmp_defalutJointPos_;
    Eigen::VectorXd vmp_JointControlMode_;
    Eigen::VectorXd vmp_JointPDMode_;
    Eigen::VectorXd vmp_jointKp_;
    Eigen::VectorXd vmp_jointKd_;
    Eigen::VectorXd vmp_torqueLimits_;
    Eigen::VectorXd vmp_actionScaleTest_;
    Eigen::VectorXd vmp_jointCmdFilterCutoffFreq_;
    Eigen::VectorXd vmp_jointCmdFilterState_;
    LowPassFilter2ndOrder vmpJointCmdFilter_;
    
    // ========== VMP推理相关 ==========
    Eigen::VectorXd vmp_actions_;
    Eigen::VectorXd vmp_ref_motion_;
    Eigen::VectorXd vmp_latent_code_;
    std::deque<Eigen::VectorXd> vmp_ref_motion_buffer_;
    // Keep an unnormalized copy for observation terms that must use raw anchor orientation.
    std::deque<Eigen::VectorXd> vmp_ref_motion_raw_buffer_;
    
    // ========== 离线轨迹数据 ==========
    std::vector<float> vmp_task_data_;
    std::vector<int> vmp_task_lengths_;
    int current_task_idx_{0};
    
    // 静止帧配置
    std::vector<float> vmp_standing_frame_;
    std::vector<float> vmp_standing_joint_pos_;
    std::vector<float> vmp_standing_end_effector_pos_;  // 末端执行器位置 (12维: L_hand + R_hand + L_foot + R_foot)
    int vmp_pre_standing_frames_{50};
    int vmp_post_standing_frames_{50};
    bool vmp_enable_loop_{true};
    int vmp_pre_interpolation_frames_{40};
    int vmp_post_interpolation_frames_{40};
    double standingHeight_{0.87};
    
    // ========== 多轨迹配置 ==========
    bool multiTrajectoryMode_{false};
    std::vector<std::string> trajectorySequence_;
    bool autoSwitchTrajectory_{true};
    bool trajectoryLoopMode_{true};
    int currentTrajectoryIndex_{0};
    int trajectoryFrameCounter_{0};
    bool trajectoryPlaybackCompleted_{false};
    bool resetSingleTrajectoryFrame_{false};
    std::vector<PreloadedTrajectory> preloadedTrajectories_;
    bool enablePreloading_{false};
    
    // ========== 在线VR配置 ==========
    std::string online_vr_data_source_{"bin_file"};
    std::string online_vr_control_mode_{"upper_body"};
    std::string online_vr_bin_file_;
    int online_buffer_size_{100};
    double online_update_rate_{100.0};
    
    OnlineReferenceBuffer online_ref_buffer_;
    std::mutex online_buffer_mutex_;
    
    // VR设备模式数据
    std::vector<float> latest_online_raw_frame_;
    std::mutex latest_frame_mutex_;
    bool has_received_online_data_{false};
    uint64_t latest_frame_seq_{0};          // callback 写入序列号（每次 callback 递增）
    uint64_t last_consumed_frame_seq_{0};   // sampling loop 上次消费的序列号
    std::thread online_sampling_thread_;
    std::atomic<bool> online_sampling_running_{false};
    std::atomic<bool> online_sampling_has_written_{false};  // 采样线程已完成第一次写入，read指针可以推进
    
    // ========== PICO推流中断/恢复控制 ==========
    std::atomic<bool> pico_streaming_paused_{false};      // PICO推流是否暂停
    std::vector<float> pico_frozen_frame_;                 // 暂停时冻结的帧数据
    std::mutex pico_frozen_frame_mutex_;                   // 冻结帧互斥锁
    ros::ServiceServer pico_stream_control_srv_;           // PICO推流控制服务
    mutable std::mutex retarget_resume_mutex_;
    bool retarget_resume_waiting_{false};
    bool retarget_frozen_frame_valid_{false};
    bool retarget_auto_freeze_valid_{false};
    ros::Time retarget_resume_min_stamp_;
    ros::Time retarget_resume_ready_stamp_;
    ros::Time latest_retargeted_frame_stamp_;
    uint64_t retarget_resume_min_frame_seq_{0};
    uint64_t latest_retargeted_frame_seq_{0};
    double retarget_heading_offset_{0.0};
    bool retarget_heading_offset_valid_{false};
    bool retarget_entry_frame_pending_{false};
    std::vector<float> retarget_entry_frame_;
    ros::Subscriber pico_stream_control_joy_sub_;          // Xsens 不调用服务，直接监听共用手柄话题
    std::atomic<bool> arm_stream_pause_requested_{false};  // RT+Y/RT+X 请求的手臂暂停状态
    std::atomic<bool> joy_pause_combo_active_{false};      // RT+Y 组合键上一帧状态
    std::atomic<bool> joy_resume_combo_active_{false};     // RT+X 组合键上一帧状态
    
    // ========== 多话题数据融合 ==========
    ArmData latest_arm_data_;
    FootData latest_foot_data_;
    std::mutex multi_topic_data_mutex_;
    
    // ========== Estimator模块 ==========
    bool vmp_enable_estimator_{false};
    int vmp_estimator_history_frames_{16};
    int vmp_estimator_input_dim_{83};
    int vmp_estimator_output_dim_{64};
    std::deque<Eigen::VectorXd> vmp_estimator_obs_buffer_;
    Eigen::VectorXd vmp_estimator_output_;
    std::mutex vmp_estimator_mtx_;
    
    // ========== GMR数据平滑 ==========
    bool enable_gmr_smoothing_{true};
    double gmr_gaussian_sigma_{1.0};
    int gmr_gaussian_kernel_size_{5};
    double gmr_velocity_clip_{20.0};
    double gmr_velocity_deadzone_{0.05};
    std::deque<std::vector<float>> gmr_history_buffer_;
    std::vector<float> gmr_gaussian_kernel_;
    bool has_gmr_history_{false};
    std::mutex gmr_smoothing_mutex_;
    
    // GMR关节位置平滑（独立开关和sigma，复用同一个history_buffer）
    bool enable_gmr_qpos_smoothing_{false};
    double gmr_qpos_sigma_{1.0};
    int gmr_qpos_kernel_size_{5};
    std::vector<float> gmr_qpos_kernel_;
    
    // ========== Qpos平滑（重复帧回溯插值） ==========
    bool enable_qpos_smoothing_{true};            // 是否启用Qpos平滑
    int qpos_smooth_buffer_size_{5};              // pending buffer大小（= 输出延迟tick数）
    double qpos_new_joint_eps_{0.001};            // 关节位置"新数据"判定阈值 (rad)
    std::mutex qpos_pending_mutex_;               // 保护 qpos_pending_buffer_（采样线程 vs reset()）
    std::deque<PendingQposSample> qpos_pending_buffer_;  // pending buffer
    std::vector<float> qpos_last_ingested_;       // 上一次入队的真正新数据（用于is_new判定）
    int qpos_stat_new_count_{0};                  // 统计：新数据帧数
    int qpos_stat_repeat_count_{0};               // 统计：重复帧数
    int qpos_stat_interp_count_{0};               // 统计：被回溯插值的帧数
    
    // ========== 离线bin文件播放（测试用） ==========
    std::vector<float> online_bin_playback_data_;
    int online_bin_playback_frame_index_{0};
    int online_bin_playback_total_frames_{0};
    ros::Time online_bin_playback_start_time_;
    
    // ========== ROS订阅器 ==========
    ros::Subscriber pico_retargeted_pose_sub_;
    ros::Subscriber mocap_retargeted_pose_sub_;  // Mocap重定向姿态订阅器
    ros::Subscriber xsense_retargeted_pose_sub_;  // Xsense重定向姿态订阅器
    ros::Subscriber retarget_yaw_resume_ready_sub_;
    ros::Publisher vmp_input_data_pub_;
    
    // ========== VMP服务 ==========
    ros::ServiceServer trigger_vmp_srv_;          // 触发VMP服务
    ros::ServiceServer switch_trajectory_srv_;    // 切换轨迹服务
  };

} // namespace humanoid_controller
