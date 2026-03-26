#pragma once

#include "humanoid_controllers/rl/RLControllerBase.h"
#include "humanoid_controllers/rl/vmp/vmp_types.h"
#include "kuavo_solver/ankle_solver.h"

#include "kuavo_msgs/GetStringList.h"
#include "kuavo_msgs/SetString.h"
#include "kuavo_msgs/VMPTrajectoryState.h"
#include "kuavo_msgs/ExecuteArmAction.h"
#include <std_srvs/Trigger.h>

#include <memory>
#include <openvino/openvino.hpp>

namespace humanoid_controller
{
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
    VMPController(const std::string& name,
                  const std::string& config_file,
                  ros::NodeHandle& nh,
                  ocs2::humanoid::TopicLogger* ros_logger = nullptr);

    ~VMPController() override = default;

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
     * @brief 暂停控制器（暂停推理过程并停止轨迹播放）
     */
    void pause() override;

    /**
     * @brief 恢复控制器（恢复推理过程并重置状态）
     */
    void resume() override;

    /**
     * @brief 检查控制器是否准备好退出
     * @return 如果控制器已完成任务并准备好退出，返回true
     */
    bool isReadyToExit() const override;

  protected:
    /**
     * @brief 更新控制器实现
     * @param time 当前时间
     * @param sensor_data 传感器数据
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 输出的关节命令
     * @return 是否更新成功
     */
    bool updateImpl(const ros::Time& time,
                    const SensorData& sensor_data,
                    const Eigen::VectorXd& measuredRbdState,
                    kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 推理过程
     * @param observation 观测数据
     * @param action 输出的动作
     * @return 是否推理成功
     */
    bool inference(const Eigen::VectorXd& observation,
                   Eigen::VectorXd& action) override;

    /**
     * @brief 更新观测数据
     * @param state_est 状态估计
     * @param sensor_data 传感器数据
     */
    void updateObservation(const Eigen::VectorXd& state_est,
                           const SensorData& sensor_data) override;

    /**
     * @brief 将动作转换为关节命令
     * @param actuation 执行器输出
     * @param measuredRbdState 测量的刚体状态
     * @param joint_cmd 关节命令
     */
    void actionToJointCmd(const Eigen::VectorXd& actuation,
                          const Eigen::VectorXd& measuredRbdState,
                          kuavo_msgs::jointCmd& joint_cmd) override;

    /**
     * @brief 检查是否应该执行推理
     * @return 如果应该执行推理，返回true
     */
    bool shouldRunInference() const override;

    void preprocessSensorData(SensorData& sensor_data) override;

    //==========================================================================
    // ROS 接口相关方法
    //==========================================================================

    /**
     * @brief 初始化 ROS 服务和发布者
     */
    void initROSServices();

    // 轨迹状态发布定时器回调 (50Hz)
    void trajectoryStateTimerCallback(const ros::TimerEvent& event);

    // 获取轨迹列表服务回调
    bool getTrajectoryListServiceCallback(kuavo_msgs::GetStringList::Request& req,
                                          kuavo_msgs::GetStringList::Response& res);

    // 执行轨迹服务回调
    bool executeTrajectoryServiceCallback(kuavo_msgs::SetString::Request& req,
                                          kuavo_msgs::SetString::Response& res);

    // 停止轨迹服务回调
    bool stopTrajectoryServiceCallback(std_srvs::Trigger::Request& req,
                                       std_srvs::Trigger::Response& res);

  private:
    //==========================================================================
    // 私有方法
    //==========================================================================

    /**
     * @brief 加载并编译VMP策略模型和编码器模型
     * @details 使用OpenVINO加载.xml模型文件，编译到CPU设备
     */
    void setupVMPModels();

    /**
     * @brief 预热VMP模型，消除首次推理延迟
     * @details 使用随机数据进行一次完整的推理流程
     */
    void warmupVMPModels();

    /**
     * @brief 加载VMP参考轨迹数据
     * @details 初始化轨迹播放状态，根据配置决定是否预加载
     */
    void loadVMPRefData();

    /**
     * @brief 在轨迹数据前后添加静止帧
     * @details 用于轨迹播放的平滑过渡
     */
    void appendStandingFramesToTaskData();

    /**
     * @brief 加载所有轨迹到内存
     * @return 是否全部加载成功
     */
    bool loadAllTrajectories();

    /**
     * @brief 加载单条轨迹
     * @param index 轨迹索引
     * @return 是否加载成功
     */
    bool loadTrajectory(size_t index);

    /**
     * @brief 处理轨迹（添加静止帧等）
     * @param index 轨迹索引
     */
    void processTrajectory(size_t index);

    /**
     * @brief 切换到指定轨迹
     * @param index 轨迹索引
     * @return 是否切换成功
     */
    bool switchToTrajectory(size_t index);

    /**
     * @brief 获取轨迹占用的内存大小
     * @return 内存占用字节数
     */
    size_t getMemoryUsage() const;

    /**
     * @brief 打印轨迹菜单（切换时显示）
     */
    void printTrajectoryMenu() const;

    /**
     * @brief 打印轨迹状态信息（调试用）
     */
    void printMultiTrajectoryStatus() const;

    /**
     * @brief 更新VMP参考运动缓冲区
     * @details 从轨迹数据中读取下一帧，维护参考运动滑动窗口
     */
    void updateVMPReferenceMotion();

    /**
     * @brief 计算VMP动作
     * @param observation 机器人观测向量
     * @return 计算得到的关节动作向量
     * @details 使用编码器处理参考运动，与观测拼接后输入策略网络
     */
    Eigen::VectorXd computeVMPAction(const Eigen::VectorXd& observation);

    /**
     * @brief 应用时序归一化到输入数据
     * @param input_data 输入数据指针
     * @param data_size 数据大小
     */
    void applyTemporalNormalization(float* input_data, size_t data_size);

    /**
     * @brief 计算关节控制命令
     * @param state 机器人状态向量
     * @return 执行器输出（力矩或位置命令）
     * @details 根据当前动作和传感器数据计算PD控制命令
     */
    Eigen::VectorXd updateRLcmd(const Eigen::VectorXd& state);

  private:
    //==========================================================================
    // 基础配置参数
    //==========================================================================
    double dt_{0.002};                              ///< 控制周期，和 /wbc_frequency 一致

    //==========================================================================
    // 机器人配置
    //==========================================================================
    bool is_real_{false};                           ///< 是否为真实机器人
    bool withArm_{true};                            ///< 是否包含手臂控制
    AnkleSolver ankleSolver_;                       ///< 脚踝解算器

    //==========================================================================
    // 观测相关参数
    //==========================================================================
    int frameStackRL_{1};                           ///< 帧堆叠数量
    int numSingleObsRL_{0};                         ///< 单一时刻观测维度
    std::vector<std::string> singleInputDataKeys_;  ///< 观测数据键列表
    std::map<std::string, Eigen::Vector3d> singleInputDataID_;  ///< key -> {startIdx, numIdx, obsScale}
    std::deque<Eigen::VectorXd> inputDeque_;        ///< 历史观测队列
    Eigen::VectorXd singleInputDataRL_;             ///< 单一时刻观测向量
    double clipObservationsRL_{18.0};               ///< 观测裁剪限制

    //==========================================================================
    // 动作相关参数
    //==========================================================================
    double actionScale_{0.25};                      ///< 动作缩放
    double clipActions_{18.0};                      ///< 动作裁剪限制
    Eigen::Vector4d velocityLimits_;                ///< 速度限制

    //==========================================================================
    // 滤波器相关
    //==========================================================================
    LowPassFilter2ndOrder jointCmdFilter_;          ///< 关节命令滤波器
    Eigen::VectorXd jointCmdFilterState_;           ///< 滤波器状态

    //==========================================================================
    // OpenVINO 模型相关
    //==========================================================================
    ov::Core core_;                                 ///< OpenVINO核心
    ov::CompiledModel vmp_policy_model_;            ///< VMP策略模型
    ov::CompiledModel vmp_encoder_model_;           ///< VMP编码器模型
    ov::InferRequest vmp_policy_request_;           ///< 策略推理请求
    ov::InferRequest vmp_encoder_request_;          ///< 编码器推理请求
    std::string vmpModelPath_;                      ///< VMP策略模型路径
    std::string vmpEncoderPath_;                    ///< VMP编码器模型路径

    //==========================================================================
    // VMP 配置参数
    //==========================================================================
    VMPConfig vmp_config_;                          ///< VMP参数配置
    bool vmp_enable_theta_normalization_{true};     ///< 是否启用theta yaw角归一化
    int numRefMotionObs_{77};                       ///< 参考运动观测维度
    int numEncoderObs_{512};                        ///< 编码器输出维度
    int episodeLengthS_{30};                        ///< episode长度(秒)
    int decimation_{10};                            ///< 降采样率

    //==========================================================================
    // 轨迹数据相关
    //==========================================================================
    std::string vmpRefDataDir_;                     ///< VMP参考数据目录
    std::vector<float> vmp_task_data_;              ///< 当前轨迹数据
    std::deque<Eigen::VectorXd> vmp_ref_motion_buffer_;  ///< 参考运动缓冲区

    //==========================================================================
    // 轨迹数据（包含配置和预加载数据）
    //==========================================================================
    std::vector<vmp::TrajectoryData> trajectories_;  ///< 轨迹数据列表
    int currentTrajectoryIndex_{0};                 ///< 当前轨迹索引
    int trajectoryFrameCounter_{0};                 ///< 当前轨迹帧计数器
    bool trajectoryPlaybackCompleted_{true};        ///< 轨迹播放完成标志（默认true，等待命令触发播放）

    //==========================================================================
    // 静止帧与插值配置
    //==========================================================================
    std::vector<float> vmp_standing_frame_;         ///< 静止帧数据
    Eigen::VectorXd vmp_standing_joint_pos_;        ///< 静止帧关节位置
    int vmp_pre_standing_frames_{50};               ///< 前置静止帧数
    int vmp_post_standing_frames_{50};              ///< 后置静止帧数
    int vmp_pre_interpolation_frames_{40};          ///< 前置插值帧数
    int vmp_post_interpolation_frames_{40};         ///< 后置插值帧数

    //==========================================================================
    // ROS 服务和发布者
    //==========================================================================
    ros::ServiceServer srv_get_trajectory_list_;    ///< 获取轨迹列表服务
    ros::ServiceServer srv_execute_trajectory_;      ///< 执行轨迹服务
    ros::ServiceServer srv_stop_trajectory_;        ///< 停止轨迹服务
    ros::Publisher pub_trajectory_state_;           ///< 轨迹状态发布者
    ros::Timer trajectory_state_timer_;             ///< 轨迹状态发布定时器 (50Hz)

    /**
     * @brief 轨迹播放状态枚举
     */
    enum class TrajectoryPlaybackState {
      PLAYING,    ///< 正在播放
      STOPPED,    ///< 已停止
      COMPLETED   ///< 播放完成
    };
    TrajectoryPlaybackState playback_state_{TrajectoryPlaybackState::STOPPED};  ///< 当前播放状态（默认STOPPED，等待命令触发播放）
  };

} // namespace humanoid_controller
