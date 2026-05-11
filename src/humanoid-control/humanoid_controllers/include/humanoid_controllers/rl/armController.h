#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/changeArmCtrlMode.h>
#include "humanoid_controllers/armTorqueController.h"
#include "humanoid_interface/common/TopicLogger.h"
#include "humanoid_controllers/LowPassFilter.h"
#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace humanoid_controller
{

/**
 * @brief 手臂控制器类
 * 
 * 封装所有手臂相关的功能，极度简化humanoidController的调用。
 * 提供统一的更新接口，内部处理所有逻辑。
 * 
 * 功能包括：
 * - 模式切换（0=全0关节控制, 1=RL控制, 2=VR控制）
 * - VR输入处理和限速跟踪
 * - 指令缓存机制
 * - 期望状态更新
 * - 力矩计算和命令生成
 * 
 * @note 关节排序：腿 + 腰 + 手
 *       - 腿部关节：索引 0 到 joint_num_-1
 *       - 腰部关节：索引 joint_num_ 到 joint_num_+joint_waist_num_-1（如果joint_waist_num_>0）
 *       - 手臂关节：索引 joint_num_+joint_waist_num_ 到 joint_num_+joint_waist_num_+joint_arm_num_-1
 */
class ArmController
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param joint_num 腿部关节数量
     * @param joint_waist_num 腰部关节数量
     * @param joint_arm_num 手臂关节数量
     * @param ros_logger ROS日志发布器（可选，用于发布状态信息）
     * @note 关节排序：腿 + 腰 + 手
     */
    ArmController(ros::NodeHandle& nh, size_t joint_num, size_t joint_waist_num, size_t joint_arm_num,
                  ocs2::humanoid::TopicLogger* ros_logger = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~ArmController();

    // ==================== 初始化函数 ====================
    
    /**
     * @brief 初始化手臂控制器
     * @param urdf_path URDF文件路径（用于初始化力矩控制器）
     * @param joint_kp 位置增益（手臂部分，可选，用于初始化力矩控制器）
     * @param joint_kd 速度增益（手臂部分，可选，用于初始化力矩控制器）
     * @return 是否初始化成功
     */
    bool initialize(const std::string& urdf_path = "",
                     const Eigen::VectorXd& joint_kp = Eigen::VectorXd(),
                     const Eigen::VectorXd& joint_kd = Eigen::VectorXd());
    
    /**
     * @brief 加载配置参数（从.info文件）
     * @param max_tracking_velocity 模式1的最大跟踪速度 (rad/s)
     * @param tracking_error_threshold 跟踪误差阈值 (rad)
     * @param mode_interpolation_velocity 模式2的插值速度 (rad/s)，用于外部控制模式
     * @param default_arm_pos 默认手臂位置（用于模式1站立时的插值目标）
     * @param mode2_cutoff_freq 模式2外部输入的截止频率 (Hz)，默认10Hz
     */
    void loadSettings(double max_tracking_velocity, 
                     double tracking_error_threshold,
                     double mode_interpolation_velocity = 1.0,
                     const Eigen::VectorXd& default_arm_pos = Eigen::VectorXd(),
                     double mode2_cutoff_freq = 2.0);

    // ==================== 主要更新函数（核心接口）====================
    
    /**
     * @brief 更新手臂控制（主循环调用，整合所有逻辑）
     *
     * 此函数整合了以下功能：
     * 1. 自动模式切换（根据cmd_stance）
     * 2. 期望状态更新（根据当前模式）
     * 3. 限速跟踪和指令缓存处理
     * 4. 力矩计算
     * 5. 命令消息填充（仅在模式0和2时，只更新joint_q、joint_v和tau，保留其他原有值）
     *
     * @param time 当前时间
     * @param dt 控制周期（秒）
     * @param joint_pos 当前关节位置（完整，包括腿+腰+手）
     * @param joint_vel 当前关节速度（完整，包括腿+腰+手）
     * @param cmd_stance 命令姿态（0=行走, 1=站立）
     * @param joint_cmd_msg 输出：关节命令消息（仅在模式0和2时更新手臂的joint_q、joint_v和tau）
     * @note 模式1（RL控制）不更新命令消息，由RL控制器处理
     */
    void update(const ros::Time& time,
                double dt,
                const Eigen::VectorXd& joint_pos,
                const Eigen::VectorXd& joint_vel,
                int cmd_stance,
                kuavo_msgs::jointCmd& joint_cmd_msg);

    /**
     * @brief 平滑插值到目标位置（专用插值函数）
     *
     * 使用五次多项式插值从当前位置平滑移动到目标位置，并直接替换joint_cmd_msg中的手臂指令。
     * 插值完成后会自动切换到RL控制（返回false）。
     *
     * @param time 当前时间
     * @param dt 控制周期（秒）
     * @param joint_pos 当前关节位置（完整，包括腿+腰+手）
     * @param joint_vel 当前关节速度（完整，包括腿+腰+手）
     * @param target_pos 目标手臂位置
     * @param joint_cmd_msg 输出：关节命令消息（直接更新手臂的joint_q、joint_v和tau）
     * @param interpolation_duration 插值持续时间（秒），默认1.0秒
     * @return true表示正在插值，false表示插值完成
     */
    bool interpolateToTarget(const ros::Time& time,
                           double dt,
                           const Eigen::VectorXd& joint_pos,
                           const Eigen::VectorXd& joint_vel,
                           const Eigen::VectorXd& target_pos,
                           kuavo_msgs::jointCmd& joint_cmd_msg,
                           double interpolation_duration = 1.0);

    // ==================== 模式控制接口 ====================
    
    /**
     * @brief 切换手臂控制模式（简化接口，使用update中保存的当前位置和速度）
     * @param target_mode 目标模式：0=固定到当前动作, 1=自动摆手, 2=外部控制
     * @return 是否切换成功（false表示指令已缓存）
     */
    bool changeMode(int target_mode);
    
    /**
     * @brief 获取当前控制模式
     * @return 当前模式：0, 1, 或 2
     */
    int getMode() const { return arm_control_mode_; }
    
    /**
     * @brief 检查VR控制是否启用
     * @return true表示VR控制启用
     */
    bool isVREnabled() const { return arm_vr_enabled_; }
    
    /**
     * @brief 获取期望手臂位置（用于RL模式）
     * @return 期望位置向量
     */
    const Eigen::VectorXd& getDesiredPosition() const { return desire_arm_q_; }
    
    /**
     * @brief 获取期望手臂速度（用于RL模式）
     * @return 期望速度向量
     */
    const Eigen::VectorXd& getDesiredVelocity() const { return desire_arm_v_; }
    
    /**
     * @brief 重置控制器状态（重置插值状态和缓存的cmd_arm_pos_）
     */
    void reset();

    // ==================== 状态查询接口 ====================
    
    /**
     * @brief 检查是否处于限速跟踪状态
     * @return true表示正在限速跟踪
     */
    bool isRateLimitedTracking() const { return is_interpolating_; }
    
    /**
     * @brief 获取手臂关节在完整关节数组中的起始索引
     * @return 手臂关节起始索引（joint_num_ + joint_waist_num_）
     */
    size_t getArmStartIndex() const { return arm_start_idx_; }

    // ==================== 回调函数（供ROS服务使用） ====================
    
    /**
     * @brief 模式切换服务回调（简化接口，使用update中保存的当前位置和速度）
     * @param req 请求
     * @param res 响应
     */
    bool changeModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                          kuavo_msgs::changeArmCtrlMode::Response &res);
    
    /**
     * @brief 获取模式服务回调
     */
    bool getModeCallback(kuavo_msgs::changeArmCtrlMode::Request &req,
                        kuavo_msgs::changeArmCtrlMode::Response &res);

private:
    // ==================== 内部辅助函数 ====================
    
    /**
     * @brief 应用模式切换的核心逻辑（使用保存的当前位置和速度）
     */
    void applyModeChange(int target_mode);
    
    /**
     * @brief 执行缓存的模式切换指令（使用保存的当前位置和速度）
     */
    void executePendingModeChange();
    
    /**
     * @brief 限速逼近函数
     */
    void applyRateLimitedApproach(double dt, 
                                  const Eigen::VectorXd& target_pos,
                                  const Eigen::VectorXd& target_vel);
    
    /**
     * @brief VR输入回调函数（处理/kuavo_arm_traj话题）
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    /**
     * @brief 更新模式0（固定到当前动作）的期望状态
     */
    void updateMode0(double dt);
    
    /**
     * @brief 更新模式1（自动摆手）的期望状态
     */
    void updateMode1(const ros::Time& time, double dt, int cmd_stance);
    
    /**
     * @brief 更新模式2（外部控制）的期望状态
     */
    void updateMode2(double dt);
    
    /**
     * @brief 限速插值函数（用于模式1和模式2）
     * @param dt 时间步长
     * @param target_pos 目标位置
     * @param target_vel 目标速度
     * @param max_velocity 最大插值速度
     */
    void applyRateLimitedInterpolation(double dt,
                                      const Eigen::VectorXd& target_pos,
                                      const Eigen::VectorXd& target_vel,
                                      double max_velocity);
    
    /**
     * @brief 处理自动模式切换（根据cmd_stance）
     * @param cmd_stance 命令姿态（0=行走, 1=站立）
     */
    void handleAutoModeSwitch(int cmd_stance);
    
    /**
     * @brief 填充关节命令消息（模式0和2使用）
     * 只更新joint_q、joint_v和tau，保留其他原有值
     */
    void fillJointCmdMessage(kuavo_msgs::jointCmd& joint_cmd_msg,
                            const Eigen::VectorXd& target_q,
                            const Eigen::VectorXd& target_v,
                            const Eigen::VectorXd& joint_pos,
                            const Eigen::VectorXd& joint_vel);

    /**
     * @brief 平滑插值函数（五次多项式）
     * @param current_time 当前时间
     * @param target_pos 目标位置
     * @param target_vel 目标速度
     */
    void applySmoothInterpolation(const ros::Time& current_time,
                                 const Eigen::VectorXd& target_pos,
                                 const Eigen::VectorXd& target_vel);
    
    /**
     * @brief 重置插值起始状态
     */
    void resetInterpolationState(const ros::Time& time, const Eigen::VectorXd& start_pos, const Eigen::VectorXd& target_pos);

    // ==================== 成员变量 ====================
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    ocs2::humanoid::TopicLogger* ros_logger_;  // ROS日志发布器（可选）
    
    // 关节信息
    size_t joint_num_;        // 腿部关节数量
    size_t joint_waist_num_;  // 腰部关节数量（默认0，后续版本可能有）
    size_t joint_arm_num_;    // 手臂关节数量
    size_t arm_start_idx_;   // 手臂关节的起始索引（joint_num_ + joint_waist_num_）
    
    // 控制模式
    int arm_control_mode_{1};    // 0=固定手臂, 1=RL控制自动摆手, 2=外部控制
    bool arm_vr_enabled_{false};     // VR控制使能标志
    
    // 当前状态（从update中保存）
    Eigen::VectorXd current_arm_pos_;  // 当前手臂位置（从update中保存）
    Eigen::VectorXd current_arm_vel_;  // 当前手臂速度（从update中保存）
    
    // 指令状态（从joint_cmd_msg中提取，用于保持指令一致性）
    Eigen::VectorXd cmd_arm_pos_;  // 指令手臂位置（从joint_cmd_msg中提取）
    Eigen::VectorXd cmd_arm_vel_;  // 指令手臂速度（从joint_cmd_msg中提取）
    
    // 期望状态（内部管理，通过getter访问）
    Eigen::VectorXd desire_arm_q_;  // 期望手臂位置
    Eigen::VectorXd desire_arm_v_;  // 期望手臂速度
    
    // 模式0相关
    Eigen::VectorXd mode0_fixed_pos_;  // 模式0固定位置（切换到模式0时的位置）
    bool mode0_fixed_pos_set_;         // 模式0固定位置是否已设置
    
    // 模式1相关
    Eigen::VectorXd default_arm_pos_;  // 默认手臂位置（用于模式1站立时的插值目标）
    bool is_interpolating_to_default_; // 是否正在插值到默认位置
    
    // 模式2相关（外部控制）
    Eigen::VectorXd raw_mode2_target_q_; // 模式2原始目标位置（从/kuavo_arm_traj获取）
    Eigen::VectorXd raw_mode2_target_v_; // 模式2原始目标速度
    Eigen::VectorXd mode2_target_q_;  // 模式2目标位置（滤波或插值后）
    Eigen::VectorXd mode2_target_v_;  // 模式2目标速度
    bool mode2_target_received_;      // 是否已收到模式2的目标
    ros::Time last_mode2_input_time_; // 上一次模式2输入的时间戳
    bool last_mode2_input_time_valid_; // 上一次时间戳是否有效
    
    // 手臂关节指令低通滤波相关
    LowPassFilter2ndOrder arm_joint_pos_filter_; // 手臂位置指令低通滤波器
    LowPassFilter2ndOrder arm_joint_vel_filter_; // 手臂速度指令低通滤波器
    double mode2_cutoff_freq_{10.0};    // 滤波器截止频率
    bool arm_filter_initialized_{false}; // 滤波器是否已初始化
    
    // 限速跟踪参数
    double arm_max_tracking_velocity_;     // 模式1的最大跟踪速度 (rad/s)
    double arm_tracking_error_threshold_;  // 跟踪误差阈值 (rad)
    double mode_interpolation_velocity_;  // 模式2的插值速度 (rad/s)
    
    // 平滑插值相关
    ros::Time interpolation_start_time_;   // 插值起始时间
    Eigen::VectorXd interpolation_start_pos_; // 插值起始位置
    double interpolation_duration_;        // 插值总持续时间

    // 专用插值函数相关（复用现有插值状态）
    Eigen::VectorXd target_interpolation_target_pos_; // 专用插值目标位置
    
    // 指令缓存机制
    int pending_arm_mode_;              // 缓存的待执行模式（-1表示无缓存）
    bool has_pending_mode_change_;      // 是否有待执行的模式切换指令
    bool is_interpolating_;             // 是否正在插值（用于判断是否可以切换模式）
    
    // 力矩控制器
    std::unique_ptr<ArmTorqueController> arm_torque_controller_;
    
    // 控制参数（从外部传入）
    Eigen::VectorXd joint_kp_;          // 位置增益（手臂部分）
    Eigen::VectorXd joint_kd_;          // 速度增益（手臂部分）
    Eigen::VectorXd torque_limits_;     // 力矩限制（手臂部分）
    
    // 自动模式切换状态
    int last_cmd_stance_;               // 上一次的cmd_stance状态
    
    // 线程安全
    mutable std::mutex state_mutex_;    // 状态访问互斥锁
};

} // namespace humanoid_controller

#endif // ARM_CONTROLLER_H
