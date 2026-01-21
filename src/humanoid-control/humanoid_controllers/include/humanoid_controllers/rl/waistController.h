#ifndef WAIST_CONTROLLER_H
#define WAIST_CONTROLLER_H

#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/robotWaistControl.h>
#include "humanoid_interface/common/TopicLogger.h"
#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace humanoid_controller
{

/**
 * @brief 腰部控制器类
 * 
 * 封装所有腰部相关的功能，参考 ArmController 的设计。
 * 提供统一的更新接口，内部处理所有逻辑。
 * 
 * 功能包括：
 * - 模式切换（0=固定位置, 1=RL控制, 2=外部控制）
 * - 外部输入处理和限速跟踪
 * - PD控制
 * - 期望状态更新
 * - 命令生成
 */
class WaistController
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param joint_waist_num 腰部关节数量
     * @param ros_logger ROS日志发布器（可选，用于发布状态信息）
     */
    WaistController(ros::NodeHandle& nh, size_t joint_waist_num,
                    ocs2::humanoid::TopicLogger* ros_logger = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~WaistController();

    // ==================== 初始化函数 ====================
    
    /**
     * @brief 加载配置参数（从.info文件）
     * @param waist_kp PD控制位置增益
     * @param waist_kd PD控制速度增益
     * @param default_waist_pos 模式1的默认位置
     * @param waist_mode_interpolation_velocity 腰部模式切换时的插值速度 (rad/s)，用于三次多项式插值
     */
    void loadSettings(const Eigen::VectorXd& waist_kp = Eigen::VectorXd(),
                     const Eigen::VectorXd& waist_kd = Eigen::VectorXd(),
                     const Eigen::VectorXd& default_waist_pos = Eigen::VectorXd(),
                     double waist_mode_interpolation_velocity = 1.0);

    // ==================== 主要更新函数 ====================
    
    /**
     * @brief 更新腰部控制
     * 
     * 此函数整合了以下功能：
     * 1. 期望状态更新（根据当前模式）
     * 2. 限速跟踪和指令缓存处理
     * 3. 命令消息填充（仅在模式0和2时，只更新joint_q、joint_v和joint_kp、joint_kd）
     * 
     * @param time 当前时间
     * @param dt 控制周期（秒）
     * @param joint_pos 当前关节位置（完整，包括腿+腰+手）
     * @param joint_vel 当前关节速度（完整，包括腿+腰+手）
     * @param cmd_stance 命令姿态（0=行走, 1=站立）
     * @param joint_cmd_msg 输出：关节命令消息（仅在模式0和2时更新腰部的joint_q、joint_v和joint_kp、joint_kd）
     * @param jointNumReal 腿部关节数量（用于正确索引jointCmdMsg中的腰部部分）
     * @note 模式1（RL控制）不更新命令消息，由RL控制器处理
     */
    void update(const ros::Time& time,
                double dt,
                const Eigen::VectorXd& joint_pos,
                const Eigen::VectorXd& joint_vel,
                int cmd_stance,
                kuavo_msgs::jointCmd& joint_cmd_msg,
                size_t jointNumReal);

    // ==================== 模式控制接口 ====================
    
    /**
     * @brief 切换腰部控制模式
     * @param target_mode 目标模式：0=固定位置, 1=RL控制, 2=外部控制
     * @return 是否切换成功
     */
    bool changeMode(int target_mode);
    
    /**
     * @brief 获取当前控制模式
     * @return 当前模式：0, 1, 或 2
     */
    int getMode() const { return waist_control_mode_; }
    
    /**
     * @brief 启用/禁用腰部控制覆盖
     * @param enabled 是否启用
     * @note 启用时：如果当前是RL模式（模式1），自动切换到外部控制模式（模式2）
     *       禁用时：自动切换回RL控制模式（模式1）
     */
    void enable(bool enabled) 
    { 
      waist_control_enabled_ = enabled;
      if (enabled)
      {
        // 启用时：如果当前是RL模式，自动切换到外部控制模式（模式2）
        if (waist_control_mode_ == 1)
        {
          changeMode(2);  // 默认外部控制模式，调用changeMode确保初始化逻辑被执行
        }
      }
      else
      {
        // 禁用时：切换回RL控制模式（模式1）
        if (waist_control_mode_ != 1)
        {
          changeMode(1);  // 调用changeMode确保初始化逻辑被执行
        }
      }
    }
    
    /**
     * @brief 获取期望腰部位置
     * @return 期望位置向量
     */
    const Eigen::VectorXd& getDesiredPosition() const { return desire_waist_q_; }
    
    /**
     * @brief 获取期望腰部速度
     * @return 期望速度向量
     */
    const Eigen::VectorXd& getDesiredVelocity() const { return desire_waist_v_; }
    
    /**
     * @brief 重置控制器状态（重置插值状态）
     */
    void reset();

    // ==================== 状态查询接口 ====================
    
    /**
     * @brief 检查是否处于限速跟踪状态
     * @return true表示正在限速跟踪
     */
    bool isRateLimitedTracking() const { return waist_is_interpolating_; }
    
    /**
     * @brief 获取腰部关节在完整关节数组中的起始索引
     * @return 腰部关节起始索引（jointNumReal）
     */
    size_t getWaistStartIndex(size_t jointNumReal) const { return jointNumReal; }

private:
    // ==================== 内部辅助函数 ====================
    
    /**
     * @brief 腰部轨迹回调函数（处理/robot_waist_motion_data话题）
     */
    void waistTrajectoryCallback(const kuavo_msgs::robotWaistControl::ConstPtr& msg);
    
    /**
     * @brief 更新模式0（固定位置）的期望腰部状态
     */
    void updateMode0(double dt);
    
    /**
     * @brief 更新模式2（外部控制）的期望腰部状态
     */
    void updateMode2(double dt);
    
    /**
     * @brief 应用模式切换的核心逻辑
     */
    void applyModeChange(int target_mode);
    
    /**
     * @brief 平滑插值函数（三次多项式，用于模式切换）
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
    void resetInterpolationState(const ros::Time& time, 
                                const Eigen::VectorXd& start_pos, 
                                const Eigen::VectorXd& target_pos);

    // ==================== 成员变量 ====================
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber waist_traj_sub_;  // 订阅/robot_waist_motion_data话题
    ocs2::humanoid::TopicLogger* ros_logger_;  // ROS日志发布器（可选）
    
    // 关节信息
    size_t joint_waist_num_;  // 腰部关节数量
    
    // 控制模式
    int waist_control_mode_{1};  // 0=固定位置, 1=RL控制, 2=外部控制
    bool waist_control_enabled_{false};  // 是否启用腰部控制覆盖
    
    // 当前状态（从update中保存）
    Eigen::VectorXd current_waist_pos_;  // 当前腰部位置
    Eigen::VectorXd current_waist_vel_;  // 当前腰部速度
    
    // 期望状态（内部管理，通过getter访问）
    Eigen::VectorXd desire_waist_q_;  // 期望腰部位置
    Eigen::VectorXd desire_waist_v_;  // 期望腰部速度
    
    // 模式0相关
    Eigen::VectorXd mode0_fixed_waist_pos_;  // 模式0固定位置
    bool mode0_fixed_waist_pos_set_;         // 模式0固定位置是否已设置
    
    // 模式1相关
    Eigen::VectorXd default_waist_pos_;  // 默认腰部位置（用于模式1站立时）
    
    // 模式2相关（外部控制）
    Eigen::VectorXd raw_mode2_waist_target_q_;  // 模式2原始目标位置
    Eigen::VectorXd mode2_waist_target_q_;  // 模式2目标位置（用于插值）
    bool mode2_waist_target_received_;  // 是否已收到模式2的目标
    
    // 限速跟踪参数
    double waist_mode_interpolation_velocity_;  // 模式切换时的平滑插值速度 (rad/s)，用于三次多项式插值
    
    // 平滑插值相关
    ros::Time waist_interpolation_start_time_;  // 插值起始时间
    Eigen::VectorXd waist_interpolation_start_pos_;  // 插值起始位置
    double waist_interpolation_duration_;  // 插值总持续时间
    bool waist_is_interpolating_;  // 是否正在插值
    
    // 控制参数（从外部传入）
    Eigen::VectorXd waist_kp_;  // 位置增益（腰部部分）
    Eigen::VectorXd waist_kd_;  // 速度增益（腰部部分）
    
    // 线程安全
    mutable std::mutex state_mutex_;    // 状态访问互斥锁
};

} // namespace humanoid_controller

#endif // WAIST_CONTROLLER_H
