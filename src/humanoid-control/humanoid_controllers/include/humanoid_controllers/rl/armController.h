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
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

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
    // ==================== 手臂控制模式枚举 ====================

    /**
     * @brief 手臂控制模式
     *
     * getMode()/changeMode() 仍以 int 对外（ROS 协议），内部存储用此枚举。
     */
    enum class ControlMode : int
    {
        kLocked    = 0,  // 锁定：固定当前姿态
        kAutoSwing = 1,  // 自动摆臂：RL 策略持臂（站立时回家）
        kExternal  = 2,   // 外部控制：VR / tact 轨迹操控
        kImpedance = 3,   // 关节空间阻抗控制（柔顺交互）
        kCartesianImpedance = 4  // 笛卡尔空间阻抗控制（末端柔顺）
    };

    // === 构造/析构 ===
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

    /**
     * @brief 更新自动摆臂/回家使用的默认手臂位置
     * @param default_arm_pos 默认手臂位置（手臂段）
     */
    void setDefaultArmPos(const Eigen::VectorXd& default_arm_pos);

    // ==================== 阻抗控制配置（Phase 1/2） ====================

    /**
     * @brief 配置关节空间阻抗控制参数（模式3）
     * @param enabled 是否启用
     * @param kp 阻抗刚度（arm-sized，左臂在前右臂在后）
     * @param kd 阻抗阻尼（arm-sized）
     * @param target_source 阻抗目标来源：0=弹簧回默认位(default_arm_pos_)，1=保持进入时的当前位
     * @param max_joint_vel 关节最大速度限制 (rad/s)，超限自动回退锁定
     * @param force_limit 关节力矩上限 (N·m)，超限自动回退锁定
     * @param use_cst 是否使用 CST 力矩模式（下位机直接力矩控制）；false 时退化为高增益位置控制
     * @param lock_wrist 是否锁死手腕关节（每侧最后 3 个关节：arm_idx%7>=4 的 l5/l6/l7 与 r5/r6/r7）。
     *                   为 true 时阻抗模式不控制手腕，手腕目标固定为默认位，且下发保持 CSP(2)+高增益位置锁定
     */
    void configureImpedance(bool enabled,
                            const Eigen::VectorXd& kp,
                            const Eigen::VectorXd& kd,
                            int target_source = 0,
                            double max_joint_vel = 1.5,
                            double force_limit = 20.0,
                            bool use_cst = true,
                            bool lock_wrist = false);

    /**
     * @brief 配置笛卡尔空间阻抗控制参数（模式4）
     * @param enabled 是否启用
     * @param kx 笛卡尔平移刚度 (N/m)
     * @param dx 笛卡尔平移阻尼 (N·s/m)
     * @param ee_frame_mode 末端坐标系模式：0=zarm_l7_end_effector（左手），1=zarm_r7_end_effector（右手）
     * @param force_est_gain 外力估计器增益（动量观测器），0=禁用外力估计
     */
    void configureCartesianImpedance(bool enabled,
                                     double kx = 100.0,
                                     double dx = 5.0,
                                     int ee_frame_mode = 0,
                                     double force_est_gain = 10.0);

    /**
     * @brief 设置实测关节力矩（用于笛卡尔阻抗外力估计，Phase 2）
     * @param joint_torque 实测关节力矩（完整关节数组或手臂段均可，内部按手臂段提取）
     */
    void setMeasuredJointTorque(const Eigen::VectorXd& joint_torque);

    /**
     * @brief 设置手臂关节力矩限制（用于阻抗模式力矩钳位）
     * @param torque_limits 手臂段力矩限制（arm-sized）
     */
    void setTorqueLimits(const Eigen::VectorXd& torque_limits);

    /**
     * @brief 配置“退出阻抗时的手臂过渡”（位置参考 + 增益 + 电机模式）
     *
     * 问题：阻抗模式下手臂的 joint_q 参考 = 阻抗目标位（默认位/进入时位姿）、kp/kd = 阻抗值、
     *       电机模式 = CST(0)；切到其他控制器时这三项会同时阶跃（位置参考跳变 + 增益跳变 + 换律）
     *       → 实测冲击/异响。
     *
     * 本功能：退出阻抗后先保持阻抗时的电机模式，在 handover_duration 内把**所有手臂关节**的：
     *   - joint_q（默认关节位置参考）从阻抗目标平滑过渡到新控制器当前下发的值；
     *   - kp/kd（非手腕关节）从阻抗值平滑过渡到切换后将使用的值（优先 motor_pdo_kp/kd）；
     * 过渡完成后才放开电机模式切换 → 驱动器在“参考与增益已匹配”时换律，冲击最小。
     *
     * @param handover_duration 过渡时长（秒）；<=0 关闭本功能（保持原行为）
     */
    void configureImpedanceHandover(double handover_duration);

    /**
     * @brief 设置交接目标增益（= 退出阻抗后这些关节将使用的 kp/kd，通常来自 info 的 motor_pdo_kp/kd）
     * @param kp/kd 全局关节索引的向量（与 joint_cmd 索引一致），尺寸不匹配时忽略
     */
    void setHandoverTargetGains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);

    /**
     * @brief 配置阻抗模式切出时的“手臂回位门禁”
     *
     * 阻抗模式下手臂偏离目标位（默认位 / 进入时位姿）超过该角度阀值时，不允许切换控制器。
     * 目的：让切出阻抗（EC 关节 CST→CSP 换律、RUIWO 关节增益切换）发生在
     *       手臂已对位、不受力的姿态，减小冲击/异响与安全风险。
     *
     * @param guard_deg 允许切出的最大关节偏差（度）；<=0 关闭门禁（默认 10°）
     */
    void configureImpedanceSwitchGuard(double guard_deg);

    /**
     * @brief 配置“肘关节随肩部 pitch 负向随动”（仅模式3阻抗生效）
     *
     * 肘目标 = 肘参考位 + k · min(0, 肩pitch位移)：
     *  - 肩 pitch 向负向偏移 Δ(<0) 时，肘同向跟随 k·|Δ|；
     *  - 肩 pitch 正向偏移时，肘保持参考位不动。
     * 随动期间肘关节以 CSP(2) 位置模式下发（不参与阻抗柔顺）。
     *
     * @param enabled        是否启用
     * @param ratio          比例系数 k（>0；例如 0.5 = pitch 动 10° 肘跟 5°）
     * @param max_offset_deg 随动角度上限（度，相对参考位；<=0 不限制）
     */
    void configureElbowFollow(bool enabled, double ratio, double max_offset_deg);

    /**
     * @brief 阻抗模式切出门禁：手臂是否已回到阻抗目标位附近
     * @param reason 输出：不允许切出时的原因描述（含最大偏差与阈值，可直接日得得打印）
     * @return true = 允许切出（非阻抗模式 / 偏差在阀值内 / 门禁已关闭）
     */
    bool isImpedanceSwitchAllowed(std::string& reason) const;
    // Physical readiness, separate from completion of the commanded homing curve.
    bool isMeasuredArmAtDefault() const;

    /// Control-thread readiness for a SOFT controller exit.
    bool isImpedanceHandoverActive() const { return impedance_handover_active_; }
    bool isHomingToDefault() const
    {
      return auto_swing_state_ == AutoSwingState::kHoming || is_returning_from_external_;
    }

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
     * @brief 仅更新内部状态（current_arm_pos_/vel_），不触发模式切换或写入 joint_cmd
     */
    void updateInternalState(const Eigen::VectorXd& joint_pos,
                              const Eigen::VectorXd& joint_vel,
                              size_t arm_start_idx);

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
     * @param target_mode 目标模式：0=锁定, 1=自动摆臂, 2=外部控制
     * @return 是否切换成功（false表示指令已缓存）
     */
    bool changeMode(int target_mode);

    /**
     * @brief 从非控制线程请求模式切换
     *
     * 只记录请求；实际模式状态由下一个 update() 控制周期修改
     */
    bool requestModeChange(int target_mode);
    void setExternalCommandBufferCallback(std::function<bool()> callback);

    /**
     * @brief Select the only external trajectory source accepted in mode 2.
     *
     * false: accept generic/VR/SDK input from /kuavo_arm_traj.
     * true:  accept offline actions from /kuavo_action_traj.
     * Switching ownership clears targets left by the previous source.
     */
    void setOfflineActionInputActive(bool active);
    bool isOfflineActionInputActive() const noexcept
    {
      return offline_action_input_active_.load(std::memory_order_acquire);
    }

    /**
     * @brief Start an arm-only return to the configured default pose before a controller switch.
     *
     * The request stays latched so the default-pose PD/gravity command is still emitted while
     * VMP is used as the live source of the subsequent RL->RL blend.
     */
    bool requestDefaultPoseReturnForSwitch();
    bool isDefaultPoseReturnForSwitchComplete() const
    {
      return default_pose_return_for_switch_state_.load(std::memory_order_acquire) ==
             DefaultPoseReturnState::kComplete;
    }
    void cancelDefaultPoseReturnForSwitch();

    /**
     * @brief Pause/resume external (mode 2) arm tracking without changing modes.
     *
     * While paused, the last emitted arm position is held with zero target
     * velocity and incoming external targets are discarded.
     */
    void setExternalControlPaused(bool paused);
    bool isExternalControlPaused() const { return external_control_pause_requested_.load(); }

    /**
     * @brief 注册模式变化回调（在 applyModeChange 末尾触发，含自动回退 Locked）
     * @param callback 回调签名：void(int old_mode, int new_mode)
     *                 模式枚举：0=Locked, 1=AutoSwing, 2=External, 3=JointImpedance, 4=CartesianImpedance
     */
    void setModeChangeCallback(std::function<void(int, int)> callback) { mode_change_callback_ = std::move(callback); }
    
    /**
     * @brief 获取当前控制模式
     * @return 当前模式：0=锁定, 1=自动摆臂, 2=外部控制
     */
    int getMode() const { return static_cast<int>(arm_control_mode_); }

    /**
     * @brief 获取当前已经接受的目标模式。
     *
     * External -> AutoSwing 归位期间，执行外壳仍是 mode2，但目标已经是 mode1。
     * 调用方用该值判断是否需要再次请求，避免每个控制周期重启归位曲线。
     */
    int getRequestedMode() const
    {
      const int pending_mode = requested_arm_mode_.load(std::memory_order_acquire);
      if (pending_mode != kNoRequestedArmMode)
      {
        return pending_mode;
      }
      if (default_pose_return_for_switch_state_.load(std::memory_order_acquire) !=
          DefaultPoseReturnState::kIdle)
      {
        return static_cast<int>(ControlMode::kAutoSwing);
      }
      if (arm_control_mode_ == ControlMode::kExternal && is_returning_from_external_)
      {
        return static_cast<int>(ControlMode::kAutoSwing);
      }
      return static_cast<int>(arm_control_mode_);
    }
    
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

    /**
     * @brief 清除外部模式下的缓冲目标
     */
    void clearExternalTarget();
    /**
     * @brief 检查是否已收到外部目标输入（VR/外部轨迹）
     * @return true表示已收到外部输入，false表示无外部输入
     */
    bool hasExternalTarget() const { return external_target_received_; }

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
    // ==================== 自动摆臂内部子状态 ====================

    /**
     * @brief 自动摆臂（RL 持臂）模式的内部子状态
     *
     * 迁移关系（详见 updateMode1）：
     *   kSwing  --站立指令-->    kHoming   发起回家：生成当前位置→默认位的平滑曲线
     *   kHoming --跑完且站立-->  kHolding  到位后继续按住默认位
     *   kHoming --跑完且行走-->  kSwing    到位后交还 RL 策略
     *   kHolding --起步走-->     kSwing    交还 RL 策略摆臂
     * 注：kHolding 在站立下没有回到 kHoming 的边 —— 到位后不会再次发起回家
     */
    enum class AutoSwingState
    {
        kSwing,    // 摆臂：手臂归 RL 策略，本控制器不填充指令
        kHoming,   // 回家中：平滑曲线回默认位途中（站立或行走都跑完）
        kHolding   // 按住：到位后持续按住默认位（仅站立）
    };

    enum class DefaultPoseReturnState
    {
        kIdle,
        kRequested,
        kStarted,
        kComplete
    };

    // ==================== 内部辅助函数 ====================
    
    /**
     * @brief 应用模式切换的核心逻辑（使用保存的当前位置和速度）
     */
    void applyModeChange(int target_mode, bool force_interpolation_to_default = false);

    // 位置滤波器重置后，下一拍速度差分输出 0，避免用上一段外部控制的位置做差分。
    void resetExternalVelocityDiff();

    /**
     * @brief 在控制线程中取出并执行外部模式请求
     */
    void executeRequestedModeChange();
    
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
    
    /** @brief Generic VR/SDK input callback for /kuavo_arm_traj. */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    /** @brief Offline-action-only input callback for /kuavo_action_traj. */
    void actionTrajectoryCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void applyBufferedMode2TargetIfReady();
    /** @pre external_input_owner_mutex_ is held. Acquires external_target_mutex_. */
    void clearExternalTargetStateForOwnerSwitch();
    void storeMode2Target(const sensor_msgs::JointState& msg,
                          Eigen::VectorXd& target_q,
                          Eigen::VectorXd& target_v) const;
    
    /**
     * @brief 更新锁定状态的期望值
     */
    void updateLockedPose(double dt);

    /**
     * @brief 更新自动摆臂的期望值（含内部子状态机）
     */
    void updateAutoSwing(const ros::Time& time, double dt, int cmd_stance);

    /**
     * @brief 更新外部控制的期望值
     */
    void updateExternalControl(double dt);

    /**
     * @brief Apply an external-control pause transition in the control thread.
     */
    void applyExternalControlPauseState();

    /**
     * @brief Reacquire an external target without the normal error-threshold snap.
     * @return true once every joint has reached the target.
     */
    bool applyStrictRateLimitedInterpolation(double dt,
                                             const Eigen::VectorXd& target_pos,
                                             double max_velocity);

    /**
     * @brief 更新关节空间阻抗控制（模式3）
     * @param dt 控制周期
     */
    void updateImpedanceControl(double dt);

    /**
     * @brief 更新笛卡尔空间阻抗控制（模式4）
     * @param dt 控制周期
     */
    void updateCartesianImpedance(double dt);
    
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
     * @brief 自动模式切换（备用：根据站立/行走指令自动切换手臂模式）
     * @param cmd_stance 命令姿态（0=行走, 1=站立）
     */
    void handleAutoModeSwitch(int cmd_stance);

    /**
     * @brief 填充关节命令消息
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
    ros::Subscriber action_traj_sub_;
    ocs2::humanoid::TopicLogger* ros_logger_;  // ROS日志发布器（可选）
    
    // 关节信息
    size_t joint_num_;        // 腿部关节数量
    size_t joint_waist_num_;  // 腰部关节数量（默认0，后续版本可能有）
    size_t joint_arm_num_;    // 手臂关节数量
    size_t arm_start_idx_;   // 手臂关节的起始索引（joint_num_ + joint_waist_num_）
    
    // 控制模式
    ControlMode arm_control_mode_{ControlMode::kAutoSwing};
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
    
    // 锁定相关
    Eigen::VectorXd locked_fixed_pos_;  // 锁定时保持的目标位置
    bool locked_pos_set_;               // 锁定目标位置是否已设置
    
    // 自动摆臂相关
    Eigen::VectorXd default_arm_pos_;  // 默认手臂位置（自动摆臂时站立回家的目标）
    AutoSwingState auto_swing_state_{AutoSwingState::kSwing}; // 自动摆臂内部子状态（门禁：!= kSwing 时本控制器填充手臂指令）
    bool is_returning_from_external_{false}; // 外部控制→自动摆臂 归位中：外壳保持外部控制，回家完成后翻回
    std::atomic<DefaultPoseReturnState> default_pose_return_for_switch_state_{
        DefaultPoseReturnState::kIdle};
    int default_pose_return_settled_cycles_{0};
    
    // 外部控制相关
    Eigen::VectorXd raw_external_target_q_; // 当前 owner 的外部控制原始目标位置
    Eigen::VectorXd raw_external_target_v_; // 外部控制原始目标速度
    Eigen::VectorXd external_target_q_;  // 外部控制目标位置（滤波或插值后）
    Eigen::VectorXd external_target_v_;  // 外部控制目标速度
    Eigen::VectorXd filtered_pos_for_velocity_; // 上一拍滤波位置，用于速度差分
    bool external_velocity_diff_valid_{false};  // false 时本拍速度差分输出 0
    // 控制线程已用于速度差分的输入源 epoch。回调只递增 external_input_owner_epoch_，
    // 由控制线程在新源的首个目标前比较并作废差分历史。
    std::uint64_t external_velocity_diff_owner_epoch_{0};
    bool external_target_received_;      // 是否已收到外部输入
    Eigen::VectorXd buffered_mode2_target_q_; // 自动切换缓冲期缓存的最新外部目标
    Eigen::VectorXd buffered_mode2_target_v_;
    bool buffered_mode2_target_received_{false};
    std::function<bool()> external_command_buffer_callback_;
    ros::Time last_external_input_time_; // 上一次外部输入的时间戳
    bool last_external_input_time_valid_; // 上一次时间戳是否有效
    // Producer arbitration. The owner mutex is always acquired before the
    // external-target mutex when both are needed.
    std::atomic_bool offline_action_input_active_{false};
    std::atomic<std::uint64_t> external_input_owner_epoch_{0};
    mutable std::mutex external_input_owner_mutex_;
    std::atomic<bool> external_control_pause_requested_{false};
    bool external_control_pause_active_{false};
    bool external_pause_resume_reacquiring_{false};
    bool has_emitted_arm_command_{false};
    Eigen::VectorXd external_pause_hold_q_;
    Eigen::VectorXd external_pause_source_q_;
    bool external_pause_source_valid_{false};
    
    // 手臂关节指令低通滤波相关
    LowPassFilter2ndOrder arm_joint_pos_filter_; // 手臂位置指令低通滤波器
    LowPassFilter2ndOrder arm_joint_vel_filter_; // 手臂速度指令低通滤波器
    double external_cutoff_freq_{10.0};    // 外部输入低通截止频率
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
    static constexpr int kNoRequestedArmMode = -1;
    std::atomic<int> requested_arm_mode_{kNoRequestedArmMode}; // 跨线程模式请求邮箱
    std::optional<ControlMode> pending_arm_mode_; // 缓存的待执行模式
    bool is_interpolating_;             // 曲线引擎正在跑（锁定/回家/外部靠拢各插值机制共用的底层标志）

    // 自动模式切换（备用）
    int prev_cmd_stance_{-1};           // 上一帧的站立/行走指令（handleAutoModeSwitch 用）

    // 力矩控制器
    std::unique_ptr<ArmTorqueController> arm_torque_controller_;
    
    // 控制参数（从外部传入）
    Eigen::VectorXd joint_kp_;          // 位置增益（手臂部分）
    Eigen::VectorXd joint_kd_;          // 速度增益（手臂部分）
    Eigen::VectorXd torque_limits_;     // 力矩限制（手臂部分）

    // ==================== 阻抗控制参数（Phase 1/2） ====================

    // 关节空间阻抗（模式3）
    bool impedance_enabled_{false};                  // 关节阻抗是否启用
    Eigen::VectorXd impedance_kp_;                   // 关节阻抗刚度（arm-sized）
    Eigen::VectorXd impedance_kd_;                   // 关节阻抗阻尼（arm-sized）
    int impedance_target_source_{0};                 // 0=弹簧回默认位，1=保持进入时位姿
    double impedance_max_joint_vel_{1.5};            // 关节最大速度限制 (rad/s)
    double impedance_force_limit_{20.0};             // 关节力矩上限 (N·m)
    bool impedance_use_cst_{true};                   // 是否使用 CST 力矩模式
    bool impedance_lock_wrist_{false};               // 阻抗模式下锁死手腕关节（目标=默认位，CSP+高增益下发）
    Eigen::VectorXd impedance_hold_pos_;             // 目标来源=1 时保持的进入位姿
    bool impedance_hold_pos_set_{false};

    // 笛卡尔空间阻抗（模式4）
    bool cartesian_impedance_enabled_{false};         // 笛卡尔阻抗是否启用
    double cartesian_kx_{100.0};                      // 笛卡尔平移刚度 (N/m)
    double cartesian_dx_{5.0};                        // 笛卡尔平移阻尼 (N·s/m)
    int cartesian_ee_frame_mode_{0};                  // 0=左手末端，1=右手末端
    double cartesian_force_est_gain_{10.0};           // 外力估计器增益
    // 实测关节力矩（Phase 2 外力估计输入，完整关节数组大小，内部按手臂段提取）
    Eigen::VectorXd measured_joint_torque_;
    bool measured_joint_torque_valid_{false};
    // 笛卡尔阻抗目标位姿缓存（进入模式4时锁定）
    Eigen::Isometry3d cartesian_hold_pose_;
    bool cartesian_hold_pose_set_{false};
    // 笛卡尔阻抗计算出的力矩（由 updateCartesianImpedance 填充，fillJointCmdMessage 直接使用）
    Eigen::VectorXd cartesian_tau_override_;
    bool cartesian_tau_override_valid_{false};

    // 模式变化回调（applyModeChange 末尾触发，含自动回退 Locked）
    std::function<void(int, int)> mode_change_callback_;

    // ==================== 退出阻抗时的手臂过渡（位置参考 + 增益 + 模式） ====================
    // 先平滑过渡 joint_q 与 kp/kd（保持阻抗时的电机模式），过渡完成后才放开模式切换
    // → 避免“位置参考跳变 + 增益跳变 + 换律”同时发生导致的冲击/异响
    double impedance_handover_duration_{0.0};   // 过渡时长（s）；<=0 = 关闭
    bool impedance_handover_active_{false};     // 是否正在过渡
    bool impedance_handover_from_cartesian_{false};
    ros::Time impedance_handover_start_time_;   // 过渡开始时间
    Eigen::VectorXd handover_from_q_;           // 过渡起点：阻抗时的位置参考（arm-sized）
    Eigen::VectorXd handover_from_v_;           // 过渡起点：阻抗时的速度参考（arm-sized，通常为 0）
    Eigen::VectorXd handover_from_kp_;          // 过渡起点 kp（arm-sized）
    Eigen::VectorXd handover_from_kd_;          // 过渡起点 kd（arm-sized）
    Eigen::VectorXd handover_target_kp_;        // 过渡目标 kp（全局关节索引，来自 motor_pdo）
    Eigen::VectorXd handover_target_kd_;        // 过渡目标 kd（全局关节索引）
    // 阻抗模式切出时的“手臂回位门禁”：关节偏差超过阈值（度）时不允许切换控制器
    double impedance_switch_guard_deg_{10.0};   // <=0 关闭
    bool elbow_follow_enabled_{false};          // 肘随肩pitch负向随动（仅模式3）
    double elbow_follow_ratio_{0.5};            // 随动比例系数 k
    double elbow_follow_max_offset_{30.0 * M_PI / 180.0};  // 随动角度上限 (rad)，<=0 不限制
    Eigen::VectorXd cartesian_hold_joint_pos_;  // 模式4进入时的关节位姿（用于回位判定）
    void applyImpedanceHandover(const ros::Time& time, kuavo_msgs::jointCmd& joint_cmd_msg);

    // 线程安全
    mutable std::mutex state_mutex_;    // 状态访问互斥锁
    mutable std::mutex external_target_mutex_; // ROS 外部目标回调与控制线程同步
};

} // namespace humanoid_controller

#endif // ARM_CONTROLLER_H
